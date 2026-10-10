/**
  ******************************************************************************
  * @file    foc_app.c
  * @brief   SguanFOC 应用适配层实现
  *
  * 实时链路（NVIC 抢占优先级见各 MspInit）：
  *   ADC_IRQn(0)   15 kHz  JEOC -> 缓存 JDR1~3 -> SguanFOC_High_Loop()
  *   TIM2_IRQn(1)   4 kHz  内部 /4 -> AS5600_Update()（1 kHz 轮询读）-> SguanFOC_Low_Loop()
  *   while(1)       最低   SguanFOC_main_Loop() + 10 Hz FocApp_DiagnoseLoop()
  *
  * 上位机 JustFloat 协议已关闭：USART1 不挂接收，User_CorrespondSet() 为空实现，
  * 速度给定直接来自本文件的 FocApp_SpeedCmd_RadS。
  * USART1 现用于 CAW_LOG 排查日志（BSP_Driver/log.c，阻塞发送），日志只允许在
  * while(1) 上下文调用：启动序列、状态机跳变、10 Hz 诊断与 500 ms 遥测。
  *
  * 中断入口沿用 HAL 的弱回调（HAL_ADCEx_InjectedConvCpltCallback /
  * HAL_TIM_PeriodElapsedCallback），stm32f4xx_it.c
  * 保持 CubeMX 原样，重新生成代码不会丢失接线。
  ******************************************************************************
  */
#include "foc_app.h"

#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"

#include "as5600.h"
#include "drv8301.h"
#include "log.h"
#include "SguanFOC.h"

/* TIM2 4 kHz 分频到 SguanFOC_Low_Loop() 的 1 kHz */
#define FOC_LOW_LOOP_DIV            4U
/* while(1) 里 DRV8301 / AS5600 状态轮询周期 */
#define FOC_DIAG_PERIOD_MS          100U
/* 每 N 个诊断周期打一组遥测日志（5 -> 500 ms），日志是阻塞串口，不能更密 */
#define FOC_LOG_TELEMETRY_DIV       5U
/* 角度数据允许的最大陈旧时间：1 kHz 刷新下正常不超过 1 ms */
#define FOC_ENCODER_MAX_AGE_MS      5U
/*
 * 连续多少个 1 kHz 周期判定角度陈旧才置故障。单次 I2C 抖动会让 AS5600 的 valid
 * 短暂为假（下一次轮询读 1 ms 后即恢复），不做去抖会误关断电机。
 * 必须大于 as5600.h 的 AS5600_MAX_STEP_REJECT(3)：步进拒绝连续 3 次后第 4 次才重同步
 * 基准，这里若也取 3 会在重同步前一拍就锁死，自愈机制永远跑不到。
 */
#define FOC_ENCODER_STALE_LIMIT     5U
/*
 * 给定值上限。全部低于 safe.Dcur_MAX/Qcur_MAX 的故障阈值，命令本身不该把状态机
 * 顶进 OVERCURRENT。HT3510：额定 0.53 A / 0.11 N.m，堵转 0.80 A，最大 965 rpm。
 */
#define FOC_IQ_LIMIT_A              1.0F    /* 与 Velocity.OutMax 一致 */
#define FOC_SPEED_LIMIT_RAD_S       100.0F  /* 965 rpm = 101 rad/s，取整留余量 */
/* 开环 Uq 是调制指令，静止时电流 = Uq/(sqrt(3)*Rs)。6.0 -> 0.81 A，约等于堵转电流。
   方案阶段 4 的方向标定从 0.5 V 缓升到 2 V，本限幅只防误输入 */
#define FOC_UQ_OPEN_LIMIT           6.0F

/*
 * 速度给定 [rad/s]，正负决定转向，上限 FOC_SPEED_LIMIT_RAD_S（约 965 rpm）。
 * 上位机协议已关闭，改这里即改转速；volatile 便于在调试器里在线改值，
 * 每个 15 kHz 周期由 FocApp_ApplyTargets() 限幅后落库。10 rad/s ≈ 95 rpm。
 */
volatile float FocApp_SpeedCmd_RadS = 10.0F;

typedef struct
{
    volatile float uq_open;   /* Velocity_OPEN_MODE：Uq 调制指令 */
    volatile float iq;        /* Current_SINGLE_MODE：Q 轴电流给定 [A] */
    volatile float pos;       /* PosVelCur_THREE_MODE：机械角度给定 [rad] */
} FocApp_Target_T;

/* I2C1 上只挂了 AS5600，实例归应用层持有 */
static AS5600_T g_encoder;

/* ADC 注入组 rank1~3 原始值：0=IA_FB(PA3) 1=IB_FB(PA4) 2=IC_FB(PA5，仅诊断) */
static volatile int32_t s_adc_raw[3];

static volatile uint8_t s_signal[FOC_SIGNAL_COUNT];
static FocApp_Target_T  s_target;

static volatile uint8_t s_mode_request;
static volatile uint8_t s_mode_pending;

static uint8_t  s_low_div;
static uint8_t  s_stale_count;
/* 诊断轮询/初始化的阻塞 I2C 与 TIM2 的轮询读互斥，置位期间跳过角度刷新 */
static volatile uint8_t s_i2c_blocking;
static uint32_t s_diag_tick;

/*
 * 排查日志用的状态缓存。日志走阻塞串口（一行约 10 ms），只能放在 while(1) 上下文，
 * 且周期性读数只在变化时打印，避免 10 Hz 轮询刷屏。初值取不可能出现的值，
 * 保证上电后第一次读到的状态必定打印一次。
 */
static volatile uint32_t s_high_loop_count;     /* 15 kHz 高频环存活计数，ADC 中断内自增 */
static uint32_t s_log_high_loop_last;
static uint8_t  s_log_enc_state = 0xFFU;        /* 0 正常 / 1 读失败 */
static uint8_t  s_log_status_group = 0xFFU;
static uint8_t  s_log_status_last = 0xFFU;
static uint8_t  s_log_div;

static float FocApp_Clamp(float value, float limit);
static void  FocApp_CheckEncoderFresh(void);
static void  FocApp_ApplyModeRequest(void);
static const char *FocApp_StatusName(uint8_t status);
static void  FocApp_LogStatusChange(void);
static void  FocApp_LogTelemetry(void);

/* ================= 生命周期 ================= */

void FocApp_Init(void)
{
    uint8_t i;

    for (i = 0U; i < FOC_SIGNAL_COUNT; i++)
    {
        s_signal[i] = 0U;
    }
    s_adc_raw[0] = 0;
    s_adc_raw[1] = 0;
    s_adc_raw[2] = 0;

    s_target.uq_open = 0.0F;
    s_target.iq = 0.0F;
    s_target.pos = 0.0F;

    s_mode_request = VelCur_DOUBLE_MODE;
    s_mode_pending = 0U;
    s_low_div = 0U;
    s_stale_count = 0U;
    s_i2c_blocking = 0U;
    s_diag_tick = HAL_GetTick();

    /*
     * 上位机协议已关闭，上电标定完成后直接进速度-电流双环，按 FocApp_SpeedCmd_RadS
     * 转动。前提是 UserData_Motor.h 的 Motor_Dir / Current_Dir 已按方案阶段 4 标定，
     * 方向反了速度环会正反馈飞车，只能靠 OVERCURRENT 兜底。
     */
    Sguan.mode = VelCur_DOUBLE_MODE;

    /* 库在 while(1) 的 Sguan_Start_Tick() 里看到 0x01 才会跑初始化与标定 */
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
}

/*
 * 由 User_InitialInit() 调用，运行在 while(1) 上下文，允许阻塞。
 * 顺序不可调换：EN_GATE 拉高前 DRV8301 的 SPI 完全不响应，必须第一步。
 */
void FocApp_HwStart(void)
{
    DRV8301_Result_t drv_result;
    uint16_t drv_status1 = 0U;
    uint16_t drv_status2 = 0U;
    int encoder_result;

    /* 日志格式串只用 ASCII：armcc(AC5) 按系统代码页解码源码，UTF-8 中文会吞掉后续引号 */
    CAW_LOG_INFO("HwStart: begin (DRV8301 -> AS5600 -> TIM2 -> TIM1/ADC)");

    /*
     * 重启路径（CO=2）进来时 CCR 还冻结在上一帧的电压矢量上，而 status<4 期间
     * 库不再更新 CCR，DRV8301_Init() 又要阻塞 10 ms，之后 Offset_CurrentRead()
     * 还要 48 ms。不先归零会把这个矢量当直流持续加在电机上近 60 ms。
     * 首次上电时 CCR 已是 ConfigChannel 写入的 ARR/2-1，本段等效空操作。
     */
    FocApp_Shutdown();
    FocApp_SetPwmDuty(PWM_PERIOD / 2U, PWM_PERIOD / 2U, PWM_PERIOD / 2U);
    Sguan.foc.Ud_in = 0.0F;
    Sguan.foc.Uq_in = 0.0F;

    /* 本函数里的 AS5600_Init() 是阻塞 I2C，重启时 TIM2 的轮询读已在跑，两者不能在
       同一总线上重入，先把角度刷新挡住。轮询读在中断内同步完成，置位后不会再有在途传输 */
    s_i2c_blocking = 1U;

    /*
     * 库的 Offset_CurrentRead() 是 24 次累加求平均，但没有先清零累加器。
     * 重启时不清会叠加上一轮结果，导致电流零偏整体偏移。
     */
    Sguan.current.Pos_offset0 = 0;
    Sguan.current.Pos_offset1 = 0;

    drv_result = DRV8301_Init(&hspi2);
    if (drv_result != DRV8301_OK)
    {
        /* 配置未落地时器件可能停在默认 6PWM 模式，必须立刻硬关断 */
        DRV8301_Shutdown();
        s_signal[FOC_SIGNAL_SENSOR_ERROR] = 1U;
        /* -2 SPI 超时 / -3 帧错误 / -4 地址不符 / -5 回读不符 / -6 FAULT 未清，见 drv8301.h */
        CAW_LOG_ERROR("DRV8301_Init FAILED ret=%d, EN_GATE pulled low", (int)drv_result);
    }
    else if (DRV8301_ReadStatus(&drv_status1, &drv_status2) == DRV8301_OK)
    {
        CAW_LOG_INFO("DRV8301 configured STATUS1=0x%04X STATUS2=0x%04X DEV_ID=%u",
                     (unsigned)drv_status1, (unsigned)drv_status2,
                     (unsigned)(drv_status2 & DRV8301_SR2_DEVICE_ID_Msk));
    }
    else
    {
        CAW_LOG_WARN("DRV8301 configured but status readback FAILED");
    }

    encoder_result = AS5600_Init(&g_encoder, &hi2c1);
    if (encoder_result != 0)
    {
        s_signal[FOC_SIGNAL_ENCODER_ERROR] = 1U;
        /* 具体失败点（STATUS 读失败 / 磁体不合法 / CONF 回读 / RAW_ANGLE）由 as5600.c 内部打点。
           空闲总线 SCL(PB6)/SDA(PB7) 都应读到 1：读到 0 说明无上拉、线被拉死或从机卡住 */
        CAW_LOG_ERROR("AS5600_Init FAILED ret=%d, I2C1 idle level SCL=%u SDA=%u (both must be 1)",
                      encoder_result,
                      (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
                      (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));
    }
    else
    {
        CAW_LOG_INFO("AS5600 ready, angle=%.4f rad raw=%u",
                     (double)g_encoder.mechanical_angle, (unsigned)g_encoder.raw_angle);
    }
    s_i2c_blocking = 0U;

    /* 低频状态机挂在 TIM2 上，故障态也要靠它锁定，必须无条件启动 */
    (void)HAL_TIM_Base_Start_IT(&htim2);
    CAW_LOG_INFO("TIM2 started (1 kHz state machine + AS5600 poll)");

    if (s_signal[FOC_SIGNAL_SENSOR_ERROR] != 0U)
    {
        CAW_LOG_ERROR("gate driver not ready, PWM/ADC NOT started, waiting for SENSOR_ERROR");
        return; /* 栅极驱动器未就绪，不开 PWM 与 ADC，等状态机进 SENSOR_ERROR */
    }

    /* 三相 CCR 初值均为 ARR/2-1，是零矢量，上电瞬间无相间电压 */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* CH4 未分配引脚，只用 OC4REF 经 TRGO 触发 ADC 注入组 */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /*
     * 重启路径（CO=2）上通道状态已是 BUSY，HAL_TIM_PWM_Start 直接返回 HAL_ERROR，
     * 不会重新置 MOE；而本函数开头的 FocApp_Shutdown() 已撤掉 MOE。CCER 与计数器
     * 仍在运行，只需显式恢复 MOE。首次上电时本行等效空操作。
     */
    __HAL_TIM_MOE_ENABLE(&htim1);

    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
    /*
     * 返回后库几微秒内就进 Offset_CurrentRead()，而首次注入触发最迟要 66.7 us。
     * 不等待则第一个采样读到 FocApp_Init() 置的 0，24 点平均被拉低约 85 LSB
     * （≈0.69 A 的假电流零偏）。等 1~2 ms 让 s_adc_raw 先刷新十几组有效数据。
     */
    HAL_Delay(1U);
    /* 零矢量下三路都应接近 2048(REF/2)；全 0 说明 TIM1 TRGO -> ADC 注入触发链没通 */
    CAW_LOG_INFO("TIM1 PWM + ADC injected started, first raw=%ld/%ld/%ld hi=%lu",
                 (long)s_adc_raw[0], (long)s_adc_raw[1], (long)s_adc_raw[2],
                 (unsigned long)s_high_loop_count);
}

void FocApp_Shutdown(void)
{
    /*
     * 先撤 MOE 让六路输出立即进 IdleState，再拉低 EN_GATE 断栅极电源。
     * htim1.Instance 判空是为了让 HardFault 等早期上下文也能安全调用。
     */
    if (htim1.Instance != NULL)
    {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    }
    DRV8301_Shutdown();
}

void FocApp_DiagnoseLoop(void)
{
    uint32_t now = HAL_GetTick();
    int encoder_result;
    uint8_t enc_state;

    /* 状态机跳变不走 100 ms 节拍，主循环每次经过都比对一次，少漏过渡态 */
    FocApp_LogStatusChange();

    if ((uint32_t)(now - s_diag_tick) < FOC_DIAG_PERIOD_MS)
    {
        return;
    }
    s_diag_tick = now;

    /* 这里是阻塞 I2C，与 TIM2 发起的轮询读不能在同一总线上重入。
       磁体检测判定已取消，CheckStatus 只返回 0 读取正常 / -2 读失败，不再触发 ENCODER_ERROR */
    s_i2c_blocking = 1U;
    encoder_result = AS5600_CheckStatus(&g_encoder); /* -2 是读失败，下个周期重试 */
    s_i2c_blocking = 0U;

    enc_state = (encoder_result == 0) ? 0U : 1U;
    if (enc_state != s_log_enc_state)
    {
        s_log_enc_state = enc_state;
        if (enc_state == 1U)
        {
            CAW_LOG_WARN("AS5600 STATUS register I2C read FAILED");
        }
        else
        {
            CAW_LOG_INFO("AS5600 STATUS read OK (magnet check disabled)");
        }
    }

    s_log_div++;
    if (s_log_div >= FOC_LOG_TELEMETRY_DIV)
    {
        s_log_div = 0U;
        FocApp_LogTelemetry();
    }
}

/* ================= 供 UserData_*.h 转发的原语 ================= */

int32_t FocApp_GetCurrentRaw(uint8_t ch)
{
    /* Current_Num = 0（AB 采样）：ch0->IA_FB，ch1->IB_FB；ch2 是 IC_FB，仅供诊断比对 */
    return (ch < 3U) ? s_adc_raw[ch] : 0;
}

float FocApp_GetEncoderRad(void)
{
    /*
     * 15 kHz ISR 内调用，必须非阻塞：只取 AS5600_Update() 发布的缓存值。
     * 单个 float 32 位对齐，Cortex-M4 上读写原子，被抢占不会读到撕裂值。
     */
    return AS5600_GetOnceAngle(&g_encoder);
}

void FocApp_SetPwmDuty(uint16_t du, uint16_t dv, uint16_t dw)
{
    /* CCR 预载已在 tim.c 打开，写入值在下一次 UEV 生效，不会在半周期内跳变 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, du);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dv);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dw);
}

uint8_t FocApp_GetFaultFlag(uint8_t which)
{
    uint8_t value;

    if (which >= FOC_SIGNAL_COUNT)
    {
        return 0U;
    }

    value = s_signal[which];
    /*
     * STANDBY / UNINITIALIZED 是边沿请求：库每 1 kHz 查一次，读后自清。
     * 不自清会把状态机永久钉在待机态。其余四个是电平语义的故障锁存。
     */
    if ((which == FOC_SIGNAL_STANDBY) || (which == FOC_SIGNAL_UNINITIALIZED))
    {
        s_signal[which] = 0U;
    }
    return value;
}

void FocApp_ApplyTargets(void)
{
    FocApp_ApplyModeRequest();

    /* 一期不做弱磁与 MTPA，D 轴励磁恒为 0 */
    switch (Sguan.mode)
    {
    case Velocity_OPEN_MODE:
        Sguan.foc.Ud_in = 0.0F;
        Sguan.foc.Uq_in = s_target.uq_open;
        break;
    case Current_SINGLE_MODE:
        Sguan.foc.Target_Id = 0.0F;
        Sguan.foc.Target_Iq = s_target.iq;
        break;
    case VelCur_DOUBLE_MODE:
        Sguan.foc.Target_Id = 0.0F;
        /* 给定来自代码内变量，可能被调试器在线改成任意值，落库前必须限幅 */
        Sguan.foc.Target_Speed = FocApp_Clamp(FocApp_SpeedCmd_RadS, FOC_SPEED_LIMIT_RAD_S);
        break;
    case PosVelCur_THREE_MODE:
        Sguan.foc.Target_Id = 0.0F;
        /* Target_Pos 是 double，跨中断的 64 位写不原子，统一在本环内落库 */
        Sguan.foc.Target_Pos = (double)s_target.pos;
        break;
    default:
        break;
    }
}

/* ================= 目标值与控制字 ================= */

void FocApp_SetTarget(float value)
{
    switch (Sguan.mode)
    {
    case Velocity_OPEN_MODE:
        /* 开环没有电流闭环兜底，限幅按静止电流不超过堵转电流取 */
        s_target.uq_open = FocApp_Clamp(value, FOC_UQ_OPEN_LIMIT);
        break;
    case Current_SINGLE_MODE:
        /* 命令上限严格低于 Qcur_MAX 故障阈值，避免正常给定就把状态机顶进 OVERCURRENT。
           要复现方案阶段 5 的过流保护测试，临时把 FOC_IQ_LIMIT_A 调到 Qcur_MAX 以上 */
        s_target.iq = FocApp_Clamp(value, FOC_IQ_LIMIT_A);
        break;
    case VelCur_DOUBLE_MODE:
        FocApp_SpeedCmd_RadS = FocApp_Clamp(value, FOC_SPEED_LIMIT_RAD_S);
        break;
    case PosVelCur_THREE_MODE:
        s_target.pos = value; /* 多圈定位不限幅，由速度环输出限幅约束过程量 */
        break;
    default:
        break;
    }
}

void FocApp_SetMode(uint8_t mode)
{
    if (mode > PosVelCur_THREE_MODE)
    {
        return;
    }
    /* 调用方只登记请求，真正切换放到 15 kHz 环，避免撕裂控制器状态 */
    s_mode_request = mode;
    s_mode_pending = 1U;
}

void FocApp_SetControlWord(float value)
{
    if (value < 0.5F)
    {
        /* CO=0：急停并锁定，由 MOTOR_STATUS_EMERGENCY_STOP_Loop() 硬关断 */
        s_signal[FOC_SIGNAL_EMERGENCY_STOP] = 1U;
        return;
    }

    /*
     * 解除锁定必须同时清掉已锁存的故障，否则状态机下一拍会立刻重新锁定。
     * 故障若仍存在，10 Hz 诊断会在 100 ms 内重新置位。
     */
    s_signal[FOC_SIGNAL_STANDBY] = 0U;
    s_signal[FOC_SIGNAL_ENCODER_ERROR] = 0U;
    s_signal[FOC_SIGNAL_SENSOR_ERROR] = 0U;
    s_signal[FOC_SIGNAL_EMERGENCY_STOP] = 0U;
    s_signal[FOC_SIGNAL_DISABLED] = 0U;
    s_stale_count = 0U;

    if (value < 1.5F)
    {
        /* CO=1：解除锁定进待机。栅极保持关断的安全空闲态，不会立刻重新上电对齐 */
        s_signal[FOC_SIGNAL_STANDBY] = 1U;
        return;
    }

    /*
     * CO=2：重新走一遍 Sguan_Start_Tick()（DRV8301 复位配置 + 电流零偏与零位重标定）。
     * 库的 STANDBY -> UNINITIALIZED 自动跳转是死代码（Status_Switch_Loop 会先
     * return），只能直接写 status。此处不能再置 STANDBY 请求，否则下一拍 1 kHz
     * 状态机会把 status 拉回 0，重启永远起不来。
     */
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
}

/* ================= 内部实现 ================= */

static float FocApp_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static void FocApp_ApplyModeRequest(void)
{
    if (s_mode_pending == 0U)
    {
        return;
    }
    s_mode_pending = 0U;

    if (s_mode_request == Sguan.mode)
    {
        return;
    }

    /* 上一模式的给定量在新模式下量纲完全不同，必须清零 */
    s_target.uq_open = 0.0F;
    s_target.iq = 0.0F;
    FocApp_SpeedCmd_RadS = 0.0F;
    /* 位置模式以当前位置为初始目标，避免切入瞬间往绝对零位大幅跳转 */
    s_target.pos = (float)Sguan.encoder.Real_Pos;
    Sguan.foc.Ud_in = 0.0F;
    Sguan.foc.Uq_in = 0.0F;

    /* 控制器历史量（含积分）在新模式下已失效，不复位会造成切入瞬间的力矩阶跃 */
    PID_Init(&Sguan.control.Current_D);
    PID_Init(&Sguan.control.Current_Q);
#if Open_PI_Control
    PID_Init(&Sguan.control.Velocity);
#else
    Ladrc_Init(&Sguan.control.Speed);
#endif
    PID_Init(&Sguan.control.Position);

    Sguan.mode = s_mode_request;
}

static void FocApp_CheckEncoderFresh(void)
{
    if (AS5600_IsFresh(&g_encoder, HAL_GetTick(), FOC_ENCODER_MAX_AGE_MS))
    {
        s_stale_count = 0U;
        return;
    }

    if (s_stale_count < FOC_ENCODER_STALE_LIMIT)
    {
        s_stale_count++;
        if (s_stale_count >= FOC_ENCODER_STALE_LIMIT)
        {
            s_signal[FOC_SIGNAL_ENCODER_ERROR] = 1U;
        }
    }
}

/* ================= 排查日志（只在 while(1) 上下文调用，串口阻塞） ================= */

/* 状态机编号 -> 短名，完整定义见 Sguan_MotorStatus.h。0x05~0x0D 统称 RUN */
static const char *FocApp_StatusName(uint8_t status)
{
    switch (status)
    {
    case MOTOR_STATUS_STANDBY:          return "STANDBY";
    case MOTOR_STATUS_UNINITIALIZED:    return "UNINIT";
    case MOTOR_STATUS_INITIALIZING:     return "INIT";
    case MOTOR_STATUS_CALIBRATING:      return "CALIB";
    case MOTOR_STATUS_IDLE:             return "IDLE";
    case MOTOR_STATUS_OVERVOLTAGE:      return "OVERVOLT";
    case MOTOR_STATUS_UNDERVOLTAGE:     return "UNDERVOLT";
    case MOTOR_STATUS_OVERTEMPERATURE:  return "OVERTEMP";
    case MOTOR_STATUS_UNDERTEMPERATURE: return "UNDERTEMP";
    case MOTOR_STATUS_OVERCURRENT:      return "OVERCURRENT";
    case MOTOR_STATUS_ENCODER_ERROR:    return "ENCODER_ERR";
    case MOTOR_STATUS_SENSOR_ERROR:     return "SENSOR_ERR";
    case MOTOR_STATUS_PWM_CALC_FAULT:   return "PWM_FAULT";
    case MOTOR_STATUS_EMERGENCY_STOP:   return "ESTOP";
    case MOTOR_STATUS_DISABLED:         return "DISABLED";
    default:
        return (status <= MOTOR_STATUS_POSITION_HOLD) ? "RUN" : "UNKNOWN";
    }
}

/*
 * 状态机跳变日志。运行态（加/减速、位置增/减等 0x05~0x0D）在库的
 * Status_Switch_Loop 里逐拍互相覆盖，归为一组避免刷屏；其余状态逐个打印。
 */
static void FocApp_LogStatusChange(void)
{
    uint8_t status = Sguan.status;
    uint8_t group = ((status >= MOTOR_STATUS_TORQUE_INCREASING) &&
                     (status <= MOTOR_STATUS_POSITION_HOLD)) ? MOTOR_STATUS_TORQUE_INCREASING : status;

    if (group != s_log_status_group)
    {
        if (s_log_status_group == 0xFFU)
        {
            CAW_LOG_INFO("status init %u(%s) mode=%u",
                         (unsigned)status, FocApp_StatusName(status), (unsigned)Sguan.mode);
        }
        else
        {
            CAW_LOG_INFO("status %u(%s) -> %u(%s) mode=%u",
                         (unsigned)s_log_status_last, FocApp_StatusName(s_log_status_last),
                         (unsigned)status, FocApp_StatusName(status), (unsigned)Sguan.mode);
        }
        s_log_status_group = group;
    }
    s_log_status_last = status;
}

/*
 * 500 ms 一组遥测。判读要点：
 *   hi      两次遥测间 15 kHz 高频环的执行次数，正常约 7500，为 0 说明 ADC 中断停了
 *   Ud/Uq   出现 nan 说明控制器被 NaN 污染（如 FW_MTPA 对 Ld==Lq 除零）
 *   duty    三路都是 0 等于三相下管全开（刹车短路），电机被锁死且无电流
 *   raw     零矢量下约 2048，不随电流变化说明 ADC 采样链没通
 *   ec/sr   AS5600 连续读失败次数 / 连续步进拒绝次数，成功一次即清零。ENCODER_ERR 时
 *           ec 大 sr 为 0 是 I2C 读失败（EMI/总线），sr 非 0 是角度跳变被拒（磁体弱）
 */
static void FocApp_LogTelemetry(void)
{
    uint32_t high_loop_delta = s_high_loop_count - s_log_high_loop_last;
    s_log_high_loop_last = s_high_loop_count;

    CAW_LOG_INFO("TLM st=%u(%s) mode=%u ts=%.1f rs=%.1f rad=%.3f eoff=%.3f ec=%u sr=%u",
                 (unsigned)Sguan.status, FocApp_StatusName(Sguan.status), (unsigned)Sguan.mode,
                 (double)Sguan.foc.Target_Speed, (double)Sguan.encoder.Real_Speed,
                 (double)Sguan.encoder.Real_Rad, (double)Sguan.encoder.Pos_offset,
                 (unsigned)g_encoder.error_count, (unsigned)g_encoder.step_reject_count);
    CAW_LOG_INFO("TLM Ia=%.3f Ib=%.3f Id=%.3f Iq=%.3f Tid=%.3f vout=%.3f",
                 (double)Sguan.current.Real_Ia, (double)Sguan.current.Real_Ib,
                 (double)Sguan.current.Real_Id, (double)Sguan.current.Real_Iq,
                 (double)Sguan.foc.Target_Id, (double)Sguan.control.Velocity.run.Output);
    CAW_LOG_INFO("TLM Ud=%.2f Uq=%.2f duty=%u/%u/%u raw=%ld/%ld hi=%lu pwm_busy=%u",
                 (double)Sguan.foc.Ud_in, (double)Sguan.foc.Uq_in,
                 (unsigned)Sguan.foc.Duty_u, (unsigned)Sguan.foc.Duty_v, (unsigned)Sguan.foc.Duty_w,
                 (long)s_adc_raw[0], (long)s_adc_raw[1],
                 (unsigned long)high_loop_delta, (unsigned)Sguan.flag.PWM_Calc);
}

/* ================= HAL 中断回调（弱符号覆盖） ================= */

/* ADC 注入组转换完成，15 kHz。整条控制链路的时基 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    /* 先整组搬走 JDR：本组已结束，下一组要到 66.67 us 之后 */
    s_adc_raw[0] = (int32_t)hadc->Instance->JDR1;
    s_adc_raw[1] = (int32_t)hadc->Instance->JDR2;
    s_adc_raw[2] = (int32_t)hadc->Instance->JDR3;
    /* 只做存活计数供遥测日志判断高频环是否还在跑，中断内不能打日志 */
    s_high_loop_count++;

    SguanFOC_High_Loop();
}

/* TIM2 更新中断，4 kHz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
    {
        return;
    }

    s_low_div++;
    if (s_low_div >= FOC_LOW_LOOP_DIV)
    {
        s_low_div = 0U;

        /*
         * 编码器刷新必须在中断里：上电标定阶段库会用 User_Delay(1000) 阻塞主循环，
         * 而 Offset_EncoderRead() 需要此时的新鲜角度。
         * 轮询读在 100 kHz 下约 0.5 ms，4 kHz 每周期都读会让本中断占满 CPU、
         * 饿死主循环，故降为 1 kHz，与低速环同拍，读完即做新鲜度判定。
         * 新鲜度判定必须和刷新放在同一个条件里：主循环主动置 s_i2c_blocking 挡住刷新
         * 的窗口内，样本必然变旧，这段时间不能反过来判成编码器故障。
         */
        if (s_i2c_blocking == 0U)
        {
            (void)AS5600_Update(&g_encoder);
            FocApp_CheckEncoderFresh();
        }

        SguanFOC_Low_Loop();
    }
}
