/**
  ******************************************************************************
  * @file    foc_app.c
  * @brief   SguanFOC 应用适配层实现
  *
  * 实时链路（NVIC 抢占优先级见各 MspInit）：
  *   ADC_IRQn(0)   15 kHz  JEOC -> 缓存 JDR1~3 -> SguanFOC_High_Loop()
  *   TIM2_IRQn(1)   4 kHz  AS5600_UpdateStart()；内部 /4 -> SguanFOC_Low_Loop()
  *   I2C1/DMA1(2,3) 事件   AS5600 角度 DMA 读完成 / 出错
  *   USART1(4)      事件   上位机调参帧接收
  *   while(1)       最低   SguanFOC_main_Loop() + 10 Hz FocApp_DiagnoseLoop()
  *
  * 中断入口沿用 HAL 的弱回调（HAL_ADCEx_InjectedConvCpltCallback /
  * HAL_TIM_PeriodElapsedCallback / HAL_I2C_* / HAL_UART_*），stm32f4xx_it.c
  * 保持 CubeMX 原样，重新生成代码不会丢失接线。
  ******************************************************************************
  */
#include "foc_app.h"

#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "as5600.h"
#include "drv8301.h"
#include "SguanFOC.h"

#include <string.h>

/* TIM2 4 kHz 分频到 SguanFOC_Low_Loop() 的 1 kHz */
#define FOC_LOW_LOOP_DIV            4U
/* while(1) 里 DRV8301 / AS5600 状态轮询周期 */
#define FOC_DIAG_PERIOD_MS          100U
/* 角度数据允许的最大陈旧时间：4 kHz 刷新下正常不超过 1 ms */
#define FOC_ENCODER_MAX_AGE_MS      5U
/*
 * 连续多少个 1 kHz 周期判定角度陈旧才置故障。单次 I2C 抖动会让 AS5600 的 valid
 * 短暂为假（下一次 DMA 读 250 us 后即恢复），不做去抖会误关断电机。
 */
#define FOC_ENCODER_STALE_LIMIT     3U
/* JustFloat 一帧 12*4+4 = 52 字节 */
#define FOC_TX_BUF_SIZE             64U
/*
 * 给定值上限。全部低于 safe.Dcur_MAX/Qcur_MAX 的故障阈值，命令本身不该把状态机
 * 顶进 OVERCURRENT。HT3510：额定 0.53 A / 0.11 N.m，堵转 0.80 A，最大 965 rpm。
 */
#define FOC_IQ_LIMIT_A              1.0F    /* 与 Velocity.OutMax 一致 */
#define FOC_SPEED_LIMIT_RAD_S       100.0F  /* 965 rpm = 101 rad/s，取整留余量 */
/* 开环 Uq 是调制指令，静止时电流 = Uq/(sqrt(3)*Rs)。6.0 -> 0.81 A，约等于堵转电流。
   方案阶段 4 的方向标定从 0.5 V 缓升到 2 V，本限幅只防误输入 */
#define FOC_UQ_OPEN_LIMIT           6.0F

typedef struct
{
    volatile float uq_open;   /* Velocity_OPEN_MODE：Uq 调制指令 */
    volatile float iq;        /* Current_SINGLE_MODE：Q 轴电流给定 [A] */
    volatile float speed;     /* VelCur_DOUBLE_MODE：机械角速度给定 [rad/s] */
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
/* 诊断轮询/初始化的阻塞 I2C 与 TIM2 的 DMA 读互斥，置位期间跳过角度刷新 */
static volatile uint8_t s_i2c_blocking;
static uint32_t s_diag_tick;

static uint8_t  s_tx_buf[FOC_TX_BUF_SIZE];
static uint8_t  s_rx_byte;
static uint16_t s_rx_len;

static float FocApp_Clamp(float value, float limit);
static void  FocApp_CheckEncoderFresh(void);
static void  FocApp_ApplyModeRequest(void);

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
    s_target.speed = 0.0F;
    s_target.pos = 0.0F;

    s_mode_request = Velocity_OPEN_MODE;
    s_mode_pending = 0U;
    s_low_div = 0U;
    s_stale_count = 0U;
    s_i2c_blocking = 0U;
    s_diag_tick = HAL_GetTick();
    s_rx_len = 0U;

    /* 上电先进开环、目标全零，方向标定完成前不允许直接进闭环 */
    Sguan.mode = Velocity_OPEN_MODE;

    /* 上位机调参通道：逐字节收，遇 '?' 交给库解析 */
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);

    /* 库在 while(1) 的 Sguan_Start_Tick() 里看到 0x01 才会跑初始化与标定 */
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
}

/*
 * 由 User_InitialInit() 调用，运行在 while(1) 上下文，允许阻塞。
 * 顺序不可调换：EN_GATE 拉高前 DRV8301 的 SPI 完全不响应，必须第一步。
 */
void FocApp_HwStart(void)
{
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

    /* 本函数里的 AS5600_Init() 是阻塞 I2C，重启时 TIM2 的 4 kHz DMA 读已在跑，
       两者不能在同一总线上重入，先把角度刷新挡住 */
    s_i2c_blocking = 1U;

    /*
     * 库的 Offset_CurrentRead() 是 24 次累加求平均，但没有先清零累加器。
     * 重启时不清会叠加上一轮结果，导致电流零偏整体偏移。
     */
    Sguan.current.Pos_offset0 = 0;
    Sguan.current.Pos_offset1 = 0;

    if (DRV8301_Init(&hspi2) != DRV8301_OK)
    {
        /* 配置未落地时器件可能停在默认 6PWM 模式，必须立刻硬关断 */
        DRV8301_Shutdown();
        s_signal[FOC_SIGNAL_SENSOR_ERROR] = 1U;
    }

    /* 挡住刷新之前可能已有一次 DMA 读在途（约 135 µs），等它收尾再动总线 */
    HAL_Delay(1U);
    if (AS5600_Init(&g_encoder, &hi2c1) != 0)
    {
        s_signal[FOC_SIGNAL_ENCODER_ERROR] = 1U;
    }
    s_i2c_blocking = 0U;

    /* 低频状态机挂在 TIM2 上，故障态也要靠它锁定，必须无条件启动 */
    (void)HAL_TIM_Base_Start_IT(&htim2);

    if (s_signal[FOC_SIGNAL_SENSOR_ERROR] != 0U)
    {
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

    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
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
    uint16_t status1 = 0U;
    uint16_t status2 = 0U;

    if ((uint32_t)(now - s_diag_tick) < FOC_DIAG_PERIOD_MS)
    {
        return;
    }
    s_diag_tick = now;

    /* 本板 nFAULT/nOCTW 未接 MCU，DRV8301 的故障只能靠 SPI 轮询发现 */
    if (DRV8301_ReadStatus(&status1, &status2) == DRV8301_OK)
    {
        if ((status1 & DRV8301_SR1_FAULT) != 0U)
        {
            s_signal[FOC_SIGNAL_SENSOR_ERROR] = 1U;
        }
    }

    /*
     * 磁体掉落后器件仍返回语法合法的角度，只读 RAW_ANGLE 发现不了。
     * 这里是阻塞 I2C，与 TIM2 发起的 DMA 读不能在同一总线上重入。
     */
    s_i2c_blocking = 1U;
    if (AS5600_CheckStatus(&g_encoder) == -1) /* -2 是总线忙，下个周期重试 */
    {
        s_signal[FOC_SIGNAL_ENCODER_ERROR] = 1U;
    }
    s_i2c_blocking = 0U;
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
     * 15 kHz ISR 内调用，必须非阻塞：只取 DMA 通道发布的缓存值。
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

void FocApp_Send(uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U) || (len > FOC_TX_BUF_SIZE))
    {
        return;
    }
    /* 上一帧未发完就丢帧：波形通道不值得阻塞主循环，会拖垮 10 Hz 诊断 */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }
    /* 库传进来的是全局 TXdata，DMA 期间主循环会继续改写它，必须先拷出来 */
    (void)memcpy(s_tx_buf, buf, len);
    (void)HAL_UART_Transmit_DMA(&huart1, s_tx_buf, len);
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
        Sguan.foc.Target_Speed = s_target.speed;
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
        s_target.speed = FocApp_Clamp(value, FOC_SPEED_LIMIT_RAD_S);
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
    /* UART 中断上下文只登记请求，真正切换放到 15 kHz 环，避免撕裂控制器状态 */
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
    s_target.speed = 0.0F;
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

    SguanFOC_High_Loop();
}

/* TIM2 更新中断，4 kHz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
    {
        return;
    }

    /*
     * 编码器刷新必须在中断里：上电标定阶段库会用 User_Delay(1000) 阻塞主循环，
     * 而 Offset_EncoderRead() 需要此时的新鲜角度。
     */
    if (s_i2c_blocking == 0U)
    {
        (void)AS5600_UpdateStart(&g_encoder);
    }

    s_low_div++;
    if (s_low_div >= FOC_LOW_LOOP_DIV)
    {
        s_low_div = 0U;
        FocApp_CheckEncoderFresh();
        SguanFOC_Low_Loop();
    }
}

/* AS5600 角度 DMA 读完成 / 出错。本板 I2C1 上只挂了 AS5600 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        AS5600_OnRxComplete(&g_encoder);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        AS5600_OnRxError(&g_encoder);
    }
}

/* 上位机调参帧：格式 AO=13.14? / BO=2? / CO=1? */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    if (s_rx_len < sizeof(Sguan_PrintfBuff))
    {
        Sguan_PrintfBuff[s_rx_len] = s_rx_byte;
        s_rx_len++;
    }
    else
    {
        s_rx_len = 0U; /* 帧超长，整帧丢弃重新同步 */
    }

    if (s_rx_byte == (uint8_t)'?')
    {
        /* 库在这里解析 Sguan_PrintfBuff 并清空缓冲 */
        SguanFOC_Printf_Loop(&s_rx_byte, 1U);
        s_rx_len = 0U;
    }

    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }
    /* 单次帧错误后若不重新挂接收，上位机调参通道会永久失效 */
    s_rx_len = 0U;
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1U);
}
