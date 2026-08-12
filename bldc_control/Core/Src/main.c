/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "foc_hal.h"
#if USE_SPEED_LOOP
#include "foc_outer_loop.h"
#endif
#include "log.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "drv8301.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  volatile uint8_t flag;
  volatile uint16_t count;
} g_Flag_t;

typedef struct
{
  volatile float I_U;
  volatile float I_V;
  volatile float I_W;
  volatile float V_U;
  volatile float V_V;
  volatile float V_W;
} Monitor_Data_t;

typedef enum
{
  BLDC_STATE_OFF = 0,
  BLDC_STATE_INIT,
  BLDC_STATE_ALIGN,
  BLDC_STATE_RUN,
  BLDC_STATE_FAULT
} BLDC_State_t;

typedef enum
{
  BLDC_FAULT_NONE = 0,
  BLDC_FAULT_STARTUP,
  BLDC_FAULT_DRV8301,
  BLDC_FAULT_SPI,
  BLDC_FAULT_ADC,
  BLDC_FAULT_SENSOR,
  BLDC_FAULT_PWM_SAMPLE,
  BLDC_FAULT_CURRENT_LIMIT,
  BLDC_FAULT_ALIGNMENT
} BLDC_FaultSource_t;

g_Flag_t g_led_status = {0};
g_Flag_t g_bldc_motor_status = {0};
Monitor_Data_t g_monitor_data;
uint16_t adc_Voltage_buf[ADC_CHANNEL_NUM];
LOWPASS_FILTER_T voltage_filter[ADC_CHANNEL_NUM];
LOWPASS_FILTER_T current_filter[ADC_CHANNEL_NUM];
float phase_voltage_calibration_gain[ADC_CHANNEL_NUM] = {1.0f, 1.0f, 1.0f};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 与文档设计值一致；提高前必须先在台架上确认电机允许的连续电流 */
#define FOC_INITIAL_IQ_REF               0.02f
#define FOC_IQ_REF_LIMIT                 0.30f
#define FOC_CURRENT_P                    0.30f
#define FOC_CURRENT_I                    0.00f
#define FOC_CURRENT_D                    0.00f
/* 电流环带宽由电压限幅约束即可，无需速率限幅：15kHz 下 200V/s 每拍仅允许输出
   变化 0.013V，远慢于 I=2000 的积分累积速度，会导致积分饱和后持续超调过流。 */
#define FOC_CURRENT_OUTPUT_RAMP          0.0f
#define FOC_CURRENT_VOLTAGE_LIMIT        4.0f
#define FOC_OUTER_LOOP_PERIOD_S          0.001f
#if USE_SPEED_LOOP
#define FOC_SPEED_P                      0.10f
#define FOC_SPEED_I                      0.00f
#define FOC_SPEED_D                      0.00f
#define FOC_SPEED_OUTPUT_RAMP            2.00f
#define FOC_SPEED_REFERENCE_LIMIT_RAD_S  10.0f
#endif
#if USE_POSITION_LOOP
#define FOC_POSITION_P                   2.00f
#define FOC_POSITION_I                   0.00f
#define FOC_POSITION_D                   0.00f
#define FOC_POSITION_OUTPUT_RAMP         20.0f
#define FOC_POSITION_SPEED_LIMIT_RAD_S   10.0f
/* 位置参考对称限幅：+-10 转，防止误设的大数值让位置环一次性输出满速指令 */
#define FOC_POSITION_REFERENCE_LIMIT_RAD 62.83f
#endif
#define FOC_ALIGNMENT_VOLTAGE            1.50f
#define FOC_ALIGNMENT_CURRENT_LIMIT_A    1.00f
#define FOC_ALIGNMENT_TIMEOUT_MS         3000U
#define FOC_ALIGNMENT_MOVEMENT_RAD       0.01f
/*
 * 电流反馈符号/通道映射的开环验证。台架专用，验证通过后必须置 0：它会在每次
 * 上电时拖动转子并阻塞约 1.2 s。测试电压按 R_phase 约 1.9 Ohm 取值，电流约
 * 0.26 A，远低于 FOC_ALIGNMENT_CURRENT_LIMIT_A。
 */
#ifndef FOC_SIGN_VERIFY_TEST
#define FOC_SIGN_VERIFY_TEST             1
#endif
#define FOC_SIGN_VERIFY_VOLTAGE          0.50f
#define FOC_SIGN_VERIFY_SETTLE_MS        200U
#define FOC_SIGN_VERIFY_SAMPLES          100U
#define MONITOR_SAMPLE_FREQ              1000U
#define CURRENT_MONITOR_DECIMATION       (PWM_FREQ / MONITOR_SAMPLE_FREQ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FOC_T foc;
#if USE_CURRENT_LOOP
PID_T pid_id;
PID_T pid_iq;
#endif
#if USE_SPEED_LOOP
PID_T pid_speed;
/* 进入 RUN 后必须由 BLDC_SetSpeedReference() 显式给定，上电不自转 */
volatile float speed_ref = 0.0f;
#endif
#if USE_POSITION_LOOP
PID_T pid_position;
volatile float position_ref = 0.0f;
#endif

volatile float iq_ref = 0.0f;
volatile BLDC_State_t bldc_state = BLDC_STATE_OFF;
volatile uint8_t bldc_fault_latched = 0U;
volatile BLDC_FaultSource_t bldc_fault_source = BLDC_FAULT_NONE;
volatile int32_t bldc_fault_detail = 0;
volatile uint8_t bldc_fault_report_pending = 0U;
/* 过流故障现场快照：ISR 内不可打印，由主循环上报时读取 */
volatile float bldc_fault_iu = 0.0f;
volatile float bldc_fault_iv = 0.0f;
volatile float bldc_fault_iw = 0.0f;
volatile float bldc_fault_id = 0.0f;
volatile float bldc_fault_iq = 0.0f;
volatile float bldc_fault_iq_ref = 0.0f;
volatile uint32_t bldc_fault_ccr1 = 0U;
volatile uint32_t bldc_fault_ccr2 = 0U;
volatile uint32_t bldc_fault_ccr3 = 0U;
/* CCR4 必须在 ISR 内取：BLDC_StopOutput() 会把它覆写成 PWM_ADC_TRIGGER_LATEST，
   上报时再读永远是那个安全值，没有任何诊断价值。 */
volatile uint32_t bldc_fault_ccr4 = 0U;
volatile float bldc_fault_raw_iu = 0.0f;
volatile float bldc_fault_raw_iv = 0.0f;
/* 区分"电角度错"与"环路振荡"必需的三个量：跳闸拍的电角度、上一拍施加的 dq
   电压（与快照的 CCR 一一对应）、观测速度。 */
volatile float bldc_fault_angle_el = 0.0f;
volatile float bldc_fault_ud = 0.0f;
volatile float bldc_fault_uq = 0.0f;
volatile float bldc_fault_omega = 0.0f;
volatile uint8_t bldc_fault_saturated = 0U;
volatile uint32_t adc_regular_callback_count = 0U;
volatile uint32_t adc_regular_callback_max_cycles = 0U;
volatile uint32_t adc_injected_callback_count = 0U;
volatile uint32_t adc_injected_callback_max_cycles = 0U;
/* 1 ms tick 丢拍与传感器读失败统计，调试变量，不参与控制也不打日志 */
volatile uint32_t outer_tick_overrun_count = 0U;
volatile uint32_t sensor_read_error_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void ADC_Filter_Init(float voltage_tf, float current_tf)
{
  int i;

  for (i = 0; i < ADC_CHANNEL_NUM; ++i)
  {
    LOWPASS_FILTER_Init(&voltage_filter[i], voltage_tf);
    LOWPASS_FILTER_Init(&current_filter[i], current_tf);
  }
}

static void Enable_DRV8301_Driver(uint8_t status)
{
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin,
                    status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t BLDC_IsDrv8301CommunicationError(DRV8301_Result_t result)
{
  return ((result == DRV8301_ERROR_SPI) ||
          (result == DRV8301_ERROR_FRAME) ||
          (result == DRV8301_ERROR_ADDRESS)) ? 1U : 0U;
}

static void BLDC_ForceHardwareOff(void)
{
  if ((htim1.Instance == TIM1) && (__HAL_RCC_TIM1_IS_CLK_ENABLED() != 0U))
  {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }

  if (__HAL_RCC_GPIOC_IS_CLK_ENABLED() != 0U)
  {
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
  }
}

static void BLDC_PerformanceCounterInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void BLDC_UpdateMaxCycles(volatile uint32_t *maximum, uint32_t start)
{
  uint32_t elapsed;

  elapsed = DWT->CYCCNT - start;
  if (elapsed > *maximum)
  {
    *maximum = elapsed;
  }
}

static void BLDC_StopOutput(BLDC_FaultSource_t source, int32_t detail)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  BLDC_ForceHardwareOff();
  iq_ref = 0.0f;
  bldc_state = BLDC_STATE_FAULT;
  if (bldc_fault_latched == 0U)
  {
    bldc_fault_source = source;
    bldc_fault_detail = detail;
    bldc_fault_latched = 1U;
    /* 日志一律推迟到 200 ms 低频任务：1 ms tick 和 ISR 内都不允许阻塞发串口 */
    bldc_fault_report_pending = 1U;
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_ADC_TRIGGER_LATEST);
  foc.current_sample_valid = 0U;
  foc.next_sample_valid = 0U;
  /* 清残留速度估计，避免下次使能时被当作真实运动 */
  FOC_ObserverReset(&foc, AS5600_GetOnceAngle(&G_SENSOR_A), HAL_GetTick());
#if USE_CURRENT_LOOP
  PID_Reset(&pid_id);
  PID_Reset(&pid_iq);
#endif
#if USE_SPEED_LOOP
  PID_Reset(&pid_speed);
#endif
#if USE_POSITION_LOOP
  PID_Reset(&pid_position);
#endif

  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void BLDC_TripFromIsr(BLDC_FaultSource_t source, int32_t detail)
{
  if (bldc_fault_latched != 0U)
  {
    return;
  }

  BLDC_StopOutput(source, detail);
}

static void BLDC_FailAndStop(BLDC_FaultSource_t source, int32_t detail,
                             const char *message)
{
  BLDC_StopOutput(source, detail);
  CAW_LOG_ERROR("%s", message);
  Error_Handler();
}

static int BLDC_StartSamplingTimer(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_ADC_TRIGGER_LATEST);
  htim1.Instance->EGR = TIM_EGR_UG;

  if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    return -1;
  }

  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  return 0;
}

static int BLDC_ArmPwmOutputs(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    return -1;
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    return -1;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    return -1;
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    return -1;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    return -1;
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    return -1;
  }

  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  return 0;
}


#if USE_CURRENT_LOOP
static int BLDC_StoreIqReference(float reference)
{
  uint32_t primask;
  int result = -1;

  if (reference != reference)
  {
    return -1;
  }

  if (reference > FOC_IQ_REF_LIMIT)
  {
    reference = FOC_IQ_REF_LIMIT;
  }
  else if (reference < -FOC_IQ_REF_LIMIT)
  {
    reference = -FOC_IQ_REF_LIMIT;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if ((bldc_state == BLDC_STATE_RUN) && (bldc_fault_latched == 0U))
  {
    iq_ref = reference;
    result = 0;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return result;
}
#endif

int BLDC_SetIqReference(float reference)
{
#if (!USE_CURRENT_LOOP || USE_SPEED_LOOP)
  (void)reference;
  return -1;
#else
  return BLDC_StoreIqReference(reference);
#endif
}

#if (USE_SPEED_LOOP && !USE_POSITION_LOOP)
int BLDC_SetSpeedReference(float reference)
{
  uint32_t primask;
  int result = -1;

  if (reference != reference)
  {
    return -1;
  }

  if (reference > FOC_SPEED_REFERENCE_LIMIT_RAD_S)
  {
    reference = FOC_SPEED_REFERENCE_LIMIT_RAD_S;
  }
  else if (reference < -FOC_SPEED_REFERENCE_LIMIT_RAD_S)
  {
    reference = -FOC_SPEED_REFERENCE_LIMIT_RAD_S;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if ((bldc_state == BLDC_STATE_RUN) && (bldc_fault_latched == 0U))
  {
    speed_ref = reference;
    result = 0;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return result;
}
#endif

#if USE_POSITION_LOOP
int BLDC_SetPositionReference(float reference)
{
  uint32_t primask;
  int result = -1;

  if (reference != reference)
  {
    return -1;
  }

  if (reference > FOC_POSITION_REFERENCE_LIMIT_RAD)
  {
    reference = FOC_POSITION_REFERENCE_LIMIT_RAD;
  }
  else if (reference < -FOC_POSITION_REFERENCE_LIMIT_RAD)
  {
    reference = -FOC_POSITION_REFERENCE_LIMIT_RAD;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if ((bldc_state == BLDC_STATE_RUN) && (bldc_fault_latched == 0U))
  {
    position_ref = reference;
    result = 0;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return result;
}
#endif

#if USE_SPEED_LOOP
/*
 * 外环，必须排在 1 ms tick 开头、AS5600_Update() 之前执行：软件 I2C 的阻塞
 * 时长和失败重试会污染外环执行时刻，PID_SetFixedDt(1 ms) 这个固定值才不成立。
 * 反馈取观测器快照，观测器在 15 kHz 预测，快照永远是新鲜的。
 * 方向约定（方案 23.5）：速度/位置反馈均需乘 dir。
 */
static void BLDC_UpdateOuterLoop(void)
{
  float next_iq_ref;
  float observed_position;
  float observed_velocity;
  float velocity_target;

  FOC_ObserverSnapshot(&foc, &observed_position, &observed_velocity);
  observed_position *= (float)foc.dir;
  observed_velocity *= (float)foc.dir;

#if USE_POSITION_LOOP
  velocity_target = Foc_PositionLoop(&pid_position, position_ref,
                                     observed_position);
  speed_ref = velocity_target;
#else
  velocity_target = speed_ref;
#endif

  next_iq_ref = Foc_VelocityLoop(&pid_speed, velocity_target,
                                 observed_velocity);
  if (BLDC_StoreIqReference(next_iq_ref) != 0)
  {
    PID_Reset(&pid_speed);
#if USE_POSITION_LOOP
    PID_Reset(&pid_position);
#endif
    return;
  }

  /*
   * 方案 23.4 第 4 条：电流环 dq 电压饱和时实际 iq 跟不上 iq_ref，而速度 PI
   * 看到的是"指令没饱和"，会继续积分。把饱和方向回传给速度 PI 的条件积分。
   */
  if (foc.voltage_saturated != 0U)
  {
    PID_ApplyExternalClip(&pid_speed, (float)foc.voltage_saturation_sign,
                          next_iq_ref);
  }
}
#endif

/*
 * 故障上报。只允许在 200 ms 低频任务里调用：115200 baud 下一条 128 字节日志
 * 需要 11.1 ms，放在 1 ms tick 或 ISR 里会直接打穿控制节拍。
 */
static void BLDC_ReportPendingFault(void)
{
  BLDC_FaultSource_t source;
  int32_t detail;
  float snap_iu;
  float snap_iv;
  float snap_iw;
  float snap_id;
  float snap_iq;
  float snap_iq_ref;
  uint32_t snap_ccr1;
  uint32_t snap_ccr2;
  uint32_t snap_ccr3;
  uint32_t snap_ccr4;
  float snap_raw_iu;
  float snap_raw_iv;
  float snap_angle_el;
  float snap_ud;
  float snap_uq;
  float snap_omega;
  uint8_t snap_saturated;

  if (bldc_fault_report_pending == 0U)
  {
    return;
  }

  __disable_irq();
  source = bldc_fault_source;
  detail = bldc_fault_detail;
  snap_iu = bldc_fault_iu;
  snap_iv = bldc_fault_iv;
  snap_iw = bldc_fault_iw;
  snap_id = bldc_fault_id;
  snap_iq = bldc_fault_iq;
  snap_iq_ref = bldc_fault_iq_ref;
  snap_ccr1 = bldc_fault_ccr1;
  snap_ccr2 = bldc_fault_ccr2;
  snap_ccr3 = bldc_fault_ccr3;
  snap_ccr4 = bldc_fault_ccr4;
  snap_raw_iu = bldc_fault_raw_iu;
  snap_raw_iv = bldc_fault_raw_iv;
  snap_angle_el = bldc_fault_angle_el;
  snap_ud = bldc_fault_ud;
  snap_uq = bldc_fault_uq;
  snap_omega = bldc_fault_omega;
  snap_saturated = bldc_fault_saturated;
  bldc_fault_report_pending = 0U;
  __enable_irq();

  CAW_LOG_ERROR("BLDC fault source=%u detail=%ld", (unsigned int)source,
                (long)detail);
  if (source == BLDC_FAULT_CURRENT_LIMIT)
  {
    CAW_LOG_ERROR("  iuvw=[%.3f %.3f %.3f] id=%.3f iq=%.3f iq_ref=%.3f",
                  snap_iu, snap_iv, snap_iw, snap_id, snap_iq, snap_iq_ref);
    /* raw 相对 offset 的偏离量，以及采样点与三相 CCR 的相对位置 */
    CAW_LOG_ERROR("  raw=[%.0f %.0f] off=[%.0f %.0f] ccr=[%lu %lu %lu] trig=%lu period=%lu",
                  snap_raw_iu, snap_raw_iv,
                  g_current_offset.offset_u, g_current_offset.offset_v,
                  (unsigned long)snap_ccr1, (unsigned long)snap_ccr2,
                  (unsigned long)snap_ccr3,
                  (unsigned long)snap_ccr4,
                  (unsigned long)PWM_PERIOD);
    /* ud/uq 顶到 FOC_CURRENT_VOLTAGE_LIMIT 而 iq_ref 很小 = 电流环在发散；
       omega 接近 FOC_OBSERVER_OMEGA_LIMIT = 观测器跑飞，电角度不可信 */
    CAW_LOG_ERROR("  th_el=%.3f ud=%.3f uq=%.3f sat=%u omega=%.2f",
                  snap_angle_el, snap_ud, snap_uq,
                  (unsigned int)snap_saturated, snap_omega);
  }
}

#if FOC_SIGN_VERIFY_TEST
/*
 * 电流反馈符号/通道映射的开环验证。必须在 BLDC_STATE_ALIGN 下运行：该状态的
 * 注入 ISR 只做 Clarke/Park 与限流判定，不跑电流环，不会覆写这里下发的占空比。
 *
 * 判据一（通道与符号）：FOC_SetTorque(uq, theta) 施加的 alpha-beta 电压矢量指向
 * theta + pi/2，转子静止、阻性主导时电流矢量必须与之同向。
 *   d_ang ~ 0          -> 通道映射与符号都正确
 *   d_ang ~ +-180      -> 两路符号都反了
 *   d_ang ~ +-60/+-120 -> 通道接错，或只有一路符号反
 * 判据二（电角度零位）：转子自由时会被拖到与电压矢量对齐，此时 id ~ |i| 且
 * iq ~ 0 才说明 zero_electric_angle 正确；四个点上 iq 有一致的残差即为零位偏差。
 */
static void BLDC_RunSignVerifyTest(float test_voltage)
{
  static const float test_angles[4] =
  {
    0.0f, 1.570796327f, 3.141592654f, 4.712388980f
  };
  const float rad_to_deg = 57.295779513f;
  uint32_t i;

  CAW_LOG_DEBUG("Sign verify: uq=%.2fV x4 points, pass = d_ang~0 and iq~0",
                test_voltage);

  for (i = 0U; i < 4U; ++i)
  {
    float sum_iu = 0.0f;
    float sum_iv = 0.0f;
    float sum_iw = 0.0f;
    float sum_ialpha = 0.0f;
    float sum_ibeta = 0.0f;
    float sum_id = 0.0f;
    float sum_iq = 0.0f;
    float delta_deg;
    float expect_deg;
    float magnitude;
    float measure_deg;
    float scale;
    uint32_t n;
    uint32_t primask;

    FOC_SetTorque(&foc, test_voltage, test_angles[i]);
    HAL_Delay(FOC_SIGN_VERIFY_SETTLE_MS);

    for (n = 0U; n < FOC_SIGN_VERIFY_SAMPLES; ++n)
    {
      /* 与 RUN 态同节拍刷新传感器，否则 cached_angle_el 冻结、id/iq 无意义 */
      (void)FOC_SensorUpdate(&foc);

      primask = __get_PRIMASK();
      __disable_irq();
      sum_iu += foc.i_a;
      sum_iv += foc.i_b;
      sum_iw += foc.i_c;
      sum_ialpha += foc.i_alpha;
      sum_ibeta += foc.i_beta;
      sum_id += foc.i_d;
      sum_iq += foc.i_q;
      if (primask == 0U)
      {
        __enable_irq();
      }
      HAL_Delay(1U);
    }

    if (bldc_fault_latched != 0U)
    {
      CAW_LOG_ERROR("Sign verify aborted at point %lu", (unsigned long)i);
      return;
    }

    scale = 1.0f / (float)FOC_SIGN_VERIFY_SAMPLES;
    sum_iu *= scale;
    sum_iv *= scale;
    sum_iw *= scale;
    sum_ialpha *= scale;
    sum_ibeta *= scale;
    sum_id *= scale;
    sum_iq *= scale;

    magnitude = sqrtf(sum_ialpha * sum_ialpha + sum_ibeta * sum_ibeta);
    measure_deg = atan2f(sum_ibeta, sum_ialpha) * rad_to_deg;
    expect_deg = (test_angles[i] + 1.570796327f) * rad_to_deg;
    delta_deg = measure_deg - expect_deg;
    while (delta_deg > 180.0f)
    {
      delta_deg -= 360.0f;
    }
    while (delta_deg <= -180.0f)
    {
      delta_deg += 360.0f;
    }

    CAW_LOG_ERROR("[%lu] th=%.0f iuvw=[%.3f %.3f %.3f] |i|=%.3f",
                  (unsigned long)i, test_angles[i] * rad_to_deg,
                  sum_iu, sum_iv, sum_iw, magnitude);
    CAW_LOG_ERROR("    ang=%.1f exp=%.1f d_ang=%+.1f id=%.3f iq=%.3f",
                  measure_deg, expect_deg, delta_deg, sum_id, sum_iq);
  }

  FOC_SetTorque(&foc, 0.0f, test_angles[0]);
  /* 测试全程在拖动转子，重锚定观测器，避免带着 omega_hat 残值进入 RUN */
  if (AS5600_Update(&G_SENSOR_A) == 0)
  {
    FOC_ObserverReset(&foc, AS5600_GetOnceAngle(&G_SENSOR_A), HAL_GetTick());
  }
}
#endif

void BLDC_Stop(void)
{
  uint32_t primask;

  if (bldc_fault_latched != 0U)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  BLDC_ForceHardwareOff();
  iq_ref = 0.0f;
  bldc_state = BLDC_STATE_OFF;
  FOC_ObserverReset(&foc, AS5600_GetOnceAngle(&G_SENSOR_A), HAL_GetTick());
#if USE_CURRENT_LOOP
  PID_Reset(&pid_id);
  PID_Reset(&pid_iq);
#endif
#if USE_SPEED_LOOP
  PID_Reset(&pid_speed);
#endif
#if USE_POSITION_LOOP
  PID_Reset(&pid_position);
#endif
  if (primask == 0U)
  {
    __enable_irq();
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  DRV8301_Result_t drv_result;
  FOC_Result_t foc_result;
  uint32_t align_start_tick;
  uint32_t last_outer_tick;
#if USE_POSITION_LOOP
  float position_ref_init;
#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  BLDC_ForceHardwareOff();
  bldc_state = BLDC_STATE_OFF;
  if (CAW_LOG_Init(&huart1, LEVEL_DEBUG, true) != 0)
  {
    Error_Handler();
  }

  BLDC_PerformanceCounterInit();
  ADC_Filter_Init(0.01f, 0.005f);
  bldc_state = BLDC_STATE_INIT;

  foc_result = FOC_Closeloop_Init(&foc, &htim1, PWM_PERIOD, 24.0f, 1, 11);
  if (foc_result != FOC_RESULT_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_STARTUP, (int32_t)foc_result,
                     "FOC init failed");
  }
  FOC_SetVoltageLimit(&foc, FOC_CURRENT_VOLTAGE_LIMIT);
  foc_result = FOC_HAL_InitA(&foc);
  if (foc_result != FOC_RESULT_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_SENSOR, (int32_t)foc_result,
                     "AS5600 init failed");
  }

#if USE_CURRENT_LOOP
  PID_Init(&pid_id, FOC_CURRENT_P, FOC_CURRENT_I, FOC_CURRENT_D,
           FOC_CURRENT_OUTPUT_RAMP, FOC_CURRENT_VOLTAGE_LIMIT);
  PID_Init(&pid_iq, FOC_CURRENT_P, FOC_CURRENT_I, FOC_CURRENT_D,
           FOC_CURRENT_OUTPUT_RAMP, FOC_CURRENT_VOLTAGE_LIMIT);
  PID_SetFixedDt(&pid_id, 1.0f / (float)PWM_FREQ);
  PID_SetFixedDt(&pid_iq, 1.0f / (float)PWM_FREQ);
  /* 电压限幅统一由 FOC 的矢量圆限幅执行：先分轴方形限幅会扭曲 d/q 比例，
     圆限幅已无法恢复。PID 内部只保留积分限幅。 */
  PID_SetExternalOutputLimit(&pid_id, 1U);
  PID_SetExternalOutputLimit(&pid_iq, 1U);
#endif
#if USE_SPEED_LOOP
  PID_Init(&pid_speed, FOC_SPEED_P, FOC_SPEED_I, FOC_SPEED_D,
           FOC_SPEED_OUTPUT_RAMP, FOC_IQ_REF_LIMIT);
  PID_SetFixedDt(&pid_speed, FOC_OUTER_LOOP_PERIOD_S);
#endif
#if USE_POSITION_LOOP
  PID_Init(&pid_position, FOC_POSITION_P, FOC_POSITION_I, FOC_POSITION_D,
           FOC_POSITION_OUTPUT_RAMP, FOC_POSITION_SPEED_LIMIT_RAD_S);
  PID_SetFixedDt(&pid_position, FOC_OUTER_LOOP_PERIOD_S);
#endif

  if (BLDC_StartSamplingTimer() != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_STARTUP, 0, "TIM1 CH4 OC start failed");
  }
  if (BLDC_ArmPwmOutputs() != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_STARTUP, 0, "TIM1 PWM arm failed");
  }

  Enable_DRV8301_Driver(1U);
  drv_result = DRV8301_Init(&hspi2);
  if (drv_result != DRV8301_OK)
  {
    BLDC_FaultSource_t source;

    source = (BLDC_IsDrv8301CommunicationError(drv_result) != 0U)
                 ? BLDC_FAULT_SPI
                 : BLDC_FAULT_DRV8301;
    BLDC_FailAndStop(source, (int32_t)drv_result, "DRV8301 init failed");
  }

  foc_result = FOC_Current_Offset_Calibration(&hadc1, 200U);
  if (foc_result != FOC_RESULT_OK)
  {
    CAW_LOG_ERROR("Current cal failed: result=%d span=[%.1f %.1f]",
                  (int)foc_result,
                  g_current_offset.max_span_u,
                  g_current_offset.max_span_v);
    BLDC_FailAndStop(BLDC_FAULT_ADC, (int32_t)foc_result,
                     "Current offset calibration failed");
  }

  FOC_SetTorque(&foc, 0.0f, 0.0f);
  if (FOC_IsNextCurrentSampleValid(&foc) == 0U)
  {
    BLDC_FailAndStop(BLDC_FAULT_PWM_SAMPLE, 0,
                     "Initial PWM current sample window invalid");
  }

  // if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Voltage_buf,
  //                       ADC_CHANNEL_NUM) != HAL_OK)
  // {
  //   BLDC_FailAndStop(BLDC_FAULT_ADC, (int32_t)hadc1.ErrorCode,
  //                    "ADC regular DMA start failed");
  // }
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_ADC, (int32_t)hadc1.ErrorCode,
                     "ADC injected start failed");
  }

  foc.alignment_current_limit = FOC_ALIGNMENT_CURRENT_LIMIT_A;
  bldc_state = BLDC_STATE_ALIGN;
  __HAL_TIM_MOE_ENABLE(&htim1);
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_STARTUP, 0, "TIM2 base start failed");
  }

  align_start_tick = HAL_GetTick();
  foc_result = FOC_AlignmentSensor(&foc, FOC_ALIGNMENT_VOLTAGE,
                                    FOC_ALIGNMENT_CURRENT_LIMIT_A,
                                    FOC_ALIGNMENT_TIMEOUT_MS,
                                    FOC_ALIGNMENT_MOVEMENT_RAD);
  if (foc_result != FOC_RESULT_OK)
  {
    CAW_LOG_ERROR("FOC alignment failed: foc_result=%d, elapsed=%lums, fault=%d detail=%ld",
                (int)foc_result,
                (unsigned long)(HAL_GetTick() - align_start_tick),
                (int)bldc_fault_source, (long)bldc_fault_detail);
    BLDC_FailAndStop(BLDC_FAULT_ALIGNMENT, (int32_t)foc_result,
                     "FOC sensor alignment failed");
  }

#if FOC_SIGN_VERIFY_TEST
  /* 仍处于 BLDC_STATE_ALIGN：电流环未接管，这里下发的开环占空比不会被覆写 */
  BLDC_RunSignVerifyTest(FOC_SIGN_VERIFY_VOLTAGE);
  if (bldc_fault_latched != 0U)
  {
    /* 此时还没进主循环，200 ms 上报任务不会执行，必须就地把现场打出来 */
    BLDC_ReportPendingFault();
    BLDC_FailAndStop(bldc_fault_source, bldc_fault_detail,
                     "Sign verify tripped");
  }
#endif

#if USE_CURRENT_LOOP
  PID_Reset(&pid_id);
  PID_Reset(&pid_iq);
#endif
#if USE_SPEED_LOOP
  PID_Reset(&pid_speed);
  speed_ref = 0.0f;
#endif
#if USE_POSITION_LOOP
  PID_Reset(&pid_position);
#endif

  /*
   * 就绪日志必须打在进入 RUN 之前：115200 baud 下一条日志阻塞约 11 ms，而
   * sensor_max_age_ms 只有 5 ms，先置 RUN 会让第一拍电流环立刻判超龄关断。
   */
#if USE_POSITION_LOOP
  CAW_LOG_DEBUG("BLDC position loop ready");
#elif USE_SPEED_LOOP
  CAW_LOG_DEBUG("BLDC speed loop ready");
#elif USE_CURRENT_LOOP
  CAW_LOG_DEBUG("BLDC current loop ready");
#else
  CAW_LOG_DEBUG("BLDC control loops disabled");
#endif

  /* 使能前刷新一次传感器，保证进入 RUN 时角度缓存是新鲜的 */
  foc_result = FOC_HAL_UpdateSensor(&foc);
  if (foc_result != FOC_RESULT_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_SENSOR, (int32_t)foc_result,
                     "AS5600 update failed before RUN");
  }
#if USE_POSITION_LOOP
  /* 闭环瞬间误差必须为 0：参考初始化为当前位置反馈，坐标系含 dir 修正 */
  FOC_ObserverSnapshot(&foc, &position_ref_init, NULL);
  position_ref = (float)foc.dir * position_ref_init;
#endif
  iq_ref = 0.0f;
  last_outer_tick = HAL_GetTick();
  bldc_state = BLDC_STATE_RUN;
#if (USE_CURRENT_LOOP && !USE_SPEED_LOOP)
  (void)BLDC_SetIqReference(FOC_INITIAL_IQ_REF);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 1 ms 前台任务，顺序固定（方案 22.7）：
     *   1. 外环（内部取观测器快照）  2. AS5600_Update()
     *   3. 成功 -> 观测器修正；失败 -> 计数，不修正、不改 last_success_tick
     *   4. 传感器超龄 -> 关断        5. 丢拍检测
     * 本任务内禁止输出日志：115200 baud 下一条日志 11 ms，会打穿 11 个 tick。
     */
    if (g_bldc_motor_status.flag != 0U)
    {
      uint32_t tick_now = HAL_GetTick();

      g_bldc_motor_status.flag = 0U;
      /*
       * 丢拍统计：flag 是置位型，主循环超过 1 ms 时中间的 tick 被静默丢弃，
       * 外环 dt 随之失真。这里按真实时基算差，覆盖所有丢拍来源。
       * 只做统计不打日志——在此处打日志会丢更多拍，形成自激。
       */
      if ((uint32_t)(tick_now - last_outer_tick) > 1U)
      {
        outer_tick_overrun_count += (uint32_t)(tick_now - last_outer_tick) - 1U;
      }
      last_outer_tick = tick_now;

      if ((bldc_state == BLDC_STATE_RUN) && (bldc_fault_latched == 0U))
      {
#if USE_SPEED_LOOP
        BLDC_UpdateOuterLoop();
#endif
        foc_result = FOC_HAL_UpdateSensor(&foc);
        if (foc_result != FOC_RESULT_OK)
        {
          ++sensor_read_error_count;
        }

        /* 丢拍不放宽超龄判据：观测器能外推不等于反馈还有效 */
        if (FOC_IsSensorFresh(&foc, HAL_GetTick()) == 0U)
        {
          BLDC_StopOutput(BLDC_FAULT_SENSOR, (int32_t)foc_result);
        }
      }
    }

    if (g_led_status.flag != 0U)
    {
      uint16_t status;
      DRV8301_Result_t result;

      g_led_status.flag = 0U;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      CAW_LOG_INFO("adc_injected_callback_max_cycles=%lu adc_regular_callback_count=%lu",
                   (unsigned long)adc_injected_callback_max_cycles,
                   (unsigned long)adc_regular_callback_count);

      if (((bldc_state == BLDC_STATE_ALIGN) ||
           (bldc_state == BLDC_STATE_RUN)) &&
          (bldc_fault_latched == 0U))
      {
        result = DRV8301_ReadStatus1(&status);
        if (result != DRV8301_OK)
        {
          BLDC_StopOutput(BLDC_FAULT_SPI, (int32_t)result);
        }
        else if ((status & DRV8301_SR1_FAULT) != 0U)
        {
          BLDC_StopOutput(BLDC_FAULT_DRV8301, (int32_t)status);
        }
        /* AS5600_Update() 只读角度寄存器，磁体掉落必须靠复检 STATUS 发现。
           只有磁体状态异常才关断；单次读失败交给 1 ms 通道的超龄判据兜底 */
        else if (AS5600_CheckStatus(&G_SENSOR_A) == -1)
        {
          BLDC_StopOutput(BLDC_FAULT_SENSOR,
                          (int32_t)AS5600_GetErrorCount(&G_SENSOR_A));
        }
      }

      BLDC_ReportPendingFault();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    g_bldc_motor_status.flag = 1U;

    if (++g_led_status.count >= 200U)
    {
      g_led_status.count = 0U;
      g_led_status.flag = 1U;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    float phase_voltage[ADC_CHANNEL_NUM];
    float raw_voltage;
    uint32_t start;
    int i;

    start = DWT->CYCCNT;
    ++adc_regular_callback_count;
    for (i = 0; i < ADC_CHANNEL_NUM; ++i)
    {
      raw_voltage = (float)adc_Voltage_buf[i] * ADC_VOLTAGE_FACTOR *
                    phase_voltage_calibration_gain[i];
      phase_voltage[i] = LOWPASS_FILTER_CalcWithDt(
        &voltage_filter[i], raw_voltage, 1.0f / MONITOR_SAMPLE_FREQ);
    }

    g_monitor_data.V_U = phase_voltage[0];
    g_monitor_data.V_V = phase_voltage[1];
    g_monitor_data.V_W = phase_voltage[2];
    BLDC_UpdateMaxCycles(&adc_regular_callback_max_cycles, start);
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    static uint16_t monitor_divider = 0U;
    BLDC_State_t state;
    float current_limit;
    float iu;
    float iv;
    float iw;
    float raw_iu;
    float raw_iv;
    FOC_Result_t current_result;
#if USE_CURRENT_LOOP
    FOC_Result_t control_result;
    uint32_t now;
#endif
    uint32_t start;

    start = DWT->CYCCNT;
    ++adc_injected_callback_count;
    state = bldc_state;
    if ((bldc_fault_latched != 0U) ||
        ((state != BLDC_STATE_ALIGN) && (state != BLDC_STATE_RUN)))
    {
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }

    FOC_CommitPwmUpdate(&foc);
    raw_iu = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    raw_iv = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

    /* 换算成电流之前先看原始码值：顶到轨的死通道折算后可能落在电流限值内 */
    current_result = FOC_ValidateRawCurrentSamples(raw_iu, raw_iv);
    if (current_result != FOC_RESULT_OK)
    {
      bldc_fault_raw_iu = raw_iu;
      bldc_fault_raw_iv = raw_iv;
      BLDC_TripFromIsr(BLDC_FAULT_ADC, (int32_t)current_result);
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }

    FOC_Get_Calibrated_Current(raw_iu, raw_iv, &iu, &iv, &iw);

    /* 观测器 15 kHz 预测：把电角度从 1 ms 一级的阶梯推进到本拍，消除 d 轴
       交叉耦合。丢拍只跳过修正，不跳过预测。 */
    FOC_ObserverPredict(&foc);

    current_limit = (state == BLDC_STATE_ALIGN)
                      ? foc.alignment_current_limit
                      : FOC_CURRENT_ABS_LIMIT_A;
    /* 本拍唯一一次 Clarke/Park，结果同时供过流快照与电流环使用 */
    FOC_Clarke(&foc, iu, iv, iw);
    FOC_Park(&foc, foc.cached_angle_el);

    current_result = FOC_ValidatePhaseCurrents(iu, iv, iw, current_limit);
    if (current_result != FOC_RESULT_OK)
    {
      /*
       * 保存现场。判读依据是"电流矢量角 vs 电压矢量角"，不是 i_d/i_q 的大小：
       * 静止转子阻性主导时电流方向恒跟随电压方向，零位偏差同时旋转测量坐标系
       * 和输出坐标系，在 i_d 里出不来东西。两者夹角远大于一拍的角度推进量
       * (pp*omega_limit/PWM_FREQ) 时，说明占空比在拍间大幅摆动，即环路在振荡。
       */
      bldc_fault_iu = iu;
      bldc_fault_iv = iv;
      bldc_fault_iw = iw;
      bldc_fault_id = foc.i_d;
      bldc_fault_iq = foc.i_q;
      bldc_fault_iq_ref = iq_ref;
      bldc_fault_ccr1 = htim1.Instance->CCR1;   /* 采样时刻的三相占空比 */
      bldc_fault_ccr2 = htim1.Instance->CCR2;
      bldc_fault_ccr3 = htim1.Instance->CCR3;
      bldc_fault_ccr4 = htim1.Instance->CCR4;   /* 本拍真实的 ADC 触发点 */
      bldc_fault_raw_iu = raw_iu;   /* 原始码值：区分真实电流与开关噪声 */
      bldc_fault_raw_iv = raw_iv;
      bldc_fault_angle_el = foc.cached_angle_el;
      bldc_fault_ud = foc.u_d;      /* 上一拍施加的 dq 电压，与上面的 CCR 同源 */
      bldc_fault_uq = foc.u_q;
      bldc_fault_saturated = foc.voltage_saturated;
      bldc_fault_omega = foc.obs_omega_hat;
      BLDC_TripFromIsr(BLDC_FAULT_CURRENT_LIMIT, current_result);
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }

    ++monitor_divider;
    if (monitor_divider >= CURRENT_MONITOR_DECIMATION)
    {
      monitor_divider = 0U;
      g_monitor_data.I_U = LOWPASS_FILTER_CalcWithDt(
        &current_filter[0], iu, 1.0f / MONITOR_SAMPLE_FREQ);
      g_monitor_data.I_V = LOWPASS_FILTER_CalcWithDt(
        &current_filter[1], iv, 1.0f / MONITOR_SAMPLE_FREQ);
      g_monitor_data.I_W = LOWPASS_FILTER_CalcWithDt(
        &current_filter[2], iw, 1.0f / MONITOR_SAMPLE_FREQ);
    }

    if (FOC_IsCurrentSampleValid(&foc) == 0U)
    {
      BLDC_TripFromIsr(BLDC_FAULT_PWM_SAMPLE, 0);
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }

    if (state == BLDC_STATE_ALIGN)
    {
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }

#if USE_CURRENT_LOOP
    now = HAL_GetTick();
    control_result = FOC_CurrentLoopUpdate(&foc, 0.0f, iq_ref, &pid_id,
                                           &pid_iq, now);
    if (control_result != FOC_RESULT_OK)
    {
      BLDC_TripFromIsr(BLDC_FAULT_SENSOR, (int32_t)control_result);
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }
    if (FOC_IsNextCurrentSampleValid(&foc) == 0U)
    {
      BLDC_TripFromIsr(BLDC_FAULT_PWM_SAMPLE, 0);
    }
#endif
    BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  BLDC_ForceHardwareOff();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

