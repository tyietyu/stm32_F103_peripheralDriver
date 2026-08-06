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
#include "foc_test.h"
#endif
#include "log.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "drv8301.h"
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
  BLDC_FAULT_CURRENT_SUM,
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
#define FOC_INITIAL_IQ_REF               0.10f
#define FOC_IQ_REF_LIMIT                 0.30f
#define FOC_CURRENT_P                    0.50f
#define FOC_CURRENT_I                    1.00f
#define FOC_CURRENT_D                    0.00f
#define FOC_CURRENT_OUTPUT_RAMP          1000.0f
#define FOC_CURRENT_VOLTAGE_LIMIT        4.0f
#define FOC_OUTER_LOOP_PERIOD_S          0.001f
#if USE_SPEED_LOOP
#define FOC_SPEED_P                      0.10f
#define FOC_SPEED_I                      0.00f
#define FOC_SPEED_D                      0.00f
#define FOC_SPEED_OUTPUT_RAMP            2.00f
#define FOC_SPEED_REFERENCE_LIMIT_RAD_S  10.0f
#define FOC_VELOCITY_FILTER_TIME_S       0.01f
#endif
#if USE_POSITION_LOOP
#define FOC_POSITION_P                   2.00f
#define FOC_POSITION_I                   0.00f
#define FOC_POSITION_D                   0.00f
#define FOC_POSITION_OUTPUT_RAMP         20.0f
#define FOC_POSITION_SPEED_LIMIT_RAD_S   10.0f
#endif
#define FOC_SENSOR_MAX_AGE_MS            5U
#define FOC_ALIGNMENT_VOLTAGE            1.50f
#define FOC_ALIGNMENT_CURRENT_LIMIT_A    1.00f
#define FOC_ALIGNMENT_TIMEOUT_MS         1500U
#define FOC_ALIGNMENT_MOVEMENT_RAD       0.01f
#define ADC_RAW_VALID_MIN                16.0f
#define ADC_RAW_VALID_MAX                4079.0f
#define MONITOR_SAMPLE_FREQ              1000U
#define CURRENT_MONITOR_DECIMATION       (PWM_FREQ / MONITOR_SAMPLE_FREQ)
#define ADC_INJECTED_MIN_CALLBACKS_MS    (CURRENT_MONITOR_DECIMATION / 2U)
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
LOWPASS_FILTER_T velocity_filter;
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
volatile uint32_t adc_regular_callback_count = 0U;
volatile uint32_t adc_regular_callback_max_cycles = 0U;
volatile uint32_t adc_injected_callback_count = 0U;
volatile uint32_t adc_injected_callback_max_cycles = 0U;
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
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_ADC_TRIGGER_LATEST);
  foc.current_sample_valid = 0U;
  foc.next_sample_valid = 0U;
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
  bldc_fault_report_pending = 1U;
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

static int BLDC_EnablePowerOutput(void)
{
  uint32_t primask;

  if (bldc_fault_latched != 0U)
  {
    return -1;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  htim1.Instance->CR1 &= ~TIM_CR1_CEN;
  htim1.Instance->CNT = 0U;
  htim1.Instance->EGR = TIM_EGR_UG;
  FOC_CommitPwmUpdate(&foc);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_CC4);
  __HAL_TIM_MOE_ENABLE(&htim1);
  htim1.Instance->CR1 |= TIM_CR1_CEN;
  if (primask == 0U)
  {
    __enable_irq();
  }

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
static void BLDC_UpdateOuterLoop(void)
{
  float next_iq_ref;

#if USE_POSITION_LOOP
  next_iq_ref = Foc_PositionLoop(&foc, &pid_position, &velocity_filter,
                                 &pid_speed, position_ref);
#else
  next_iq_ref = Foc_VelocityLoop(&foc, &velocity_filter, &pid_speed,
                                 speed_ref);
#endif
  if (BLDC_StoreIqReference(next_iq_ref) != 0)
  {
    PID_Reset(&pid_speed);
#if USE_POSITION_LOOP
    PID_Reset(&pid_position);
#endif
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

  FOC_Closeloop_Init(&foc, &htim1, PWM_PERIOD, 24.0f, 1, 11);
  FOC_SetVoltageLimit(&foc, FOC_CURRENT_VOLTAGE_LIMIT);
  foc.sensor_max_age_ms = FOC_SENSOR_MAX_AGE_MS;
  if (FOC_HAL_InitA(&foc) != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_SENSOR, 0, "AS5600 init failed");
  }

#if USE_CURRENT_LOOP
  PID_Init(&pid_id, FOC_CURRENT_P, FOC_CURRENT_I, FOC_CURRENT_D,
           FOC_CURRENT_OUTPUT_RAMP, FOC_CURRENT_VOLTAGE_LIMIT);
  PID_Init(&pid_iq, FOC_CURRENT_P, FOC_CURRENT_I, FOC_CURRENT_D,
           FOC_CURRENT_OUTPUT_RAMP, FOC_CURRENT_VOLTAGE_LIMIT);
  PID_SetFixedDt(&pid_id, 1.0f / (float)PWM_FREQ);
  PID_SetFixedDt(&pid_iq, 1.0f / (float)PWM_FREQ);
#endif
#if USE_SPEED_LOOP
  PID_Init(&pid_speed, FOC_SPEED_P, FOC_SPEED_I, FOC_SPEED_D,
           FOC_SPEED_OUTPUT_RAMP, FOC_IQ_REF_LIMIT);
  PID_SetFixedDt(&pid_speed, FOC_OUTER_LOOP_PERIOD_S);
  LOWPASS_FILTER_Init(&velocity_filter, FOC_VELOCITY_FILTER_TIME_S);
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

  if (FOC_Current_Offset_Calibration(&hadc1, 200U) != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_ADC, (int32_t)hadc1.ErrorCode,
                     "Current offset calibration failed");
  }

  FOC_SetTorque(&foc, 0.0f, 0.0f);
  if (FOC_IsNextCurrentSampleValid(&foc) == 0U)
  {
    BLDC_FailAndStop(BLDC_FAULT_PWM_SAMPLE, 0,
                     "Initial PWM current sample window invalid");
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Voltage_buf,
                        ADC_CHANNEL_NUM) != HAL_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_ADC, (int32_t)hadc1.ErrorCode,
                     "ADC regular DMA start failed");
  }
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_ADC, (int32_t)hadc1.ErrorCode,
                     "ADC injected start failed");
  }
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    BLDC_FailAndStop(BLDC_FAULT_STARTUP, 0, "TIM2 base start failed");
  }

  foc.alignment_current_limit = FOC_ALIGNMENT_CURRENT_LIMIT_A;
  bldc_state = BLDC_STATE_ALIGN;
  if (BLDC_EnablePowerOutput() != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_STARTUP, 0,
                     "TIM1 power output enable failed");
  }
  if (FOC_AlignmentSensor(&foc, FOC_ALIGNMENT_VOLTAGE,
                          FOC_ALIGNMENT_CURRENT_LIMIT_A,
                          FOC_ALIGNMENT_TIMEOUT_MS,
                          FOC_ALIGNMENT_MOVEMENT_RAD) != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_ALIGNMENT, 0,
                     "FOC sensor alignment failed");
  }
  if (bldc_fault_latched != 0U)
  {
    BLDC_FailAndStop(BLDC_FAULT_ALIGNMENT, 0,
                     "FOC alignment protection tripped");
  }
  if (FOC_HAL_UpdateSensor(&foc, FOC_SENSOR_MAX_AGE_MS) != 0)
  {
    BLDC_FailAndStop(BLDC_FAULT_SENSOR,
                     (int32_t)FOC_HAL_GetSensorErrorCount(),
                     "AS5600 update failed after alignment");
  }
  if (FOC_IsNextCurrentSampleValid(&foc) == 0U)
  {
    BLDC_FailAndStop(BLDC_FAULT_PWM_SAMPLE, 0,
                     "PWM current sample window invalid after alignment");
  }

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
  position_ref = (float)foc.dir * foc.Sensor_GetAngle();
#endif
  iq_ref = 0.0f;
  bldc_state = BLDC_STATE_RUN;
#if USE_POSITION_LOOP
  CAW_LOG_DEBUG("BLDC position loop ready");
#elif USE_SPEED_LOOP
  CAW_LOG_DEBUG("BLDC speed loop ready");
#elif USE_CURRENT_LOOP
  (void)BLDC_SetIqReference(FOC_INITIAL_IQ_REF);
  CAW_LOG_DEBUG("BLDC current loop ready");
#else
  CAW_LOG_DEBUG("BLDC control loops disabled");
#endif
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_bldc_motor_status.flag != 0U)
    {
      g_bldc_motor_status.flag = 0U;
      if ((bldc_state == BLDC_STATE_RUN) && (bldc_fault_latched == 0U))
      {
        if (FOC_HAL_UpdateSensor(&foc, FOC_SENSOR_MAX_AGE_MS) != 0)
        {
          BLDC_StopOutput(BLDC_FAULT_SENSOR,
                          (int32_t)FOC_HAL_GetSensorErrorCount());
          CAW_LOG_ERROR("AS5600 update failed, driver disabled");
        }
#if USE_SPEED_LOOP
        else
        {
          BLDC_UpdateOuterLoop();
        }
#endif
      }
    }

    if (bldc_fault_report_pending != 0U)
    {
      BLDC_FaultSource_t source;
      int32_t detail;

      __disable_irq();
      source = bldc_fault_source;
      detail = bldc_fault_detail;
      bldc_fault_report_pending = 0U;
      __enable_irq();
      CAW_LOG_ERROR("BLDC fault source=%u detail=%ld", (unsigned int)source,
                    (long)detail);
    }

    if (g_led_status.flag != 0U)
    {
      uint16_t status;
      DRV8301_Result_t result;

      g_led_status.flag = 0U;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      if (((bldc_state == BLDC_STATE_ALIGN) ||
           (bldc_state == BLDC_STATE_RUN)) &&
          (bldc_fault_latched == 0U))
      {
        result = DRV8301_ReadStatus1(&status);
        if (result != DRV8301_OK)
        {
          BLDC_StopOutput(BLDC_FAULT_SPI, (int32_t)result);
          CAW_LOG_ERROR("DRV8301 status read failed: %d", (int)result);
        }
        else if ((status & DRV8301_SR1_FAULT) != 0U)
        {
          BLDC_StopOutput(BLDC_FAULT_DRV8301, (int32_t)status);
          CAW_LOG_ERROR("DRV8301 fault status=0x%03X", (unsigned int)status);
        }
      }
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
    static uint32_t last_injected_callback_count = 0U;
    uint32_t callback_delta;
    uint32_t current_injected_callback_count;

    current_injected_callback_count = adc_injected_callback_count;
    callback_delta = current_injected_callback_count -
                     last_injected_callback_count;
    if (((bldc_state == BLDC_STATE_ALIGN) ||
         (bldc_state == BLDC_STATE_RUN)) &&
        (callback_delta < ADC_INJECTED_MIN_CALLBACKS_MS))
    {
      BLDC_TripFromIsr(BLDC_FAULT_ADC, (int32_t)callback_delta);
    }
    last_injected_callback_count = current_injected_callback_count;
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
    float raw_iw;
    int current_result;
#if USE_CURRENT_LOOP
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
    raw_iw = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    if ((raw_iu < ADC_RAW_VALID_MIN) || (raw_iu > ADC_RAW_VALID_MAX) ||
        (raw_iv < ADC_RAW_VALID_MIN) || (raw_iv > ADC_RAW_VALID_MAX) ||
        (raw_iw < ADC_RAW_VALID_MIN) || (raw_iw > ADC_RAW_VALID_MAX) ||
        (FOC_Get_Calibrated_Current(raw_iu, raw_iv, raw_iw,
                                    &iu, &iv, &iw) != 0))
    {
      BLDC_TripFromIsr(BLDC_FAULT_ADC, (int32_t)hadc->ErrorCode);
      BLDC_UpdateMaxCycles(&adc_injected_callback_max_cycles, start);
      return;
    }

    current_limit = (state == BLDC_STATE_ALIGN)
                      ? foc.alignment_current_limit
                      : FOC_CURRENT_ABS_LIMIT_A;
    current_result = FOC_ValidatePhaseCurrents(iu, iv, iw, current_limit);
    if (current_result != 0)
    {
      BLDC_TripFromIsr((current_result == FOC_CURRENT_ERROR_SUM)
                         ? BLDC_FAULT_CURRENT_SUM
                         : BLDC_FAULT_CURRENT_LIMIT,
                       current_result);
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
    if (FOC_CurrentLoopControl(&foc, 0.0f, iq_ref, iu, iv, iw,
                               &pid_id, &pid_iq, now) != 0)
    {
      BLDC_TripFromIsr(BLDC_FAULT_SENSOR,
                       (int32_t)(now - FOC_HAL_GetSensorLastSuccessTick()));
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

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    BLDC_TripFromIsr(BLDC_FAULT_ADC, (int32_t)hadc->ErrorCode);
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

