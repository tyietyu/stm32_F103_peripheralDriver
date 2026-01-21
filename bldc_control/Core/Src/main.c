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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "foc_hal.h"
#include "foc_test.h"
#include "hal_iic.h"
#include "log.h"
#include "lowpass_filter.h"
#include "math.h"
#include "pid.h"
#include "as5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern AS5600_T G_SENSOR_A;

typedef struct flag
{
  volatile uint8_t flag;
  volatile uint16_t count;
}g_Flag_t;
g_Flag_t g_led_status={0};
g_Flag_t g_bldc_motor_status={0};

typedef struct {
  volatile float I_U; // U相电流 (A)
  volatile float I_V; // V相电流 (A)
  volatile float I_W; // W相电流 (A)
  volatile float V_U; // U相电压 (V)
  volatile float V_V; // V相电压 (V)
  volatile float V_W; // W相电压 (V)
} Monitor_Data_t;
Monitor_Data_t g_monitor_data;

typedef struct{
  volatile uint32_t adc_raw_data[ADC_CHANNEL_NUM];								
}uAdcValue_t;
uAdcValue_t adc_value;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FOC_T foc;
PID_T velPID;
PID_T anglePID;
LOWPASS_FILTER_T velFilter;
float target_velocity = 20.0f; // 目标速度 20 rad/s
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value.adc_raw_data, ADC_CHANNEL_NUM);
  CAW_LOG_Init(&huart1, LEVEL_DEBUG, true);
  CAW_LOG_DEBUG("CAW FOC start ...");

  /*  FOC 初始化配置 */
  FOC_Closeloop_Init(&foc, &htim1, PWM_PERIOD, 24.0f, 1, 11);
  FOC_SetVoltageLimit(&foc, 24.0f);
  FOC_HAL_InitA(&foc);
  PID_Init(&velPID, 0.05f, 0.01f, 0.0f, 1000.0f,foc.voltage_power_supply / 2);
  PID_Init(&anglePID, 2, 0, 0, 100000.0f, 100);
  LOWPASS_FILTER_Init(&velFilter, 0.01f);
  FOC_AlignmentSensor(&foc);
  FOC_CurrentLoop_Init(&foc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(g_bldc_motor_status.flag)
    {
      g_bldc_motor_status.flag = 0;
      FOC_SensorUpdate(&foc);
      // 2. 速度环计算
      float current_vel = foc.Sensor_GetVelocity();
      current_vel = LOWPASS_FILTER_Calc(&velFilter, foc.dir * current_vel);
      float Iq_cmd = PID_Calc(&velPID, target_velocity - current_vel);
      // 3. 执行电流闭环
      FOC_SetCurrent(&foc, Iq_cmd, FOC_CloseloopElectricalAngle(&foc));
    }

    if(g_led_status.flag)
    {
      g_led_status.flag = 0;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
    if(g_bldc_motor_status.count ++ >= 1)
    {
      g_bldc_motor_status.count = 0;
      g_bldc_motor_status.flag = 1;
    }

    if (g_led_status.count ++ >= 200)
    {
      g_led_status.count = 0;
      g_led_status.flag = 1;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
  {
    g_monitor_data.V_U = adc_value.adc_raw_data[0] * ADC_VOLTAGE_FACTOR;
    g_monitor_data.V_V = adc_value.adc_raw_data[1] * ADC_VOLTAGE_FACTOR;
    g_monitor_data.V_W = adc_value.adc_raw_data[2] * ADC_VOLTAGE_FACTOR;
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  const float factor = ADC_REF_VOLTAGE / ADC_RESOLUTION;
  if (hadc->Instance == ADC1)
  {
    float raw_iu = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    float raw_iv = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    float raw_iw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    foc.I_u = (raw_iu * factor - ADC_BIAS_VOLTAGE) * VOLTAGE_TO_CURRENT;
    foc.I_v = (raw_iv * factor - ADC_BIAS_VOLTAGE) * VOLTAGE_TO_CURRENT;
    foc.I_w = (raw_iw * factor - ADC_BIAS_VOLTAGE) * VOLTAGE_TO_CURRENT;
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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

