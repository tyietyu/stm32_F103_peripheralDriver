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
#include "foc_test.h"
#include "hal_iic.h"
#include "log.h"
#include "lowpass_filter.h"
#include "math.h"
#include "pid.h"
#include "as5600.h"
#include "drv8301.h"  
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
  volatile float I_U; // U Current(A)
  volatile float I_V; // V Current(A)
  volatile float I_W; // W Current(A)
  volatile float V_U; // U Voltage(V)
  volatile float V_V; // V Voltage(V)
  volatile float V_W; // W Voltage(V)
} Monitor_Data_t;
Monitor_Data_t g_monitor_data;

uint32_t adc_Voltage_buf[ADC_CHANNEL_NUM];
LOWPASS_FILTER_T voltage_filter[ADC_CHANNEL_NUM];
LOWPASS_FILTER_T current_filter[ADC_CHANNEL_NUM];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG_TO_RAD(deg)     ((deg) * 3.14159f / 180.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FOC_T foc;
PID_T velPID;
PID_T anglePID;
PID_T pid_id, pid_iq;
LOWPASS_FILTER_T velFilter;

float target_velocity = 20.0f;  // TargetSpeed 20 rad/s
float target_angle = 180.0f;  // TargetAngle 180°
volatile float iq_ref = 0.0f; // QCurrentRef
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void ADC_Filter_Init(float voltage_tf, float current_tf)
{
  for (int i = 0; i < ADC_CHANNEL_NUM; i++) 
  {
    LOWPASS_FILTER_Init(&voltage_filter[i], voltage_tf);
    LOWPASS_FILTER_Init(&current_filter[i], current_tf);
  }
}

static void Enable_DRV8301_Driver(uint8_t status)
{
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_PERIOD / 2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Voltage_buf, ADC_CHANNEL_NUM);

  DRV8301_ConfigInit(&hspi2,
                      DRV8301_CR1_GATE_CURRENT_1_7A,    // 1.7A 栅极驱动电流
                      DRV8301_CR1_PWM_MODE_6PWM,        // 6-PWM 模式
                      DRV8301_CR1_OC_MODE_LATCH_SD,     // 过流锁存关断
                      DRV8301_OC_ADJ_SET_0_250V,        // 0.25V VDS 阈值
                      DRV8301_CR2_GAIN_10,              // 放大器增益 10V/V
                      DRV8301_CR2_OCTW_MODE_OT_OC,      // 过温报告模式
                      DRV8301_CR2_OC_TOFF_CYCLE);       // 过温关断关闭


  /*  FOC init */
  ADC_Filter_Init(0.01f, 0.005f);
  FOC_Closeloop_Init(&foc, &htim1, PWM_PERIOD, 24.0f, 1, 11);   // motor 24V  volatge input,1 dir,  11 pole pairs
  FOC_SetVoltageLimit(&foc, 24.0f);                             // voltage limit 24V
  FOC_HAL_InitA(&foc);
  
  //P I D Ramp Limit
  PID_Init(&velPID, 2.0f, 0.5f, 0.0f, 1000.0f, 0.7f);      //0.7A max
  PID_Init(&anglePID, 5.0f, 0.1f, 0.05f, 500.0f, 40.0f);    //40rad/s max
  PID_Init(&pid_id, 1.5f, 0.05f, 0.0f, 1000.0f, 8.0f);      //8V max
  PID_Init(&pid_iq, 1.5f, 0.05f, 0.0f, 1000.0f, 8.0f);      //8V max
  
  LOWPASS_FILTER_Init(&velFilter, 0.01f);                   //filter time constant 10ms
  FOC_Current_Offset_Calibration(&hadc1, 200);
  Enable_DRV8301_Driver(1);
  FOC_AlignmentSensor(&foc);
  FOC_SensorUpdate(&foc);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  CAW_LOG_Init(&huart1, LEVEL_DEBUG, true);
  CAW_LOG_DEBUG("BLDC Motor ready ...");
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

      // VelocityLoop iq_ref
      float temp_iq = Foc_VelocityLoop(&foc, &velFilter, &velPID, target_velocity);
      __disable_irq();
      iq_ref = temp_iq;
      __enable_irq();

      //PositionLoop iq_ref
      //float temp_iq = Foc_PositionLoop(&foc, &anglePID, &velFilter, &velPID,DEG_TO_RAD(target_angle));
      //__disable_irq();
      //iq_ref = temp_iq;
      //__enable_irq();
    }

    if(g_led_status.flag)
    {
      g_led_status.flag = 0;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      if(DRV8301_HasFault())
      {
        char fault_str[64];
        uint16_t status = DRV8301_ReadStatus1();
        DRV8301_GetFaultString(status, fault_str, sizeof(fault_str));
        CAW_LOG_ERROR("DRV8301 Fault: %s", fault_str);
        DRV8301_ClearFaults();
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
    if(g_bldc_motor_status.count ++ >= 10) //1ms FOC angle update
    {
      g_bldc_motor_status.count = 0;
      g_bldc_motor_status.flag = 1;
    }

    if (g_led_status.count ++ >= 2000) //200ms LED
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
    float phase_voltage[ADC_CHANNEL_NUM];
    for (int i = 0; i < ADC_CHANNEL_NUM; i++) 
    {
      float raw_voltage = (float)adc_Voltage_buf[i] * ADC_VOLTAGE_FACTOR;
       phase_voltage[i] = LOWPASS_FILTER_Calc(&voltage_filter[i], raw_voltage);
    }

    g_monitor_data.V_U = phase_voltage[0];
    g_monitor_data.V_V = phase_voltage[1];
    g_monitor_data.V_W = phase_voltage[2];
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    float raw_iu = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    float raw_iv = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    float raw_iw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    float iu, iv, iw;
    FOC_Get_Calibrated_Current(raw_iu, raw_iv, raw_iw, &iu, &iv, &iw);

    g_monitor_data.I_U = LOWPASS_FILTER_Calc(&current_filter[0], iu);
    g_monitor_data.I_V = LOWPASS_FILTER_Calc(&current_filter[1], iv);
    g_monitor_data.I_W = LOWPASS_FILTER_Calc(&current_filter[2], iw);

    FOC_CurrentLoopControl(&foc, 0.0f, iq_ref, 
                          g_monitor_data.I_U, 
                          g_monitor_data.I_V, 
                          g_monitor_data.I_W, 
                          &pid_id, &pid_iq);
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

