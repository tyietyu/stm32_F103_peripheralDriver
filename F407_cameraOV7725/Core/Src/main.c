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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "ov7725.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OV7725_TEST_WIDTH   320
#define OV7725_TEST_HEIGHT  240
#define USB_TX_BUFFER_SIZE  2048

/* 帧头定义 */
#define FRAME_HEADER_SYNC1  0xAA
#define FRAME_HEADER_SYNC2  0x55
#define FRAME_HEADER_END1   0xA5
#define FRAME_HEADER_END2   0x5A
#define FRAME_HEADER_SIZE   8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t camera_frame_flag = 0;
static volatile uint16_t camera_cnt = 0;
/* 乒乓缓冲区 */
static uint8_t usb_tx_buffer[2][USB_TX_BUFFER_SIZE];
static volatile uint8_t usb_tx_buf_idx = 0;  /* 当前写入的缓冲区索引 */
extern OV7725_Handle_t OV7725_Camera;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceHS;
extern volatile OV7725_Capture_State_t capture_state;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief OV7725摄像头初始化
 * @return 0: 成功, 其他: 失败
 */
static uint8_t OV7725_Setup(void)
{
  uint8_t ret;
  CAW_LOG_INFO("OV7725 Initializing...");
  ret = ov7725_Init();
  if (ret != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Init Failed!");
    return ret;
  }
    
  /* 配置OV7725: QVGA 320x240, 自动灯光, 默认参数 */
  CAW_LOG_INFO("OV7725 Configuring...");
  ret = ov7725_Config(OV7725_TEST_WIDTH, OV7725_TEST_HEIGHT, 
                        OV7725_OUTPUT_MODE_QVGA,
                        OV7725_LIGHT_MODE_AUTO,
                        OV7725_COLOR_SATURATION_4,
                        OV7725_BRIGHTNESS_4,
                        OV7725_CONTRAST_4,
                        OV7725_SPECIAL_EFFECT_NORMAL);
  if (ret != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Config Failed!");
    return ret;
  }
  CAW_LOG_INFO("OV7725 Ready! Width=%d, Height=%d", OV7725_Camera.image_width, OV7725_Camera.image_height);
  return OV7725_OK;
}

/**
 * @brief 等待USB发送完成
 * @param timeout_ms 超时时间(ms)
 * @return 1: 成功, 0: 超时
 */
static uint8_t USB_Wait_Tx_Complete(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  while (hcdc->TxState != 0)
  {
    if (HAL_GetTick() - start_tick > timeout_ms)
    {
      return 0;
    }
  }
  return 1;
}

/**
 * @brief 发送帧头
 * @param frame_idx 帧索引
 * @return 1: 成功, 0: 失败
 */
static uint8_t USB_Send_Frame_Header(uint16_t frame_idx)
{
  uint8_t header[FRAME_HEADER_SIZE];
  header[0] = FRAME_HEADER_SYNC1;
  header[1] = FRAME_HEADER_SYNC2;
  header[2] = (uint8_t)(frame_idx & 0xFF);
  header[3] = (uint8_t)((frame_idx >> 8) & 0xFF);
  header[4] = (uint8_t)(OV7725_Camera.image_width & 0xFF);
  header[5] = (uint8_t)(OV7725_Camera.image_height & 0xFF);
  header[6] = FRAME_HEADER_END1;
  header[7] = FRAME_HEADER_END2;

  if (!USB_Wait_Tx_Complete(500))
  {
    CAW_LOG_ERROR("USB Busy before header!");
    return 0;
  }

  if (CDC_Transmit_HS(header, FRAME_HEADER_SIZE) != USBD_OK)
  {
    CAW_LOG_ERROR("Header TX Failed!");
    return 0;
  }
  return 1;
}

/**
 * @brief 通过USB CDC分块发送一帧图像数据（乒乓缓冲）
 * @note 使用双缓冲：当一个缓冲区正在USB发送时，另一个缓冲区同时读取数据
 *       capture_state 由中断和 ov7725_Frame_Read_End() 管理
 */
static void OV7725_Send_Frame_USB(void)
{
  uint32_t total_pixels = OV7725_Camera.image_width * OV7725_Camera.image_height;
  uint32_t pixels_per_chunk = USB_TX_BUFFER_SIZE / 2;  /* 每像素2字节 */
  uint32_t pixels_sent = 0;
  uint32_t pixels_read = 0;
  uint8_t write_idx = 0;
  uint8_t send_idx = 0;
  uint8_t send_ok = 1;

  /* 只有在 CAPTURE_DONE 状态才能开始发送 */
  if (capture_state != CAPTURE_DONE)
  {
    return;
  }

  if (ov7725_Frame_Read_Start() != OV7725_OK) 
  {
    return;
  }
  
  CAW_LOG_DEBUG("Frame %d sending...", OV7725_Camera.frame.frame_count);

  /* 发送帧头 */
  if (!USB_Send_Frame_Header(OV7725_Camera.frame.frame_count))
  {
    ov7725_Frame_Read_End();
    return;
  }

  /* 预填充第一个缓冲区 */
  uint16_t first_chunk = (total_pixels > pixels_per_chunk) ? pixels_per_chunk : total_pixels;
  ov7725_Frame_Read_Chunk(usb_tx_buffer[write_idx], first_chunk);
  pixels_read += first_chunk;

  while (pixels_sent < total_pixels && send_ok)
  {
    send_idx = write_idx;
    write_idx = 1 - write_idx;
    uint16_t current_chunk = (total_pixels - pixels_sent) > pixels_per_chunk ? pixels_per_chunk : (total_pixels - pixels_sent);

    if (!USB_Wait_Tx_Complete(500))
    {
      CAW_LOG_ERROR("USB Hardware Busy Timeout!");
      send_ok = 0;
      break;
    }

    uint8_t result = CDC_Transmit_HS(usb_tx_buffer[send_idx], current_chunk * 2);
    if (result != USBD_OK)
    {
      CAW_LOG_WARN("USB TX Failed, Code: %d", result);
    }

    /* 在 USB 正在发送的同时，读取下一块数据到另一个缓冲区 (异步并行) */
    if (pixels_read < total_pixels)
    {
      uint16_t next_chunk = (total_pixels - pixels_read) > pixels_per_chunk ? pixels_per_chunk : (total_pixels - pixels_read);
      ov7725_Frame_Read_Chunk(usb_tx_buffer[write_idx], next_chunk);
      pixels_read += next_chunk;
    }
    pixels_sent += current_chunk;
  }

  if (send_ok)
  {
    CAW_LOG_DEBUG("Frame %d sent OK", OV7725_Camera.frame.frame_count - 1);
  }
  ov7725_Frame_Read_End();
}

/**
 * @brief OV7725摄像头主循环处理（非阻塞，分块发送模式）
 */
static void OV7725_Process(void)
{
  if (camera_frame_flag == 0) return;
  camera_frame_flag = 0;

  if (OV7725_Camera.frame.frame_process_flag)
  {
    OV7725_Send_Frame_USB();
  }
}

/**
 * @brief TIM2定时器中断回调函数（1ms周期）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    camera_cnt++;
    if (camera_cnt >= 500)  /* 每500ms置位一次标志 */
    {
      camera_cnt = 0;
      camera_frame_flag = 1;
    }
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  CAW_LOG_Init(&huart1, LEVEL_DEBUG, true);
  CAW_LOG_INFO("System Started!");
  CAW_LOG_INFO("USB CDC + OV7725 Camera Test");

  if (OV7725_Setup() != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Setup Failed! System Halted.");
    while (1);
  }
  
  HAL_TIM_Base_Start_IT(&htim2);
  CAW_LOG_INFO("TIM2 Started, capturing frames...");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 非阻塞处理OV7725帧数据 */
    OV7725_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
