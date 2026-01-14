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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 图像分辨率配置 */
#define OV7725_TEST_WIDTH   320
#define OV7725_TEST_HEIGHT  240

/* 协议常量：必须与上位机 Python 脚本严格一致 */
#define FRAME_HEADER_SYNC1  0xAA
#define FRAME_HEADER_SYNC2  0x55
#define FRAME_HEADER_END1   0xA5
#define FRAME_HEADER_END2   0x5A
#define FRAME_HEADER_SIZE   10  /* 同步字(2) + 帧号(2) + 总包数(2) + 每包大小(2) + 结束字(2) */

#define PACKET_DATA_SIZE    2048 /* 每包负载大小 */
#define PACKET_HEADER_SIZE  4    /* 包序号(2) + 数据长度(2) */
#define TOTAL_PACKET_SIZE   (PACKET_DATA_SIZE + PACKET_HEADER_SIZE)

#define USB_TX_BUFFER_SIZE  PACKET_DATA_SIZE  /* USB发送缓冲区大小 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 双缓冲：当一个缓冲区正在USB发送时，另一个缓冲区同时读取FIFO数据 */
static uint8_t usb_tx_buffer[2][USB_TX_BUFFER_SIZE];

extern OV7725_Handle_t OV7725_Camera;
extern volatile OV7725_Capture_State_t capture_state;
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint32_t frame_count = 0;
static volatile uint16_t led_toggle_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t OV7725_Setup(void);
static void OV7725_Process(void);
static uint8_t USB_Wait_Tx_Complete(uint32_t timeout_ms);
static uint8_t USB_Send_Frame_Header(uint16_t frame_idx, uint16_t total_packets, uint16_t packet_size);
static uint8_t USB_Send_Data_Packet(uint8_t *data, uint16_t data_len, uint16_t packet_idx);
static void OV7725_Send_Frame_USB(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief OV7725 初始化与配置
 */
static uint8_t OV7725_Setup(void)
{
  if (ov7725_Init() != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Hardware Init Failed!");
    return OV7725_ERROR;
  }
    
  /* 配置为 QVGA (320x240) RGB565 格式 */
  if (ov7725_Config(OV7725_TEST_WIDTH, OV7725_TEST_HEIGHT, 
                        OV7725_OUTPUT_MODE_QVGA,
                        OV7725_LIGHT_MODE_AUTO,
                        OV7725_COLOR_SATURATION_4,
                        OV7725_BRIGHTNESS_4,
                        OV7725_CONTRAST_4,
                        OV7725_SPECIAL_EFFECT_NORMAL) != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Logic Config Failed!");
    return OV7725_ERROR;
  }
  
  CAW_LOG_INFO("OV7725 Ready! Resolution: %dx%d", OV7725_Camera.image_width, OV7725_Camera.image_height);
  return OV7725_OK;
}

/**
 * @brief 等待 USB CDC 发送缓冲区空闲
 */
static uint8_t USB_Wait_Tx_Complete(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  
  if (hcdc == NULL) return HAL_ERROR;
  
  while (hcdc->TxState != 0)
  {
    if (HAL_GetTick() - start > timeout_ms)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

/**
 * @brief 发送 10 字节同步帧头
 * @return HAL_OK成功, HAL_ERROR失败
 */
static uint8_t USB_Send_Frame_Header(uint16_t frame_idx, uint16_t total_packets, uint16_t packet_size)
{
  uint8_t header[FRAME_HEADER_SIZE];
  
  header[0] = FRAME_HEADER_SYNC1;
  header[1] = FRAME_HEADER_SYNC2;
  header[2] = (uint8_t)(frame_idx & 0xFF);
  header[3] = (uint8_t)((frame_idx >> 8) & 0xFF);
  header[4] = (uint8_t)(total_packets & 0xFF);
  header[5] = (uint8_t)((total_packets >> 8) & 0xFF);
  header[6] = (uint8_t)(packet_size & 0xFF);
  header[7] = (uint8_t)((packet_size >> 8) & 0xFF);
  header[8] = FRAME_HEADER_END1;
  header[9] = FRAME_HEADER_END2;

  if (USB_Wait_Tx_Complete(500) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if (CDC_Transmit_FS(header, FRAME_HEADER_SIZE) != USBD_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
 * @brief 发送数据包（包头+数据）
 * @return HAL_OK成功, HAL_ERROR失败
 */
static uint8_t USB_Send_Data_Packet(uint8_t *data, uint16_t data_len, uint16_t packet_idx)
{
  uint8_t pkt_header[PACKET_HEADER_SIZE];
  
  /* 构建包头: 包序号(2B) + 数据长度(2B) */
  pkt_header[0] = (uint8_t)(packet_idx & 0xFF);
  pkt_header[1] = (uint8_t)((packet_idx >> 8) & 0xFF);
  pkt_header[2] = (uint8_t)(data_len & 0xFF);
  pkt_header[3] = (uint8_t)((data_len >> 8) & 0xFF);
  
  /* 等待上一次发送完成 */
  if (USB_Wait_Tx_Complete(500) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* 发送包头 */
  if (CDC_Transmit_FS(pkt_header, PACKET_HEADER_SIZE) != USBD_OK)
  {
    return HAL_ERROR;
  }
  
  /* 等待包头发送完成 */
  if (USB_Wait_Tx_Complete(500) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* 发送数据 */
  if (CDC_Transmit_FS(data, data_len) != USBD_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief 通过USB CDC分块发送一帧图像数据（带包序号验证）
 * @note 使用双缓冲：当一个缓冲区正在USB发送时，另一个缓冲区同时读取数据
 *       每个数据包带有包序号，上位机可验证完整性
 */
static void OV7725_Send_Frame_USB(void)
{
  uint32_t total_pixels = OV7725_Camera.image_width * OV7725_Camera.image_height;
  uint32_t total_bytes = total_pixels * 2;  /* RGB565每像素2字节 */
  uint32_t bytes_per_chunk = USB_TX_BUFFER_SIZE;
  uint16_t total_packets = (total_bytes + bytes_per_chunk - 1) / bytes_per_chunk;
  uint32_t bytes_sent = 0;
  uint32_t bytes_read = 0;
  uint16_t packet_idx = 0;
  uint8_t write_idx = 0;
  uint8_t send_idx = 0;
  uint8_t send_ok = 1;

  /* 只有在 CAPTURE_DONE 状态才能开始发送 */
  if (capture_state != CAPTURE_DONE)
  {
    return;
  }
  
  /* 检查USB是否已连接 */

  if (ov7725_Frame_Read_Start() != OV7725_OK) 
  {
    return;
  }
  
  CAW_LOG_INFO("Frame %d sending, %d packets, USB connected", frame_count, total_packets);

  /* 发送帧头（包含总包数和每包大小） */
  if (USB_Send_Frame_Header(frame_count, total_packets, bytes_per_chunk) != HAL_OK)
  {
    ov7725_Frame_Read_End();
    return;
  }

  /* 预填充第一个缓冲区 */
  uint16_t first_chunk_bytes = (total_bytes > bytes_per_chunk) ? bytes_per_chunk : total_bytes;
  uint16_t first_chunk_pixels = first_chunk_bytes / 2;
  ov7725_Frame_Read_Chunk(usb_tx_buffer[write_idx], first_chunk_pixels);
  bytes_read += first_chunk_bytes;

  while (bytes_sent < total_bytes && send_ok)
  {
    send_idx = write_idx;
    write_idx = 1 - write_idx;
    uint16_t current_bytes = (total_bytes - bytes_sent) > bytes_per_chunk ? bytes_per_chunk : (total_bytes - bytes_sent);

    /* 发送带包序号的数据包 */
    if (USB_Send_Data_Packet(usb_tx_buffer[send_idx], current_bytes, packet_idx) != HAL_OK)
    {
      send_ok = 0;
      break;
    }
    packet_idx++;

    /* 在 USB 正在发送的同时，读取下一块数据到另一个缓冲区 (异步并行) */
    if (bytes_read < total_bytes)
    {
      uint16_t next_bytes = (total_bytes - bytes_read) > bytes_per_chunk ? bytes_per_chunk : (total_bytes - bytes_read);
      uint16_t next_pixels = next_bytes / 2;
      ov7725_Frame_Read_Chunk(usb_tx_buffer[write_idx], next_pixels);
      bytes_read += next_bytes;
    }
    bytes_sent += current_bytes;
  }

  if (send_ok)
  {
    CAW_LOG_DEBUG("Frame %d sent OK, %d packets", frame_count, packet_idx);
    frame_count++;
  }
  ov7725_Frame_Read_End();
}

/**
 * @brief 处理一帧图像的读取与 USB 传输
 * @note OV7725_Send_Frame_USB 内部已调用 ov7725_Frame_Read_End()
 *       发送完成后状态重置为 IDLE，允许 VSYNC 触发新采集
 */
static void OV7725_Process(void)
{
  if (ov7725_Frame_Available())
  {
    OV7725_Send_Frame_USB();
    CAW_LOG_DEBUG("Frame sent, capturing next...");
  }
}

/**
 * @brief 定时器回调用于 LED 指示
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (led_toggle_cnt++ >= 500) 
    {
       led_toggle_cnt = 0;
       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  Main program entry point
  */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  CAW_LOG_Init(&huart1, LEVEL_DEBUG, true);
  CAW_LOG_INFO("System Core Started");

  /* 初始化摄像头 */
  if (OV7725_Setup() != OV7725_OK)
  {
    CAW_LOG_ERROR("Camera Setup Failed!");
    Error_Handler();
  }
  
  HAL_TIM_Base_Start_IT(&htim2);
  CAW_LOG_INFO("Ready to Stream. Waiting for VSYNC...");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
