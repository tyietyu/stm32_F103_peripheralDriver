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
#define USB_TX_BUFFER_SIZE  2048  /* 增大缓冲区减少包数量 */

/* 帧头定义 */
#define FRAME_HEADER_SYNC1  0xAA
#define FRAME_HEADER_SYNC2  0x55
#define FRAME_HEADER_END1   0xA5
#define FRAME_HEADER_END2   0x5A
#define FRAME_HEADER_SIZE   10  /* 同步字(2B) + 帧号(2B) + 总包数(2B) + 每包大小(2B) + 结束字(2B) */

/* 数据包头定义 */
#define PACKET_HEADER_SIZE  4   /* 包序号(2B) + 数据长度(2B) */
#define PACKET_BUFFER_SIZE  (USB_TX_BUFFER_SIZE + PACKET_HEADER_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 乒乓缓冲区 */
static uint8_t usb_tx_buffer[2][USB_TX_BUFFER_SIZE];
extern OV7725_Handle_t OV7725_Camera;
/* LED闪烁计数 */
static volatile uint16_t led_toggle_cnt = 0;
/* 帧计数 */
static uint32_t frame_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
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
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  while (hcdc->TxState != 0)
  {
    if (HAL_GetTick() - start_tick > timeout_ms)
    {
      /* 超时，强制清除TxState以恢复发送能力 */
      CAW_LOG_WARN("TxState timeout, force clear");
      hcdc->TxState = 0;
      return 1;  /* 返回1继续发送，不要中断整个帧传输 */
    }  
  }
  return 1;
}

/**
 * @brief 发送帧头
 * @param frame_idx 帧索引
 * @param total_packets 总包数
 * @param packet_size 每包数据大小(字节)
 * @return 1: 成功, 0: 失败
 */
static uint8_t USB_Send_Frame_Header(uint16_t frame_idx, uint16_t total_packets, uint16_t packet_size)
{
  uint8_t header[FRAME_HEADER_SIZE];
  uint8_t result;
  
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

  /* 打印帧头内容用于调试 */
  CAW_LOG_INFO("Header: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
               header[0], header[1], header[2], header[3], header[4],
               header[5], header[6], header[7], header[8], header[9]);

  if (!USB_Wait_Tx_Complete(2000))
  {
    CAW_LOG_ERROR("USB not connected or busy!");
    return 0;
  }

  result = CDC_Transmit_FS(header, FRAME_HEADER_SIZE);
  if (result != USBD_OK)
  {
    CAW_LOG_ERROR("Header TX Failed! result=%d", result);
    return 0;
  }
  
  CAW_LOG_INFO("Header TX OK");
  
  /* 等待帧头发送完成 */
  if (!USB_Wait_Tx_Complete(2000))
  {
    CAW_LOG_ERROR("Header TX complete timeout!");
    return 0;
  }
  
  return 1;
}

/**
 * @brief 发送带包头的数据包（零拷贝方式）
 * @param data 数据指针（直接发送，不复制）
 * @param data_len 数据长度(字节)
 * @param packet_idx 包序号
 * @return 1: 成功, 0: 失败
 */
static uint8_t USB_Send_Data_Packet(uint8_t *data, uint16_t data_len, uint16_t packet_idx)
{
  uint8_t pkt_header[PACKET_HEADER_SIZE];
  uint8_t result;
  
  /* 构建包头: 包序号(2B) + 数据长度(2B) */
  pkt_header[0] = (uint8_t)(packet_idx & 0xFF);
  pkt_header[1] = (uint8_t)((packet_idx >> 8) & 0xFF);
  pkt_header[2] = (uint8_t)(data_len & 0xFF);
  pkt_header[3] = (uint8_t)((data_len >> 8) & 0xFF);
  
  /* 先等待上一次发送完成 */
  if (!USB_Wait_Tx_Complete(2000))
  {
    CAW_LOG_ERROR("Pkt%d: Wait1 timeout", packet_idx);
    return 0;
  }
  
  /* 发送包头 */
  result = CDC_Transmit_FS(pkt_header, PACKET_HEADER_SIZE);
  if (result != USBD_OK)
  {
    CAW_LOG_ERROR("Pkt%d: Header TX fail=%d", packet_idx, result);
    return 0;
  }
  
  /* 等待包头发送完成 */
  if (!USB_Wait_Tx_Complete(2000))
  {
    CAW_LOG_ERROR("Pkt%d: Wait2 timeout", packet_idx);
    return 0;
  }
  
  /* 发送数据 */
  result = CDC_Transmit_FS(data, data_len);
  if (result != USBD_OK)
  {
    CAW_LOG_ERROR("Pkt%d: Data TX fail=%d, len=%d", packet_idx, result, data_len);
    return 0;
  }
  
  return 1;
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
  if (!USB_Send_Frame_Header(frame_count, total_packets, bytes_per_chunk))
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
    if (!USB_Send_Data_Packet(usb_tx_buffer[send_idx], current_bytes, packet_idx))
    {
      CAW_LOG_ERROR("Packet %d TX Failed!", packet_idx);
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
 * @brief OV7725摄像头主循环处理（VSYNC触发模式）
 * @note VSYNC信号触发帧采集完成后，立即发送帧数据
 *       发送完成后等待下一个VSYNC信号，避免阻塞
 */
static void OV7725_Process(void)
{
  /* 检查是否有帧数据可读（VSYNC触发） */
  if (ov7725_Frame_Available())
  {
    OV7725_Send_Frame_USB();
  }
}

/**
 * @brief TIM2定时器中断回调函数（1ms周期）
 * @note 仅用于LED闪烁指示系统运行状态
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (led_toggle_cnt++ >= 200)  /* 每500ms翻转一次LED状态 */
    {
       led_toggle_cnt = 0;
       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
  if (OV7725_Setup() != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Setup Failed! System Halted.");
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

