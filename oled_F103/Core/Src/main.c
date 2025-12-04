/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "WS2812B.h"
#include "esp8266.h"
#include "ee.h"
#include "otadriver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char oled_buff[20] = {0};
uint8_t Rxbuff = 0;

extern uint8_t index_send_msg;
extern uint16_t index_led;
uint8_t led_status = 0;
uint8_t led_vol = 0;

volatile uint16_t mqtt_receive_count = 0;
volatile uint8_t mqtt_receive_complete = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 1. 定义三组不同的测试数据和对应的句柄 (与之前相同)
// 第一组: 用户ID
uint32_t user_id = 19981231;
EE_HandleTypeDef handle_user_id;

// 第二组: GPS坐标
typedef struct {
    double latitude;
    double longitude;
} GpsCoord;
GpsCoord last_location = { .latitude = 39.9042, .longitude = 116.4074 };
EE_HandleTypeDef handle_gps;

// 第三组: 设备状态
uint8_t device_status_flags = 0xAC;  // 8个标志位
EE_HandleTypeDef handle_status;


// 2. 规划每组数据在EEPROM中的存储布局（偏移地址）
#define OFFSET_USER_ID          0
#define OFFSET_GPS_COORD        32  // 预留一些空间
#define OFFSET_STATUS_FLAGS     64
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t app_payload_buffer[256];
void ESP8266_demo(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  void flash_test(void)
  {
     printf("========= EEPROM Emulation Test Case V2 =========\n");

    // 根据编译时选择的配置，打印当前测试的内存布局

    printf("--- Running in [STANDALONE / NO BOOTLOADER] mode ---\n");
    printf("--- Simulated EEPROM Address: 0x%X (At the end of Flash) ---\n\n", (unsigned int)EE_ADDRESS);

    // ================== 写入阶段 ==================
    printf("========= Phase 1: Writing data =========\n");

    EE_Init(&handle_user_id, &user_id, sizeof(user_id), OFFSET_USER_ID);
    EE_Init(&handle_gps, &last_location, sizeof(last_location), OFFSET_GPS_COORD);
    EE_Init(&handle_status, &device_status_flags, sizeof(device_status_flags), OFFSET_STATUS_FLAGS);

    if (!EE_Format()) {
        printf("Error: Failed to format EEPROM area.\n");
    }

    printf("Writing User ID: %u at offset %d\n", user_id, OFFSET_USER_ID);
    EE_Write(&handle_user_id);

    printf("Writing GPS Location: lat=%.4f, lon=%.4f at offset %d\n", last_location.latitude, last_location.longitude, OFFSET_GPS_COORD);
    EE_Write(&handle_gps);

    printf("Writing Status Flags: 0x%02X at offset %d\n", device_status_flags, OFFSET_STATUS_FLAGS);
    EE_Write(&handle_status);
    
    printf("\n========= Write phase finished. =========\n\n");


    // ================== 读取与验证阶段 ==================
    printf("========= Phase 2: Reading and Verifying data =========\n");

    uint32_t read_user_id = 0;
    GpsCoord read_gps = {0.0, 0.0};
    uint8_t read_status = 0;

    EE_Init(&handle_user_id, &read_user_id, sizeof(read_user_id), OFFSET_USER_ID);
    EE_Init(&handle_gps, &read_gps, sizeof(read_gps), OFFSET_GPS_COORD);
    EE_Init(&handle_status, &read_status, sizeof(read_status), OFFSET_STATUS_FLAGS);
    
    printf("\nReading data...\n");
    EE_Read(&handle_user_id);
    EE_Read(&handle_gps);
    EE_Read(&handle_status);

    printf("\n--- Verification ---\n");
    printf("User ID:         Original = %-12u | Read = %-12u\n", user_id, read_user_id);
    printf("GPS Location:    Original = %.4f, %.4f | Read = %.4f, %.4f\n", last_location.latitude, last_location.longitude, read_gps.latitude, read_gps.longitude);
    printf("Status Flags:    Original = 0x%-12X | Read = 0x%-12X\n", device_status_flags, read_status);

    bool success = (user_id == read_user_id) &&
                   (memcmp(&last_location, &read_gps, sizeof(GpsCoord)) == 0) &&
                   (device_status_flags == read_status);
    
    printf("\n==============================================\n");
    if (success) {
        printf("SUCCESS: All data read back matches original data!\n");
    } else {
        printf("FAILURE: Read-back data does not match!\n");
    }
    printf("==============================================\n");

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  ESP8266_demo();
  time2_start();
  // while (MPU6050_Init(&hi2c2) == 1) {};
  flash_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ESP8266_receive_msg(NULL, app_payload_buffer, 256);
    OTA_Loop();
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ESP8266_demo(void)
{
  if (ESP8266_init(1, 1) == ESP8266_EOK) 
  {
    printf("MQTT Connected!\r\n");
    OTA_Init(); 
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

#ifdef USE_FULL_ASSERT
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
