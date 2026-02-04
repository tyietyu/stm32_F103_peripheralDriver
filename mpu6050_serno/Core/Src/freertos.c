/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float pitch;
  float roll;
  float yaw;
} MPU6050_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void Servo_Control(uint16_t angle)   
{
    float temp;
    temp =  50.0 + ((float)angle * 200.0 / 180.0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)temp);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for mpu6050_task */
osThreadId_t mpu6050_taskHandle;
const osThreadAttr_t mpu6050_task_attributes = {
  .name = "mpu6050_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for serno_task */
osThreadId_t serno_taskHandle;
const osThreadAttr_t serno_task_attributes = {
  .name = "serno_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for usart_task */
osThreadId_t usart_taskHandle;
const osThreadAttr_t usart_task_attributes = {
  .name = "usart_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu6050_data */
osMessageQueueId_t mpu6050_dataHandle;
const osMessageQueueAttr_t mpu6050_data_attributes = {
  .name = "mpu6050_data"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
}
/* USER CODE END FunctionPrototypes */

void MPU6050Task(void *argument);
void SERNO_TASK(void *argument);
void USART_TASK(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    MUP6050_Init();
    DMP_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */	
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of mpu6050_data */
  mpu6050_dataHandle = osMessageQueueNew (10, sizeof(MPU6050_Data_t), &mpu6050_data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mpu6050_task */
  mpu6050_taskHandle = osThreadNew(MPU6050Task, NULL, &mpu6050_task_attributes);

  /* creation of serno_task */
  serno_taskHandle = osThreadNew(SERNO_TASK, NULL, &serno_task_attributes);

  /* creation of usart_task */
  usart_taskHandle = osThreadNew(USART_TASK, NULL, &usart_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MPU6050Task */
/**
  * @brief  Function implementing the mpu6050_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MPU6050Task */
void MPU6050Task(void *argument)
{
  /* USER CODE BEGIN MPU6050Task */
  float pitch,roll,yaw;
  uint8_t result;
  MPU6050_Data_t mpu6050_data;

  /* Infinite loop */
  for(;;)
  {
    result = Read_DMP(&pitch,&roll,&yaw);
    if(result == 0){
    mpu6050_data.pitch = pitch;
    mpu6050_data.roll = roll;
    mpu6050_data.yaw = yaw;
    osMessageQueuePut(mpu6050_dataHandle,&mpu6050_data,0,0);
    }else
    {
      printf("mpu_dmp_get_data error :%d\r\n",result);
    }
    osDelay(5);
  }
  /* USER CODE END MPU6050Task */
}

/* USER CODE BEGIN Header_SERNO_TASK */
/**
* @brief Function implementing the serno_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SERNO_TASK */
void SERNO_TASK(void *argument)
{
  /* USER CODE BEGIN SERNO_TASK */
  uint16_t angle = 90;
  int8_t direction = -1; // 1表示增加角度，-1表示减少角度
  uint16_t step = 5;   // 角度步进值
  const uint16_t delay_servo = 25; // 每步延时
  
  /* Infinite loop */
  for(;;)
  {
    Servo_Control(angle);
    osDelay(delay_servo);
    uint16_t new_angle = angle + direction * step;
    if(new_angle >= 180) {
      new_angle = 180;
      direction = -1;
      osDelay(100);
    } else if(new_angle <= 0) {
      new_angle = 0;
      direction = 1;
      osDelay(100);
    }
   angle = new_angle;
  }
  /* USER CODE END SERNO_TASK */
}

/* USER CODE BEGIN Header_USART_TASK */
/**
* @brief Function implementing the usart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USART_TASK */
void USART_TASK(void *argument)
{
  /* USER CODE BEGIN USART_TASK */
  MPU6050_Data_t mpu6050_data;
  osStatus_t status;
  uint32_t msg_count = 0;
  
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(mpu6050_dataHandle, &mpu6050_data, 0, osWaitForever);
    if(status == osOK) {
      printf("msg[%lu] pitch:%.2f,roll:%.2f,yaw:%.2f\n", 
             msg_count++, mpu6050_data.pitch, mpu6050_data.roll, mpu6050_data.yaw);
    } else {
      printf("Error getting message from queue: %d\n", status);
    }
    osDelay(10);
  }
  /* USER CODE END USART_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

