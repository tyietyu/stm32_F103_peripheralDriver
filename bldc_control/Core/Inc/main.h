/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define CKTIM       168000000
#define PWM_PRSC    0
#define PWM_FREQ    15000
#define PWM_PERIOD  CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))

#define ADC_CHANNEL_NUM       3
#define ADC_DATA_LEN          100
#define ADC_REF_VOLTAGE       3.3f
#define ADC_RESOLUTION        4096.0f

#define CURRENT_GAIN            10.0f
#define SHUNT_RESISTOR          0.05f
#define VOLTAGE_TO_CURRENT      (1.0f / (CURRENT_GAIN * SHUNT_RESISTOR))
#define ADC_BIAS_VOLTAGE        1.65f

#define VOLTAGE_DIVIDER_RATIO   25.0f 
#define ADC_VOLTAGE_FACTOR      (ADC_REF_VOLTAGE / ADC_RESOLUTION * VOLTAGE_DIVIDER_RATIO)

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZERO_U_Pin GPIO_PIN_0
#define ZERO_U_GPIO_Port GPIOB
#define ZERO_V_Pin GPIO_PIN_1
#define ZERO_V_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define ZERO_W_Pin GPIO_PIN_3
#define ZERO_W_GPIO_Port GPIOB
#define AS5600_SCL_Pin GPIO_PIN_6
#define AS5600_SCL_GPIO_Port GPIOB
#define AS5600_SDA_Pin GPIO_PIN_7
#define AS5600_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
