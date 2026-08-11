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
/*
 * 环路级联关系：位置环 -> 速度环 -> 电流环。
 * 默认仅启用电流环，外环需完成参数整定后显式开启。
 */
#ifndef USE_CURRENT_LOOP
#define USE_CURRENT_LOOP  1
#endif

#ifndef USE_SPEED_LOOP
#define USE_SPEED_LOOP    0
#endif

#ifndef USE_POSITION_LOOP
#define USE_POSITION_LOOP 0
#endif

#if ((USE_CURRENT_LOOP != 0) && (USE_CURRENT_LOOP != 1))
#error "USE_CURRENT_LOOP must be 0 or 1"
#endif

#if ((USE_SPEED_LOOP != 0) && (USE_SPEED_LOOP != 1))
#error "USE_SPEED_LOOP must be 0 or 1"
#endif

#if ((USE_POSITION_LOOP != 0) && (USE_POSITION_LOOP != 1))
#error "USE_POSITION_LOOP must be 0 or 1"
#endif

#if (USE_SPEED_LOOP && !USE_CURRENT_LOOP)
#error "USE_SPEED_LOOP requires USE_CURRENT_LOOP"
#endif

#if (USE_POSITION_LOOP && !USE_SPEED_LOOP)
#error "USE_POSITION_LOOP requires USE_SPEED_LOOP"
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define ADC_CHANNEL_NUM       3
#define ADC_DATA_LEN          8
#define ADC_REF_VOLTAGE       3.3f
#define ADC_RESOLUTION        4096.0f

#define CURRENT_GAIN            10.0f
#define SHUNT_RESISTOR          0.01f
#define VOLTAGE_TO_CURRENT      (1.0f / (CURRENT_GAIN * SHUNT_RESISTOR))
#define ADC_BIAS_VOLTAGE        1.65f

#define VOLTAGE_DIVIDER_RATIO   8.0f 
#define ADC_VOLTAGE_FACTOR      (ADC_REF_VOLTAGE / ADC_RESOLUTION * VOLTAGE_DIVIDER_RATIO)

/* 21 MHz ADC: 2 channels x (15 sample + 12 conversion) cycles x 8 TIM clocks
   = 432 tick = 2.571 us。必须与 adc.c 的 InjectedSamplingTime 保持一致。 */
#define PWM_ADC_SEQUENCE_TICKS       432U
#define PWM_ADC_END_MARGIN_TICKS     100U
#define PWM_ADC_PEAK_BLANKING_TICKS  200U
#define PWM_MIN_COMPARE              PWM_ADC_PEAK_BLANKING_TICKS
#define PWM_MAX_COMPARE              (PWM_PERIOD - PWM_ADC_SEQUENCE_TICKS - \
                                      PWM_ADC_END_MARGIN_TICKS - \
                                      PWM_ADC_PEAK_BLANKING_TICKS)
#define PWM_ADC_TRIGGER_LATEST       (PWM_PERIOD - PWM_ADC_PEAK_BLANKING_TICKS)

/*
 * 采样窗口的真正约束是"转换结束点仍在低侧共同导通窗口内"：
 *   trigger + SEQUENCE <= 2*ARR - max_compare - END_MARGIN
 * FOC_SetPwm() 的运行时判据把 SEQUENCE 加在触发下界上，方向与此相反，靠
 * PWM_ADC_TRIGGER_LATEST 取值保守才兜住。此处按最坏情况(trigger 取最晚、
 * max_compare 取最大)做编译期校验，避免下次改 ADC 配置再次静默失效。
 */
#if ((PWM_ADC_TRIGGER_LATEST + PWM_ADC_SEQUENCE_TICKS) > \
     (2 * PWM_PERIOD - PWM_MAX_COMPARE - PWM_ADC_END_MARGIN_TICKS))
#error "ADC injected sequence overruns the low-side conduction window"
#endif

/* 参考值单位：iq 为 A，速度为 rad/s，位置为累计机械角 rad。 */
int BLDC_SetIqReference(float reference);
#if (USE_SPEED_LOOP && !USE_POSITION_LOOP)
int BLDC_SetSpeedReference(float reference);
#endif
#if USE_POSITION_LOOP
int BLDC_SetPositionReference(float reference);
#endif
void BLDC_Stop(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CKTIM 168000000
#define PWM_PRSC 0
#define PWM_FREQ 15000
#define PWM_PERIOD CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))
#define EN_GATE_Pin GPIO_PIN_0
#define EN_GATE_GPIO_Port GPIOC
#define DC_CAL_Pin GPIO_PIN_1
#define DC_CAL_GPIO_Port GPIOC
#define V_CHANNEL_U_Pin GPIO_PIN_0
#define V_CHANNEL_U_GPIO_Port GPIOA
#define V_CHANNEL_V_Pin GPIO_PIN_1
#define V_CHANNEL_V_GPIO_Port GPIOA
#define V_CHANNEL_W_Pin GPIO_PIN_2
#define V_CHANNEL_W_GPIO_Port GPIOA
#define I_CHANNEL_U_Pin GPIO_PIN_3
#define I_CHANNEL_U_GPIO_Port GPIOA
#define I_CHANNEL_V_Pin GPIO_PIN_4
#define I_CHANNEL_V_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
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
