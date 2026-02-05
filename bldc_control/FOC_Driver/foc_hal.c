/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 13:34:45
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 18:25:48
 * @Description:
 */
#include "foc_hal.h"
#include "hal_iic.h"
#include "as5600.h"
#include "main.h"

AS5600_T G_SENSOR_A;
Current_Offset_T g_current_offset = {0};

float Sensor_GetOnceAngleA() { return AS5600_GetOnceAngle(&G_SENSOR_A); }
float Sensor_GetAngleA() { return AS5600_GetAngle(&G_SENSOR_A); }
void Sensor_UpdateA() { AS5600_Update(&G_SENSOR_A); }
float Sensor_GetVelocityA() { return AS5600_GetVelocity(&G_SENSOR_A); }

void FOC_HAL_InitA(FOC_T *hfoc)
{
  AS5600_Init(&G_SENSOR_A);
  FOC_Bind_SensorUpdate(hfoc, Sensor_UpdateA);
  FOC_Bind_SensorGetOnceAngle(hfoc, Sensor_GetOnceAngleA);
  FOC_Bind_SensorGetAngle(hfoc, Sensor_GetAngleA);
  FOC_Bind_SensorGetVelocity(hfoc, Sensor_GetVelocityA);
}

/**
 * @brief  电流零点偏置校准
 * @param  hadc: ADC句柄指针
 * @param  sample_count: 采样次数 (建议100-500)
 * @retval 0: 校准成功, -1: 校准失败
 */
int FOC_Current_Offset_Calibration(ADC_HandleTypeDef *hadc, uint16_t sample_count)
{
  float sum_u = 0.0f;
  float sum_v = 0.0f;
  float sum_w = 0.0f;

  if (sample_count == 0) 
  {
    sample_count = 100;
  }

  HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  for (uint16_t i = 0; i < sample_count; i++) 
  {
    HAL_ADCEx_InjectedStart(hadc);
    HAL_ADCEx_InjectedPollForConversion(hadc, 10);

    sum_u += (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    sum_v += (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    sum_w += (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  g_current_offset.offset_u = sum_u / (float)sample_count;
  g_current_offset.offset_v = sum_v / (float)sample_count;
  g_current_offset.offset_w = sum_w / (float)sample_count;
  g_current_offset.calibrated = 1;

  return 0;
}

/**
 * @brief  获取校准后的电流值
 * @param  raw_u, raw_v, raw_w: ADC原始采样值
 * @param  iu, iv, iw: 输出的电流值指针 (单位: A)
 * @retval 无
 */
void FOC_Get_Calibrated_Current(float raw_u, float raw_v, float raw_w,
                                 float *iu, float *iv, float *iw)
{
  const float factor = ADC_REF_VOLTAGE / ADC_RESOLUTION * VOLTAGE_TO_CURRENT;

  if (g_current_offset.calibrated)
  {
    // 使用校准后的偏置
    *iu = (raw_u - g_current_offset.offset_u) * factor;
    *iv = (raw_v - g_current_offset.offset_v) * factor;
    *iw = (raw_w - g_current_offset.offset_w) * factor;
  } 
  else 
  {
    // 未校准，使用固定偏置 (ADC中点值)
    const float default_offset = ADC_RESOLUTION / 2.0f;
    *iu = (raw_u - default_offset) * factor;
    *iv = (raw_v - default_offset) * factor;
    *iw = (raw_w - default_offset) * factor;
  }
}

