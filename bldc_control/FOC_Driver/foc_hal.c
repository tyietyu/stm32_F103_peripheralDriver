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

#include <math.h>

AS5600_T G_SENSOR_A;
Current_Offset_T g_current_offset = {
  0.0f, 0.0f, 0.0f,
  1.0f, 1.0f, 1.0f,
  0.0f, 0.0f, 0.0f,
  FOC_CURRENT_CALIBRATION_VERSION, 0U, 0U
};

static float Sensor_GetOnceAngleA(void) { return AS5600_GetOnceAngle(&G_SENSOR_A); }
static float Sensor_GetAngleA(void) { return AS5600_GetAngle(&G_SENSOR_A); }
static int Sensor_UpdateA(void) { return AS5600_Update(&G_SENSOR_A); }
static float Sensor_GetVelocityA(void) { return AS5600_GetVelocity(&G_SENSOR_A); }

int FOC_HAL_InitA(FOC_T *hfoc)
{
  if ((hfoc == NULL) || (AS5600_Init(&G_SENSOR_A) != 0))
  {
    return -1;
  }

  FOC_Bind_SensorUpdate(hfoc, Sensor_UpdateA);
  FOC_Bind_SensorGetOnceAngle(hfoc, Sensor_GetOnceAngleA);
  FOC_Bind_SensorGetAngle(hfoc, Sensor_GetAngleA);
  FOC_Bind_SensorGetVelocity(hfoc, Sensor_GetVelocityA);

  hfoc->sensor_valid = 1U;
  hfoc->sensor_last_success_tick = AS5600_GetLastSuccessTick(&G_SENSOR_A);
  hfoc->cached_angle_el = FOC_SensorAngleToElectricalAngle(hfoc,
                                                           AS5600_GetOnceAngle(&G_SENSOR_A));
  return 0;
}

int FOC_HAL_UpdateSensor(FOC_T *hfoc, uint32_t max_age_ms)
{
  uint32_t now;

  if ((hfoc == NULL) || (AS5600_Update(&G_SENSOR_A) != 0))
  {
    if (hfoc != NULL)
    {
      hfoc->sensor_valid = 0U;
    }
    return -1;
  }

  now = HAL_GetTick();
  if (!AS5600_IsFresh(&G_SENSOR_A, now, max_age_ms))
  {
    hfoc->sensor_valid = 0U;
    return -1;
  }

  FOC_UpdateCachedSensorAngle(hfoc, AS5600_GetOnceAngle(&G_SENSOR_A),
                              AS5600_GetLastSuccessTick(&G_SENSOR_A));
  return 0;
}

uint32_t FOC_HAL_GetSensorLastSuccessTick(void)
{
  return AS5600_GetLastSuccessTick(&G_SENSOR_A);
}

uint16_t FOC_HAL_GetSensorErrorCount(void)
{
  return AS5600_GetErrorCount(&G_SENSOR_A);
}

/**
 * @brief  电流零点偏置校准
 * @param  hadc: ADC句柄指针
 * @param  sample_count: 采样次数 (建议100-500)
 * @retval 0: 校准成功, -1: 校准失败
 */
int FOC_Current_Offset_Calibration(ADC_HandleTypeDef *hadc, uint16_t sample_count)
{
  float max_u = 0.0f;
  float max_v = 0.0f;
  float max_w = 0.0f;
  float min_u = ADC_RESOLUTION;
  float min_v = ADC_RESOLUTION;
  float min_w = ADC_RESOLUTION;
  float raw_u;
  float raw_v;
  float raw_w;
  float sum_u = 0.0f;
  float sum_v = 0.0f;
  float sum_w = 0.0f;
  uint16_t i;

  if ((hadc == NULL) || (hadc->Instance != ADC1) ||
      (htim1.Instance != TIM1) ||
      ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U) ||
      ((htim1.Instance->CCER & TIM_CCER_CC4E) == 0U) ||
      ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U))
  {
    return -1;
  }

  if (sample_count == 0U)
  {
    sample_count = 100;
  }
  g_current_offset.calibrated = 0U;

  HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  for (i = 0U; i < sample_count; ++i)
  {
    if (HAL_ADCEx_InjectedStart(hadc) != HAL_OK)
    {
      (void)HAL_ADCEx_InjectedStop(hadc);
      HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
      return -1;
    }

    if (HAL_ADCEx_InjectedPollForConversion(hadc, 10) != HAL_OK)
    {
      (void)HAL_ADCEx_InjectedStop(hadc);
      HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
      return -1;
    }

    raw_u = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    raw_v = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    raw_w = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    sum_u += raw_u;
    sum_v += raw_v;
    sum_w += raw_w;
    min_u = (raw_u < min_u) ? raw_u : min_u;
    min_v = (raw_v < min_v) ? raw_v : min_v;
    min_w = (raw_w < min_w) ? raw_w : min_w;
    max_u = (raw_u > max_u) ? raw_u : max_u;
    max_v = (raw_v > max_v) ? raw_v : max_v;
    max_w = (raw_w > max_w) ? raw_w : max_w;

    HAL_Delay(1);
  }

  (void)HAL_ADCEx_InjectedStop(hadc);
  HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  g_current_offset.max_span_u = max_u - min_u;
  g_current_offset.max_span_v = max_v - min_v;
  g_current_offset.max_span_w = max_w - min_w;
  if ((g_current_offset.max_span_u > FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS) ||
      (g_current_offset.max_span_v > FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS) ||
      (g_current_offset.max_span_w > FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS) ||
      (min_u <= FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS) ||
      (min_v <= FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS) ||
      (min_w <= FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS) ||
      (max_u >= (ADC_RESOLUTION - FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS)) ||
      (max_v >= (ADC_RESOLUTION - FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS)) ||
      (max_w >= (ADC_RESOLUTION - FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS)))
  {
    return -1;
  }

  g_current_offset.offset_u = sum_u / (float)sample_count;
  g_current_offset.offset_v = sum_v / (float)sample_count;
  g_current_offset.offset_w = sum_w / (float)sample_count;
  g_current_offset.calibrated = 1U;

  return 0;
}

int FOC_Set_Current_Gain_Calibration(float gain_u, float gain_v, float gain_w,
                                     uint16_t version)
{
  if ((gain_u <= 0.0f) || (gain_v <= 0.0f) || (gain_w <= 0.0f) ||
      (version != FOC_CURRENT_CALIBRATION_VERSION))
  {
    return -1;
  }

  g_current_offset.gain_u = gain_u;
  g_current_offset.gain_v = gain_v;
  g_current_offset.gain_w = gain_w;
  g_current_offset.version = version;
  g_current_offset.gain_calibrated = 1U;
  return 0;
}

/**
 * @brief  获取校准后的电流值
 * @param  raw_u, raw_v, raw_w: ADC原始采样值
 * @param  iu, iv, iw: 输出的电流值指针 (单位: A)
 * @retval 0: 转换成功, -1: 参数或校准状态无效
 */
int FOC_Get_Calibrated_Current(float raw_u, float raw_v, float raw_w,
                               float *iu, float *iv, float *iw)
{
  const float factor = ADC_REF_VOLTAGE / ADC_RESOLUTION * VOLTAGE_TO_CURRENT;

  if ((iu == NULL) || (iv == NULL) || (iw == NULL) ||
      (g_current_offset.calibrated == 0U) ||
      (g_current_offset.version != FOC_CURRENT_CALIBRATION_VERSION))
  {
    return -1;
  }

  *iu = (raw_u - g_current_offset.offset_u) * factor *
        g_current_offset.gain_u * FOC_CURRENT_SIGN_U;
  *iv = (raw_v - g_current_offset.offset_v) * factor *
        g_current_offset.gain_v * FOC_CURRENT_SIGN_V;
  *iw = (raw_w - g_current_offset.offset_w) * factor *
        g_current_offset.gain_w * FOC_CURRENT_SIGN_W;
  return 0;
}

int FOC_ValidatePhaseCurrents(float iu, float iv, float iw, float current_limit)
{
  if ((current_limit <= 0.0f) ||
      (fabsf(iu) > current_limit) ||
      (fabsf(iv) > current_limit) ||
      (fabsf(iw) > current_limit))
  {
    return FOC_CURRENT_ERROR_LIMIT;
  }

  if (fabsf(iu + iv + iw) > FOC_CURRENT_SUM_TOLERANCE_A)
  {
    return FOC_CURRENT_ERROR_SUM;
  }

  return FOC_CURRENT_VALID;
}

