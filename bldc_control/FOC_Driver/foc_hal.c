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
  0.0f, 0.0f,
  1.0f, 1.0f,
  0.0f, 0.0f,
  FOC_CURRENT_CALIBRATION_VERSION, 0U, 0U
};

static float Sensor_GetOnceAngleA(void) { return AS5600_GetOnceAngle(&G_SENSOR_A); }
static float Sensor_GetAngleA(void) { return AS5600_GetAngle(&G_SENSOR_A); }
static int Sensor_UpdateA(void) { return AS5600_Update(&G_SENSOR_A); }
static float Sensor_GetVelocityA(void) { return AS5600_GetVelocity(&G_SENSOR_A); }

FOC_Result_t FOC_HAL_InitA(FOC_T *hfoc)
{
  if (hfoc == NULL)
  {
    return FOC_ERROR_INVALID_ARGUMENT;
  }
  if (AS5600_Init(&G_SENSOR_A) != 0)
  {
    return FOC_ERROR_SENSOR_INIT_FAILED;
  }

  FOC_Bind_SensorUpdate(hfoc, Sensor_UpdateA);
  FOC_Bind_SensorGetOnceAngle(hfoc, Sensor_GetOnceAngleA);
  FOC_Bind_SensorGetAngle(hfoc, Sensor_GetAngleA);
  FOC_Bind_SensorGetVelocity(hfoc, Sensor_GetVelocityA);

  hfoc->sensor_valid = 1U;
  hfoc->sensor_last_success_tick = AS5600_GetLastSuccessTick(&G_SENSOR_A);
  /* 观测器以实测角度起步，避免上电初期 theta_hat 从 0 收敛的暂态 */
  FOC_ObserverReset(hfoc, AS5600_GetOnceAngle(&G_SENSOR_A),
                    AS5600_GetLastSuccessTick(&G_SENSOR_A));
  return FOC_RESULT_OK;
}

FOC_Result_t FOC_HAL_UpdateSensor(FOC_T *hfoc)
{
  if (AS5600_Update(&G_SENSOR_A) != 0)
  {
    /*
     * 单次读失败不清 sensor_valid：观测器按 omega_hat 继续外推，由调用方按
     * now - sensor_last_success_tick > sensor_max_age_ms 决定是否关断。
     */
    return FOC_ERROR_SENSOR_UPDATE_FAILED;
  }

  FOC_UpdateCachedSensorAngle(hfoc, AS5600_GetOnceAngle(&G_SENSOR_A),
                              AS5600_GetLastSuccessTick(&G_SENSOR_A));
  return FOC_RESULT_OK;
}

/**
 * @brief  电流零点偏置校准
 * @param  hadc: ADC句柄指针
 * @param  sample_count: 采样次数 (建议100-500)
 * @retval 0: 校准成功, -1: 校准失败
 */
FOC_Result_t FOC_Current_Offset_Calibration(ADC_HandleTypeDef *hadc,
                                            uint16_t sample_count)
{
  float max_u = 0.0f;
  float max_v = 0.0f;
  float min_u = ADC_RESOLUTION;
  float min_v = ADC_RESOLUTION;
  float raw_u;
  float raw_v;
  float sum_u = 0.0f;
  float sum_v = 0.0f;
  uint16_t i;

  if ((hadc == NULL) || (hadc->Instance != ADC1))
  {
    return FOC_ERROR_ADC_CONFIGURATION;
  }

  if ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U)
  {
    return FOC_ERROR_PWM_OUTPUT_ACTIVE;
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
      return FOC_ERROR_ADC_START_FAILED;
    }

    if (HAL_ADCEx_InjectedPollForConversion(hadc, 10) != HAL_OK)
    {
      (void)HAL_ADCEx_InjectedStop(hadc);
      HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
      return FOC_ERROR_ADC_CONVERSION_FAILED;
    }

    /* Rank 3 (PA5/IC_FB) 为外置 LM2904 通道，不参与两相采样，故不采集 */
    raw_u = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    raw_v = (float)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    sum_u += raw_u;
    sum_v += raw_v;
    min_u = (raw_u < min_u) ? raw_u : min_u;
    min_v = (raw_v < min_v) ? raw_v : min_v;
    max_u = (raw_u > max_u) ? raw_u : max_u;
    max_v = (raw_v > max_v) ? raw_v : max_v;

    HAL_Delay(1);
  }

  (void)HAL_ADCEx_InjectedStop(hadc);
  HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  g_current_offset.max_span_u = max_u - min_u;
  g_current_offset.max_span_v = max_v - min_v;
  if ((g_current_offset.max_span_u > FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS) ||
      (g_current_offset.max_span_v > FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS) ||
      (min_u <= FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS) ||
      (min_v <= FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS) ||
      (max_u >= (ADC_RESOLUTION - FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS)) ||
      (max_v >= (ADC_RESOLUTION - FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS)))
  {
    return FOC_ERROR_CURRENT_OFFSET_INVALID;
  }

  g_current_offset.offset_u = sum_u / (float)sample_count;
  g_current_offset.offset_v = sum_v / (float)sample_count;
  g_current_offset.calibrated = 1U;

  return FOC_RESULT_OK;
}

FOC_Result_t FOC_Set_Current_Gain_Calibration(float gain_u, float gain_v,
                                               uint16_t version)
{
  if ((gain_u <= 0.0f) || (gain_v <= 0.0f) ||
      (version != FOC_CURRENT_CALIBRATION_VERSION))
  {
    return FOC_ERROR_CURRENT_GAIN_INVALID;
  }

  g_current_offset.gain_u = gain_u;
  g_current_offset.gain_v = gain_v;
  g_current_offset.version = version;
  g_current_offset.gain_calibrated = 1U;
  return FOC_RESULT_OK;
}

FOC_Result_t FOC_ValidateRawCurrentSamples(float raw_u, float raw_v)
{
  if ((raw_u < FOC_CURRENT_RAW_MIN_COUNTS) ||
      (raw_u > FOC_CURRENT_RAW_MAX_COUNTS) ||
      (raw_v < FOC_CURRENT_RAW_MIN_COUNTS) ||
      (raw_v > FOC_CURRENT_RAW_MAX_COUNTS))
  {
    return FOC_ERROR_CURRENT_RAW_SATURATED;
  }

  return FOC_RESULT_OK;
}

/**
 * @brief  获取校准后的电流值
 * @note   仅在启动校准成功后调用
 * @param  raw_u, raw_v: U/V 相 ADC 原始采样值
 * @param  iu, iv, iw: 输出的电流值指针 (单位: A)，iw 由 -(iu+iv) 推算
 */
void FOC_Get_Calibrated_Current(float raw_u, float raw_v,
                                float *iu, float *iv, float *iw)
{
  const float factor = ADC_REF_VOLTAGE / ADC_RESOLUTION * VOLTAGE_TO_CURRENT;

  *iu = (raw_u - g_current_offset.offset_u) * factor *
        g_current_offset.gain_u * FOC_CURRENT_SIGN_U;
  *iv = (raw_v - g_current_offset.offset_v) * factor *
        g_current_offset.gain_v * FOC_CURRENT_SIGN_V;
  /* 三相星型无中线，由基尔霍夫电流定律推算 W 相 */
  *iw = -(*iu + *iv);
}

FOC_Result_t FOC_ValidatePhaseCurrents(float iu, float iv, float iw,
                                       float current_limit)
{
  /* iw 为推算值，三相和恒为 0，故不再做和值一致性校验 */
  if ((fabsf(iu) > current_limit) ||
      (fabsf(iv) > current_limit) ||
      (fabsf(iw) > current_limit))
  {
    return FOC_CURRENT_ERROR_LIMIT;
  }

  return FOC_CURRENT_VALID;
}

