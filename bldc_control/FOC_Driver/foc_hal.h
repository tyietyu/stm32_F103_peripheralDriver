/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 13:34:37
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 18:26:02
 * @Description:
 */
#ifndef __FOC_HAL_H__
#define __FOC_HAL_H__

#include "foc.h"
#include "adc.h"

/* 电流偏置校准 */
#define FOC_CURRENT_CALIBRATION_VERSION       1U
#define FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS    64.0f
#define FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS 16.0f

typedef struct {
  float offset_u;   // U相电流偏置 (ADC原始值)
  float offset_v;   // V相电流偏置 (ADC原始值)
  float offset_w;   // W相电流偏置 (ADC原始值)
  float gain_u;     // U相生产校准增益
  float gain_v;     // V相生产校准增益
  float gain_w;     // W相生产校准增益
  float max_span_u; // U相零点校准最大跨度 (ADC原始值)
  float max_span_v; // V相零点校准最大跨度 (ADC原始值)
  float max_span_w; // W相零点校准最大跨度 (ADC原始值)
  uint16_t version; // 校准数据版本
  uint8_t calibrated;  // 校准完成标志
  uint8_t gain_calibrated; // 生产增益校准有效标志
} Current_Offset_T;
extern Current_Offset_T g_current_offset;

int FOC_HAL_InitA(FOC_T *hfoc);

/**
 * @brief  电流零点偏置校准
 * @param  hadc: ADC句柄指针
 * @param  sample_count: 采样次数 (建议100-500)
 * @retval 0: 校准成功, -1: 校准失败
 */
int FOC_Current_Offset_Calibration(ADC_HandleTypeDef *hadc, uint16_t sample_count);
int FOC_Set_Current_Gain_Calibration(float gain_u, float gain_v, float gain_w,
                                     uint16_t version);

/**
 * @brief  获取校准后的电流值
 * @param  raw_u, raw_v, raw_w: ADC原始采样值
 * @param  iu, iv, iw: 输出的电流值指针 (单位: A)
 * @retval 0: 转换成功, -1: 参数或校准状态无效
 */
int FOC_Get_Calibrated_Current(float raw_u, float raw_v, float raw_w,
                               float *iu, float *iv, float *iw);

#endif
