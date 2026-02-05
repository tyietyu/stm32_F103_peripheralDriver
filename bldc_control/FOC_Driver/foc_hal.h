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
typedef struct {
  float offset_u;   // U相电流偏置 (ADC原始值)
  float offset_v;   // V相电流偏置 (ADC原始值)
  float offset_w;   // W相电流偏置 (ADC原始值)
  uint8_t calibrated;  // 校准完成标志
} Current_Offset_T;
extern Current_Offset_T g_current_offset;

void FOC_HAL_InitA(FOC_T *hfoc);

/**
 * @brief  电流零点偏置校准
 * @param  hadc: ADC句柄指针
 * @param  sample_count: 采样次数 (建议100-500)
 * @retval 0: 校准成功, -1: 校准失败
 */
int FOC_Current_Offset_Calibration(ADC_HandleTypeDef *hadc, uint16_t sample_count);

/**
 * @brief  获取校准后的电流值
 * @param  raw_u, raw_v, raw_w: ADC原始采样值
 * @param  iu, iv, iw: 输出的电流值指针 (单位: A)
 * @retval 无
 */
void FOC_Get_Calibrated_Current(float raw_u, float raw_v, float raw_w,
                                 float *iu, float *iv, float *iw);

#endif
