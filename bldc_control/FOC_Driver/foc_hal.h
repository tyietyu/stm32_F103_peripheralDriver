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
#include "as5600.h"

/*
 * 原始码值合理区间。放大器饱和、分流虚焊、ADC 通道错配都会把读数顶到轨上，
 * 这类失效不会触发绝对值上限判定（折算电流反而可能落在限值内），必须在换算成
 * 电流之前用原始码值直接拦下。
 */
/*
 * 极性已由实测反推确认为正：占空比 [U 49.7% V 56.0% W 30.6%] 对应的相电压
 * [+1.02 +2.53 -3.56]V，与 sign=+1 换算出的电流 [+0.258 +0.798 -1.056]A 三相
 * 逐个同向，等效相阻 3.2~4.0Ω 一致。即"ADC 读数高于 offset"对应相电流为正。
 */
#define FOC_CURRENT_VALID        FOC_RESULT_OK
#define FOC_CURRENT_ERROR_LIMIT   FOC_ERROR_CURRENT_LIMIT

/*
 * 采用 U/V 两相采样：W 相板载 LM2904 (3.3V 单电源) 静态工作点 1.65V 已逼近其
 * 输出摆幅上限 (V+ - 1.5V ≈ 1.8V)，且不受 DRV8301 的 DC_CAL 短接控制，实测零点
 * 跨度约为 U/V 的 4 倍 (折算 0.8A 峰峰)。W 相改由基尔霍夫电流定律推算。
 */
typedef struct {
  float offset_u;   // U相电流偏置 (ADC原始值)
  float offset_v;   // V相电流偏置 (ADC原始值)
  float gain_u;     // U相生产校准增益
  float gain_v;     // V相生产校准增益
  float max_span_u; // U相零点校准最大跨度 (ADC原始值)
  float max_span_v; // V相零点校准最大跨度 (ADC原始值)
  uint16_t version; // 校准数据版本
  uint8_t calibrated;  // 校准完成标志
  uint8_t gain_calibrated; // 生产增益校准有效标志
} Current_Offset_T;
extern Current_Offset_T g_current_offset;

extern AS5600_T G_SENSOR_A;

FOC_Result_t FOC_HAL_InitA(FOC_T *hfoc);
FOC_Result_t FOC_HAL_UpdateSensor(FOC_T *hfoc);

/**
 * @brief  电流零点偏置校准
 * @param  hadc: ADC句柄指针
 * @param  sample_count: 采样次数 (建议100-500)
 * @retval FOC_RESULT_OK: 校准成功
 * @retval FOC_ERROR_*: 具体失败原因
 */
FOC_Result_t FOC_Current_Offset_Calibration(ADC_HandleTypeDef *hadc,
                                            uint16_t sample_count);
FOC_Result_t FOC_Set_Current_Gain_Calibration(float gain_u, float gain_v,
                                               uint16_t version);

/**
 * @brief  原始 ADC 码值饱和检查
 * @note   必须在 FOC_Get_Calibrated_Current() 之前调用
 * @retval FOC_RESULT_OK 或 FOC_ERROR_CURRENT_RAW_SATURATED
 */
FOC_Result_t FOC_ValidateRawCurrentSamples(float raw_u, float raw_v);

/**
 * @brief  获取校准后的电流值
 * @note   仅在启动校准成功后调用
 * @param  raw_u, raw_v: U/V 相 ADC 原始采样值
 * @param  iu, iv, iw: 输出的电流值指针 (单位: A)，iw 由 -(iu+iv) 推算
 */
void FOC_Get_Calibrated_Current(float raw_u, float raw_v,
                                float *iu, float *iv, float *iw);
FOC_Result_t FOC_ValidatePhaseCurrents(float iu, float iv, float iw,
                                       float current_limit);

#endif
