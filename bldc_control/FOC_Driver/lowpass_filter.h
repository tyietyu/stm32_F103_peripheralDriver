/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:57:43
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 12:58:46
 * @Description:
 */
#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__

typedef struct {
  float tf;
  float prev_y;
} LOWPASS_FILTER_T;

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *f, float time_const);
/*
 * 只保留显式传 dt 的接口。原 LOWPASS_FILTER_Calc() 按 HAL_GetTick 差值取 dt，
 * 在 1 ms tick 分辨率下 delta 可能为 0，会返回原值且不更新状态，滤波被静默旁路。
 */
float LOWPASS_FILTER_CalcWithDt(LOWPASS_FILTER_T *f, float x, float dt_s);

#endif
