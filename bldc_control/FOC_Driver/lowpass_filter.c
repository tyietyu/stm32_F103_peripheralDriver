/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:57:48
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 14:39:32
 * @Description:
 */
#include "lowpass_filter.h"
#include "stm32f4xx_hal.h"

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *f, float time_const)
{
  f->tf = time_const;
  f->prev_y = 0.0f;
  f->prev_timestamp = HAL_GetTick();
}

float LOWPASS_FILTER_CalcWithDt(LOWPASS_FILTER_T *f, float x, float dt_s)
{
  float alpha;
  float y;

  if ((f == NULL) || (dt_s <= 0.0f))
  {
    return x;
  }

  alpha = f->tf / (f->tf + dt_s);
  y = alpha * f->prev_y + (1.0f - alpha) * x;
  f->prev_y = y;
  f->prev_timestamp = HAL_GetTick();
  return y;
}

float LOWPASS_FILTER_Calc(LOWPASS_FILTER_T *f, float x)
{
  unsigned long timestamp = HAL_GetTick();
  float delta = (timestamp - f->prev_timestamp) * 1e-3f;
  if (delta < 0.0f)
  {
    delta = 1e-3f;
  }
  else if (delta > 0.3f)
  {
    f->prev_y = x;
    f->prev_timestamp = timestamp;
    return x;
  }

  return LOWPASS_FILTER_CalcWithDt(f, x, delta);
}


