/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:57:48
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 14:39:32
 * @Description:
 */
#include "lowpass_filter.h"

#include <stddef.h>

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *f, float time_const)
{
  f->tf = time_const;
  f->prev_y = 0.0f;
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
  return y;
}
