/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:58:39
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 12:48:51
 * @Description:
 */
#include "pid.h"
#include "stm32f4xx_hal.h"

#define _constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void PID_Init(PID_T *pid, float P, float I, float D, float ramp, float limit)
{
  pid->p = P;
  pid->i = I;
  pid->d = D;
  pid->output_ramp = ramp;
  pid->limit = limit;
  pid->fixed_dt = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_output = 0.0f;
  pid->prev_integral = 0.0f;
  pid->prev_timestamp = HAL_GetTick(); // 获取毫秒时间
}

void PID_SetFixedDt(PID_T *pid, float dt_s)
{
  pid->fixed_dt = (dt_s > 0.0f) ? dt_s : 0.0f;
}

void PID_Reset(PID_T *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->prev_error = 0.0f;
  pid->prev_output = 0.0f;
  pid->prev_integral = 0.0f;
  pid->prev_timestamp = HAL_GetTick();
}

float PID_Calc(PID_T *pid, float error)
{
  unsigned long timestamp_now;
  float derivative;
  float integral;
  float output;
  float proportional;
  float ts;
  float unsaturated_output;

  if (pid == NULL)
  {
    return 0.0f;
  }

  timestamp_now = HAL_GetTick();
  if (pid->fixed_dt > 0.0f)
  {
    ts = pid->fixed_dt;
  }
  else
  {
    ts = (timestamp_now - pid->prev_timestamp) * 1e-3f;
    if ((ts <= 0.0f) || (ts > 0.5f))
    {
      ts = 1e-3f;
    }
  }

  proportional = pid->p * error;
  integral = pid->prev_integral +
             pid->i * ts * 0.5f * (error + pid->prev_error);
  integral = _constrain(integral, -pid->limit, pid->limit);
  derivative = pid->d * (error - pid->prev_error) / ts;

  unsaturated_output = proportional + integral + derivative;
  if (((unsaturated_output > pid->limit) && (error > 0.0f)) ||
      ((unsaturated_output < -pid->limit) && (error < 0.0f)))
  {
    integral = pid->prev_integral;
    unsaturated_output = proportional + integral + derivative;
  }
  output = _constrain(unsaturated_output, -pid->limit, pid->limit);

  if (pid->output_ramp > 0.0f)
  {
    float output_rate = (output - pid->prev_output) / ts;

    if (output_rate > pid->output_ramp)
    {
      output = pid->prev_output + pid->output_ramp * ts;
    }
    else if (output_rate < -pid->output_ramp)
    {
      output = pid->prev_output - pid->output_ramp * ts;
    }
  }

  pid->prev_integral = integral;
  pid->prev_output = output;
  pid->prev_error = error;
  pid->prev_timestamp = timestamp_now;
  return output;
}

void PID_Set(PID_T *pid, float P, float I, float D, float ramp)
{
  pid->p = P;
  pid->i = I;
  pid->d = D;
  pid->output_ramp = ramp;
}

