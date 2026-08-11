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
  pid->external_output_limit = 0U;
  pid->prev_error = 0.0f;
  pid->prev_output = 0.0f;
  pid->prev_integral = 0.0f;
  pid->integral_before = 0.0f;
  pid->last_integral_delta = 0.0f;
  pid->prev_timestamp = HAL_GetTick(); // 获取毫秒时间
}

void PID_SetFixedDt(PID_T *pid, float dt_s)
{
  pid->fixed_dt = (dt_s > 0.0f) ? dt_s : 0.0f;
}

void PID_SetExternalOutputLimit(PID_T *pid, unsigned char enable)
{
  if (pid != NULL)
  {
    pid->external_output_limit = (enable != 0U) ? 1U : 0U;
  }
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
  pid->integral_before = 0.0f;
  pid->last_integral_delta = 0.0f;
  pid->prev_timestamp = HAL_GetTick();
}

float PID_Calc(PID_T *pid, float error)
{
  unsigned long timestamp_now;
  float clipped;
  float derivative;
  float integral;
  float integral_delta;
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
  integral_delta = pid->i * ts * 0.5f * (error + pid->prev_error);
  integral = _constrain(pid->prev_integral + integral_delta,
                        -pid->limit, pid->limit);
  derivative = pid->d * (error - pid->prev_error) / ts;

  unsaturated_output = proportional + integral + derivative;
  /*
   * external_output_limit 时不做分轴方形限幅：限幅统一在外部矢量圆限幅处发生，
   * 削掉的量由 PID_ApplyExternalClip() 回传。pid->limit 仍然约束积分器本身。
   */
  output = (pid->external_output_limit != 0U)
             ? unsaturated_output
             : _constrain(unsaturated_output, -pid->limit, pid->limit);

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

  /*
   * 抗积分饱和：clipped 为限幅+限速合计削掉的输出量。仅当积分增量还在朝被削掉的
   * 同方向推时才冻结积分，反向增量始终放行，保证退出饱和时能立刻回落。
   *
   * 用冻结而非回算(integral = output - P - D)：本项目电流环 P=4、limit=4，误差
   * 1A 时 P 项单独即吃满限幅，回算会把积分打到 0 甚至反向，误差回落后输出符号
   * 会翻转。冻结无此风险。
   *
   * 必须覆盖 output_ramp 路径：15kHz 下 ramp 每拍只放行极小的输出增量，若只按
   * limit 判饱和，被 ramp 削掉的部分会持续累积进积分器。
   */
  clipped = unsaturated_output - output;
  if (((clipped > 0.0f) && (integral_delta > 0.0f)) ||
      ((clipped < 0.0f) && (integral_delta < 0.0f)))
  {
    integral = pid->prev_integral;
  }

  pid->integral_before = pid->prev_integral;
  pid->last_integral_delta = integral_delta;
  pid->prev_integral = integral;
  pid->prev_output = output;
  pid->prev_error = error;
  pid->prev_timestamp = timestamp_now;
  return output;
}

void PID_ApplyExternalClip(PID_T *pid, float clipped, float applied_output)
{
  if (pid == NULL)
  {
    return;
  }

  if (((clipped > 0.0f) && (pid->last_integral_delta > 0.0f)) ||
      ((clipped < 0.0f) && (pid->last_integral_delta < 0.0f)))
  {
    pid->prev_integral = pid->integral_before;
  }
  pid->prev_output = applied_output;
}

void PID_Set(PID_T *pid, float P, float I, float D, float ramp)
{
  pid->p = P;
  pid->i = I;
  pid->d = D;
  pid->output_ramp = ramp;
}

