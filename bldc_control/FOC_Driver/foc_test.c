/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 13:27:39
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 16:52:09
 * @Description:
 */
#include "foc_test.h"
#include "foc.h"

#define _PI 3.141592653589793f

// 测试开环速度控制
void Foc_TestOpenloopVelocity(FOC_T *hfoc, float target_velocity)
{
  unsigned long now_ms = HAL_GetTick();
  // 计算每个Loop的运行时间间隔
  float Ts = (now_ms - hfoc->open_loop_timestamp) * 1e-3f;
  hfoc->shaft_angle = _normalizeAngle(hfoc->shaft_angle + target_velocity * Ts);
  float Uq = hfoc->voltage_limit;
  FOC_SetTorque(hfoc, Uq, _openloop_electricalAngle(hfoc->shaft_angle, hfoc->pp));
  hfoc->open_loop_timestamp = now_ms;
}

// 测试闭环速度控制
void Foc_TestCloseloopVelocity(FOC_T *hfoc, LOWPASS_FILTER_T *filter,
                               PID_T *pid, float target_velocity)
{
  float SensorVel = hfoc->Sensor_GetVelocity();
  SensorVel = LOWPASS_FILTER_Calc(filter, hfoc->dir * SensorVel);
  FOC_SetTorque(hfoc, PID_Calc(pid, (target_velocity - SensorVel) * 180 / _PI), FOC_CloseloopElectricalAngle(hfoc));
}

// 测试闭环位置控制和力矩控制 (旧版单环，保留兼容)
void Foc_TestCloseloopAngle(FOC_T *hfoc, PID_T *pid, float angle)
{
  float SensorAngle = hfoc->Sensor_GetAngle();
  FOC_SetTorque(hfoc,
    PID_Calc(pid, (angle - hfoc->dir * SensorAngle) * 180.0f / _PI),
    FOC_CloseloopElectricalAngle(hfoc));
}

// 位置环控制 (三环串级架构: 位置环 -> 速度环 -> 电流环)
// 返回 iq_ref 供电流环使用
float Foc_PositionLoop(FOC_T *hfoc, PID_T *anglePID, LOWPASS_FILTER_T *velFilter,
                       PID_T *velPID, float target_angle)
{
  // 获取当前位置 (累计角度，单位: rad)
  float current_angle = hfoc->dir * hfoc->Sensor_GetAngle();

  // 位置环: 位置误差 -> 目标速度 (rad/s)
  float position_error = target_angle - current_angle;
  float target_velocity = PID_Calc(anglePID, position_error);

  // 速度环: 速度误差 -> iq_ref (A)
  float current_velocity = hfoc->dir * hfoc->Sensor_GetVelocity();
  current_velocity = LOWPASS_FILTER_Calc(velFilter, current_velocity);
  float velocity_error = target_velocity - current_velocity;

  return PID_Calc(velPID, velocity_error);
}

float Foc_VelocityLoop(FOC_T *hfoc, LOWPASS_FILTER_T *filter,
                       PID_T *pid, float target_velocity)
{
  float SensorVel = hfoc->Sensor_GetVelocity();
  SensorVel = LOWPASS_FILTER_Calc(filter, hfoc->dir * SensorVel);
  return PID_Calc(pid, (target_velocity - SensorVel) * 180.0f / 3.14159f);
}
