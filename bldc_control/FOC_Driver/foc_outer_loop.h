/*
 * @Description: 外环（位置环 / 速度环）。仅负责由参考量和观测反馈算出下一级
 *               的参考量，不直接操作 PWM，也不承担调度职责。
 *               调度顺序见方案 22.7：位置环 -> 速度环 -> 电流环。
 */
#ifndef __FOC_OUTER_LOOP_H__
#define __FOC_OUTER_LOOP_H__

#include "pid.h"
#include "foc.h"

#if USE_POSITION_LOOP
/*
 * 位置环：位置误差 -> speed_ref (rad/s)。
 * position_feedback 必须与 target_angle 同坐标系（均为 dir 修正后的累计机械角）。
 * anglePID 的输出限幅必须等于 speed_ref 的限幅。
 */
float Foc_PositionLoop(PID_T *anglePID, float target_angle,
                       float position_feedback);
#endif

#if USE_SPEED_LOOP
/*
 * 速度环：速度误差 -> iq_ref (A)。
 * velocity_feedback 为 dir 修正后的观测机械角速度，直接取自跟踪观测器，
 * 不再串低通滤波器——观测器本身已完成滤波，再叠加只会额外引入相位滞后。
 * velPID 的输出限幅必须等于 FOC_IQ_REF_LIMIT。
 */
float Foc_VelocityLoop(PID_T *velPID, float target_velocity,
                       float velocity_feedback);
#endif

#endif
