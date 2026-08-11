/*
 * @Description: 外环（位置环 / 速度环）实现，接口约定见 foc_outer_loop.h
 */
#include "foc_outer_loop.h"

#include <stddef.h>

#if USE_POSITION_LOOP
float Foc_PositionLoop(PID_T *anglePID, float target_angle,
                       float position_feedback)
{
  if (anglePID == NULL)
  {
    return 0.0f;
  }

  return PID_Calc(anglePID, target_angle - position_feedback);
}
#endif

#if USE_SPEED_LOOP
float Foc_VelocityLoop(PID_T *velPID, float target_velocity,
                       float velocity_feedback)
{
  if (velPID == NULL)
  {
    return 0.0f;
  }

  return PID_Calc(velPID, target_velocity - velocity_feedback);
}
#endif
