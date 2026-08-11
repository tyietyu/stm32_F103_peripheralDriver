/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:58:34
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 12:48:37
 * @Description:
 */
#ifndef __PID_H__
#define __PID_H__

typedef struct {
  float p;  // 比例增益
  float i;  // 积分增益
  float d;  // 微分增益
  float output_ramp;
  float limit;                   // 积分限幅；external_output_limit==0 时同时作为输出限幅
  float fixed_dt;                // >0 时使用固定积分步长(秒)，0 时按 HAL_GetTick 实时计算
  unsigned char external_output_limit;  // !=0 时输出限幅由外部统一执行(如矢量圆限幅)

  float prev_error;              // 最后跟踪误差
  float prev_output;             // 最后一个pid输出值
  float prev_integral;           // 最后一个积分分量
  float integral_before;         // 本拍累加前的积分值，供外部削幅回滚
  float last_integral_delta;     // 本拍积分增量，供外部削幅判方向
  unsigned long prev_timestamp;  // 上次执行的时间戳
} PID_T;

void PID_Init(PID_T *pid, float P, float I, float D, float ramp, float limit);
void PID_Set(PID_T *pid, float P, float I, float D, float ramp);
void PID_SetFixedDt(PID_T *pid, float dt_s);
void PID_SetExternalOutputLimit(PID_T *pid, unsigned char enable);
void PID_Reset(PID_T *pid);
float PID_Calc(PID_T *pid, float error);
/*
 * 外部限幅回传。clipped 只取符号：与本拍积分增量同向时冻结积分到累加前的值；
 * applied_output 为实际生效的输出，回写 prev_output 以保证限速判据一致。
 */
void PID_ApplyExternalClip(PID_T *pid, float clipped, float applied_output);

#endif

