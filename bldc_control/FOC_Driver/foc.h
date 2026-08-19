#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

#include "tim.h"
#include "foc_config.h"

typedef int (*FUNC_SENSOR_UPDATE)(void);
typedef float (*FUNC_SENSOR_GET_ONCE_ANGLE)(void);
typedef float (*FUNC_SENSOR_GET_ANGLE)(void);
typedef float (*FUNC_SENSOR_GET_VELOCITY)(void);

typedef enum
{
  FOC_RESULT_OK = 0,
  FOC_ERROR_CURRENT_LIMIT = -1,

  FOC_ERROR_INVALID_ARGUMENT = -10,
  FOC_ERROR_SENSOR_NOT_BOUND = -11,
  FOC_ERROR_SENSOR_INIT_FAILED = -12,
  FOC_ERROR_SENSOR_UPDATE_FAILED = -13,
  FOC_ERROR_SENSOR_STALE = -14,

  FOC_ERROR_PWM_OUTPUT_DISABLED = -20,
  FOC_ERROR_ALIGNMENT_TIMEOUT = -21,
  FOC_ERROR_PWM_SAMPLE_INVALID = -22,
  FOC_ERROR_ALIGNMENT_NO_MOVEMENT = -23,

  FOC_ERROR_PID_NOT_BOUND = -30,

  FOC_ERROR_ADC_CONFIGURATION = -40,
  FOC_ERROR_PWM_TRIGGER_NOT_READY = -41,
  FOC_ERROR_PWM_OUTPUT_ACTIVE = -42,
  FOC_ERROR_ADC_START_FAILED = -43,
  FOC_ERROR_ADC_CONVERSION_FAILED = -44,
  FOC_ERROR_CURRENT_OFFSET_INVALID = -45,
  FOC_ERROR_CURRENT_GAIN_INVALID = -46,
  FOC_ERROR_CURRENT_CALIBRATION_INVALID = -47,
  FOC_ERROR_CURRENT_RAW_SATURATED = -48
} FOC_Result_t;

typedef struct
{
  TIM_HandleTypeDef *tim;
  float pwm_period;
  float voltage_power_supply;
  float voltage_limit;
  float alignment_current_limit;
  float zero_electric_angle;
  float u_alpha;
  float u_beta;
  /* 圆限幅之后真正施加出去的 dq 电压。只供故障快照/台架诊断读取，控制路径
     不依赖它——没有这两个量，过流现场就只能靠 CCR 反算 dq 电压。 */
  float u_d;
  float u_q;
  float u_a;
  float u_b;
  float u_c;
  float dc_a;
  float dc_b;
  float dc_c;

  float i_alpha;
  float i_beta;
  float i_d;
  float i_q;
  float i_a;
  float i_b;
  float i_c;
  /* Park 变换的 sin/cos 缓存，同一拍内供电流环复用，避免重复三角函数 */
  float sin_el;
  float cos_el;
  volatile float cached_angle_el;
  volatile uint32_t sensor_last_success_tick;
  uint32_t sensor_max_age_ms;
  uint32_t adc_trigger_compare;
  volatile uint8_t sensor_valid;
  volatile uint8_t current_sample_valid;
  volatile uint8_t next_sample_valid;
  /* dq 电压矢量圆限幅信息：clip_uq 是请求值与实际 uq 的差值，供慢环抗饱和 */
  volatile uint8_t voltage_saturated;
  volatile int8_t voltage_saturation_sign;
  volatile float voltage_clip_uq;

  /*
   * 角度/速度跟踪观测器（PLL 型）：15 kHz 预测、1 kHz 修正。
   * obs_theta_hat 归一化到 [0, 2pi)，整圈数单独用 obs_turns 记，避免多圈连续
   * 累计后 float32 有效位不足（1000 转时 eps 已达 5e-4 rad，接近单拍增量）。
   */
  volatile float obs_theta_hat;
  volatile float obs_omega_hat;
  volatile int32_t obs_turns;
  volatile uint32_t obs_last_correct_tick;
  float obs_k1;
  float obs_k2;
  float obs_predict_ts;
  float obs_omega_limit;

  int dir;
  int pp;

  FUNC_SENSOR_UPDATE Sensor_Update;
  FUNC_SENSOR_GET_ONCE_ANGLE Sensor_GetOnceAngle;
  FUNC_SENSOR_GET_ANGLE Sensor_GetAngle;
  FUNC_SENSOR_GET_VELOCITY Sensor_GetVelocity;
} FOC_T;

FOC_Result_t FOC_Closeloop_Init(FOC_T *hfoc, TIM_HandleTypeDef *tim,
                                float pwm_period, float voltage, int dir,
                                int pp);
FOC_Result_t FOC_AlignmentSensor(FOC_T *hfoc, float alignment_voltage,
                                 float alignment_current_limit,
                                 uint32_t timeout_ms,
                                 float movement_threshold);
void FOC_SetVoltageLimit(FOC_T *hfoc, float voltage);
float FOC_SensorAngleToElectricalAngle(const FOC_T *hfoc, float sensor_angle);
void FOC_AlignmentSetVoltage(FOC_T *hfoc, float uq, float angle_el);
FOC_Result_t FOC_SensorUpdate(FOC_T *hfoc);
void FOC_UpdateCachedSensorAngle(FOC_T *hfoc, float mechanical_angle,
                                 uint32_t success_tick);
uint8_t FOC_IsSensorFresh(const FOC_T *hfoc, uint32_t now);
void FOC_Bind_SensorUpdate(FOC_T *hfoc, FUNC_SENSOR_UPDATE sensor_update);
void FOC_Bind_SensorGetOnceAngle(FOC_T *hfoc,
                                 FUNC_SENSOR_GET_ONCE_ANGLE sensor_get_angle);
void FOC_Bind_SensorGetAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ANGLE sensor_get_angle);
void FOC_Bind_SensorGetVelocity(FOC_T *hfoc,
                                FUNC_SENSOR_GET_VELOCITY sensor_get_velocity);

/*
 * 角度/速度跟踪观测器。omega_n/zeta 给出增益，correct_ts 为修正周期(1 ms)，
 * predict_ts 为预测周期(1/15000 s)，omega_limit 为观测速度的合理性钳位。
 */
void FOC_ObserverInit(FOC_T *hfoc, float omega_n, float zeta,
                      float correct_ts, float predict_ts, float omega_limit);
void FOC_ObserverReset(FOC_T *hfoc, float mechanical_angle, uint32_t now);
void FOC_ObserverPredict(FOC_T *hfoc);
void FOC_ObserverSnapshot(const FOC_T *hfoc, float *position, float *velocity);

/* 坐标变换不依赖 PID，过流快照在未启用电流环时同样需要 */
void FOC_Clarke(FOC_T *hfoc, float ia, float ib, float ic);
void FOC_Park(FOC_T *hfoc, float angle_el);

#if USE_CURRENT_LOOP
/* 需在同一拍先调用 FOC_Clarke() + FOC_Park()，本函数复用其 i_d/i_q 与 sin/cos */
FOC_Result_t FOC_CurrentLoopUpdate(FOC_T *hfoc, float id_ref, float iq_ref,
                                   void *pid_id, void *pid_iq, uint32_t now);
#endif
void FOC_CommitPwmUpdate(FOC_T *hfoc);
uint8_t FOC_IsCurrentSampleValid(const FOC_T *hfoc);
uint8_t FOC_IsNextCurrentSampleValid(const FOC_T *hfoc);

float _normalizeAngle(float angle);

#endif
