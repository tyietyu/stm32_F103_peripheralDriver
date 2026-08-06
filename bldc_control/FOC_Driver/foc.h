#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

#include "tim.h"

typedef int (*FUNC_SENSOR_UPDATE)(void);
typedef float (*FUNC_SENSOR_GET_ONCE_ANGLE)(void);
typedef float (*FUNC_SENSOR_GET_ANGLE)(void);
typedef float (*FUNC_SENSOR_GET_VELOCITY)(void);

typedef struct
{
  TIM_HandleTypeDef *tim;
  float pwm_period;
  float voltage_power_supply;
  float voltage_limit;
  float alignment_current_limit;
  float shaft_angle;
  uint32_t open_loop_timestamp;
  float zero_electric_angle;
  float u_alpha;
  float u_beta;
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
  volatile float cached_angle_el;
  volatile uint32_t sensor_last_success_tick;
  uint32_t sensor_max_age_ms;
  uint32_t adc_trigger_compare;
  volatile uint8_t sensor_valid;
  volatile uint8_t current_sample_valid;
  volatile uint8_t next_sample_valid;

  int dir;
  int pp;

  FUNC_SENSOR_UPDATE Sensor_Update;
  FUNC_SENSOR_GET_ONCE_ANGLE Sensor_GetOnceAngle;
  FUNC_SENSOR_GET_ANGLE Sensor_GetAngle;
  FUNC_SENSOR_GET_VELOCITY Sensor_GetVelocity;
} FOC_T;

void FOC_Closeloop_Init(FOC_T *hfoc, TIM_HandleTypeDef *tim, float pwm_period,
                        float voltage, int dir, int pp);
int FOC_AlignmentSensor(FOC_T *hfoc, float alignment_voltage,
                        float alignment_current_limit, uint32_t timeout_ms,
                        float movement_threshold);
void FOC_SetVoltageLimit(FOC_T *hfoc, float voltage);
float FOC_CloseloopElectricalAngle(FOC_T *hfoc);
float FOC_SensorAngleToElectricalAngle(const FOC_T *hfoc, float sensor_angle);
void FOC_SetTorque(FOC_T *hfoc, float uq, float angle_el);
int FOC_SensorUpdate(FOC_T *hfoc);
void FOC_UpdateCachedSensorAngle(FOC_T *hfoc, float mechanical_angle,
                                 uint32_t success_tick);
uint8_t FOC_IsSensorFresh(const FOC_T *hfoc, uint32_t now);
void FOC_Bind_SensorUpdate(FOC_T *hfoc, FUNC_SENSOR_UPDATE sensor_update);
void FOC_Bind_SensorGetOnceAngle(FOC_T *hfoc,
                                 FUNC_SENSOR_GET_ONCE_ANGLE sensor_get_angle);
void FOC_Bind_SensorGetAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ANGLE sensor_get_angle);
void FOC_Bind_SensorGetVelocity(FOC_T *hfoc,
                                FUNC_SENSOR_GET_VELOCITY sensor_get_velocity);

#if USE_CURRENT_LOOP
void FOC_Clarke(FOC_T *hfoc, float ia, float ib, float ic);
void FOC_Park(FOC_T *hfoc, float angle_el);
void FOC_SetTorqueWithCurrent(FOC_T *hfoc, float ud, float uq, float angle_el);
int FOC_CurrentLoopControl(FOC_T *hfoc, float id_ref, float iq_ref,
                           float ia, float ib, float ic,
                           void *pid_id, void *pid_iq, uint32_t now);
#endif
void FOC_CommitPwmUpdate(FOC_T *hfoc);
uint8_t FOC_IsCurrentSampleValid(const FOC_T *hfoc);
uint8_t FOC_IsNextCurrentSampleValid(const FOC_T *hfoc);

float _normalizeAngle(float angle);
float _openloop_electricalAngle(float shaft_angle, int pole_pairs);

#endif
