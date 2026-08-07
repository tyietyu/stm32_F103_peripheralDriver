#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

#include "tim.h"

typedef int (*FUNC_SENSOR_UPDATE)(void);
typedef float (*FUNC_SENSOR_GET_ONCE_ANGLE)(void);
typedef float (*FUNC_SENSOR_GET_ANGLE)(void);
typedef float (*FUNC_SENSOR_GET_VELOCITY)(void);

typedef enum
{
  FOC_RESULT_OK = 0,
  FOC_ERROR_CURRENT_LIMIT = -1,
  FOC_ERROR_CURRENT_SUM = -2,

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
  FOC_ERROR_CURRENT_CALIBRATION_INVALID = -47
} FOC_Result_t;

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

FOC_Result_t FOC_Closeloop_Init(FOC_T *hfoc, TIM_HandleTypeDef *tim,
                                float pwm_period, float voltage, int dir,
                                int pp);
FOC_Result_t FOC_AlignmentSensor(FOC_T *hfoc, float alignment_voltage,
                                 float alignment_current_limit,
                                 uint32_t timeout_ms,
                                 float movement_threshold);
void FOC_SetVoltageLimit(FOC_T *hfoc, float voltage);
float FOC_CloseloopElectricalAngle(FOC_T *hfoc);
float FOC_SensorAngleToElectricalAngle(const FOC_T *hfoc, float sensor_angle);
void FOC_SetTorque(FOC_T *hfoc, float uq, float angle_el);
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

#if USE_CURRENT_LOOP
void FOC_Clarke(FOC_T *hfoc, float ia, float ib, float ic);
void FOC_Park(FOC_T *hfoc, float angle_el);
void FOC_SetTorqueWithCurrent(FOC_T *hfoc, float ud, float uq, float angle_el);
FOC_Result_t FOC_CurrentLoopControl(FOC_T *hfoc, float id_ref, float iq_ref,
                                    float ia, float ib, float ic,
                                    void *pid_id, void *pid_iq, uint32_t now);
#endif
void FOC_CommitPwmUpdate(FOC_T *hfoc);
uint8_t FOC_IsCurrentSampleValid(const FOC_T *hfoc);
uint8_t FOC_IsNextCurrentSampleValid(const FOC_T *hfoc);

float _normalizeAngle(float angle);
float _openloop_electricalAngle(float shaft_angle, int pole_pairs);

#endif
