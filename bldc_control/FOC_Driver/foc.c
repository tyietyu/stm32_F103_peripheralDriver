#include "foc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "main.h"
#if USE_CURRENT_LOOP
#include "pid.h"
#endif

#define FOC_PI                  3.141592653589793f
#define FOC_TWO_PI              6.283185307179586f
#define FOC_THREE_PI_2          4.712388980384690f
#define FOC_SQRT3               1.732050808f
#define FOC_ONE_SQRT3           0.577350269f
#define FOC_TWO_THIRDS          0.666666667f
#define FOC_ALIGNMENT_SWEEP     (FOC_PI / 2.0f)
/* 每步含一次软件 I²C 读 AS5600，开销远大于 FOC_ALIGNMENT_STEP_MS。
   20 步扫 π/2 (每步 4.5°) 已满足零位精度，同时把 I²C 次数降到 1/3。 */
#define FOC_ALIGNMENT_STEPS     20U
#define FOC_ALIGNMENT_STEP_MS   5U
#define FOC_ALIGNMENT_SETTLE_MS 200U
#define FOC_DEFAULT_SENSOR_AGE  5U

/*
 * 角度/速度跟踪观测器起始设计值（方案 22.3 / 23.1）：
 * K1 = 2*zeta*omega_n*Ts_s = 0.452，K2 = omega_n^2*Ts_s = 63.2。
 * omega_n 必须 >= 2 倍速度环带宽；提高前先确认观测速度抖动仍达标。
 */
#define FOC_OBSERVER_OMEGA_N     (2.0f * FOC_PI * 40.0f)
#define FOC_OBSERVER_ZETA        0.9f
#define FOC_OBSERVER_CORRECT_TS  1.0e-3f
#define FOC_OBSERVER_PREDICT_TS  (1.0f / (float)PWM_FREQ)
/* 观测速度合理性钳位：远高于本电机工况，仅用于阻止异常值污染多圈计数 */
#define FOC_OBSERVER_OMEGA_LIMIT 200.0f

#define FOC_CONSTRAIN(value, low, high) \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

static float FOC_GetEffectiveVoltageLimit(const FOC_T *hfoc);
static void FOC_SetPwm(FOC_T *hfoc, float ua, float ub, float uc);
static void FOC_SetVoltageDqWithTrig(FOC_T *hfoc, float ud, float uq,
                                     float sin_angle, float cos_angle,
                                     float *ud_applied, float *uq_applied);
static FOC_Result_t FOC_AlignmentStep(FOC_T *hfoc, float voltage,
                                        float angle_el, uint32_t start_tick,
                                        uint32_t timeout_ms);
static FOC_Result_t FOC_AlignmentHold(FOC_T *hfoc, float voltage,
                                      float angle_el, uint32_t duration_ms,
                                      uint32_t start_tick,
                                      uint32_t timeout_ms);

float _normalizeAngle(float angle)
{
  float normalized = fmodf(angle, FOC_TWO_PI);

  return (normalized >= 0.0f) ? normalized : (normalized + FOC_TWO_PI);
}

float FOC_SensorAngleToElectricalAngle(const FOC_T *hfoc, float sensor_angle)
{
  if (hfoc == NULL)
  {
    return 0.0f;
  }

  return _normalizeAngle((float)(hfoc->dir * hfoc->pp) * sensor_angle -
                         hfoc->zero_electric_angle);
}

static float FOC_GetEffectiveVoltageLimit(const FOC_T *hfoc)
{
  float modulation_limit;
  float supply_limit;


  supply_limit = hfoc->voltage_power_supply / 2.0f;
  modulation_limit = hfoc->voltage_power_supply *
                     ((float)(PWM_MAX_COMPARE - PWM_MIN_COMPARE) /
                      hfoc->pwm_period) / FOC_SQRT3;
  if (modulation_limit < supply_limit)
  {
    supply_limit = modulation_limit;
  }

  return (hfoc->voltage_limit < supply_limit) ? hfoc->voltage_limit
                                               : supply_limit;
}

static void FOC_SetPwm(FOC_T *hfoc, float ua, float ub, float uc)
{
  float allowed_span;
  float common_mode;
  float dc_a;
  float dc_b;
  float dc_c;
  float dc_max;
  float dc_min;
  float max_dc;
  float min_dc;
  float offset_high;
  float offset_low;
  float scale;
  float span;
  uint32_t compare_a;
  uint32_t compare_b;
  uint32_t compare_c;
  uint32_t max_compare;
  uint32_t trigger_compare;
  uint32_t trigger_earliest;


  ua = FOC_CONSTRAIN(ua, 0.0f, hfoc->voltage_power_supply);
  ub = FOC_CONSTRAIN(ub, 0.0f, hfoc->voltage_power_supply);
  uc = FOC_CONSTRAIN(uc, 0.0f, hfoc->voltage_power_supply);

  dc_a = FOC_CONSTRAIN(ua / hfoc->voltage_power_supply, 0.0f, 1.0f);
  dc_b = FOC_CONSTRAIN(ub / hfoc->voltage_power_supply, 0.0f, 1.0f);
  dc_c = FOC_CONSTRAIN(uc / hfoc->voltage_power_supply, 0.0f, 1.0f);

  min_dc = (float)PWM_MIN_COMPARE / hfoc->pwm_period;
  max_dc = (float)PWM_MAX_COMPARE / hfoc->pwm_period;
  allowed_span = max_dc - min_dc;
  dc_min = fminf(dc_a, fminf(dc_b, dc_c));
  dc_max = fmaxf(dc_a, fmaxf(dc_b, dc_c));
  span = dc_max - dc_min;

  if (span > allowed_span)
  {
    scale = allowed_span / span;
    dc_a = min_dc + (dc_a - dc_min) * scale;
    dc_b = min_dc + (dc_b - dc_min) * scale;
    dc_c = min_dc + (dc_c - dc_min) * scale;
  }
  else
  {
    offset_low = min_dc - dc_min;
    offset_high = max_dc - dc_max;
    common_mode = (offset_low + offset_high) * 0.5f;
    dc_a += common_mode;
    dc_b += common_mode;
    dc_c += common_mode;
  }

  dc_a = FOC_CONSTRAIN(dc_a, min_dc, max_dc);
  dc_b = FOC_CONSTRAIN(dc_b, min_dc, max_dc);
  dc_c = FOC_CONSTRAIN(dc_c, min_dc, max_dc);
  compare_a = (uint32_t)(dc_a * hfoc->pwm_period + 0.5f);
  compare_b = (uint32_t)(dc_b * hfoc->pwm_period + 0.5f);
  compare_c = (uint32_t)(dc_c * hfoc->pwm_period + 0.5f);

  max_compare = compare_a;
  if (compare_b > max_compare)
  {
    max_compare = compare_b;
  }
  if (compare_c > max_compare)
  {
    max_compare = compare_c;
  }

  trigger_earliest = max_compare + PWM_ADC_SEQUENCE_TICKS +
                     PWM_ADC_END_MARGIN_TICKS;
  if ((trigger_earliest > PWM_ADC_TRIGGER_LATEST) ||
      (PWM_MIN_COMPARE >= PWM_MAX_COMPARE))
  {
    compare_a = PWM_MIN_COMPARE;
    compare_b = PWM_MIN_COMPARE;
    compare_c = PWM_MIN_COMPARE;
    trigger_compare = PWM_ADC_TRIGGER_LATEST;
    hfoc->next_sample_valid = 0U;
  }
  else
  {
    trigger_compare = trigger_earliest +
                      (PWM_ADC_TRIGGER_LATEST - trigger_earliest) / 2U;
    hfoc->next_sample_valid = 1U;
  }

  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_1, compare_a);
  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_2, compare_b);
  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_3, compare_c);
  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_4, trigger_compare);

  hfoc->u_a = ua;
  hfoc->u_b = ub;
  hfoc->u_c = uc;
  hfoc->dc_a = dc_a;
  hfoc->dc_b = dc_b;
  hfoc->dc_c = dc_c;
  hfoc->adc_trigger_compare = trigger_compare;
}

static void FOC_SetVoltageDqWithTrig(FOC_T *hfoc, float ud, float uq,
                                     float sin_angle, float cos_angle,
                                     float *ud_applied, float *uq_applied)
{
  float limit;
  float magnitude;
  float scale;
  float u_alpha;
  float u_beta;
  float ua;
  float ub;
  float uc;

  /*
   * 全部电压限幅集中在此处的矢量圆限幅：保持 dq 电压矢量方向不变。PID 内部
   * 不再做分轴方形限幅，被削掉的量由 FOC_CurrentLoopUpdate() 回传给条件积分。
   */
  limit = FOC_GetEffectiveVoltageLimit(hfoc);
  magnitude = sqrtf(ud * ud + uq * uq);
  if ((magnitude > limit) && (magnitude > 0.0f))
  {
    scale = limit / magnitude;
    ud *= scale;
    uq *= scale;
    hfoc->voltage_saturated = 1U;
    hfoc->voltage_saturation_sign = (uq >= 0.0f) ? (int8_t)1 : (int8_t)-1;
  }
  else
  {
    hfoc->voltage_saturated = 0U;
    hfoc->voltage_saturation_sign = 0;
  }

  u_alpha = ud * cos_angle - uq * sin_angle;
  u_beta = ud * sin_angle + uq * cos_angle;
  ua = u_alpha + hfoc->voltage_power_supply / 2.0f;
  ub = (-u_alpha + FOC_SQRT3 * u_beta) / 2.0f +
       hfoc->voltage_power_supply / 2.0f;
  uc = (-u_alpha - FOC_SQRT3 * u_beta) / 2.0f +
       hfoc->voltage_power_supply / 2.0f;

  hfoc->u_d = ud;
  hfoc->u_q = uq;
  hfoc->u_alpha = u_alpha;
  hfoc->u_beta = u_beta;
  FOC_SetPwm(hfoc, ua, ub, uc);

  if (ud_applied != NULL)
  {
    *ud_applied = ud;
  }
  if (uq_applied != NULL)
  {
    *uq_applied = uq;
  }
}

FOC_Result_t FOC_Closeloop_Init(FOC_T *hfoc, TIM_HandleTypeDef *tim,
                                float pwm_period, float voltage, int dir,
                                int pp)
{
  if ((hfoc == NULL) || (tim == NULL) || (tim->Instance == NULL) ||
      (pwm_period <= 0.0f) || (voltage <= 0.0f) || (pp <= 0))
  {
    return FOC_ERROR_INVALID_ARGUMENT;
  }

  memset(hfoc, 0, sizeof(*hfoc));
  hfoc->tim = tim;
  hfoc->pwm_period = pwm_period;
  hfoc->voltage_power_supply = voltage;
  hfoc->voltage_limit = voltage;
  hfoc->dir = (dir >= 0) ? 1 : -1;
  hfoc->pp = pp;
  hfoc->sensor_max_age_ms = FOC_DEFAULT_SENSOR_AGE;
  FOC_ObserverInit(hfoc, FOC_OBSERVER_OMEGA_N, FOC_OBSERVER_ZETA,
                   FOC_OBSERVER_CORRECT_TS, FOC_OBSERVER_PREDICT_TS,
                   FOC_OBSERVER_OMEGA_LIMIT);
  return FOC_RESULT_OK;
}

void FOC_SetVoltageLimit(FOC_T *hfoc, float voltage)
{
  if (hfoc != NULL)
  {
    hfoc->voltage_limit = (voltage > 0.0f) ? voltage : 0.0f;
  }
}

float FOC_CloseloopElectricalAngle(FOC_T *hfoc)
{
  if ((hfoc == NULL) || (hfoc->Sensor_GetOnceAngle == NULL))
  {
    return 0.0f;
  }

  return FOC_SensorAngleToElectricalAngle(hfoc,
                                          hfoc->Sensor_GetOnceAngle());
}

void FOC_SetTorque(FOC_T *hfoc, float uq, float angle_el)
{
  float normalized_angle;

  if (hfoc == NULL)
  {
    return;
  }

  normalized_angle = _normalizeAngle(angle_el);
  FOC_SetVoltageDqWithTrig(hfoc, 0.0f, uq, sinf(normalized_angle),
                           cosf(normalized_angle), NULL, NULL);
}

void FOC_Bind_SensorUpdate(FOC_T *hfoc, FUNC_SENSOR_UPDATE sensor_update)
{
  if (hfoc != NULL)
  {
    hfoc->Sensor_Update = sensor_update;
  }
}

void FOC_Bind_SensorGetOnceAngle(FOC_T *hfoc,
                                 FUNC_SENSOR_GET_ONCE_ANGLE sensor_get_angle)
{
  if (hfoc != NULL)
  {
    hfoc->Sensor_GetOnceAngle = sensor_get_angle;
  }
}

void FOC_Bind_SensorGetAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ANGLE sensor_get_angle)
{
  if (hfoc != NULL)
  {
    hfoc->Sensor_GetAngle = sensor_get_angle;
  }
}

void FOC_Bind_SensorGetVelocity(FOC_T *hfoc,
                                FUNC_SENSOR_GET_VELOCITY sensor_get_velocity)
{
  if (hfoc != NULL)
  {
    hfoc->Sensor_GetVelocity = sensor_get_velocity;
  }
}

void FOC_ObserverInit(FOC_T *hfoc, float omega_n, float zeta,
                      float correct_ts, float predict_ts, float omega_limit)
{
  if ((hfoc == NULL) || (omega_n <= 0.0f) || (correct_ts <= 0.0f) ||
      (predict_ts <= 0.0f) || (omega_limit <= 0.0f))
  {
    return;
  }

  hfoc->obs_k1 = 2.0f * zeta * omega_n * correct_ts;
  hfoc->obs_k2 = omega_n * omega_n * correct_ts;
  hfoc->obs_predict_ts = predict_ts;
  hfoc->obs_omega_limit = omega_limit;
  hfoc->obs_theta_hat = 0.0f;
  hfoc->obs_omega_hat = 0.0f;
  hfoc->obs_turns = 0;
  hfoc->obs_last_correct_tick = 0U;
}

void FOC_ObserverReset(FOC_T *hfoc, float mechanical_angle, uint32_t now)
{
  uint32_t primask;

  if (hfoc == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  hfoc->obs_theta_hat = _normalizeAngle(mechanical_angle);
  hfoc->obs_omega_hat = 0.0f;
  hfoc->obs_turns = 0;
  hfoc->obs_last_correct_tick = now;
  hfoc->cached_angle_el = FOC_SensorAngleToElectricalAngle(hfoc,
                                                           hfoc->obs_theta_hat);
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/*
 * 预测：在 15 kHz ADC 注入 ISR 内调用。该 ISR 抢占优先级最高，其读改写相对
 * 前台的修正是原子的，无需再关中断。
 * 电角度按增量推进而非由 theta_hat 重算，避免在 15 kHz 路径上执行 fmodf；
 * 每次跨越 2pi 才回退到 _normalizeAngle()，且每拍都会在 1 kHz 修正处重锚定。
 */
void FOC_ObserverPredict(FOC_T *hfoc)
{
  float angle_el;
  float omega;
  float theta;

  omega = hfoc->obs_omega_hat;
  theta = hfoc->obs_theta_hat + omega * hfoc->obs_predict_ts;
  if (theta >= FOC_TWO_PI)
  {
    theta -= FOC_TWO_PI;
    ++hfoc->obs_turns;
  }
  else if (theta < 0.0f)
  {
    theta += FOC_TWO_PI;
    --hfoc->obs_turns;
  }
  hfoc->obs_theta_hat = theta;

  angle_el = hfoc->cached_angle_el +
             (float)(hfoc->dir * hfoc->pp) * omega * hfoc->obs_predict_ts;
  if ((angle_el >= FOC_TWO_PI) || (angle_el < 0.0f))
  {
    angle_el = _normalizeAngle(angle_el);
  }
  hfoc->cached_angle_el = angle_el;
}

void FOC_ObserverSnapshot(const FOC_T *hfoc, float *position, float *velocity)
{
  uint32_t primask;

  if (hfoc == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (position != NULL)
  {
    *position = (float)hfoc->obs_turns * FOC_TWO_PI + hfoc->obs_theta_hat;
  }
  if (velocity != NULL)
  {
    *velocity = hfoc->obs_omega_hat;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void FOC_UpdateCachedSensorAngle(FOC_T *hfoc, float mechanical_angle,
                                 uint32_t success_tick)
{
  float error;
  float omega;
  float theta;
  uint32_t primask;

  if (hfoc == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  hfoc->sensor_valid = 0U;

  /* 单圈误差归一化到 (-pi, pi]，跨 0/4095 边界无需额外判定 */
  error = mechanical_angle - hfoc->obs_theta_hat;
  if (error > FOC_PI)
  {
    error -= FOC_TWO_PI;
  }
  else if (error <= -FOC_PI)
  {
    error += FOC_TWO_PI;
  }

  theta = hfoc->obs_theta_hat + hfoc->obs_k1 * error;
  if (theta >= FOC_TWO_PI)
  {
    theta -= FOC_TWO_PI;
    ++hfoc->obs_turns;
  }
  else if (theta < 0.0f)
  {
    theta += FOC_TWO_PI;
    --hfoc->obs_turns;
  }
  hfoc->obs_theta_hat = theta;

  omega = hfoc->obs_omega_hat + hfoc->obs_k2 * error;
  hfoc->obs_omega_hat = FOC_CONSTRAIN(omega, -hfoc->obs_omega_limit,
                                      hfoc->obs_omega_limit);
  hfoc->obs_last_correct_tick = success_tick;

  hfoc->cached_angle_el = FOC_SensorAngleToElectricalAngle(hfoc, theta);
  hfoc->sensor_last_success_tick = success_tick;
  __DMB();
  hfoc->sensor_valid = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

FOC_Result_t FOC_SensorUpdate(FOC_T *hfoc)
{
  uint32_t now;

  /* 对齐流程内使用：此时观测器尚未投入，单次读失败即视为对齐失败 */
  if (hfoc->Sensor_Update() != 0)
  {
    hfoc->sensor_valid = 0U;
    return FOC_ERROR_SENSOR_UPDATE_FAILED;
  }

  now = HAL_GetTick();
  FOC_UpdateCachedSensorAngle(hfoc, hfoc->Sensor_GetOnceAngle(), now);
  return FOC_RESULT_OK;
}

uint8_t FOC_IsSensorFresh(const FOC_T *hfoc, uint32_t now)
{
  if ((hfoc == NULL) || (hfoc->sensor_valid == 0U))
  {
    return 0U;
  }

  return ((uint32_t)(now - hfoc->sensor_last_success_tick) <=
          hfoc->sensor_max_age_ms) ? 1U : 0U;
}

static FOC_Result_t FOC_AlignmentStep(FOC_T *hfoc, float voltage,
                                        float angle_el, uint32_t start_tick,
                                        uint32_t timeout_ms)
{
  if ((hfoc->tim->Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    return FOC_ERROR_PWM_OUTPUT_DISABLED;
  }
  if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms)
  {
    return FOC_ERROR_ALIGNMENT_TIMEOUT;
  }

  FOC_SetTorque(hfoc, voltage, angle_el);
  if (FOC_IsNextCurrentSampleValid(hfoc) == 0U)
  {
    return FOC_ERROR_PWM_SAMPLE_INVALID;
  }

  HAL_Delay(FOC_ALIGNMENT_STEP_MS);
  return FOC_SensorUpdate(hfoc);
}

static FOC_Result_t FOC_AlignmentHold(FOC_T *hfoc, float voltage,
                                      float angle_el, uint32_t duration_ms,
                                      uint32_t start_tick,
                                      uint32_t timeout_ms)
{
  FOC_Result_t result;
  uint32_t hold_start = HAL_GetTick();

  /* 以真实时钟判定保持时长：每步除 HAL_Delay 外还含一次软件 I²C 读取，
     按 FOC_ALIGNMENT_STEP_MS 累加会严重低估实际耗时。 */
  while ((uint32_t)(HAL_GetTick() - hold_start) < duration_ms)
  {
    result = FOC_AlignmentStep(hfoc, voltage, angle_el, start_tick,
                               timeout_ms);
    if (result != FOC_RESULT_OK)
    {
      return result;
    }
  }

  return FOC_RESULT_OK;
}

FOC_Result_t FOC_AlignmentSensor(FOC_T *hfoc, float alignment_voltage,
                                 float alignment_current_limit,
                                 uint32_t timeout_ms,
                                 float movement_threshold)
{
  FOC_Result_t result;
  float angle_command;
  float end_angle;
  float movement;
  float start_angle;
  uint32_t i;
  uint32_t start_tick;

  if ((hfoc == NULL) || (hfoc->tim == NULL) ||
      (hfoc->tim->Instance == NULL) ||
      (alignment_voltage <= 0.0f) ||
      (alignment_current_limit <= 0.0f) ||
      (timeout_ms == 0U) || (movement_threshold <= 0.0f))
  {
    return FOC_ERROR_INVALID_ARGUMENT;
  }
  if ((hfoc->Sensor_Update == NULL) ||
      (hfoc->Sensor_GetOnceAngle == NULL) ||
      (hfoc->Sensor_GetAngle == NULL))
  {
    return FOC_ERROR_SENSOR_NOT_BOUND;
  }

  hfoc->alignment_current_limit = alignment_current_limit;
  start_tick = HAL_GetTick();
  result = FOC_AlignmentHold(hfoc, alignment_voltage, FOC_THREE_PI_2,
                             FOC_ALIGNMENT_SETTLE_MS, start_tick,
                             timeout_ms);
  if (result != FOC_RESULT_OK)
  {
    FOC_SetTorque(hfoc, 0.0f, FOC_THREE_PI_2);
    return result;
  }

  start_angle = hfoc->Sensor_GetAngle();
  for (i = 1U; i <= FOC_ALIGNMENT_STEPS; ++i)
  {
    angle_command = FOC_THREE_PI_2 +
                    FOC_ALIGNMENT_SWEEP * (float)i /
                    (float)FOC_ALIGNMENT_STEPS;
    result = FOC_AlignmentStep(hfoc, alignment_voltage, angle_command,
                               start_tick, timeout_ms);
    if (result != FOC_RESULT_OK)
    {
      FOC_SetTorque(hfoc, 0.0f, FOC_THREE_PI_2);
      return result;
    }
  }

  end_angle = hfoc->Sensor_GetAngle();
  movement = end_angle - start_angle;
  if (fabsf(movement) < movement_threshold)
  {
    FOC_SetTorque(hfoc, 0.0f, FOC_THREE_PI_2);
    return FOC_ERROR_ALIGNMENT_NO_MOVEMENT;
  }
  hfoc->dir = (movement > 0.0f) ? 1 : -1;

  for (i = FOC_ALIGNMENT_STEPS; i > 0U; --i)
  {
    angle_command = FOC_THREE_PI_2 +
                    FOC_ALIGNMENT_SWEEP * (float)(i - 1U) /
                    (float)FOC_ALIGNMENT_STEPS;
    result = FOC_AlignmentStep(hfoc, alignment_voltage, angle_command,
                               start_tick, timeout_ms);
    if (result != FOC_RESULT_OK)
    {
      FOC_SetTorque(hfoc, 0.0f, FOC_THREE_PI_2);
      return result;
    }
  }

  result = FOC_AlignmentHold(hfoc, alignment_voltage, FOC_THREE_PI_2,
                             FOC_ALIGNMENT_SETTLE_MS, start_tick,
                             timeout_ms);
  if (result != FOC_RESULT_OK)
  {
    FOC_SetTorque(hfoc, 0.0f, FOC_THREE_PI_2);
    return result;
  }

  hfoc->zero_electric_angle = _normalizeAngle(
    (float)(hfoc->dir * hfoc->pp) * hfoc->Sensor_GetOnceAngle());
  /* 进入 RUN 之前复位观测器：theta_hat = 实测机械角，omega_hat = 0，
     避免对齐期间的强制转动在观测速度里留下残值。 */
  FOC_ObserverReset(hfoc, hfoc->Sensor_GetOnceAngle(), HAL_GetTick());
  FOC_UpdateCachedSensorAngle(hfoc, hfoc->Sensor_GetOnceAngle(),
                              HAL_GetTick());
  FOC_SetTorque(hfoc, 0.0f, FOC_THREE_PI_2);
  return FOC_RESULT_OK;
}

/* 坐标变换不依赖 PID，过流快照在未启用电流环时同样需要，故不做宏裁剪 */
void FOC_Clarke(FOC_T *hfoc, float ia, float ib, float ic)
{
  hfoc->i_a = ia;
  hfoc->i_b = ib;
  hfoc->i_c = ic;
  hfoc->i_alpha = FOC_TWO_THIRDS * (ia - 0.5f * ib - 0.5f * ic);
  hfoc->i_beta = FOC_ONE_SQRT3 * (ib - ic);
}

void FOC_Park(FOC_T *hfoc, float angle_el)
{
  if (hfoc == NULL)
  {
    return;
  }

  hfoc->sin_el = sinf(angle_el);
  hfoc->cos_el = cosf(angle_el);
  hfoc->i_d = hfoc->i_alpha * hfoc->cos_el + hfoc->i_beta * hfoc->sin_el;
  hfoc->i_q = -hfoc->i_alpha * hfoc->sin_el + hfoc->i_beta * hfoc->cos_el;
}

#if USE_CURRENT_LOOP
void FOC_SetTorqueWithCurrent(FOC_T *hfoc, float ud, float uq, float angle_el)
{
  float normalized_angle;

  if (hfoc == NULL)
  {
    return;
  }

  normalized_angle = _normalizeAngle(angle_el);
  FOC_SetVoltageDqWithTrig(hfoc, ud, uq, sinf(normalized_angle),
                           cosf(normalized_angle), NULL, NULL);
}

/*
 * 电流环。调用前必须在同一拍内先执行 FOC_Clarke() + FOC_Park(cached_angle_el)，
 * 本函数直接复用其 i_d/i_q 与 sin/cos，不重复计算三角函数。
 */
FOC_Result_t FOC_CurrentLoopUpdate(FOC_T *hfoc, float id_ref, float iq_ref,
                                   void *pid_id, void *pid_iq, uint32_t now)
{
  float ud;
  float ud_applied;
  float uq;
  float uq_applied;

  if (FOC_IsSensorFresh(hfoc, now) == 0U)
  {
    return FOC_ERROR_SENSOR_STALE;
  }

  ud = PID_Calc((PID_T *)pid_id, id_ref - hfoc->i_d);
  uq = PID_Calc((PID_T *)pid_iq, iq_ref - hfoc->i_q);
  FOC_SetVoltageDqWithTrig(hfoc, ud, uq, hfoc->sin_el, hfoc->cos_el,
                           &ud_applied, &uq_applied);
  /* 圆限幅削掉的量回传给条件积分判据，否则圆限幅路径存在积分饱和 */
  PID_ApplyExternalClip((PID_T *)pid_id, ud - ud_applied, ud_applied);
  PID_ApplyExternalClip((PID_T *)pid_iq, uq - uq_applied, uq_applied);
  return FOC_RESULT_OK;
}
#endif

void FOC_CommitPwmUpdate(FOC_T *hfoc)
{
  hfoc->current_sample_valid = hfoc->next_sample_valid;
}

uint8_t FOC_IsCurrentSampleValid(const FOC_T *hfoc)
{
  return (hfoc->current_sample_valid != 0U) ? 1U : 0U;
}

uint8_t FOC_IsNextCurrentSampleValid(const FOC_T *hfoc)
{
  return (hfoc->next_sample_valid != 0U) ? 1U : 0U;
}
