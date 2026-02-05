/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-25 14:17:26
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 20:13:42
 * @Description:
 */
#include "foc.h"
#include <string.h>
#include <math.h>
#include "pid.h"

#define _3PI_2 4.71238898038469f
#define _PI 3.141592653589793f
#define _SQRT3 1.732050808f
#define _1_SQRT3 0.577350269f
#define _2_SQRT3 1.154700538f

#define _constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 归一化角度到 [0, 2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2.0f * _PI);
  return a >= 0 ? a : (a + 2.0f * _PI);
}

// 求解电角度（开环）
float _openloop_electricalAngle(float shaft_angle, int pole_pairs)
{
  return (shaft_angle * pole_pairs);
}

// 求解电角度（闭环）
float _closeloop_electricalAngle(FOC_T *hfoc)
{
  return _normalizeAngle((float)(hfoc->dir * hfoc->pp) * hfoc->Sensor_GetOnceAngle() - hfoc->zero_electric_angle);
}

void _setPwm(FOC_T *hfoc, float Ua, float Ub, float Uc)
{
  Ua = _constrain(Ua, 0.0f, hfoc->voltage_limit);
  Ub = _constrain(Ub, 0.0f, hfoc->voltage_limit);
  Uc = _constrain(Uc, 0.0f, hfoc->voltage_limit);

  float dc_a = _constrain(Ua / hfoc->voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / hfoc->voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / hfoc->voltage_power_supply, 0.0f, 1.0f);

  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_1, dc_a * hfoc->pwm_period);
  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_2, dc_b * hfoc->pwm_period);
  __HAL_TIM_SET_COMPARE(hfoc->tim, TIM_CHANNEL_3, dc_c * hfoc->pwm_period);
}

/**
 * @description: 获取闭环控制电角度数据
 * @param {FOC_T} *hfoc foc句柄
 * @return {float} 电角度值
 */
float FOC_CloseloopElectricalAngle(FOC_T *hfoc)
{
  return _closeloop_electricalAngle(hfoc);
}

/**
 * @description: 设置力矩
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} Uq 力矩值
 * @param {float} angle_el 电角度
 * @return {*}
 */
void FOC_SetTorque(FOC_T *hfoc, float Uq, float angle_el)
{
  Uq = _constrain(Uq, -hfoc->voltage_power_supply / 2.0f, hfoc->voltage_power_supply / 2.0f);
  // float Ud = 0.0f;
  angle_el = _normalizeAngle(angle_el);

  // Park逆变换
  float Ualpha = -Uq * sin(angle_el);
  float Ubeta = Uq * cos(angle_el);

  // Clark逆变换
  float Ua = Ualpha + hfoc->voltage_power_supply / 2.0f;
  float Ub = (sqrt(3) * Ubeta - Ualpha) / 2.0f + hfoc->voltage_power_supply / 2.0f;
  float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2.0f + hfoc->voltage_power_supply / 2.0f;
  _setPwm(hfoc, Ua, Ub, Uc);
}

/**
 * @description: FOC闭环控制初始化
 * @param {FOC_T} *hfoc foc句柄
 * @param {TIM_HandleTypeDef} *tim PWM定时器句柄
 * @param {float} pwm_period PWM的重装载值
 * @param {float} voltage 电源电压值
 * @param {int} dir 方向
 * @param {int} pp 极对数
 * @return {*}
 */
void FOC_Closeloop_Init(FOC_T *hfoc, TIM_HandleTypeDef *tim, float pwm_period, float voltage, int dir, int pp)
{
  memset((void *)hfoc, 0, sizeof(FOC_T));
  hfoc->tim = tim;
  hfoc->pwm_period = pwm_period;
  // 配置电源电压和限压
  hfoc->voltage_power_supply = voltage;
  hfoc->voltage_limit = voltage;
  // 配置方向和极对数
  hfoc->dir = dir;
  hfoc->pp = pp;
}

/**
 * @description: 设置电压限制
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} v 电压值
 * @return {*}
 */
void FOC_SetVoltageLimit(FOC_T *hfoc, float v) { hfoc->voltage_limit = v; }

/**
 * @description: 编码器零位较准（需要配置传感器相关函数指针）
 * @param {FOC_T} *hfoc foc句柄
 * @return {*}
 */
void FOC_AlignmentSensor(FOC_T *hfoc)
{
  // 较准0位电角度
  if (hfoc->Sensor_GetAngle && hfoc->Sensor_GetOnceAngle)
  {
    FOC_SetTorque(hfoc, 3, _3PI_2);
    HAL_Delay(1500);
    hfoc->zero_electric_angle = _closeloop_electricalAngle(hfoc);
    FOC_SetTorque(hfoc, 0, _3PI_2);
  }
}

/**
 * @description: 绑定用于获取单圈弧度值的函数（0 - 6.28）
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_ONCE_ANGLE} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetOnceAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ONCE_ANGLE s)
{
  hfoc->Sensor_GetOnceAngle = s;
}

/**
 * @description: 绑定用于获取累计弧度值的函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_ANGLE} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ANGLE s)
{
  hfoc->Sensor_GetAngle = s;
}

/**
 * @description: 绑定用于更新计数值相关的函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_UPDATE} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorUpdate(FOC_T *hfoc, FUNC_SENSOR_UPDATE s)
{
  hfoc->Sensor_Update = s;
}

/**
 * @description: 绑定用户获取速度值得函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_VELOCITY} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetVelocity(FOC_T *hfoc, FUNC_SENSOR_GET_VELOCITY s)
{
  hfoc->Sensor_GetVelocity = s;
}

/**
 * @description: 更新传感器数据并缓存电角度
 * @param {FOC_T} *hfoc foc句柄
 * @return {*}
 */
void FOC_SensorUpdate(FOC_T *hfoc)
{
  hfoc->Sensor_Update();
  hfoc->cached_angle_el = _closeloop_electricalAngle(hfoc);  // 缓存电角度
}

/**
 * @description: Clarke变换 (abc -> αβ)
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} ia, ib, ic 三相电流
 * @return {*}
 */
void FOC_Clarke(FOC_T *hfoc, float ia, float ib, float ic)
{
  hfoc->i_a = ia;
  hfoc->i_b = ib;
  hfoc->i_c = ic;

  // Clarke变换: i_alpha = ia, i_beta = (ia + 2*ib) / sqrt(3)
  hfoc->i_alpha = ia;
  hfoc->i_beta = _1_SQRT3 * ia + _2_SQRT3 * ib;
}

/**
 * @description: Park变换 (αβ -> dq)
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} angle_el 电角度
 * @return {*}
 */
void FOC_Park(FOC_T *hfoc, float angle_el)
{
  float sin_angle = sinf(angle_el);
  float cos_angle = cosf(angle_el);

  // Park变换
  hfoc->i_d =  hfoc->i_alpha * cos_angle + hfoc->i_beta * sin_angle;
  hfoc->i_q = -hfoc->i_alpha * sin_angle + hfoc->i_beta * cos_angle;
}

/**
 * @description: 设置力矩（带Ud/Uq输入）
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} Ud d轴电压
 * @param {float} Uq q轴电压
 * @param {float} angle_el 电角度
 * @return {*}
 */
void FOC_SetTorqueWithCurrent(FOC_T *hfoc, float Ud, float Uq, float angle_el)
{
  float limit = hfoc->voltage_power_supply / 2.0f;
  Ud = _constrain(Ud, -limit, limit);
  Uq = _constrain(Uq, -limit, limit);
  angle_el = _normalizeAngle(angle_el);

  float sin_angle = sinf(angle_el);
  float cos_angle = cosf(angle_el);

  // Park逆变换
  float Ualpha = Ud * cos_angle - Uq * sin_angle;
  float Ubeta  = Ud * sin_angle + Uq * cos_angle;

  // Clarke逆变换 (SPWM)
  float Ua = Ualpha + hfoc->voltage_power_supply / 2.0f;
  float Ub = (-Ualpha + _SQRT3 * Ubeta) / 2.0f + hfoc->voltage_power_supply / 2.0f;
  float Uc = (-Ualpha - _SQRT3 * Ubeta) / 2.0f + hfoc->voltage_power_supply / 2.0f;

  _setPwm(hfoc, Ua, Ub, Uc);
}

/**
 * @description: 电流环控制
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} id_ref d轴电流参考值 (通常为0)
 * @param {float} iq_ref q轴电流参考值 (力矩指令)
 * @param {float} ia, ib, ic 采样的三相电流
 * @param {void*} pid_id d轴PID控制器
 * @param {void*} pid_iq q轴PID控制器
 * @return {*}
 */
void FOC_CurrentLoopControl(FOC_T *hfoc, float id_ref, float iq_ref,
                            float ia, float ib, float ic,
                            void *pid_id, void *pid_iq)
{
  // 使用缓存的电角度（由速度环更新，避免在电流环中读取I2C）
  float angle_el = hfoc->cached_angle_el;

  // Clarke变换: abc -> αβ
  FOC_Clarke(hfoc, ia, ib, ic);

  // Park变换: αβ -> dq
  FOC_Park(hfoc, angle_el);

  // 电流环PID控制
  float Ud = PID_Calc((PID_T *)pid_id, id_ref - hfoc->i_d);
  float Uq = PID_Calc((PID_T *)pid_iq, iq_ref - hfoc->i_q);

  // 输出电压
  FOC_SetTorqueWithCurrent(hfoc, Ud, Uq, angle_el);
}
