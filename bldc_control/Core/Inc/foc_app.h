/**
  ******************************************************************************
  * @file    foc_app.h
  * @brief   SguanFOC 应用适配层：把算法层、BSP 驱动层与 CubeMX 外设接成一条
  *          可运行的实时链路。
  *
  * 分层约定：
  *   - SguanFOC/UserData_*.h 只写一行转发到本文件的原语，不含业务逻辑；
  *   - 本文件不包含 SguanFOC.h，避免与 UserData_Status.h 形成头文件环。
  ******************************************************************************
  */
#ifndef __FOC_APP_H
#define __FOC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* MOTOR_STATUS_*_Signal() 的信号编号，供 UserData_Status.h 一行转发 */
#define FOC_SIGNAL_STANDBY          0U  /* 解除锁定请求（边沿语义，读后自清） */
#define FOC_SIGNAL_UNINITIALIZED    1U  /* 开始初始化请求（边沿语义，读后自清） */
#define FOC_SIGNAL_ENCODER_ERROR    2U  /* 编码器故障（电平语义，需显式解除） */
#define FOC_SIGNAL_SENSOR_ERROR     3U  /* 栅极驱动器故障（电平语义） */
#define FOC_SIGNAL_EMERGENCY_STOP   4U  /* 急停（电平语义） */
#define FOC_SIGNAL_DISABLED         5U  /* 失能（电平语义） */
#define FOC_SIGNAL_COUNT            6U

/* ================= 生命周期与调度 ================= */
/* main() 中调用：复位内部状态、写 Sguan.status = 0x01 触发库启动 */
void FocApp_Init(void);
/* while(1) 中调用，内部按 10 Hz 自限速：DRV8301 / AS5600 状态轮询，置故障标志 */
void FocApp_DiagnoseLoop(void);
/* 硬关断：先撤 TIM1 的 MOE 再拉低 EN_GATE。任何上下文可调，含 HardFault */
void FocApp_Shutdown(void);

/*
 * 速度给定 [rad/s]，正负决定转向，落库前按 FOC_SPEED_LIMIT_RAD_S 限幅。
 * 上位机协议已关闭，上电标定完成后电机直接按该值转动；可在代码或调试器里改。
 */
extern volatile float FocApp_SpeedCmd_RadS;

/* ================= 供 UserData_*.h 一行转发的原语 ================= */
void     FocApp_HwStart(void);                       /* <- User_InitialInit()   */
int32_t  FocApp_GetCurrentRaw(uint8_t ch);           /* <- User_ReadADC_Raw()   */
float    FocApp_GetEncoderRad(void);                 /* <- User_Encoder_ReadRad */
void     FocApp_SetPwmDuty(uint16_t du,
                           uint16_t dv,
                           uint16_t dw);             /* <- User_PwmDuty_Set()   */
uint8_t  FocApp_GetFaultFlag(uint8_t which);         /* <- MOTOR_STATUS_*_Signal*/
void     FocApp_ApplyTargets(void);                  /* <- User_UserControl()   */

/* ================= 目标值与控制字下发 ================= */
/* 以下三个接口由 User_AO/BO/CO_Adjust() 转发；上位机协议关闭后无人触发，保留供重新接入 */
/* 按当前 mode 分发目标值，内部按 safe 限幅后再落到库结构体 */
void FocApp_SetTarget(float value);
/* 登记模式切换请求，实际切换在 15 kHz 环内完成（含控制器历史量复位） */
void FocApp_SetMode(uint8_t mode);
/* 控制字：0 = 急停锁定，1 = 解除锁定进待机，>=2 = 重新初始化并运行 */
void FocApp_SetControlWord(float value);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_APP_H */
