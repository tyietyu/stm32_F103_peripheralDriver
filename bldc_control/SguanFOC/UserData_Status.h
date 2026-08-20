#ifndef __USERDATA_STATUS_H
#define __USERDATA_STATUS_H
#include <stdint.h>
/* 电机控制User用户设置·状态管理 */
/* 用户自己的CODE BEGIN Includes */
#include "foc_app.h"
/* 用户自己的CODE END Includes */

/* ================= 状态机任务信号(输入) =================
 * 六个 Signal 都在 1 kHz 的 SguanFOC_Low_Loop() 里被调用，只能读全局标志。
 * DRV8301_ReadStatus() 是阻塞 SPI、AS5600_CheckStatus() 是阻塞 I2C，
 * 二者放在 while(1) 的 FocApp_DiagnoseLoop() 里 10 Hz 轮询，只负责置标志。
 */
static inline uint8_t MOTOR_STATUS_STANDBY_Signal(void){
    /* 解除锁定信号：边沿语义，FocApp 内读后自清 */
    return FocApp_GetFaultFlag(FOC_SIGNAL_STANDBY);
}

static inline uint8_t MOTOR_STATUS_UNINITIALIZED_Signal(void){
    /* 准备开始初始化信号：库内此分支是死代码（Status_Switch_Loop 会先 return），
       启动改由 FocApp 直接写 Sguan.status = 0x01 触发 */
    return FocApp_GetFaultFlag(FOC_SIGNAL_UNINITIALIZED);
}

static inline uint8_t MOTOR_STATUS_ENCODER_ERROR_Signal(void){
    /* 磁体掉落/过弱（10 Hz 轮询）或角度数据陈旧（1 kHz 去抖判定） */
    return FocApp_GetFaultFlag(FOC_SIGNAL_ENCODER_ERROR);
}

static inline uint8_t MOTOR_STATUS_SENSOR_ERROR_Signal(void){
    /* DRV8301 的 STATUS1.FAULT 位，或上电时 DRV8301_Init() 失败 */
    return FocApp_GetFaultFlag(FOC_SIGNAL_SENSOR_ERROR);
}

static inline uint8_t MOTOR_STATUS_EMERGENCY_STOP_Signal(void){
    /* 上位机 CO<0.5 置位，需再发 CO>=0.5 才解除 */
    return FocApp_GetFaultFlag(FOC_SIGNAL_EMERGENCY_STOP);
}

static inline uint8_t MOTOR_STATUS_DISABLED_Signal(void){
    return FocApp_GetFaultFlag(FOC_SIGNAL_DISABLED);
}


/* ================= 状态机任务处理(执行) =================
 * 24 个 Loop 同样在 1 kHz 中断里执行，只允许做寄存器级操作。
 * FocApp_Shutdown() 是两次寄存器写（撤 MOE + 拉低 EN_GATE），幂等，可重复调用。
 */
static inline void MOTOR_STATUS_STANDBY_Loop(void){
    /* 待机态下库不再更新 CCR，上一帧的电压矢量会被冻结住持续加在电机上，
       必须硬关断，否则堵转直流会烧绕组 */
    FocApp_Shutdown();
}

static inline void MOTOR_STATUS_UNINITIALIZED_Loop(void){
    /* 此时 Sguan_Start_Tick() 正在主循环里启动外设，不能关断 */
}

static inline void MOTOR_STATUS_INITIALIZING_Loop(void){
    /* 参数加载中，PWM 已是零矢量，无需干预 */
}

static inline void MOTOR_STATUS_CALIBRATING_Loop(void){
    /* 电流零偏与零位对齐依赖 PWM 输出，绝对不能在此关断 */
}

static inline void MOTOR_STATUS_IDLE_Loop(void){
    /* 已使能、零指令，库每周期输出零矢量 */
}

static inline void MOTOR_STATUS_TORQUE_INCREASING_Loop(void){
}

static inline void MOTOR_STATUS_TORQUE_DECREASING_Loop(void){
}

static inline void MOTOR_STATUS_TORQUE_CONTROL_Loop(void){
}

static inline void MOTOR_STATUS_ACCELERATING_Loop(void){
}

static inline void MOTOR_STATUS_DECELERATING_Loop(void){
}

static inline void MOTOR_STATUS_CONST_SPEED_Loop(void){
}

static inline void MOTOR_STATUS_POSITION_INCREASING_Loop(void){
}

static inline void MOTOR_STATUS_POSITION_DECREASING_Loop(void){
}

static inline void MOTOR_STATUS_POSITION_HOLD_Loop(void){
}

static inline void MOTOR_STATUS_OVERVOLTAGE_Loop(void){
    /* 不可达：User_VBUS_DataGet() 返回哨兵值，库跳过母线电压判定 */
}

static inline void MOTOR_STATUS_UNDERVOLTAGE_Loop(void){
    /* 不可达，同上 */
}

static inline void MOTOR_STATUS_OVERTEMPERATURE_Loop(void){
    /* 不可达：User_Temperature_DataGet() 返回哨兵值 */
}

static inline void MOTOR_STATUS_UNDERTEMPERATURE_Loop(void){
    /* 不可达，同上 */
}

static inline void MOTOR_STATUS_OVERCURRENT_Loop(void){
    /* 过流在库里是“稳态”而非锁定态，DQcur_watchdog_limit*10 个周期后自动回 IDLE。
       目标值已由 FocApp_SetTarget() 按 Qcur_MAX 限幅，这里出现的多是阶跃过冲，
       不做硬关断，避免正常调试被误停。真正的瞬时过流由 DRV8301 的 OC 锁存兜底 */
}

static inline void MOTOR_STATUS_ENCODER_ERROR_Loop(void){
    /* 角度失效后电角度无从计算，继续输出等于失控 */
    FocApp_Shutdown();
}

static inline void MOTOR_STATUS_SENSOR_ERROR_Loop(void){
    FocApp_Shutdown();
}

static inline void MOTOR_STATUS_PWM_CALC_FAULT_Loop(void){
    /* 高频环连续计算超时，控制量已不可信 */
    FocApp_Shutdown();
}

static inline void MOTOR_STATUS_EMERGENCY_STOP_Loop(void){
    FocApp_Shutdown();
}

static inline void MOTOR_STATUS_DISABLED_Loop(void){
    FocApp_Shutdown();
}


#endif // USERDATA_STATUS_H
