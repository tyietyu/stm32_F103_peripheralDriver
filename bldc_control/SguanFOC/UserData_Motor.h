#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 用户自己的CODE BEGIN Includes */
#include "main.h"       /* PWM_PERIOD / PWM_FREQ，与 tim.c 共用同一套定义 */
#include "drv8301.h"    /* 采样电阻与运放增益，与 BSP 驱动共用同一套标定 */
/* 用户自己的CODE END Includes */
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码) */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 1.mode选择电机的运行模式
    // 由 FocApp_Init() / FocApp_SetMode() 管理，此处不写，否则每次启动都会被刷回
    // 2.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10; // (uint8_t)PWM错误限幅
    // 3.identify电机参数辨识结果
    /* 一期全填 0 关闭前馈解耦：Ud_in/Uq_in 是以 VBUS 为满刻度的调制指令，
       而前馈项 we*Lq*Iq、we*Flux 是真实伏特，两者量纲差 sqrt(3) 倍，
       直接相加会偏小。填 0 后电流环纯 PI 闭环，同时规避 FW_MTPA 的除零。
       二期若要开前馈，填“真实值的 sqrt(3) 倍”，不需要改库。 */
    Sguan.identify.Ld = 0.0f;           // (float)D轴电感
    Sguan.identify.Lq = 0.0f;           // (float)Q轴电感
    Sguan.identify.Ls = 0.0f;           // (float)相线电感
    Sguan.identify.Rs = 0.0f;           // (float)相线电阻
    Sguan.identify.Flux = 0.0f;         // (float)磁链
    // 4.motor电机参数辨识
    Sguan.motor.Poles = 11;             // (uint8_t)极对极数，实测确认
    Sguan.motor.VBUS = 24.0f;           // (float)母线电压

    /* 以下四个方向量必须在台架上按方案 6.5 的顺序逐项确认，不能猜 */
    Sguan.motor.Motor_Dir = 1;          // (int8_t)电机方向，-1 时库交换 U/V 相序
    Sguan.motor.PWM_Dir = 1;            // (int8_t)PWM1模式 + OCPolarity HIGH：CCR↑ -> 高边导通↑
    Sguan.motor.Duty = PWM_PERIOD;      // (uint16_t)SVPWM_Tick 用 Du*Duty 得 CCR，须等于 ARR

    Sguan.motor.Encoder_Dir = 1;        // (int8_t)编码器方向，台架待定

    Sguan.motor.Current_Dir0 = 1;       // (int8_t)相线电流方向，台架待定
    Sguan.motor.Current_Dir1 = 1;       // (int8_t)相线电流方向，台架待定
    /* AB 采样：IC_FB 走外接 LM2904，与 DRV8301 内部放大器存在增益/失调失配，
       不让它进控制环，只在诊断中与 -(Ia+Ib) 的重构值比对 */
    Sguan.motor.Current_Num = 0;        // (uint8_t)电流通道0->AB相，1->AC相，2->BC相
    Sguan.motor.ADC_Precision = 4096;   // (uint32_t)12 bit
    Sguan.motor.Amplifier = (float)DRV8301_SHUNT_GAIN;      // (float)运放增益 10 V/V
    Sguan.motor.MCU_Voltage = 3.3f;     // (float)ADC 基准
    Sguan.motor.Sampling_Rs = DRV8301_SHUNT_RESISTANCE;     // (float)采样电阻 10 mΩ
    /* Final_Gain = 3.3/(4096*10*0.010) = 8.057 mA/LSB，满量程 ±16.5 A */
    // 5.电机安全设计
    /* VBUS/Temp 两类保护已由 User_VBUS/Temperature_DataGet() 返回 -9999.0f 关闭，
       下面的阈值不生效；但 watchdog_limit 会被 Status_RUN_Loop 用作取模的除数，
       必须保持非零 */
    Sguan.safe.VBUS_MAX = 28.0f;        // (float)母线电压值波动MAX阈值(不生效)
    Sguan.safe.VBUS_MIM = 20.0f;        // (float)母线电压值波动MIN阈值(不生效)
    Sguan.safe.VBUS_watchdog_limit = 1000;

    Sguan.safe.Temp_MAX = 60.0f;        // (float)驱动器允许最大温度(不生效)
    Sguan.safe.Temp_MIN = -20.0f;       // (float)驱动器允许最小温度(不生效)
    Sguan.safe.Temp_watchdog_limit = 1000;

    /* HT3510 额定电流 0.53 A、堵转电流 0.80 A。库默认的 60 A、方案里的 10 A 对本
       电机都形同虚设，取 1.5 A 作为故障阈值（约 1.9 倍堵转电流）。
       给定值另在 FocApp_SetTarget() 里限到 1.0 A，命令上限低于故障阈值 */
    Sguan.safe.Dcur_MAX = 1.5f;         // (float)电机最大电流D轴限制
    Sguan.safe.Qcur_MAX = 1.5f;         // (float)电机最大电流Q轴限制
    Sguan.safe.DQcur_watchdog_limit = 1000;

    Sguan.safe.DISABLED_watchdog_limit = 1000;
    // 6.系统定时中断周期设计
    /* 168e6/(2*5600) = 15.000 kHz 精确整除 -> 66.667 us */
    Sguan.PMSM_RUN_T = 1.0f/(float)PWM_FREQ;
}


#endif // USERDATA_MOTOR_H
