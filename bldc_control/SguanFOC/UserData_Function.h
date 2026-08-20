#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
#include "main.h"
#include "foc_app.h"
/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    /* DRV8301 -> AS5600 -> TIM2 -> TIM1 六路 PWM + CH4 -> ADC 注入组，顺序不可换 */
    FocApp_HwStart();
}

static inline void User_Delay(unsigned int ms){
    /* 只在 while(1) 上下文的上电标定流程里被调用，允许阻塞 */
    HAL_Delay(ms);
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    /* Current_Num = 0（AB采样）：CH0->IA_FB(注入rank1)，CH1->IB_FB(rank2)
       IC 由库按 -(Ia+Ib) 重构，不用外接 LM2904 那一路进控制环 */
    return (signed int)FocApp_GetCurrentRaw((uint8_t)Current_CH);
}

static inline float User_Encoder_ReadRad(void){
    /* 15 kHz ISR 内调用，只能返回 AS5600 的缓存单圈机械角 [0,2pi) */
    return FocApp_GetEncoderRad();
}

static inline void User_PwmDuty_Set(unsigned short int Duty_u,
                                unsigned short int Duty_v,
                                unsigned short int Duty_w){
    /* 死区由 TIM1 的 DTG 硬件生成（84 -> 500 ns），此处不做死区补偿 */
    FocApp_SetPwmDuty((uint16_t)Duty_u,(uint16_t)Duty_v,(uint16_t)Duty_w);
}

static inline float User_VBUS_DataGet(void){
    /* 本板只有 VA/VB/VC_FB 三相电压，采样时刻下管全开、相电压接近 0，无法反推母线。
       返回哨兵值让库跳过过/欠压保护，SVPWM 归一化用 motor.VBUS 固定 24 V */
    return -9999.0f;
}

static inline float User_Temperature_DataGet(void){
    /* 本板无温度通道，返回哨兵值让库跳过过/低温保护 */
    return -9999.0f;
}


#endif // USERDATA_FUNCTION_H
