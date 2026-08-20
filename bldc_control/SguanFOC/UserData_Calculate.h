#ifndef __USERDATA_CALCULATE_H
#define __USERDATA_CALCULATE_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0或1决定“闭环控制系统”是否使用PI控制(默认开启)
 * @reminder: 0->电流环“PI控制”，转速环“LADRC”，位置环“PD控制”
 * @reminder: 1->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @return {*}
 */
#define Open_PI_Control 1

/**
 * @description: 宏定义0或1决定“FW弱磁控制”是否开启(默认开启)
 * @reminder: 0->不开启IPMSM的弱磁控制
 * @reminder: 1->开启基于MTPA的弱磁控制
 * @return {*}
 */
#define Open_FW_Calculate 0

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Printf_Send)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Printf_Debug 0

/**
 * @description: 上电零位对齐时施加的 d 轴电压占 VBUS 的比例
 * @reminder: SVPWM 按 VBUS 归一化，实际相电压 = ratio * VBUS / sqrt(3)
 * @reminder: 静止对齐电流 I ≈ ratio * VBUS / (sqrt(3) * Rs)
 * @reminder: HT3510 相电阻 4.265 Ω（相间 8.53/2），Kt ≈ 0.2075 N.m/A
 * @reminder: 0.15 -> 相电压 2.08 V、电流 0.487 A（约额定 0.53 A）、力矩 0.10 N.m
 *            比例太小转子克服不了齿槽力矩，Pos_offset 会记错，闭环必然失败
 * @reminder: 库原值 0.3 对本电机是 0.975 A（1.2 倍堵转电流），换电机务必按上式复核
 * @return {*}
 */
#define Align_Voltage_Ratio 0.15f


#endif // USERDATA_CALCULATE_H
