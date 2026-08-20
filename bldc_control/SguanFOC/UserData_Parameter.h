#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

/* ==================== 电流环整定依据 ====================
 * Sguan_GeneratePWM_Loop() 送进 SVPWM 的是 Ud_in/VBUS、Uq_in/VBUS，而 Overmod()
 * 把矢量幅值限制在 1。代入 SVPWM 验算 m=1、theta=0：占空比 (0.933,0.067,0.067)，
 * 均值 0.356，相电压 = VBUS*(0.933-0.356) = VBUS/sqrt(3)。
 * 即：实际相电压幅值 = |U_in| / sqrt(3)，等效对象增益被缩小 sqrt(3) 倍，
 * 整定公式相应放大：
 *      Kp = sqrt(3) * Ls * wc
 *      Ki = sqrt(3) * Rs * wc     (库的 Ki 是连续域增益，PID_Init 内 I_num = T*Ki)
 *
 * ★ 换电机只需改下面两行 Rs/Ls，Kp/Ki 由编译期常量折叠算出 ★
 * 当前值来自 HT3510 铭牌：相间电阻 8.53 Ω、相间电感 1.90 mH。
 * 星接绕组的相间值是两相串联，折算相值需除以 2。
 */
#define MOTOR_PHASE_RS      (8.53f / 2.0f)      /* [Ohm] 4.265，相间 8.53 折算 */
#define MOTOR_PHASE_LS      (0.00190f / 2.0f)   /* [H]   0.00095，相间 1.90 mH 折算 */
/* 15 kHz 载波的 1/10，2*pi*1500。电气极点 Rs/Ls = 4489 rad/s，取 2.1 倍带宽 */
#define CURRENT_LOOP_WC     9424.778f
#define VALUE_SQRT3         1.7320508f

/* Kp = 15.508，Ki = 69623（连续域），T*Ki = 4.64 */
#define CURRENT_LOOP_KP     (VALUE_SQRT3 * MOTOR_PHASE_LS * CURRENT_LOOP_WC)
#define CURRENT_LOOP_KI     (VALUE_SQRT3 * MOTOR_PHASE_RS * CURRENT_LOOP_WC)
/* Ud_in 的物理上限就是 VBUS。取 0.5*VBUS：单轴 0.5 -> 矢量幅值最大 0.707 < 1，
   不进过调制区，留足余量。对应相电压 6.93 V，恰好覆盖 965 rpm 的反电动势 */
#define CURRENT_LOOP_OUTMAX 12.0f
/* ==================== 速度环整定依据 ====================
 * 被控对象：J = 76 g.cm2 = 7.6e-6 kg.m2，Kt = 额定扭矩/额定电流 = 0.11/0.53
 *          = 0.2075 N.m/A（堵转 0.16/0.80 = 0.20 交叉验证一致）。
 * 反馈通道最慢的一级是 bpf.Encoder 的 48 Hz（300 rad/s），穿越频率必须留够相位
 * 裕度，取 80 rad/s（12.7 Hz）：
 *      Kp = wc_v * J / Kt = 80 * 7.6e-6 / 0.2075 = 0.0029
 *      Ki = Kp * wc_v/5   （PI 零点放在穿越频率的 1/5，约 16 rad/s）
 * 库默认的 Kp = 0.06 对应穿越 1639 rad/s，远高于反馈滤波极点，本电机上会振荡，
 * 不能沿用。以下为起调值，阶段 6 台架整定。
 * 惯量只算了转子本体，带负载后可上调。
 */
#define VELOCITY_KP         0.003f
#define VELOCITY_KI         0.05f
/* 速度环输出即 Iq 给定。额定电流 0.53 A、堵转电流 0.80 A，取 1.0 A 允许短时加速 */
#define VELOCITY_OUTMAX     1.0f
/* 位置环输出即速度给定。电机最大转速 965 rpm = 101 rad/s，取整留余量 */
#define POSITION_OUTMAX     100.0f

static inline void User_ParameterSet(void){
    // 1.bpf低通滤波器设计
    /* T 从 20 kHz 的 50 us 变成 15 kHz 的 66.7 us，Wc 保持默认值：
       双线性预畸后实际截止约 3.9 kHz，仍远低于 7.5 kHz 的 Nyquist，极点模 0.41 稳定 */
    Sguan.bpf.CurrentD.Wc = 31415.96f;              // 电机D轴电流滤波->截止频率
    Sguan.bpf.CurrentQ.Wc = 31415.96f;              // 电机Q轴电流滤波->截止频率
    Sguan.bpf.Encoder.Wc = 300.0f;                  // 速度信号滤波->截止约 48 Hz
    // 2.pid闭环控制系统设计
    /* IntMax/IntMin 取与 OutMax/OutMin 同值：本库 Output = Kp*e + Io + Do，
       积分项超过输出限幅后只会积攒无法输出的量，库默认的 150 相当于对 ±12 的
       输出几乎没有抗饱和 */
    Sguan.control.Current_D.Wc = 100.0f;            // 微分环节一阶低通，Kd=0 时不参与运算
    Sguan.control.Current_D.Kp = CURRENT_LOOP_KP;   // = 15.508
    Sguan.control.Current_D.Ki = CURRENT_LOOP_KI;   // = 69623
    Sguan.control.Current_D.Kd = 0.0f;
    Sguan.control.Current_D.OutMax = CURRENT_LOOP_OUTMAX;
    Sguan.control.Current_D.OutMin = -CURRENT_LOOP_OUTMAX;
    Sguan.control.Current_D.IntMax = CURRENT_LOOP_OUTMAX;
    Sguan.control.Current_D.IntMin = -CURRENT_LOOP_OUTMAX;
    /* =========================== 分割线 ========================== */
    /* SPMSM 的 Ld = Lq，D/Q 两轴用同一组增益 */
    Sguan.control.Current_Q.Wc = 100.0f;
    Sguan.control.Current_Q.Kp = CURRENT_LOOP_KP;
    Sguan.control.Current_Q.Ki = CURRENT_LOOP_KI;
    Sguan.control.Current_Q.Kd = 0.0f;
    Sguan.control.Current_Q.OutMax = CURRENT_LOOP_OUTMAX;
    Sguan.control.Current_Q.OutMin = -CURRENT_LOOP_OUTMAX;
    Sguan.control.Current_Q.IntMax = CURRENT_LOOP_OUTMAX;
    Sguan.control.Current_Q.IntMin = -CURRENT_LOOP_OUTMAX;

    #if Open_PI_Control
    /* 一期用参数直观、易整定的 PI 建立速度环基线，增益按上面的惯量/Kt 推算 */
    Sguan.control.Velocity.Wc = 100.0f;
    Sguan.control.Velocity.Kp = VELOCITY_KP;
    Sguan.control.Velocity.Ki = VELOCITY_KI;
    Sguan.control.Velocity.Kd = 0.0f;
    Sguan.control.Velocity.OutMax = VELOCITY_OUTMAX;
    Sguan.control.Velocity.OutMin = -VELOCITY_OUTMAX;
    Sguan.control.Velocity.IntMax = VELOCITY_OUTMAX;
    Sguan.control.Velocity.IntMin = -VELOCITY_OUTMAX;
    #else // Open_PI_Control
    Sguan.control.Speed.r = 60.0f;                  // LADRC线自抗扰“速度环”跟踪系数
    Sguan.control.Speed.b0 = 1200000.0f;            // LADRC线自抗扰“速度环”补偿系数
    Sguan.control.Speed.wc = 210.0f;                // LADRC线自抗扰“速度环”控制器带宽
    Sguan.control.Speed.OutMax = VELOCITY_OUTMAX;   // LADRC线自抗扰“速度环”输出上限
    Sguan.control.Speed.OutMin = -VELOCITY_OUTMAX;  // LADRC线自抗扰“速度环”输出下限
    #endif // Open_PI_Control

    /* 纯 P，定位演示够用；Ki=0 时 IntMax 只作兜底 */
    Sguan.control.Position.Wc = 100.0f;
    Sguan.control.Position.Kp = 8.0f;
    Sguan.control.Position.Ki = 0.0f;
    Sguan.control.Position.Kd = 0.0f;
    Sguan.control.Position.OutMax = POSITION_OUTMAX;
    Sguan.control.Position.OutMin = -POSITION_OUTMAX;
    Sguan.control.Position.IntMax = POSITION_OUTMAX;
    Sguan.control.Position.IntMin = -POSITION_OUTMAX;

    /* 电流环 15 kHz -> 速度环 3 kHz -> 位置环 600 Hz */
    Sguan.control.Response = 5;                     // (uint8_t)响应带宽倍数
    // 3.pll锁相环跟踪系统
    /* wn = sqrt(Ki) = 458 rad/s (73 Hz)，zeta = Kp/(2*sqrt(Ki)) = 0.71，
       远低于 4 kHz 的角度更新率，起调值，按方案 6.5 在台架上复核 */
    Sguan.encoder.pll.Kp = 650.0f;                  // 锁相环比例项增益
    Sguan.encoder.pll.Ki = 210000.0f;               // 锁相环积分项增益
}


#endif // USERDATA_PARAMETER_H
