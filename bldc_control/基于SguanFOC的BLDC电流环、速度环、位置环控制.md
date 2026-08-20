# 基于SguanFOC的BLDC电流环、速度环、位置环控制

## 1.目标：实现BLDC电流环、速度环、位置环控制

## 2.已确认硬件的实现：

* 电路原理图文件去读取README.md内容。
* 使用SguanFOC驱动库
* MCU采集U、V、W三相电压使用的是ADC的规则转换组 ，在原理图中的网络标签是：VA_FB,VB_FB,VC_FB,对应ADC1的channel 0,1,2
* MCU采集U、V、W三相电流使用的是ADC的注入转换组，在原理图中的网络标签是：IA_FB,IB_FB,IC_FB,其中IA_FB,IB_FB是DRV8301内部的运放输出，IC_FB是通过LM2904差分运放输出，对应 ADC1的channel 3,4,5
* 母线电压是24V
* PWM驱动使用的是TIM1的channel 1，2，3互补通道，channel 4 是PWM Generation No Output,用于触发adc采样

---

## 3.软件方案架构

### 3.1 起点判定

本方案不是"从零实现 FOC"。现状盘点：

* `SguanFOC/` 是一套 16 文件、约 2740 行的**完整** FOC 框架。Clarke/Park/iPark、SVPWM、双线性离散 PID、二阶 LADRC、角度跟踪 PLL、二阶巴特沃斯滤波、24 态状态机、JustFloat 上位机协议，以及四种控制模式（开环 / 电流单环 / 速度-电流双环 / 位置-速度-电流三环）**全部已实现**，且 9 个 `.c` 已加入 MDK 编译（`MDK-ARM/bldc_control.uvprojx`）。
* 外设已由 CubeMX 配好且配置正确：TIM1 中心对齐 ARR=5600 @168 MHz、死区 500 ns、CH4 产生 OC4REF 经 TRGO 触发 ADC1 注入组。
* **但整条实时链路没有接通**：`Core/Src/main.c` 与 `stm32f4xx_it.c` 中对 SguanFOC 零调用；TIM1 的 MOE 从未使能、ADC 从未启动、TIM2 从未启动、`DRV8301_Init()` 从未调用；7 个 `UserData_*.h` 扩展点全部是空注释模板。

所以本方案要解决的是**接线**：把算法层、驱动层、外设配置接成一条可运行的实时链路，并处理接线过程中暴露的量化约束与库缺陷。

### 3.2 分层与边界

```
┌────────────────────────────────────────────────────────────┐
│ Core/Src/main.c          启动序列、while(1) 调度、目标下发      │  可改
├────────────────────────────────────────────────────────────┤
│ Core/Src/stm32f4xx_it.c  中断入口，只做转发，不含业务逻辑        │  可改
├────────────────────────────────────────────────────────────┤
│ Core/Src/foc_app.c/.h    ★新建★ 应用适配层                    │  新建
│   全局对象 / ISR 处理 / 诊断轮询 / 供库回调的原语                │
├────────────────────────────────────────────────────────────┤
│ SguanFOC/UserData_*.h    库的既定扩展点，只写一行转发            │  仅填空
├────────────────────────────────────────────────────────────┤
│ SguanFOC/*.c             算法层                               │  只读*
├────────────────────────────────────────────────────────────┤
│ BSP_Driver/              as5600 / drv8301 / delay / log      │  只读
├────────────────────────────────────────────────────────────┤
│ Drivers/                 STM32 HAL + CMSIS                   │  只读
└────────────────────────────────────────────────────────────┘
        * 例外：3 处必要缺陷修复，见第 8 节，改动登记在 docs/sguanfoc-patch.md
```

**为什么业务代码不直接写进 `UserData_*.h`**：这 7 个文件里全是 `static inline` 函数，被多个编译单元包含。把业务逻辑写进去会导致代码重复展开、无法设断点单步、且模糊了"库只读"的边界。正确做法是每个钩子只写一行转发：

```c
static inline float User_Encoder_ReadRad(void){
    return FocApp_GetEncoderRad();
}
```

### 3.3 库的四个入口

| 函数                                         | 调用位置             | 频率       |
| -------------------------------------------- | -------------------- | ---------- |
| `SguanFOC_High_Loop(void)`                 | ADC 注入组 JEOC 中断 | 15 kHz     |
| `SguanFOC_Low_Loop(void)`                  | TIM2 中断内 /4 分频  | 1 kHz      |
| `SguanFOC_Printf_Loop(uint8_t*, uint16_t)` | USART1 接收完成中断  | 事件驱动   |
| `SguanFOC_main_Loop(void)`                 | `while(1)`         | 最低优先级 |

---

## 4.中断与任务时序

### 4.1 频率与优先级分配

NVIC 优先级分组为 `NVIC_PRIORITYGROUP_4`（4 位全抢占）。

| 中断源                             | 频率   | 抢占优先级  | 职责                                                                        |
| ---------------------------------- | ------ | ----------- | --------------------------------------------------------------------------- |
| `ADC_IRQn`（注入组 JEOC）        | 15 kHz | **0** | 读 JDR1~3 存入缓存 →`SguanFOC_High_Loop()`                               |
| `TIM2_IRQn`                      | 4 kHz  | 2           | 每次调`AS5600_UpdateStart()`；内部 /4 → 1 kHz 调 `SguanFOC_Low_Loop()` |
| `DMA1_Stream0/6`、`I2C1_EV/ER` | 事件   | 3           | HAL 转发 →`AS5600_OnRxComplete()` / `OnRxError()`                      |
| `USART1_IRQn` + TX DMA           | 事件   | 4           | RX →`SguanFOC_Printf_Loop()`；TX 完成清忙标志                            |
| `SysTick`                        | 1 kHz  | 15          | `HAL_IncTick()`                                                           |
| `while(1)`                       | —     | —          | `SguanFOC_main_Loop()` + 10 Hz 诊断轮询                                   |

### 4.2 单个 PWM 周期内的时序

TIM1：`CK_CNT = 168 MHz`（PSC=0），中心对齐，`ARR = 5600`
→ 周期 = 2×5600 / 168e6 = **66.67 µs**，**f_PWM = 15.000 kHz**（精确整除）
死区 `DTG = 84`，`t_DTS = 1/168 MHz` → **500 ns**
`CCR4 = ARR − 168`，PWM2 模式 → OC4REF 上升沿在计数峰值前 168 tick = **1.00 µs**

以计数器峰值（三相下管导通中点）为 t = 0：

| 时刻                | 事件                                                   |
| ------------------- | ------------------------------------------------------ |
| −1.00 µs          | 向上计数越过 CCR4=5432，OC4REF↑ → TRGO → 注入组启动 |
| −1.00 ~ −0.86 µs | rank1 采样 IA_FB（CH3）                                |
| −0.29 ~ −0.14 µs | rank2 采样 IB_FB（CH4）                                |
| +0.43 ~ +0.57 µs   | rank3 采样 IC_FB（CH5，仅诊断）                        |
| +1.14 µs           | JEOC →`ADC_IRQHandler`                              |
| ISR 内              | 读 JDR →`SguanFOC_High_Loop()` → 写 CCR1/2/3       |
| 下一次 UEV          | CCR 预装载生效（`tim.c:110-113` 已开 OCxPreload）    |

ADC 时钟 = PCLK2/4 = 21 MHz，注入采样时间 3 cycles → 单通道 (3+12)/21 MHz = 0.714 µs，三通道共 2.14 µs。三个采样点全部落在下管导通窗口内。

**CPU 预算**：66.67 µs 周期内 `SguanFOC_High_Loop()` 必须 < 20 µs（30%），工程当前优化等级为 -O2.

### 4.3 三条硬约束

1. **`User_Encoder_ReadRad()` 在 15 kHz ISR 内被调用，必须非阻塞。** 只能返回 `AS5600_GetOnceAngle()` 的缓存值。该函数返回单个 `float`（`sensor->mechanical_angle`），Cortex-M4 上 32 位对齐读写是原子的，被高优先级 ISR 抢占不会读到撕裂值。
2. **编码器刷新必须在中断里，不能放主循环。** 上电标定阶段库会调用 `User_Delay(1000)` 阻塞主循环（`SguanFOC.c:919`），而 `Offset_EncoderRead()` 需要此时的新鲜角度。因此 `AS5600_UpdateStart()` 挂在 TIM2 4 kHz 中断。
   AS5600 走硬件 I2C 400 kHz，单次读 RAW_ANGLE 约 6 字节 × 9 bit / 400 kHz ≈ **135 µs**，4 kHz（250 µs）留 46% 余量。
   `AS5600_UpdateStart()` 内的 5 ms 超时是**时限判定而非忙等**（`as5600.c:168-174`），DeInit/Init 仅数十微秒，放中断安全。
3. **`MOTOR_STATUS_*_Signal()` 六个钩子在 1 kHz 中断里被调，只能读全局 `volatile` 标志。** `DRV8301_ReadStatus()` 是阻塞 SPI（328 kHz，2×16 bit ≈ 100 µs），`AS5600_CheckStatus()` 是阻塞 I2C 且与 DMA 读互斥（返回 −2 表示总线忙）。二者放 `while(1)` 里 10 Hz 轮询，只负责置标志。

---

## 5.数据流与接口

### 5.1 `foc_app.h` 对外接口

```c
/* ---- 生命周期与调度 ---- */
void     FocApp_Init(void);          /* main() 中调用：装配全局对象，写 Sguan.status = 0x01 触发库启动 */
void     FocApp_OnAdcInjected(void); /* ADC_IRQHandler 转发：缓存 JDR → SguanFOC_High_Loop() */
void     FocApp_OnTim2Tick(void);    /* TIM2_IRQHandler 转发：AS5600_UpdateStart() + /4 → SguanFOC_Low_Loop() */
void     FocApp_DiagnoseLoop(void);  /* while(1) 中 10 Hz：DRV8301/AS5600 状态轮询，置故障标志 */
void     FocApp_Shutdown(void);      /* 急停：关 MOE + DRV8301_Shutdown() */

/* ---- 供 UserData_*.h 一行转发的原语 ---- */
void     FocApp_HwStart(void);                                  /* ← User_InitialInit() */
int32_t  FocApp_GetCurrentRaw(uint8_t ch);                      /* ← User_ReadADC_Raw()，ch: 0=IA 1=IB */
float    FocApp_GetEncoderRad(void);                            /* ← User_Encoder_ReadRad()，返回 [0,2π) */
void     FocApp_SetPwmDuty(uint16_t du, uint16_t dv, uint16_t dw); /* ← User_PwmDuty_Set() */
void     FocApp_Send(uint8_t *buf, uint16_t len);               /* ← User_CorrespondSet()，UART TX DMA，忙则丢帧 */
uint8_t  FocApp_GetFaultFlag(uint8_t which);                    /* ← MOTOR_STATUS_*_Signal() */
```

对应的库侧空壳原型分别在 `UserData_Function.h:9,26,31,51,58,70,78`、`UserData_Correspond.h:6`、`UserData_Status.h:10-50`。

### 5.2 七个扩展点各自填什么

| 文件                       | 内容                                                                                                                                                                                                     |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `UserData_Function.h`    | 7 个钩子转发到`FocApp_*`。`User_Delay` → `HAL_Delay`；`User_VBUS_DataGet` / `User_Temperature_DataGet` 保持返回 `-9999.0f`（哨兵值，库据此跳过对应保护，见 `SguanFOC.c:501-506,585,596`） |
| `UserData_Motor.h`       | 板级与电机参数，按 6.1 表格逐项改写                                                                                                                                                                      |
| `UserData_Parameter.h`   | PID / LADRC / PLL / 滤波器参数，按 6.2 重算                                                                                                                                                              |
| `UserData_Calculate.h`   | `Open_PI_Control = 1`（速度环用 PI）、`Open_FW_Calculate = 0`（关弱磁）、`Printf_Debug = 0`                                                                                                        |
| `UserData_Status.h`      | 6 个`*_Signal()` 转发到 `FocApp_GetFaultFlag()`；24 个 `*_Loop()` 一期只在急停/失能/各故障态里调 `FocApp_Shutdown()`，其余留空                                                                   |
| `UserData_UserControl.h` | `User_UserControl()` 读 `foc_app` 的目标值；`User_AO/BO/CO_Adjust()` 按库注释示例接上位机调参；`User_UserTX()` 沿用现有 12 通道映射                                                              |
| `UserData_Correspond.h`  | 一行转发到`FocApp_Send()`                                                                                                                                                                              |

### 5.3 电流通道映射

库只读两路电流（`User_ReadADC_Raw(0)` / `(1)`），第三相由 `I2 = −(I0+I1)` 重构（`SguanFOC.c:227`）。

取 **`Current_Num = 0`（AB 采样）**：CH0 → IA_FB（注入 rank1），CH1 → IB_FB（注入 rank2），IC 由重构得出。

理由：IC_FB 走外接 LM2904（R45=1k / R47=10k，增益 10 V/V），与 DRV8301 内部放大器不是同一条通道，存在增益与失调失配（README.md:40-47）。让它不进控制环，只在诊断中与重构值比对，正好用于验证采样链路。

### 5.4 上位机波形通道

沿用 `UserData_UserControl.h:69-80` 已定义的映射，无需改动：

| CH | 变量                   | CH | 变量                   |
| -- | ---------------------- | -- | ---------------------- |
| 0  | `Sguan.status`       | 6  | `foc.Target_Iq`      |
| 1  | `encoder.Real_Speed` | 7  | `foc.Uq_in`          |
| 2  | `foc.Target_Speed`   | 8  | `current.Real_Ia`    |
| 3  | `current.Real_Id`    | 9  | `encoder.Real_Pos`   |
| 4  | `current.Real_Iq`    | 10 | `encoder.Pos_offset` |
| 5  | `foc.Target_Id`      | 11 | `Sguan.mode`         |

一帧 12×4 + 4 = 52 字节。当前 USART1 是 **921600（一帧 0.56 ms）并加了 USART1_TX DMA。**

---

## 6.参数标定与配置

### 6.1 必须改的库默认值

`UserData_Motor.h` 的默认值来自一块 12 V / 5 mΩ / 20 kHz 的板子，与本板全面不符。

| 项                                 | 库默认                 | 本板取值           | 依据                                                                      |
| ---------------------------------- | ---------------------- | ------------------ | ------------------------------------------------------------------------- |
| `PMSM_RUN_T`                     | `0.00005f`（20 kHz） | `0.00006666667f` | 168e6/(2×5600) = 15.000 kHz，`tim.c:47-49`                             |
| `Duty`                           | `4249`               | `5600`           | `SVPWM_Tick` 用 `Du × Duty` 得 CCR（`SguanFOC.c:809`），须等于 ARR |
| `VBUS`                           | `12.0f`              | `24.0f`          | 第 2 节已确认                                                             |
| `Sampling_Rs`                    | `0.005f`             | `0.010f`         | `drv8301.h:29`                                                          |
| `Amplifier`                      | `10.0f`              | `10.0f`          | 内部增益 10 V/V，C 相外部亦为 10 V/V                                      |
| `ADC_Precision`                  | `4096`               | `4096`           | 12 bit，`adc.c:47`                                                      |
| `MCU_Voltage`                    | `3.3f`               | `3.3f`           | —                                                                        |
| `Current_Num`                    | `1`（AC）            | `0`（AB）        | 见 5.3                                                                    |
| `Poles`                          | `7`                  | 11                 | 实测确认是11极对                                                          |
| `PWM_Dir`                        | `-1`                 | `+1`（台架复核） | PWM1 模式 +`OCPolarity HIGH`，CCR↑ → 高边导通↑                       |
| `Dcur_MAX` / `Qcur_MAX`        | `60.0f`              | `10.0f`          | 硬件量程仅 ±16.5 A，60 A 的保护形同虚设                                  |
| `VBUS_MAX/MIM`、`Temp_MAX/MIN` | 12 V 一套              | 不适用             | `User_VBUS/Temperature_DataGet()` 返回 `-9999.0f`，库跳过这两类保护   |

采样增益校验：

```
Final_Gain = MCU_Voltage / (ADC_Precision × Amplifier × Sampling_Rs)
           = 3.3 / (4096 × 10 × 0.010) = 8.057 mA/LSB
满量程 = 8.057 mA × 2048 = ±16.5 A          （与 README.md:44 一致）
```

### 6.2 电压量纲：SVPWM 归一化引入 √3 因子

这是本方案最容易踩错的一点，必须先说清。

`Sguan_GeneratePWM_Loop()` 送进 SVPWM 的是 `Ud_in / VBUS`、`Uq_in / VBUS`（`SguanFOC.c:841-842`），而 `Overmod()` 把矢量幅值 `m` 限制在 1（`Sguan_math.c:167`）。

代入 `SVPWM()` 验算 `m = 1、θ = 0` 的情形：`t_m = 0`、`t_n = 0.866`、`t_0 = 0.134`，sector 6，得
`(d_u, d_v, d_w) = (0.933, 0.067, 0.067)`，均值 0.356，
相电压 `v_a = VBUS × (0.933 − 0.356) = 0.577 × VBUS = VBUS/√3`。

也就是说：

```
实际相电压幅值 = |U_in| / √3       （m = 1 时对应 VBUS/√3 = 13.86 V）
```

`Ud_in` / `Uq_in` **不是真实伏特**，而是"以 VBUS 为满刻度的调制指令"。两个后果：

* **电流环 PI 的等效对象增益被缩小 √3 倍**，整定公式需相应放大：

  ```
  Kp = √3 · Ls · ωc
  Ki = √3 · Rs · ωc          （库的 Ki 是连续域增益，PID_Init 内 I_num = T × Ki）
  ```
* **前馈解耦项量纲不一致。** `Control_Current_SINGLE` 的前馈用 `ω_e·Lq·Iq`、`ω_e·Ld·Id + ω_e·Flux`（`SguanFOC.c:299-301`），这些是真实伏特，直接加到 `Ud_in/Uq_in` 上会偏小 √3 倍。

  **一期处理：把 `identify.Ld / Lq / Ls / Flux` 全部填 `0.0f`，关闭前馈解耦**，纯靠 PI 闭环。理由：演示级带宽下前馈不是必需，且能规避量纲问题与 `FW_MTPA_Loop` 的除零风险。二期若要开前馈，填**真实值的 √3 倍**即可，不需改库。

**限幅**：`Ud_in` 的物理上限就是 `VBUS`（超出被 `Overmod` 等比缩放）。一期取

```
Current_D.OutMax / Current_Q.OutMax = ±12.0f       （= 0.5 × VBUS）
```

单轴 0.5 → 矢量幅值最大 0.707 < 1，不进过调制区，留足余量。

**带宽取值**：15 kHz 载波下建议 `ωc = 2π × 1500 ≈ 9425 rad/s`（载波的 1/10）。
反推校验：库默认 `Current_Q.Kp = 1.198`、`Ki = 3849.6`，配 `Ls = 51.9 µH`、`Rs = 0.1907 Ω`，对应 `ωc ≈ 1.3e4 rad/s` @20 kHz 载波 —— 默认参数是为另一台电机、另一个载波频率整定的，**不能直接搬用**。

### 6.3 外环参数

| 项                         | 库默认           | 本板取值            | 依据                                                             |
| -------------------------- | ---------------- | ------------------- | ---------------------------------------------------------------- |
| `Response`               | `5`            | `5`               | 速度环 3 kHz、位置环 600 Hz（`SguanFOC.c:854-863`），合理      |
| `Velocity.OutMax/OutMin` | `±10.5f`      | `±3.0f`          | 速度环输出 = Iq 给定，一期按电机额定电流保守取                   |
| `Position.OutMax/OutMin` | `±210.0f`     | `±105.0f`        | 位置环输出 = 速度给定，105 rad/s ≈ 1000 rpm，即本方案的转速上限 |
| `Position.Kp/Ki/Kd`      | `8/0/0`        | `8/0/0` 起调      | 纯 P，定位演示够用                                               |
| `bpf.Encoder.Wc`         | `300.0f`       | `300.0f`          | 速度反馈滤波，48 Hz 截止                                         |
| `pll.Kp/Ki`              | `650 / 210000` | 起调值，按 6.5 验证 | 需确认在 4 kHz 角度更新率下不振荡                                |

**速度环用 PI 而非 LADRC**：`UserData_Calculate.h` 设 `Open_PI_Control = 1`。理由见第 8 节 —— `Ladrc_Init()` 会覆盖用户限幅。虽然该缺陷已在 patch 中修复，一期仍先用参数直观、易整定的 PI 建立基线。

### 6.4 上电启动与标定时序

库的 `Sguan_Start_Tick()`（`SguanFOC.c:904-934`）在 `while(1)` 里执行，仅当 `status == 0x01` 时生效一次：

```
main(): HAL 外设 MX_*_Init() → FocApp_Init() → Sguan.status = 0x01

status=0x01  User_InitialInit()
               ├ DRV8301_Init(&hspi2)          ← 必须第一步，见 7.2
               ├ AS5600_Init(&g_encoder,&hi2c1)
               ├ HAL_TIM_Base_Start_IT(&htim2)  ← 编码器刷新起跑
               ├ HAL_TIMEx_PWMN_Start ×3 + HAL_TIM_PWM_Start ×3 + CH4
               └ HAL_ADCEx_InjectedStart_IT(&hadc1)
             User_MotorSet() / User_ParameterSet() / Sguan_SystemT_Set()

status=0x02  BPF / Control / PLL / Printf 初始化

status=0x03  Offset_CurrentRead()     24 次平均求电流零偏
               （三相 CCR 初值均为 2799 = 50%，零矢量，理论电流为 0）
             Sguan_Positioning_Set(0.3×VBUS, 0) + User_Delay(1000)
               （强制 d 轴对齐到电角度 0）
             Offset_EncoderRead()      记录 Pos_offset
             Sguan_Positioning_Set(0, 0) + User_Delay(800)

status=0x04  IDLE，等待 mode 与目标下发
```

注意 `status < 4` 期间 `SguanFOC_High_Loop()` 内的控制分支不执行（`SguanFOC.c` 判 `status 4..18`），所以 TIM1 一启动就来的 ADC 中断是安全的。

**已知风险：转子极性辨识未实现。** 库在 `SguanFOC.c:929-930` 留了 TODO —— SPMSM 加 d 轴电压后可能稳定在 0° 也可能停在 180°。一期靠 `0.3 × VBUS = 7.2 V` 的较大对齐电压强制拉到 0° 规避。人工验证方法：反复上电 5 次，每次记录 `Pos_offset`（CH10），若出现两簇相差约 `π/Poles` 的机械角，说明存在 180° 电角度歧义，需在二期补辨识。

### 6.5 方向标定四元组

`Motor_Dir` / `PWM_Dir` / `Encoder_Dir` / `Current_Dir0,1` 必须在台架上按顺序确定，**不能猜**：

1. **`PWM_Dir`**：开环模式给 `Uq_in = 1.0f`，示波器确认 U 相占空比增大时高边导通时间增大 → `+1`；反之 `-1`。
2. **`Motor_Dir`**：同上条件下观察电机实际转向。定义逆时针（面向轴端）为正 → 若反转则置 `-1`（库通过交换 U/V 相实现，`SguanFOC.c:818-822`）。
3. **`Encoder_Dir`**：手动正向转轴，看 CH1 `Real_Speed` 应为正。为负则置 `-1`。
4. **`Current_Dir0` / `Current_Dir1`**：前三项确定后，开环给恒定 `Uq_in` 让电机稳速空转，观察 CH3 `Real_Id`。稳态 `Real_Id` 应接近 0；若 `Real_Id` 明显偏离且 `Real_Iq` 符号与 `Uq_in` 相反，说明电流极性反了，翻转两个 `Current_Dir`。

每一项确定后写回 `UserData_Motor.h` 并记录实测依据。

---

## 7.保护与故障处理

### 7.1 保护路径

本板 `nFAULT` / `nOCTW` **未接 MCU**（原理图上经 SN74AHC1G08 与门只驱动 LED1 与 Q1），TIM1 的 BKIN 也未启用且未分配引脚。硬件级即时保护只剩 DRV8301 内部 OC latch shutdown（CTRL1 已配为 `OC_MODE_LATCH_SD`）。软件必须补上其余部分：

| 故障           | 检测点                                                    | 频率   | 响应                                                       |
| -------------- | --------------------------------------------------------- | ------ | ---------------------------------------------------------- |
| DRV8301 FAULT  | `while(1)` `DRV8301_ReadStatus()` 查 STATUS1 FAULT 位 | 10 Hz  | 置`SENSOR_ERROR` 标志 → 库锁定 → `FocApp_Shutdown()` |
| 磁体掉落/过弱  | `while(1)` `AS5600_CheckStatus()`                     | 10 Hz  | 置`ENCODER_ERROR` 标志                                   |
| 编码器数据陈旧 | `SguanFOC_Low_Loop()` 前 `AS5600_IsFresh(now, 5)`     | 1 kHz  | 置`ENCODER_ERROR` 标志                                   |
| D/Q 轴过流     | 库内`Status_Switch_Loop` 比 `Dcur_MAX/Qcur_MAX`       | 1 kHz  | 库置`OVERCURRENT`                                        |
| 高频环计算超时 | 库内`PWM_watchdog`                                      | 15 kHz | 库置`PWM_CALC_FAULT`                                     |

`AS5600_CheckStatus()` 是阻塞 I2C，与 DMA 角度读互斥，返回 −2 表示总线忙 —— 主循环里直接跳过、下个周期重试即可，不计为错误。

### 7.2 母线电压

本板没有独立的母线电压 ADC 通道，只有 VA/VB/VC_FB 三相电压。在下管全开的采样时刻相电压接近 0，无法反推母线。

决策：`User_VBUS_DataGet()` 返回 `-9999.0f` 关闭过/欠压保护，SVPWM 归一化用 `motor.VBUS = 24.0f` 固定值。

---

## 8.库缺陷清单

调研中确认的问题。改动登记在 `docs/sguanfoc-patch.md`，便于日后跟上游。

### 8.1 需要修复（3 处，约 10 行）

| 位置                    | 问题                                                                                                                                                                                                        | 处理               |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------ |
| `Sguan_PID.c:39`      | `static uint8_t IntegralFrozen_flag` 是**函数级 static**，`Current_D` / `Current_Q` / `Velocity` / `Position` 四个实例共享同一个抗饱和状态 —— D 轴积分饱和会把 Q 轴的积分一起冻住         | 移入`PID_STRUCT` |
| `Sguan_Ladrc.c:80-82` | `Ladrc_Init()` 硬编码 `OutMax = ±2000`。调用顺序是 `User_ParameterSet()`(设 ±10.5) → `Sguan_Control_Init()` → `Ladrc_Init()`，用户限幅**必然被冲掉**，速度环输出（= Iq 给定）实质无限幅 | 删除这两行         |
| `SguanFOC.c:987` 附近 | PWM 重入保护的`else` 分支把忙标志 `PWM_Calc` 清零（写反了），下一帧会误判为空闲，看门狗形同虚设                                                                                                         | 改为不清零         |

### 8.2 仅记录、用配置规避

| 问题                                                                                                                                                                        | 规避方式                                                                                                  |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `Status_Switch_Loop` 稳态判定的 `if` 串联无 `else`，条件区间重叠，`TORQUE_CONTROL` / `CONST_SPEED` / `POSITION_HOLD` 等稳态实际不可达（`SguanFOC.c:630-679`） | 只影响状态显示，不影响控制量。CH0 上看到状态在增/减之间跳动属正常                                         |
| `STANDBY → UNINITIALIZED` 自动跳转是死代码：`SguanFOC.c:574-581` 先 `return`，`MOTOR_STATUS_UNINITIALIZED_Signal()` 永远不被检查                                   | 由`FocApp_Init()` 直接写 `Sguan.status = 0x01` 启动，与库示例（`UserData_UserControl.h:59-63`）一致 |
| `FW_MTPA_Loop` 对 SPMSM（`Ld == Lq`）分母为 0（`Sguan_FW.c:16`）                                                                                                      | `Open_FW_Calculate = 0` 保持关闭；且一期 `Ld/Lq` 填 0，前馈也关                                       |
| `Real_Pos` / `Target_Pos` 是 `double` 但 PLL 输出 `OutRe` 是 `float`，多圈累积后精度受 float 限制                                                                 | 1000 rad 处分辨率约 6e-5 rad，定位演示可接受                                                              |
| `flag.Angle_Calc`（`SguanFOC.h:22`）声明后全库零引用                                                                                                                    | 死字段，忽略                                                                                              |
| 参数与离散系数用`double`（`PID_STRUCT.Wc/T/Ki/Kd` 等）                                                                                                                  | 只在 Init 里算一次，运行期是 float，不影响 15 kHz 实时性                                                  |

---

## 9.分阶段落地与验证

验证手段限定为：Keil 编译输出、示波器、VOFA+ 波形、手动转轴。每阶段通过后才进下一阶段。

### 阶段 1 · 接线打通

改动：新建 `Core/Src/foc_app.c` + `Core/Inc/foc_app.h` 并加入 MDK 的 `Application/User/Core` 组；填 7 个 `UserData_*.h`；`stm32f4xx_it.c` 里接 `ADC_IRQHandler` / `TIM2_IRQHandler` 转发；TIM2 改 4 kHz；调整 NVIC 优先级；应用 8.1 的 3 处 patch。

- [ ] Keil 编译 0 error 0 warning
- [ ] 示波器：U/V/W 上下桥波形为中心对齐，周期 66.67 µs
- [ ] 示波器：同相上下桥死区 ≈ 500 ns，无直通
- [ ] GPIO 翻转测得 `SguanFOC_High_Loop()` 执行时间 < 20 µs
- [ ] `DRV8301_Init()` 返回 `DRV8301_OK`

### 阶段 2 · 采样链路

**不接电机。**

- [ ] `Offset_CurrentRead()` 完成后 `Pos_offset0/1` ≈ 2048（±100 以内）
- [ ] 换算后静态电流 `|Real_Ia|`、`|Real_Ib|` < 0.1 A
- [ ] 重构的 `Real_Ic` 与 CH5 实测 IC_FB 换算值之差 < 0.3 A（验证 LM2904 通道与内部放大器的一致性）
- [ ] USART1 提到 921600 + TX DMA，VOFA+ JustFloat 稳定出 12 通道波形无丢帧

### 阶段 3 · 编码器

- [ ] 手动缓慢转轴，CH9 `Real_Pos` 连续无跳变，转满一圈增量 ≈ 2π
- [ ] CH1 `Real_Speed` 符号与转向一致，静止时 ≈ 0 且无明显抖动
- [ ] 快速转轴（约 2 转/秒）不出现跳变拒绝告警（`error_count` 不增长）
- [ ] 拔开磁体，10 Hz 诊断在 200 ms 内置起 `ENCODER_ERROR`

### 阶段 4 · 开环与方向标定

`mode = Velocity_OPEN_MODE`，**电机空载**。

- [ ] `Uq_in` 从 0.5 V 缓升到 2 V，电机平稳转动，无异响
- [ ] 按 6.5 依次确定 `PWM_Dir` / `Motor_Dir` / `Encoder_Dir` / `Current_Dir0,1` 并写回
- [ ] 标定完成后稳速空转，CH3 `Real_Id` 稳态 |值| < 0.3 A
- [ ] 反复上电 5 次，CH10 `Pos_offset` 无 180° 电角度歧义（见 6.4）

### 阶段 5 · 电流环

`mode = Current_SINGLE_MODE`，**电机堵转固定**。

- [ ] `Target_Iq` 阶跃 0 → 1 A，CH4 `Real_Iq` 无持续振荡
- [ ] 上升时间与 `ωc = 2π×1500` 的理论值（约 0.1 ms）同数量级，超调 < 20%
- [ ] `Target_Id = 0` 时 CH3 `Real_Id` 稳态 |值| < 0.15 A
- [ ] `Target_Iq` 阶跃到 `Qcur_MAX` 以上，库正确置 `OVERCURRENT` 并限流

### 阶段 6 · 速度环

`mode = VelCur_DOUBLE_MODE`。

- [ ] `Target_Speed` 阶跃到 50 rad/s，CH1 跟踪无持续振荡，稳态误差 < 2%
- [ ] 阶跃到 105 rad/s（≈1000 rpm 上限），仍稳定
- [ ] 手动加载（手捏轴），转速跌落后能恢复，Iq 相应增大
- [ ] 反向阶跃 +50 → −50 rad/s 过零平稳

### 阶段 7 · 位置环

`mode = PosVelCur_THREE_MODE`。

- [ ] `Target_Pos` 阶跃 ±π，收敛无极限环振荡
- [ ] 定位后手动施加扰动，能回到目标位，稳态误差 < 0.05 rad
- [ ] 连续多圈定位（±10π）位置不累积漂移
- [ ] 全程 CH0 状态无 `PWM_CALC_FAULT` / `OVERCURRENT`

---

## 附：本方案新增/修改的文件清单

| 文件                                                        | 动作                                                                                                |
| ----------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `Core/Inc/foc_app.h`、`Core/Src/foc_app.c`              | 新建，并加入 MDK`Application/User/Core` 组                                                        |
| `Core/Src/main.c`                                         | 调用`FocApp_Init()`；`while(1)` 改为 `SguanFOC_main_Loop()` + 10 Hz `FocApp_DiagnoseLoop()` |
| `Core/Src/stm32f4xx_it.c`                                 | `ADC_IRQHandler` / `TIM2_IRQHandler` 加转发                                                     |
| `SguanFOC/UserData_*.h` × 7                              | 填写扩展点                                                                                          |
| `SguanFOC/Sguan_PID.c`、`Sguan_Ladrc.c`、`SguanFOC.c` | 8.1 的 3 处 patch                                                                                   |
| `docs/sguanfoc-patch.md`                                  | 新建，登记库改动                                                                                    |
