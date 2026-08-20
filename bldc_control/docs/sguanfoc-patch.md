# SguanFOC 库改动登记

本项目把 `SguanFOC/` 视为只读算法层，唯一例外是下面登记的 4 处改动。
每处都标了 `[PATCH-n]` 注释，便于日后跟上游或整体回滚。

参照方案：`基于SguanFOC的BLDC电流环、速度环、位置环控制.md` 第 8 节。

---

## PATCH-1 · PID 抗饱和状态被四个实例共享

**文件**：`Sguan_PID.h`、`Sguan_PID.c`

**问题**：`PID_Loop()` 里的 `static uint8_t IntegralFrozen_flag` 是函数级
`static`，`Current_D` / `Current_Q` / `Velocity` / `Position` 四个实例共用同一份
冻结状态。D 轴积分饱和会把 Q 轴的积分一起冻住，反之亦然，四个环互相干扰。

**改动**：

- `Sguan_PID.h`：`RUN_STRUCT` 增加 `uint8_t IntegralFrozen;`
- `Sguan_PID.c`：`PID_Loop()` 内所有 `IntegralFrozen_flag` 改为
  `pid->run.IntegralFrozen`；`PID_Init()` 增加一行复位

**影响面**：`PID_STRUCT` 尺寸增加 4 字节（含对齐填充），无外部接口变化。

---

## PATCH-2 · `Ladrc_Init()` 覆盖用户限幅

**文件**：`Sguan_Ladrc.c`

**问题**：`Ladrc_Init()` 里硬编码 `OutMax = ±2000`。调用顺序是
`User_ParameterSet()`（设 ±3.0）→ `Sguan_Control_Init()` → `Ladrc_Init()`，
用户限幅必然被冲掉。速度环输出就是 Iq 给定，实质等于无限幅。

**改动**：删除 `ladrc->OutMax = 2000.0f;` / `ladrc->OutMin = -2000.0f;` 两行。

**影响面**：仅在 `Open_PI_Control = 0`（速度环走 LADRC）时参与编译。本项目
一期设为 1，此路径未编译，改动为二期预留。删除后用户**必须**在
`User_ParameterSet()` 里显式设置 `Speed.OutMax/OutMin`，本项目已设。

---

## PATCH-3 · PWM 重入保护把忙标志写反

**文件**：`SguanFOC.c`（`SguanFOC_High_Loop()`）

**问题**：重入分支（`else`）里执行 `Sguan.flag.PWM_Calc = 0;`，把正被上一次调用
持有的忙标志清掉了。下一帧会误判为空闲，重入保护与 `PWM_watchdog` 同时失效。

**改动**：删除 `else` 分支里的 `Sguan.flag.PWM_Calc = 0;`。忙标志只由持有方在
正常退出时清零。

**影响面**：高频环超时才会走到该分支。修复后连续超时超过
`flag.PWM_watchdog_limit`（本项目 10）次会正确置 `MOTOR_STATUS_PWM_CALC_FAULT`。

---

## PATCH-4 · 上电零位对齐电压固定 0.3×VBUS（安全）

**文件**：`SguanFOC.c`（`Sguan_Start_Tick()`）、`UserData_Calculate.h`

**问题**：`Sguan_Positioning_Set(&Sguan, 0.3f*Sguan.motor.VBUS, 0.0f)` 的比例写死。
SVPWM 按 VBUS 归一化、`Overmod()` 限幅到 m=1，实际相电压 = `ratio × VBUS / √3`：

```
ratio = 0.3, VBUS = 24 V   ->  相电压 = 0.3 × 24 / 1.732 = 4.16 V
HT3510 相电阻 4.265 Ω      ->  对齐电流 = 4.16 / 4.265 = 0.975 A
```

对 HT3510（额定 0.53 A / 堵转 0.80 A）来说 0.975 A 已是 1.2 倍堵转电流，持续 1 s
偏热；而该默认值来自 12 V / 5 mΩ 的参考板，换到低阻电机上会直接超出 ±16.5 A 的
采样量程并触发 DRV8301 的 OC 锁存。比例必须随电机走，不能写死。

**改动**：

- `UserData_Calculate.h`：新增 `#define Align_Voltage_Ratio 0.15f`
- `SguanFOC.c`：`0.3f*Sguan.motor.VBUS` 改为 `Align_Voltage_Ratio*Sguan.motor.VBUS`

**取值方法**：

```
I_align ≈ Align_Voltage_Ratio × VBUS / (√3 × Rs)
```

当前 `0.15` 对应 2.08 V 相电压、0.487 A（约等于额定电流）、0.10 N.m 对齐力矩。
换电机按上式反解，目标对齐电流建议取额定电流量级。

**影响面**：只改标定阶段的对齐力矩。比例过小会导致转子克服不了齿槽力矩、对不到
电角度 0，`Pos_offset` 记录错误，闭环必然失败 —— 阶段 4 需确认转子确实被锁死在一个
固定位置，并按 6.4 反复上电 5 次检查 `Pos_offset` 无 180° 电角度歧义。

---

## 仅记录、用配置规避的问题

| 问题 | 规避方式 |
| --- | --- |
| `Status_Switch_Loop` 稳态判定的 `if` 串联无 `else`，区间重叠，`TORQUE_CONTROL` / `CONST_SPEED` / `POSITION_HOLD` 实际不可达 | 只影响 CH0 的状态显示，不影响控制量。状态在增/减之间跳动属正常 |
| `STANDBY → UNINITIALIZED` 自动跳转是死代码（`Status_Switch_Loop` 先 `return`） | 启动与重启由 `FocApp_Init()` / `FocApp_SetControlWord()` 直接写 `Sguan.status = 0x01` |
| `Offset_CurrentRead()` 累加求平均前未清零 `Pos_offset0/1`，重启会叠加上一轮结果 | `FocApp_HwStart()` 在调用链更上游先清零，未改库 |
| `status < 4` 期间库不更新 CCR，重启时上一帧电压矢量会被冻结着持续输出 | `FocApp_HwStart()` 入口先 `FocApp_Shutdown()` + 三相拉回零矢量 |
| `Status_Current_OVERCURRENT` 只判正向（`Real_Id > Dcur_MAX`），负向过流不触发 | 目标值已在 `FocApp_SetTarget()` 按 `Qcur_MAX` 双向限幅；瞬时过流由 DRV8301 的 OC 锁存兜底 |
| `FW_MTPA_Loop` 对 SPMSM（`Ld == Lq`）分母为 0 | `Open_FW_Calculate = 0` 保持关闭，且 `identify.Ld/Lq` 填 0 |
| `PLL_Loop()` 运行期用 `double` 做 `T*OutWe`，在 15 kHz 环内是软件浮点 | 实测执行时间需在阶段 1 用 GPIO 翻转确认高频环 < 20 µs；超标再把 `PLL_STRUCT.T` 降为 `float` |
| `Real_Pos` / `Target_Pos` 是 `double` 但 PLL 输出 `OutRe` 是 `float` | 1000 rad 处分辨率约 6e-5 rad，定位演示可接受 |
| `flag.Angle_Calc`（`SguanFOC.h:22`）声明后全库零引用 | 死字段，忽略 |
