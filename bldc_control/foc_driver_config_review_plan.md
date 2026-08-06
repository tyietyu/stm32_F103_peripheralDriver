# 基于 SimpleFOC 思路的裁剪版 BLDC 驱动实施文档

> 文档状态：实施方案，仅落实文档，不直接修改源码。
>
> 复查日期：2026-08-06。
>
> 实施边界：只使用项目当前已有的 AS5600、三相电流采样、三相电压采样、DRV8301、TIM1 六路互补 PWM 和现有 STM32 外设，不增加新硬件、新传感器或与驱动电机无关的功能。

## 1. 目标

在现有硬件和代码基础上，参考 SimpleFOC 的职责划分、传感器对齐、电流检测和 FOC 执行顺序，裁剪实现一套最小、清晰、可验证的有感无刷直流电机驱动：

```text
AS5600 获取转子机械角度
  -> 计算电角度
  -> 同步采样三相电流
  -> Clarke/Park 变换
  -> id/iq 电流 PI
  -> 逆 Park/Clarke
  -> 三相 PWM
  -> DRV8301 驱动功率桥
```

本方案只实现：

- AS5600 转子位置检测；
- 电角度计算和零位对齐；
- 三相电流同步采样、零点校准和 dq 电流环；
- 三相电压采样及监控；
- DRV8301 初始化、使能、故障读取和六路 PWM 驱动；
- `id_ref=0`、`iq_ref` 控制的电流/转矩模式；
- 运行必需的故障关断。

## 2. 明确不实现的功能

为避免把 SimpleFOC 的通用框架完整搬入本项目，本次明确不增加：

- 速度闭环；
- 位置闭环；
- 开环速度或开环位置运行模式；
- 无感反电势、滑模观测器或其他位置估算；
- 新增编码器、霍尔传感器或其他角度传感器；
- 新增母线电压采样硬件；
- 相电压闭环、反电势重构或在线电机参数辨识；
- MTPA、弱磁、前馈解耦等扩展控制；
- 完整移植 SimpleFOC 的 Arduino 类库和平台抽象；
- RTOS、动态内存、命令行配置或复杂参数管理；
- 自动识别所有电流 ADC 通道映射并在线改写配置；
- 为追求高速而增加角度预测器或更换通信接口。

现有速度、位置相关代码如暂不影响构建，可以先保留文件，但不得进入本次运行路径；后续是否删除另行评审。

## 3. 当前外设与职责

| 外设/模块   | 当前资源                     | 裁剪后的唯一职责                                     |
| ----------- | ---------------------------- | ---------------------------------------------------- |
| AS5600      | PB6/PB7 软件 I²C            | 提供单圈机械角、多圈位置监控、有效状态和最近成功时间 |
| ADC1 注入组 | PA3/PA4/PA5，TIM1_CH4 触发   | 同步采样 U/V/W 三相电流，作为 15 kHz 电流环反馈      |
| ADC1 普通组 | PA0/PA1/PA2，TIM2 TRGO + DMA | 采样三相端电压，仅用于监控和调试，不参与 FOC 计算    |
| TIM1        | CH1~CH3 及互补输出、CH4 触发 | 输出六路中心对齐 PWM，并产生电流 ADC 采样时刻        |
| DRV8301     | SPI2、EN_GATE、DC_CAL        | 配置栅极驱动和电流放大器、驱动功率桥、提供故障状态   |
| TIM2        | 1 ms 周期                    | 触发 AS5600/监控任务和普通 ADC；不执行电流内环       |
| FOC 算法    | `FOC_Driver`               | 电角计算、Clarke/Park、PI、逆变换和 PWM 电压输出     |

## 4. SimpleFOC 思路的裁剪映射

SimpleFOC 将电机控制分为 Driver、Sensor、CurrentSense 和 Motor。项目不照搬类结构，只保留相同职责边界。

### 4.1 Driver：DRV8301 与 TIM1

对应项目：

- `BSP_Driver/drv8301.c/.h`
- `Core/Src/tim.c`
- `FOC_Driver/foc.c` 中的 PWM 输出逻辑

保留职责：

1. 初始化 DRV8301 SPI 和控制寄存器；
2. 设置 6PWM 模式和电流放大器增益；
3. 控制 `EN_GATE`；
4. 启停 TIM1 主输出 MOE；
5. 将三相目标电压转换为 CCR1/CCR2/CCR3；
6. 更新 CH4 电流采样触发点；
7. 读取并上报 DRV8301 故障。

不增加通用 Driver 类、运行时 PWM 模式切换或多型号驱动适配。

### 4.2 Sensor：AS5600

对应项目：

- `BSP_Driver/as5600.c/.h`
- `BSP_Driver/hal_iic.c/.h`
- `FOC_Driver/foc_hal.c`

保留职责：

1. 读取 AS5600 12 位原始角度；
2. 转换为 `[0, 2π)` 单圈机械角；
3. 跨越 0/4095 时维护多圈位置，供监控使用；
4. 保存最近一次成功读取时间；
5. 提供读取是否有效；
6. 将机械角转换为 FOC 使用的电角度。

速度估算不进入本次控制链路。已有 `AS5600_GetVelocity()` 可以保留，但当前模式不得依赖它。

### 4.3 CurrentSense：三相电流采样

对应项目：

- `Core/Src/adc.c`
- `FOC_Driver/foc_hal.c/.h`
- `Core/Src/main.c` 的 ADC 注入回调

保留职责：

1. PWM 关闭时执行三相电流零点校准；
2. TIM1_CH4 同步触发 ADC 注入组；
3. 将 ADC 原始值转换为 U/V/W 三相电流；
4. 应用固定的通道映射、方向符号和增益；
5. 检查 ADC 饱和、采样窗口和软件过流；
6. 把有效电流交给 FOC 电流环。

不实现 SimpleFOC 通用库中的多驱动、多 ADC 架构。由于本项目硬件固定，通道顺序和符号采用明确配置加台架验证，不做复杂的运行时自动重映射。

### 4.4 Motor：裁剪版 FOC 电流模式

对应项目：

- `FOC_Driver/foc.c/.h`
- `FOC_Driver/pid.c/.h`
- `Core/Src/main.c`

只保留以下运行步骤：

```text
读取缓存电角度
  -> 读取三相电流
  -> Clarke
  -> Park
  -> id/iq PI
  -> dq 电压限幅
  -> 逆 Park
  -> 逆 Clarke
  -> 三相居中 PWM
```

控制目标固定为：

```text
id_ref = 0 A
-iq_limit <= iq_ref <= iq_limit
```

`iq_ref` 是本次唯一电机控制输入。正负值分别产生正反方向转矩。

## 5. 裁剪后的软件结构

不新增复杂目录，只在现有文件中明确职责。

```text
BSP_Driver/
  drv8301.c/.h       DRV8301 配置、使能和故障
  as5600.c/.h        原始角、机械角、多圈位置、有效状态
  hal_iic.c/.h       AS5600 软件 I²C

FOC_Driver/
  foc.c/.h           对齐、电角度、坐标变换、调制、电流环
  foc_hal.c/.h       AS5600 绑定、电流采样转换和校准
  pid.c/.h           id/iq PI
  lowpass_filter.c   只用于监控数据，不进入电流反馈

Core/Src/
  main.c             启动流程、1 ms 任务、ADC ISR、故障关断
  tim.c              TIM1 PWM/CH4，TIM2 1 ms
  adc.c              电流注入组和电压普通组
```

不为模仿 SimpleFOC 而新建大量基类或抽象层。

## 6. 必须统一的物理定义

FOC 能否正确运行，首先取决于以下定义一致，而不是 PI 参数。

### 6.1 三相定义

实施前必须形成一张固定映射表：

| 软件定义 | PWM           | DRV8301/功率桥 | 电机相线 | 分流/反馈 | ADC                      |
| -------- | ------------- | -------------- | -------- | --------- | ------------------------ |
| U 相     | TIM1_CH1/CH1N | A 桥臂         | 电机 U   | IA_FB     | ADC rank1/PA3 或实测通道 |
| V 相     | TIM1_CH2/CH2N | B 桥臂         | 电机 V   | IB_FB     | ADC rank2/PA4 或实测通道 |
| W 相     | TIM1_CH3/CH3N | C 桥臂         | 电机 W   | IC_FB     | ADC rank3/PA5 或实测通道 |

表中最终映射必须通过原理图、线束和小电流激励共同确认。

原理图显示 A/B 相使用 DRV8301 电流放大器，C 相使用 LM2904，因此三相需分别保留：

```text
channel_u/channel_v/channel_w
sign_u/sign_v/sign_w
gain_u/gain_v/gain_w
offset_u/offset_v/offset_w
```

这些值使用固定配置，不增加运行时自动改线功能。

### 6.2 角度方向

统一公式：

```text
θe = normalize(sensor_direction × pole_pairs × θm - zero_electric_angle)
```

其中：

- `θm`：AS5600 单圈机械角；
- `sensor_direction`：`+1` 或 `-1`；
- `pole_pairs`：当前电机固定配置为 11；
- `zero_electric_angle`：启动对齐得到的零电角；
- `θe`：范围 `[0, 2π)` 的电角度。

极对数本次不做自动辨识。若更换电机，由配置修改并重新验证。

### 6.3 电流方向

规定从逆变桥流向电机绕组为相电流正方向，ADC 转换后的 U/V/W 电流必须符合这一约定。

运行时可使用以下关系做基本合理性检查：

```text
iu + iv + iw ≈ 0
```

该检查只用于发现明显的通道、符号、增益或采样窗口错误，不增加复杂电流诊断算法。

## 7. AS5600 最小实现

### 7.1 保留的数据

AS5600 状态最少包含：

```text
raw_angle              最近成功的 12 位角度
mechanical_angle       最近成功的单圈机械角
full_angle             多圈机械位置，供监控
last_success_tick       最近成功读取时间
error_count             连续读取错误数
valid                   当前数据是否有效
```

不增加角度预测器、速度观测器或复杂滤波器。

### 7.2 更新规则

1. TIM2 每 1 ms 只设置任务标志；
2. 前台任务调用一次 `AS5600_Update()`；
3. 读取成功后：
   - 更新原始角；
   - 处理 0/4095 跨圈；
   - 更新单圈角和多圈位置；
   - 更新最近成功时间；
   - 清连续错误并设置有效；
4. 读取失败后：
   - 不改写最近成功角度；
   - 连续错误计数加一；
   - 当前周期不发布新角度；
5. FOC 只使用最近成功角度，但必须检查数据年龄。

### 7.3 最小安全检查

保留以下检查即可：

- 初始化读取必须成功；
- 启动前确认 AS5600 检测到磁体；
- 运行中读取失败或角度数据超过允许时间未更新时关断；
- 对齐期间角度必须产生可辨识变化；
- 原始角不能出现超过当前最大允许转速的异常跳变。

不加入 AGC 趋势分析、磁场幅值统计或装配质量评估功能。

### 7.4 软件 I²C 修正范围

保留现有软件 I²C，不改成硬件 I²C。只做正确驱动 AS5600 所必需的修正：

1. SDA、SCL 使用开漏输出，高电平通过释放总线获得；
2. 初始化后总线保持空闲高电平；
3. ACK 等待有明确超时；
4. 读写失败能够返回错误；
5. 不在 ADC ISR 中执行 I²C；
6. 用逻辑分析仪确认时序和上升沿满足当前板级连接。

不额外实现 DMA、异步事务或通用 I²C 设备框架。

## 8. 传感器对齐

参考 SimpleFOC 的传感器对齐思路，但只保留本电机必需步骤。

### 8.1 对齐目的

对齐只解决两个参数：

```text
sensor_direction
zero_electric_angle
```

极对数固定使用 11，不在运行时搜索。

### 8.2 对齐步骤

1. 确认 DRV8301 已初始化，电流零点已校准；
2. 启动 ADC 注入采样和软件过流检查；
3. 使用受限对齐电压建立固定定子磁场；
4. 缓慢改变一小段命令电角度，读取 AS5600 机械角变化；
5. 根据机械角变化正负确定 `sensor_direction`；
6. 如果机械角变化小于阈值，则判定电机未运动或传感器异常，立即退出；
7. 将转子稳定锁定到已知电角位置；
8. 读取机械角并计算 `zero_electric_angle`；
9. 将对齐电压降为零；
10. 清空 id/iq PI 状态，再进入运行态。

不实现长距离往返扫描、极对数自动估算或校准参数永久存储。

### 8.3 对齐限制

独立设置：

```text
alignment_voltage
alignment_current_limit
alignment_timeout
```

这三个量只用于防止对齐阶段过流或长时间堵转，不作为新的控制功能。

## 9. 电流采样最小实现

### 9.1 零点校准

保留当前 DC_CAL 方式：

1. MOE 关闭、功率 PWM 不输出；
2. DRV8301 使能并进入 DC_CAL；
3. TIM1_CH4 继续产生 ADC 注入触发；
4. 采集约 200 组样本；
5. 分别计算 U/V/W offset；
6. 检查样本跨度和是否靠近 ADC 电源轨；
7. 校准失败不得使能功率输出。

### 9.2 电流转换

统一为：

```text
phase_current = (raw - offset) × adc_factor × gain × sign
```

其中 channel、gain 和 sign 使用固定配置。

### 9.3 PWM 同步采样

继续使用 TIM1_CH4 触发注入组，不改变现有外设方案。

必须确认：

- CH4 位于三相低侧电流均可测的稳定窗口；
- 采样点避开死区和开关尖峰；
- 三个注入通道完整转换结束前采样窗口仍有效；
- CCR1~CCR4 在同一更新边界生效；
- 一个 PWM 周期只产生一次期望的电流环回调。

当前动态 CH4 计算逻辑可以保留，但必须用示波器或调试 GPIO验证。

### 9.4 电流有效性检查

ADC ISR 只保留必要检查：

1. 三路原始 ADC 未接近 0/满量程；
2. 电流 offset 已校准；
3. 当前 PWM 采样窗口有效；
4. 任一相电流未超过软件绝对限值；
5. `iu+iv+iw` 未超过基本容差；
6. 注入 ADC 回调未停止。

任一检查失败，立即关闭 MOE 和 EN_GATE，并锁存故障。

### 9.5 电流滤波

15 kHz FOC 电流反馈不使用当前 5 ms 低通滤波器。

现有低通只用于降低监控数据显示噪声，不进入 Clarke/Park 和 PI。

## 10. 三相电压采样的裁剪用途

三相电压采样不是当前有感电流 FOC 的控制反馈，因此只保留：

1. TIM2 TRGO 以约 1 kHz 触发 ADC 普通组；
2. DMA 获取 U/V/W 三相端电压；
3. 使用现有比例系数换算为电压；
4. 使用低通滤波后写入监控数据；
5. 用于台架确认 PWM、桥臂和电机相线是否有输出。

本次不使用三相电压进行：

- 电压闭环；
- 反电势过零检测；
- 转子位置估算；
- 母线电压估算；
- 相电阻/电感辨识；
- PWM 自动补偿。

电压采样异常只记录监控状态，不作为高频电流环输入。是否触发关断沿用现有项目策略，不新增复杂判定。

## 11. DRV8301 最小实现

### 11.1 初始化

保留当前 SPI 和寄存器配置流程：

1. `EN_GATE=0`；
2. SPI2 初始化；
3. `EN_GATE=1` 并等待驱动器稳定；
4. 写 CTRL1/CTRL2：
   - 6PWM；
   - 既定栅极驱动电流；
   - 既定过流模式/阈值；
   - 电流放大增益 10；
5. 读回控制寄存器；
6. 读取状态寄存器；
7. 任一步失败立即关断。

### 11.2 运行

运行时只保留：

- DRV8301 已使能；
- 按现有周期读取故障状态；
- 发现有效故障后锁存并关断；
- 不提供运行中动态修改栅极电流、增益或过流阈值的接口。

### 11.3 关断

统一关断顺序：

```text
关闭 TIM1 MOE
  -> 拉低 EN_GATE
  -> iq_ref = 0
  -> 清空 id/iq PI
  -> 锁存首个故障
```

不增加新的硬件 Break 连接或修改 PCB。本次只使用当前硬件已经具备的 DRV8301 保护和软件关断路径。

## 12. 裁剪版 FOC 算法

### 12.1 电角度

AS5600 更新在 1 ms 前台任务中执行，成功后更新缓存电角度：

```text
cached_angle_el = normalize(direction × pp × angle_m - zero_el)
```

ADC 电流 ISR 不读取 I²C，只使用缓存电角度。

为避免长期使用旧角度，ISR 或进入 ISR 前的共享状态必须能够判断：

```text
current_time - last_success_time <= sensor_max_age
```

当前目标机械速度 5 rad/s、`pp=11` 时，1 ms 对应约 3.15° 电角变化。当前方案只面向既定低速调试范围，不增加角度预测；若后续提高转速，需重新评估 AS5600 更新周期，但不属于本次实现。

### 12.2 Clarke 变换

三相电流变换为 α/β：

```text
i_alpha = 2/3 × (iu - iv/2 - iw/2)
i_beta  = 1/√3 × (iv - iw)
```

保持当前三电流实现，不增加两相重构模式。

### 12.3 Park 变换

```text
id =  i_alpha × cos(θe) + i_beta × sin(θe)
iq = -i_alpha × sin(θe) + i_beta × cos(θe)
```

### 12.4 电流 PI

控制误差：

```text
error_d = 0 - id
error_q = iq_ref - iq
```

输出：

```text
Ud = PI_d(error_d)
Uq = PI_q(error_q)
```

裁剪要求：

- D 固定为 0；
- 电流环使用固定 `dt=1/15000 s`；
- 初始从低 P、低 I 开始；
- PI 输出进行简单限幅；
- 输出饱和时停止继续向同方向积分，采用条件积分即可；
- 不增加解耦、反电势前馈或自动整定。

现有 `I=0` 只能算电流 P 控制，后续实现必须加入可调但默认较小的 I。

### 12.5 逆变换与调制

使用同一组 `sin(θe)`、`cos(θe)` 完成逆 Park：

```text
U_alpha = Ud × cos(θe) - Uq × sin(θe)
U_beta  = Ud × sin(θe) + Uq × cos(θe)
```

再通过逆 Clarke 得到三相电压，并加入半母线偏置生成 PWM。

保留当前三相共模居中和占空比范围限制，不新增扇区式 SVPWM。调制必须优先满足低侧电流采样窗口。

### 12.6 唯一运行接口

裁剪后上层只需要：

```text
BLDC_SetIqReference(float iq_ref)
BLDC_Start()
BLDC_Stop()
BLDC_GetMonitorData()
BLDC_GetFault()
```

实际命名可保持当前接口风格，不要求为此重构所有文件。

不暴露速度、位置和多控制模式接口。

## 13. 调度与运行流程

### 13.1 15 kHz ADC 注入 ISR

只执行：

```text
读取三相 ADC
  -> ADC/电流/采样窗口检查
  -> 电流转换
  -> Clarke/Park
  -> id/iq PI
  -> 逆变换和 PWM 更新
```

禁止执行：

- AS5600 I²C；
- SPI 故障轮询；
- 电压普通组处理；
- 日志输出；
- HAL_Delay；
- 速度或位置控制。

### 13.2 1 ms 前台任务

只执行：

```text
AS5600_Update
  -> 检查传感器有效性和数据年龄
  -> 更新缓存机械角/电角
  -> 更新 iq_ref 斜坡（如保留）
```

不执行速度环和位置环。

### 13.3 普通 ADC 回调

只完成三相电压换算、低通和监控数据更新。

### 13.4 低频故障任务

沿用现有周期读取 DRV8301 状态并更新监控。硬件过流仍由 DRV8301 本身处理，软件检测到故障后保持锁存关断。

## 14. 最小启动状态

不建立复杂状态机，只保留五个明确状态：

```text
OFF
  -> INIT
  -> ALIGN
  -> RUN
  -> FAULT
```

### 14.1 OFF

- MOE 关闭；
- EN_GATE 低；
- `iq_ref=0`；
- PI 清零。

### 14.2 INIT

执行：

1. STM32 GPIO/SPI/TIM/ADC 初始化；
2. TIM1 CH4 计时和 PWM 通道预装载，但 MOE 保持关闭；
3. AS5600 初始化和磁体检查；
4. DRV8301 初始化和寄存器回读；
5. 电流零点校准；
6. 检查 ADC 和传感器状态。

任一步失败进入 FAULT。

### 14.3 ALIGN

执行：

1. 启动 ADC 注入中断；
2. 打开功率输出；
3. 在软件电流限制下完成方向和零电角对齐；
4. 将输出降为零；
5. 清 PI。

失败进入 FAULT。

### 14.4 RUN

- 15 kHz 电流环运行；
- 1 ms 更新 AS5600；
- 1 kHz 更新三相电压监控；
- 只接受 `iq_ref`；
- 任一必要故障进入 FAULT。

### 14.5 FAULT

- 关闭 MOE；
- 拉低 EN_GATE；
- 清参考和 PI；
- 保留首个故障原因；
- 不自动恢复运行。

## 15. 必要故障范围

只保留与当前驱动直接相关的故障：

```text
DRV8301 初始化/SPI/状态故障
AS5600 初始化、通信或数据超时
ADC 原始值异常
电流零点校准失败
电流超过软件限值
PWM 电流采样窗口无效
ADC 注入回调停止
传感器对齐失败
```

不增加与当前功能无关的复杂故障管理、历史数据库或自动恢复策略。

## 16. 文件修改计划

### 16.1 `BSP_Driver/as5600.c/.h`

必须修改：

- 明确最近成功角度和时间；
- 读取失败时不伪造新样本；
- 提供有效状态和连续错误计数；
- 初始化检查磁体存在；
- 保留单圈角和多圈位置；
- 不向电流 ISR 暴露阻塞读取函数。

不实现速度控制、预测和复杂滤波。

### 16.2 `BSP_Driver/hal_iic.c/.h`

必须修改：

- SDA/SCL 开漏；
- ACK/事务错误返回；
- 保持现有软件 I²C 和阻塞调用方式；
- 不增加新驱动框架。

### 16.3 `FOC_Driver/foc_hal.c/.h`

必须修改：

- 明确电流 channel/sign/gain 配置；
- 保留 DC_CAL offset 校准；
- 电流转换后执行基本范围和三相和检查；
- AS5600 适配改为返回缓存状态，而不是控制过程中直接读取 I²C。

### 16.4 `FOC_Driver/foc.c/.h`

必须修改：

- 只保留当前模式需要的 FOC 电流路径；
- 完成简单方向和零电角对齐；
- 检查角度缓存是否有效且未超时；
- 复用同一组 sin/cos；
- 保留三相居中调制和动态 ADC 触发；
- 明确当前/下一 PWM 周期采样窗口状态。

不增加速度、位置和其他运动控制模式。

### 16.5 `FOC_Driver/pid.c/.h`

必须修改：

- 电流控制器按 PI 使用，D=0；
- 保留固定周期；
- 加入简单条件积分；
- 不增加通用控制器框架。

### 16.6 `Core/Src/main.c`

必须修改：

- 删除运行路径中的速度/位置测试阶段；
- 只保留 `iq_ref` 电流模式；
- 启动流程改为 OFF/INIT/ALIGN/RUN/FAULT；
- ALIGN 前启动注入电流监控；
- 1 ms 任务只更新 AS5600 和电流目标；
- 统一故障关断。

### 16.7 `Core/Src/tim.c`、`Core/Src/adc.c`

原则上只做必要修正：

- 保留 15 kHz 中心对齐 PWM；
- 保留 CH4 动态触发；
- 保留三相注入电流采样；
- 保留 TIM2 触发三相电压普通组；
- 不改变 MCU 外设方案。

## 17. 实施顺序

### 第一步：固定硬件映射

- 确认 TIM1 CH1/2/3 到功率桥 A/B/C；
- 确认电机 U/V/W 相线；
- 确认 IA/IB/IC 到 PA3/PA4/PA5；
- 确认三相电流正方向和增益；
- 形成固定配置表。

**通过条件**：单相/两相小电流激励与 ADC 响应一致。

### 第二步：整理 AS5600

- 修正软件 I²C 开漏和错误返回；
- 增加最近成功时间和数据超时；
- 保留单圈角与多圈位置；
- 验证正反手动旋转和跨零点。

**通过条件**：连续读取稳定，断开传感器能够进入故障。

### 第三步：验证电流同步采样

- 执行 offset 校准；
- 验证三相 gain/sign/channel；
- 示波器确认 CH4 采样窗口；
- 验证 `iu+iv+iw`；
- 验证软件电流上限关断。

**通过条件**：不同 PWM 占空比下三相电流样本有效。

### 第四步：实现对齐

- 启动电流监控；
- 小电压扫描确定方向；
- 固定位置取得零电角；
- 验证无运动和过流退出。

**通过条件**：重复对齐方向一致，零电角稳定。

### 第五步：闭合电流环

- 固定 `id_ref=0`；
- `iq_ref` 从 0.05~0.10 A 小电流开始；
- 先增加 P，再加入小 I；
- 验证正负 `iq_ref`；
- 检查 `id`、`iq`、PWM 和电流峰值。

**通过条件**：电流跟踪稳定、方向正确、故障可关断。

## 18. 验收清单

### 18.1 AS5600

- [x] 软件 I²C SDA/SCL 为开漏行为。<!-- hal_iic.c IICInit/SDA_Output_Mode 均使用 GPIO_MODE_OUTPUT_OD -->
- [x] 初始化读取成功并检测到磁体。<!-- as5600.c AS5600_Init 读 STATUS 寄存器并检查 AS5600_STATUS_MAGNET_DETECT -->
- [ ] 正反旋转时机械角方向连续。<!-- 需台架：手动正反旋转确认输出连续递增/递减 -->
- [x] 0/4095 跨界时多圈位置无跳变。<!-- as5600.c AS5600_Update 用有符号 delta 检测跨圈并修正 rotation_offset -->
- [x] 读取失败不会更新成功时间和角度序号。<!-- AS5600_RecordReadError 只递增 error_count 并置 valid=false，不修改 last_success_tick/raw_angle -->
- [x] 数据超时后关闭 MOE 和 EN_GATE。<!-- main.c 1ms 任务中 FOC_HAL_UpdateSensor 超时返回 -1 → BLDC_StopOutput → BLDC_ForceHardwareOff -->

### 18.2 电流采样

- [x] 三路 offset 校准通过。<!-- foc_hal.c FOC_Current_Offset_Calibration：DC_CAL 模式采样 200 次，检查跨度和电源轨裕量，失败阻止上电 -->
- [ ] U/V/W channel、sign、gain 经实测确认。<!-- 需台架：FOC_CURRENT_SIGN_U/V/W 当前均为 1.0f，需单相激励实测后修正 -->
- [ ] 已知正负电流下换算值方向正确。<!-- 需台架：施加已知方向电流验证符号 -->
- [x] `iu+iv+iw` 满足基本容差。<!-- foc_hal.c FOC_ValidatePhaseCurrents 检查 |iu+iv+iw| <= FOC_CURRENT_SUM_TOLERANCE_A(0.30A) -->
- [x] ADC 未饱和且三通道完整采样。<!-- main.c ISR 中检查 raw_iu/iv/iw 均在 [16, 4079] 范围内，越界立即触发 FAULT_ADC -->
- [ ] CH4 位于有效低侧采样窗口。<!-- 需台架：示波器确认 CCR4 触发点处三路低侧管同时导通 -->
- [x] 软件过流立即关断。<!-- main.c ISR：FOC_ValidatePhaseCurrents 返回非零 → BLDC_TripFromIsr → BLDC_ForceHardwareOff -->

### 18.3 电压采样

- [x] TIM2 触发频率符合配置。<!-- 第21节已审查：TIM2 TRGO 触发普通组，约 1 kHz，与方案一致 -->
- [ ] U/V/W 电压通道对应正确。<!-- 需台架：单相施加已知电压，确认 V_CHANNEL_U/V/W 通道与实际相线对应 -->
- [ ] 电压比例经已知输入验证。<!-- 需台架：已知直流电压验证 ADC_VOLTAGE_FACTOR × VOLTAGE_DIVIDER_RATIO(8.0) 精度 -->
- [x] 电压采样只更新监控，不进入 FOC 电流环。<!-- main.c HAL_ADC_ConvCpltCallback 只写 g_monitor_data.V_U/V/W，不参与任何 FOC 计算路径 -->

### 18.4 DRV8301

- [x] SPI 寄存器写入和回读一致。<!-- drv8301.c DRV8301_WriteRegVerified：写入后立即读回比对，不一致返回 DRV8301_ERROR_VERIFY -->
- [x] 6PWM、电流增益和保护参数符合当前硬件。<!-- drv8301.c CTRL1_CONFIG=6PWM/1.7A 栅极/锁存过流/0.25V；CTRL2_CONFIG=GAIN_10/OT_OC 监控；与方案要求一致 -->
- [x] EN_GATE 低时功率输出关闭。<!-- main.c BLDC_ForceHardwareOff 先 MOE_DISABLE 再 EN_GATE=0；EN_GATE 不使能则 DRV8301 不驱动功率桥 -->
- [x] 状态故障能够锁存并关断。<!-- main.c 低频任务 DRV8301_ReadStatus1 → SR1_FAULT → BLDC_StopOutput 锁存，不自动恢复 -->

### 18.5 对齐

- [ ] 方向检测结果与手动旋转方向一致。<!-- 需台架：手动旋转验证 dir 与机械方向吻合 -->
- [ ] 固定 `pp=11` 时电角方向正确。<!-- 需台架：对齐后确认 cached_angle_el 随机械角单调变化且极对数正确 -->
- [x] 无运动时对齐失败。<!-- foc.c FOC_AlignmentSensor：movement = end_angle - start_angle，|movement| < movement_threshold 时返回 -1 -->
- [x] 对齐期间电流超过限制时关断。<!-- main.c ISR：state==ALIGN 时使用 foc.alignment_current_limit，超限 → BLDC_TripFromIsr；foc.c AlignmentStep 检查 MOE 状态 -->
- [ ] 多次对齐的零电角结果稳定。<!-- 需台架：重复上电对齐，比较 zero_electric_angle 波动是否在可接受范围 -->

### 18.6 FOC 电流环

- [x] `id_ref` 始终为 0。<!-- main.c FOC_CurrentLoopControl 调用时硬编码 id_ref=0.0f，无任何路径修改该值 -->
- [ ] 正负 `iq_ref` 产生预期方向转矩。<!-- 需台架：小电流下确认正负 iq_ref 对应预期转向 -->
- [ ] `id` 均值接近 0。<!-- 需台架：示波器或调试变量确认稳态 id 收敛到 0 附近 -->
- [ ] `iq` 能跟踪小电流目标。<!-- 需台架：0.05~0.10 A 阶跃验证 iq 跟踪响应 -->
- [x] PI 饱和时不继续同方向积分。<!-- pid.c PID_Calc：unsaturated_output 超限且误差同向时回退 integral = prev_integral（条件积分反饱和） -->
- [ ] 15 kHz ISR 最坏执行时间小于 66.7 μs，并保留裕量。<!-- 需台架：DWT CYCCNT 已埋点(adc_injected_callback_max_cycles)，上电后读取确认 -->
- [x] AS5600 I²C、SPI 和日志不在 ADC ISR 中执行。<!-- main.c ISR 只调用 HAL_ADCEx_InjectedGetValue / FOC_Get_Calibrated_Current / FOC_CurrentLoopControl，无 I²C/SPI/printf/HAL_Delay -->

### 18.7 启停和故障

- [x] INIT/ALIGN 失败不能进入 RUN。<!-- main.c：AS5600/DRV8301/calibration/alignment 任一失败均调用 BLDC_FailAndStop → Error_Handler 死循环，bldc_state 不会到达 BLDC_STATE_RUN -->
- [x] Stop 和任一故障均先关闭 MOE/EN_GATE。<!-- BLDC_StopOutput / BLDC_ForceHardwareOff：先 __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY，再 EN_GATE=0 -->
- [x] 故障后 `iq_ref=0` 且 PI 清零。<!-- BLDC_StopOutput：iq_ref=0.0f，PID_Reset(&pid_id/pid_iq/pid_speed/pid_position) 均在同一临界区内完成 -->
- [x] 故障锁存后不自动重新启动。<!-- bldc_fault_latched 置 1 后无任何代码路径将其清零或将 bldc_state 改回 RUN；BLDC_Stop 也先检查 fault_latched 并直接返回 -->

## 19. 当前实施边界

本次文档只定义现有硬件上的最小有感电流 FOC。以下实测信息仍需在代码实施和台架验证阶段补齐：

- 电机相电阻、相电感和允许连续/峰值电流；
- AS5600 实际安装方向和磁体状态；
- 电流反馈三相真实增益、符号和线束映射；
- CH4 触发与低侧导通窗口波形；
- 15 kHz ADC ISR 最坏执行时间；
- 合适的 `alignment_voltage`、软件电流限值和 PI 参数。

这些参数不明确前，只允许使用限流电源和小 `iq_ref` 调试，不启用现有速度/位置控制代码。

## 20. 最终裁剪结论

本项目按 SimpleFOC 思路只保留四个必要组成：

```text
DRV8301/TIM1 Driver
AS5600 Sensor
ADC CurrentSense
Current-mode FOC Motor
```

最终运行路径固定为：

```text
1 ms：AS5600 更新机械角和缓存电角
15 kHz：同步电流采样 -> Clarke/Park -> id/iq PI -> PWM
1 kHz：三相电压采样，仅监控
低频：DRV8301 状态检查
```

最终功能固定为：

- AS5600 有感；
- 三相电流闭环；
- `id_ref=0`；
- `iq_ref` 转矩控制；
- 速度闭环；
- 位置、速度、电流三级串级闭环；
- 三相电压监控；
- DRV8301 六路 PWM 驱动和故障关断。

### 20.1 环路宏配置

环路功能通过 `Core/Inc/main.h` 中的编译宏选择：

```c
#define USE_CURRENT_LOOP  1
#define USE_SPEED_LOOP    0
#define USE_POSITION_LOOP 0
```

| 控制模式 | `USE_CURRENT_LOOP` | `USE_SPEED_LOOP` | `USE_POSITION_LOOP` |
| -------- | ------------------ | ---------------- | ------------------- |
| 电流环   | 1                  | 0                | 0                   |
| 速度环   | 1                  | 1                | 0                   |
| 位置环   | 1                  | 1                | 1                   |

宏配置遵循以下依赖关系：

- 三个宏只能定义为 `0` 或 `1`；
- 速度环必须依赖电流环，即 `USE_SPEED_LOOP=1` 时必须设置 `USE_CURRENT_LOOP=1`；
- 位置环必须依赖速度环，即 `USE_POSITION_LOOP=1` 时必须设置 `USE_SPEED_LOOP=1`；
- 不合法的组合通过 `#error` 在编译阶段终止构建；
- 默认配置为仅启用电流环，避免外环参数未完成台架整定时电机意外运动；
- `0/0/0` 可以用于代码裁剪和构建检查，但不作为正常电机控制模式。

### 20.2 控制环运行链路

启用位置环时，控制器按以下三级串级链路运行：

```text
position_ref（累计机械角 rad）
    -> 位置 PID
    -> speed_ref（rad/s，限制为 +/-10 rad/s）
    -> 速度 PID
    -> iq_ref（A，限制为 +/-0.3 A）
    -> id/iq 电流 PI，id_ref 固定为 0 A
    -> SVPWM
```

各环路调度周期如下：

| 环路 | 执行位置 | 周期/频率 | 输出 |
| ---- | -------- | --------- | ---- |
| 位置环 | 主循环中 AS5600 更新成功后 | 1 ms / 1 kHz | 目标速度 `speed_ref` |
| 速度环 | 主循环中 AS5600 更新成功后 | 1 ms / 1 kHz | 目标电流 `iq_ref` |
| 电流环 | ADC 注入转换完成中断 | 15 kHz | `ud/uq` 和下一周期 PWM |

速度反馈由 AS5600 计算，经一阶低通滤波后进入速度 PID。位置反馈使用 AS5600 的累计机械角，因此位置参考值允许跨越多圈。

### 20.3 参考值接口

对外控制接口声明在 `Core/Inc/main.h`：

```c
int BLDC_SetIqReference(float reference);
int BLDC_SetSpeedReference(float reference);
int BLDC_SetPositionReference(float reference);
```

| 接口 | 编译/使用条件 | 单位 | 处理方式 |
| ---- | ------------- | ---- | -------- |
| `BLDC_SetIqReference()` | 仅电流环模式 | A | 限制为 `+/-0.3 A` |
| `BLDC_SetSpeedReference()` | 速度环开启且位置环关闭 | rad/s | 限制为 `+/-10 rad/s` |
| `BLDC_SetPositionReference()` | 位置环开启 | 累计机械角 rad | 作为多圈位置目标 |

接口仅在电机处于 `BLDC_STATE_RUN` 且没有锁存故障时接受新参考值。返回 `0` 表示设置成功，返回 `-1` 表示当前控制模式或运行状态不允许设置。主循环更新参考值时使用临界区，与 15 kHz 电流中断之间不发生数据竞争。

### 20.4 启动、停止和故障行为

- 仅电流环模式启动后使用原有 `FOC_INITIAL_IQ_REF`，当前为 `0.10 A`；
- 速度环模式启动目标为 `0 rad/s`，必须由上层显式设置速度目标；
- 位置环模式将启动目标初始化为当前位置，避免进入闭环时产生位置跳变；
- `BLDC_Stop()` 或故障关断时，将 `iq_ref` 清零并复位所有已启用环路的 PID；
- 传感器更新失败、ADC 异常、相电流越限、采样窗口无效或 DRV8301 故障时，仍沿用统一锁存故障和硬件关断路径。

### 20.5 代码实现位置

| 文件 | 实现内容 |
| ---- | -------- |
| `Core/Inc/main.h` | 环路宏、依赖校验和参考值接口声明 |
| `Core/Src/main.c` | PID 初始化、参考值管理、1 kHz 外环调度和 15 kHz 电流环调用 |
| `FOC_Driver/foc.c/.h` | 电流环 Clarke/Park、DQ 电流 PI 及宏裁剪 |
| `FOC_Driver/foc_test.c/.h` | `Foc_VelocityLoop()`、`Foc_PositionLoop()` 及宏裁剪 |
| `FOC_Driver/pid.c/.h` | 固定周期 PID、输出限幅、积分抗饱和和输出斜率限制 |
| `FOC_Driver/lowpass_filter.c/.h` | AS5600 速度反馈低通滤波 |

### 20.6 构建验证结果

使用 Keil ARMCC5 对环路组合进行构建验证：

| 配置 | 构建结果 | Code | RO-data | RW-data | ZI-data |
| ---- | -------- | ---: | ------: | ------: | ------: |
| 电流环 `1/0/0` | 0 Error，0 Warning | 29560 | 512 | 152 | 4912 |
| 速度环 `1/1/0` | 0 Error，0 Warning | 29876 | 512 | 156 | 4964 |
| 位置环 `1/1/1` | 0 Error，0 Warning | 30020 | 512 | 160 | 5008 |
| 全部关闭 `0/0/0` | 0 Error，0 Warning | 28440 | 512 | 152 | 4832 |

最终提交前配置恢复为电流环 `1/0/0`。上述结果只证明编译和链接通过；速度环、位置环的 PID 参数仍需在限流电源、小目标指令和可靠机械限位条件下完成实机整定。

后续代码修改必须围绕该最小运行链路实施，不因参考 SimpleFOC 而扩大项目范围。

---

## 21. TIM1 与 ADC 配置审查（2026-08-06）

本节对 `Core/Src/tim.c` 和 `Core/Src/adc.c` 的当前配置进行逐项审查，确认是否满足第 3、9、13 节的方案要求。

### 21.1 时钟基础计算

```text
MCU：STM32F407，SYSCLK = 168 MHz
TIM1 挂载 APB2，APB2 预分频 = 2，定时器时钟 = 168 MHz
ADC 时钟 = 84 MHz / DIV4 = 21 MHz
```

### 21.2 TIM1 配置审查

**已正确的配置：**

| 配置项                 | 当前值                          | 方案要求          | 结论 |
| ---------------------- | ------------------------------- | ----------------- | ---- |
| CounterMode            | CENTERALIGNED1                  | 中心对齐          | ✓   |
| PWM 频率               | 168 MHz / (2×5600×1) = 15 kHz | 15 kHz            | ✓   |
| CH1/2/3 模式           | PWM1                            | 互补 PWM 输出     | ✓   |
| CH4 模式               | TIMING（仅比较触发）            | 产生 ADC 注入触发 | ✓   |
| CH1~CH4 预加载         | 全部启用                        | CCR 同边界生效    | ✓   |
| ARPE（自动重装预加载） | ENABLE                          | 保持稳定 ARR      | ✓   |
| 死区时间               | 100 ticks ≈ 595 ns             | 需台架确认        | 待定 |

**存在问题：**

| 配置项                      | 当前值      | 应为        | 影响                                            |
| --------------------------- | ----------- | ----------- | ----------------------------------------------- |
| **RepetitionCounter** | **0** | **1** | **每 PWM 周期产生两次 UEV，CCR 提前生效** |

**RepetitionCounter = 0 的根因分析：**

STM32F4 中心对齐模式下计数器每个完整 PWM 周期有两次溢出事件：

- 上溢：CNT 从 0 计到 ARR（峰值）
- 下溢：CNT 从 ARR 计回 0（谷值）

`RepetitionCounter = 0` 表示每次溢出都立即产生 UEV，即每个 PWM 周期产生 **两次** UEV（频率 30 kHz）。后果：

1. CCR1~CCR4 的预加载值在每个峰值和谷值都会更新生效，若 FOC 算法在计数器运行中途写入新 CCR，可能在下一个峰值（而非谷值）提前生效，导致同一 PWM 周期内三路 PWM 和 CH4 触发点不一致。
2. 第 9.3 节要求"CCR1~CCR4 在同一更新边界生效"——这在 RC=0 的情况下无法保证。
3. 方案目标"一个 PWM 周期只产生一次期望的电流环回调"依赖 UEV 每周期只触发一次，RC=0 会导致注入 ADC 在谷值和峰值各被触发一次（若 CCR4 设置不当），电流 ISR 频率变为 30 kHz，严重超出预期。

**推荐配置：**

```c
htim1.Init.RepetitionCounter = 1;
```

将 RepetitionCounter 设为 1，两次溢出才产生一次 UEV，UEV 在下溢（谷值）时触发：

- 所有 CCR 在同一个谷值原子更新；
- 每个完整 15 kHz PWM 周期只产生一次 UEV；
- ADC 注入 CH4 触发严格与 PWM 谷值对齐，每周期只触发一次；
- 电流 ISR 固定 15 kHz，符合方案要求。

### 21.3 ADC 配置审查

**注入组（电流采样，方案核心路径）：**

| 配置项           | 当前值                 | 方案要求      | 结论 |
| ---------------- | ---------------------- | ------------- | ---- |
| 触发源           | T1_CC4 上升沿          | TIM1_CH4 触发 | ✓   |
| 注入通道数       | 3                      | U/V/W 三相    | ✓   |
| 通道映射         | CH3/4/5 → PA3/PA4/PA5 | PA3/PA4/PA5   | ✓   |
| GPIO 标识        | I_CHANNEL_U/V/W        | 三相电流输入  | ✓   |
| 采样时间         | 15 cycles              | ≥ 15 cycles  | ✓   |
| AutoInjectedConv | DISABLE                | 独立触发      | ✓   |

**注入组 ADC 时序验证：**

```text
ADC 时钟 = 21 MHz，采样时间 = 15 cycles，转换时间 = 12 cycles
单通道耗时 = (15 + 12) / 21 MHz = 1.286 μs
三通道共 = 3 × 27 cycles / 21 MHz = 3.857 μs
折算 TIM1 ticks = 3.857 μs × 168 MHz ≈ 648 ticks

main.h 中 PWM_ADC_SEQUENCE_TICKS = 648U  → 吻合 ✓
```

ADC 转换时序计算与宏定义完全一致，注入组采样窗口计算逻辑正确。

**普通组（电压监控，非 FOC 路径）：**

| 配置项       | 当前值                 | 方案要求           | 结论         |
| ------------ | ---------------------- | ------------------ | ------------ |
| 触发源       | T2_TRGO 上升沿         | TIM2 TRGO 约 1 kHz | ✓           |
| 通道数       | 3                      | U/V/W 三相端电压   | ✓           |
| 通道映射     | CH0/1/2 → PA0/PA1/PA2 | PA0/PA1/PA2        | ✓           |
| GPIO 标识    | V_CHANNEL_U/V/W        | 三相端电压输入     | ✓           |
| DMA 模式     | CIRCULAR               | 持续采样           | ✓           |
| DMA 数据宽度 | HALFWORD（16 位）      | ADC 12 位数据对齐  | ✓（已修正） |

DMA 宽度已于 2026-08-06 由 WORD 改为 HALFWORD，与 ADC 12 位右对齐数据匹配，目标缓冲区应声明为 `uint16_t[3]`，代码审查阶段确认。

### 21.4 ADC 采样窗口有效性分析

方案第 9.3 节要求 CH4 触发点位于"三相低侧电流均可测的稳定窗口"。在中心对齐 PWM 中，三路低侧管同时导通的窗口位于 CNT 接近谷值（0）附近。

当前 `main.h` 的窗口约束宏：

```c
PWM_MIN_COMPARE              = 200   (PWM_ADC_PEAK_BLANKING_TICKS)
PWM_MAX_COMPARE              = 5600 - 648 - 100 - 200 = 4652
PWM_ADC_TRIGGER_LATEST       = 5600 - 200 = 5400
```

CH4 = CCR4 必须满足 CCR4 ≤ PWM_ADC_TRIGGER_LATEST，且三路 CCR1~3 均处于低侧导通状态时触发。这套逻辑在修正 RepetitionCounter 后仍然有效，动态 CCR4 计算逻辑可以保留。
