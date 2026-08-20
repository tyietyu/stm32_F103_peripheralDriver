# BLDC FOC 驱动项目代码工程

## DRV8301的芯片手册路径

* "E:\0001_LCWork\datasheet\drv8301.pdf"

## BLDC驱动板的原理图

* 原理图文件路径"E:\0001_LCWork\project_STM32\BLDC_HW_Driver\SCH_Motor.pdf"

### AS5600的芯片手册

* 路径："E:\0001_LCWork\datasheet\as5600.pdf"

## BSP 驱动使用说明

主控实际为 **STM32F407V(E-G)Tx**

### 初始化顺序

```c
delay_init();                 /* AS5600 写 CONF 后的建立等待依赖它 */
AS5600_Init(&sensor, &hi2c1); /* 须在 MX_I2C1_Init() 之后；返回 0 表示磁体正常且 CONF 已回读校验 */
DRV8301_Init(&hspi2);         /* 返回 DRV8301_OK 表示已使能并完成配置 */
```

### DRV8301

* `EN_GATE`(PC0) 上电默认低电平，此时器件休眠、栅极关断、**SPI 完全不响应**。
  `DRV8301_Init()` 会先拉高 EN_GATE 并等待 10 ms（tSPI_READY 最大值）再访问寄存器。
* `DRV8301_Shutdown()` 拉低 EN_GATE 立即硬关断栅极。**本板 `nFAULT`/`nOCTW` 未接 MCU**
  （原理图上经 SN74AHC1G08 与门只驱动 LED1 与 Q1），故障无法中断上报，上层必须：
  1. 由低频任务轮询 `DRV8301_ReadStatus()` 检查 STATUS1 的 FAULT 位；
  2. 急停路径直接调用 `DRV8301_Shutdown()`。
     即时过流保护依赖芯片内部 OC latch shutdown（CTRL1 已配置为 `OC_MODE_LATCH_SD`）。
* Device ID 手册未给出期望值，驱动不做校验，需要时由上层从 `status2` 的 bit3:0 自行读取。

### 电流采样标定

三相 shunt 均为 10 mΩ；C 相不走 DRV8301 内部放大器而是外接 LM2904（R45=1k / R47=10k，
增益 10 V/V），因此内部增益同样取 10 V/V，三相共用一套系数：

* `DRV8301_VOLTS_PER_AMP` = 0.010 Ω × 10 V/V = **0.1 V/A**
* 3.3 V 量程叠加 REF/2 偏置后约 **±16.5 A**

需要改增益时覆盖 `DRV8301_SHUNT_GAIN`（可选 10/20/40/80），但会与 C 相外部运放失配，
需在上层对 C 相单独用一套标定系数。

### AS5600

* 走硬件 I2C1（PB6/PB7，400 kHz）。角度读取为 DMA 非阻塞：周期任务调
  `AS5600_UpdateStart()` 发起，解算与发布在 `AS5600_OnRxComplete()` 里完成，
  由 `HAL_I2C_MemRxCpltCallback`（实现在 `main.c`）转发；错误走 `AS5600_OnRxError()`。
  返回 -2 表示上一次尚未完成，本周期跳过，不算错误。
* 从机拉死 SDA 时 DMA 不产生任何回调，`AS5600_UpdateStart()` 内置 5 ms 超时，
  超时后 DeInit/Init 复位 I2C1，避免角度通道永久失效。
* 单次步进超过 1/8 圈（512 counts）按误码丢弃，
  连续 `AS5600_MAX_STEP_REJECT`(3) 次后以当前读数重新同步，避免永久失效。
* `AS5600_CheckStatus()` 需由低频任务定期调用：磁体掉落后器件仍返回语法合法的角度值，
  只读 RAW_ANGLE 发现不了。返回 -1 表示磁体异常须关断，-2 表示总线读失败或 DMA 读进行中
  （阻塞读与 DMA 读不能在同一总线上重入）。
* `AS5600_GetAngle()` 为累计角（含圈数，供位置环），`AS5600_GetOnceAngle()` 为单圈机械角
  （供电角度换算）。驱动不提供速度输出，速度反馈须由 FOC 侧的观测器给出。
