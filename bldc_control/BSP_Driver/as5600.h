#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 * 应用层角度传感器接口。底层寄存器读写由官方 LibDriver（AS5600_driver.c/.h）
 * 经 as5600_handle_t 回调完成，本文件只暴露应用所需的配置与查询接口。
 */

#define AS5600_CONF_HIGH_REGISTER   0x07U
#define AS5600_CONF_LOW_REGISTER    0x08U
#define AS5600_STATUS_MAGNET_DETECT 0x20U  /* MD: 磁体存在 */
#define AS5600_STATUS_MAGNET_WEAK   0x10U  /* ML: AGC 触顶，磁场过弱 */
#define AS5600_STATUS_MAGNET_STRONG 0x08U  /* MH: AGC 触底，磁场过强 */
#define AS5600_RESOLUTION           4096U

/*
 * CONF 是 14 位寄存器，0x07 为高字节(bit13:8)、0x08 为低字节(bit7:0)。
 * 高字节内：bit1:0 = SF(CONF bit9:8)，bit4:2 = FTH(bit12:10)，bit5 = WD(bit13)。
 * SF 出厂默认 16x，阶跃延迟约 2.2 ms，对 1 kHz 修正是显著相位滞后。改 2x 后
 * 延迟约 0.286 ms、RMS 噪声约 0.043° = 0.75 mrad，仍小于一个 LSB(1.534 mrad)，
 * 噪声代价被角度量化淹没。
 * FTH = 000 只用慢速滤波，不引入快速滤波的额外噪声。
 * WD = 0：看门狗会在角度长时间不变时切到 LPM3，电机静止时必须关闭。
 */
#define AS5600_CONF_SF_2X           0x03U
#define AS5600_CONF_FTH_SLOW_ONLY   0x00U
#define AS5600_CONF_WD_OFF          0x00U
#define AS5600_CONF_HIGH_VALUE      (AS5600_CONF_SF_2X | \
                                     (AS5600_CONF_FTH_SLOW_ONLY << 2) | \
                                     (AS5600_CONF_WD_OFF << 5))
#define AS5600_CONF_HIGH_MASK       0x3FU
/* CONF 低字节 (0x08)：PM=NOM、HYST=OFF、OUTS/PWMF 不使用，全部写 0 */
#define AS5600_CONF_LOW_VALUE       0x00U
#define AS5600_CONF_LOW_MASK        0xFFU

/*
 * 拒绝明显异常跳变的阈值（1/8 圈）。连续拒绝达到 AS5600_MAX_STEP_REJECT 次后
 * 强制以当前读数重新同步基准，避免单次误码或调用周期抖动导致传感器永久失效。
 */
#define AS5600_MAX_STEP_COUNTS      512
#define AS5600_MAX_STEP_REJECT      3U

/* 单次阻塞读/写的总线超时。轮询读在 TIM2 中断内执行（100 kHz 下正常约 0.5 ms），
   该值同时决定总线故障时中断的最长占用时间，取 2 ms 为正常读的 4 倍余量。
   不能放大：必须小于 foc_app.c 的 FOC_ENCODER_MAX_AGE_MS(5)，否则一次超时就会把
   角度样本拖过新鲜度窗口，直接锁 ENCODER_ERR */
#define AS5600_I2C_TIMEOUT_MS       2U

/*
 * I2C1 引脚，总线恢复时临时切成 GPIO 位敲，必须与 i2c.c 的 HAL_I2C_MspInit 一致。
 * 位敲半周期 5 us（100 kHz），最多 9 个 SCL 脉冲（8 位数据 + ACK）+ STOP，总计约 100 us。
 */
#define AS5600_SCL_PORT             GPIOB
#define AS5600_SCL_PIN              GPIO_PIN_6
#define AS5600_SDA_PORT             GPIOB
#define AS5600_SDA_PIN              GPIO_PIN_7
#define AS5600_RECOVER_HALF_US      5U
#define AS5600_RECOVER_MAX_CLOCKS   9U

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t raw_angle;
  float mechanical_angle;
  float full_angle;
  float rotation_offset;
  uint32_t last_success_tick;
  uint16_t error_count;
  uint8_t step_reject_count;
  bool valid;
} AS5600_T;

/*
 * 阻塞式初始化，只在上电跑一次：读 STATUS 作诊断打点、写 CONF 并回读校验、读取首个角度。
 * 磁体检测判定已取消：MD/ML/MH 只记录不判定，不再作为初始化成败条件。
 * 必须在 MX_I2C1_Init() 与 delay_init() 之后调用（CONF 写入后的建立等待用 delay_us）。
 */
int AS5600_Init(AS5600_T *sensor, I2C_HandleTypeDef *hi2c);
/*
 * 轮询读取一次 RAW_ANGLE 并完成解算发布，由周期任务（TIM2 中断）调用。
 * 100 kHz 总线下单次读约 0.5 ms，不能按 4 kHz 每周期调用。
 * 返回 0 成功；-1 失败（总线卡死时内部做 I2C 总线恢复：补 SCL 时钟 + STOP + 外设复位）。
 */
int AS5600_Update(AS5600_T *sensor);
/* 运行期 STATUS 观测（只打点不判定磁体）：0 读取正常，-2 读失败 */
int AS5600_CheckStatus(AS5600_T *sensor);
/* 单圈机械角 [0, 2π)，供电角度换算 */
float AS5600_GetOnceAngle(const AS5600_T *sensor);
/* 累计机械角（含圈数），供位置环 */
float AS5600_GetAngle(const AS5600_T *sensor);
bool AS5600_IsValid(const AS5600_T *sensor);
bool AS5600_IsFresh(const AS5600_T *sensor, uint32_t now, uint32_t max_age_ms);

#endif
