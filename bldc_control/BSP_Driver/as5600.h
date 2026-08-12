#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <stdint.h>

#include "hal_iic.h"

#define AS5600_RAW_ADDR             0x36U
#define AS5600_RAW_ANGLE_REGISTER   0x0CU
#define AS5600_STATUS_REGISTER      0x0BU
#define AS5600_CONF_HIGH_REGISTER   0x07U
#define AS5600_CONF_LOW_REGISTER    0x08U
#define AS5600_STATUS_MAGNET_DETECT 0x20U  /* MD: 磁体存在 */
#define AS5600_STATUS_MAGNET_WEAK   0x10U  /* ML: AGC 触顶，磁场过弱 */
#define AS5600_STATUS_MAGNET_STRONG 0x08U  /* MH: AGC 触底，磁场过强 */
#define AS5600_STATUS_MASK          (AS5600_STATUS_MAGNET_DETECT | \
                                     AS5600_STATUS_MAGNET_WEAK | \
                                     AS5600_STATUS_MAGNET_STRONG)
#define AS5600_RESOLUTION           4096U
#define AS5600_RAW_MASK             (AS5600_RESOLUTION - 1U)

/*
 * CONF 高字节 (0x07)：bit1:0 = SF，bit4:2 = FTH，bit5 = WD。
 * SF 出厂默认 16x，阶跃延迟约 2.2 ms，对 1 kHz 修正是显著相位滞后。改 2x 后
 * 延迟约 0.286 ms、RMS 噪声约 0.043° = 0.75 mrad，仍小于一个 LSB(1.534 mrad)，
 * 噪声代价被角度量化淹没。
 * FTH = 000 只用慢速滤波，不引入快速滤波的额外噪声。
 * WD = 0：看门狗会在角度长时间不变时切到 LPM3，电机静止时必须关闭。
 *
 * 注意：本次整改无法联网核对 AMS AS5600 datasheet，位定义按已知寄存器表填写，
 * 台架首次运行必须用读回值和阶跃响应实测确认。
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

/* 仅用于拒绝明显异常跳变，当前低速调试下正常步进远小于该阈值。 */
#define AS5600_MAX_STEP_COUNTS      512

typedef struct
{
  iic_bus_t *i2c_ins;
  uint16_t raw_angle;
  float mechanical_angle;
  float full_angle;
  float rotation_offset;
  uint32_t last_success_tick;
  float velocity_previous_angle;
  uint32_t velocity_previous_tick;
  uint16_t error_count;
  bool magnet_detected;
  bool valid;
} AS5600_T;

int AS5600_Init(AS5600_T *sensor);
int AS5600_Update(AS5600_T *sensor);
/* 运行期磁体状态复检：0 正常，-1 磁体异常（须关断），-2 读失败（总线问题） */
int AS5600_CheckStatus(AS5600_T *sensor);
uint16_t AS5600_GetRawAngle(const AS5600_T *sensor);
float AS5600_GetOnceAngle(const AS5600_T *sensor);
float AS5600_GetAngle(const AS5600_T *sensor);
float AS5600_GetVelocity(AS5600_T *sensor);
uint32_t AS5600_GetLastSuccessTick(const AS5600_T *sensor);
uint16_t AS5600_GetErrorCount(const AS5600_T *sensor);
bool AS5600_IsValid(const AS5600_T *sensor);
bool AS5600_IsFresh(const AS5600_T *sensor, uint32_t now, uint32_t max_age_ms);

#endif
