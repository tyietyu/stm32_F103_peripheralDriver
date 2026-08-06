#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <stdint.h>

#include "hal_iic.h"

#define AS5600_RAW_ADDR             0x36U
#define AS5600_RAW_ANGLE_REGISTER   0x0CU
#define AS5600_STATUS_REGISTER      0x0BU
#define AS5600_STATUS_MAGNET_DETECT 0x20U
#define AS5600_RESOLUTION           4096U
#define AS5600_RAW_MASK             (AS5600_RESOLUTION - 1U)

/* 仅用于拒绝明显异常跳变，当前低速调试下正常步进远小于该阈值。 */
#define AS5600_MAX_STEP_COUNTS      512

typedef struct
{
  iic_bus_t *i2c_ins;
  uint16_t raw_angle;
  uint16_t previous_raw_angle;
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
uint16_t AS5600_GetRawAngle(const AS5600_T *sensor);
float AS5600_GetOnceAngle(const AS5600_T *sensor);
float AS5600_GetAngle(const AS5600_T *sensor);
float AS5600_GetVelocity(AS5600_T *sensor);
uint32_t AS5600_GetLastSuccessTick(const AS5600_T *sensor);
uint16_t AS5600_GetErrorCount(const AS5600_T *sensor);
bool AS5600_IsValid(const AS5600_T *sensor);
bool AS5600_IsFresh(const AS5600_T *sensor, uint32_t now, uint32_t max_age_ms);

#endif
