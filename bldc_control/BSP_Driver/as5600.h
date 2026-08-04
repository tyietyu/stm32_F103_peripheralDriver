#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <stdint.h>

#include "hal_iic.h"

#define AS5600_RAW_ADDR             0x36
#define AS5600_WRITE_ADDR           (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR            ((AS5600_RAW_ADDR << 1) | 1)
#define AS5600_RAW_ANGLE_REGISTER   0x0C

#define AS5600_RESOLUTION           4096U
#define AS5600_RAW_MASK             (AS5600_RESOLUTION - 1U)

typedef struct
{
  iic_bus_t *i2c_ins;
  float prev_angle;
  uint32_t prev_angle_ts;
  float rotation_offset;

  float vel_rotation_offset;
  float vel_prev_angle;
  uint32_t vel_prev_angle_ts;

  uint16_t error_count;
  bool valid;
} AS5600_T;

int AS5600_Init(AS5600_T *a);

uint16_t AS5600_GetRawAngle(AS5600_T *a);
float AS5600_GetOnceAngle(AS5600_T *a);
float AS5600_GetAngle(AS5600_T *a);
void AS5600_Update(AS5600_T *a);
float AS5600_GetVelocity(AS5600_T *a);
bool AS5600_IsValid(AS5600_T *a);

#endif
