#include "as5600.h"
#include "hal_iic.h"
#include "log.h"
#include "main.h"
#include "delay.h"
#include <math.h>

#define _2PI    6.283185307179586f
#define AS5600_MAX_ERROR_COUNT  0xFFFFU

iic_bus_t i2c_bus = {
    .IIC_SCL_PORT = AS5600_SCL_GPIO_Port,
    .IIC_SCL_PIN = AS5600_SCL_Pin,
    .IIC_SDA_PORT = AS5600_SDA_GPIO_Port,
    .IIC_SDA_PIN = AS5600_SDA_Pin,
};

static int AS5600_ReadRawAngle(AS5600_T *a, uint16_t *raw_angle)
{
  uint8_t buffer[2] = {0};

  if ((a == NULL) || (a->i2c_ins == NULL) || (raw_angle == NULL))
  {
    return -1;
  }

  if (IIC_Read_Multi_Byte(a->i2c_ins, AS5600_RAW_ADDR, AS5600_RAW_ANGLE_REGISTER, 2, buffer) != 0)
  {
    return -1;
  }

  *raw_angle = (((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]) & AS5600_RAW_MASK;
  return 0;
}

static void AS5600_RecordReadError(AS5600_T *a)
{
  if (a == NULL)
  {
    return;
  }

  a->valid = false;
  if (a->error_count < AS5600_MAX_ERROR_COUNT)
  {
    a->error_count++;
  }
}

static void AS5600_RecordReadOk(AS5600_T *a)
{
  if (a == NULL)
  {
    return;
  }

  a->valid = true;
  a->error_count = 0;
}

int AS5600_Init(AS5600_T *a)
{
  uint16_t angle = 0;
  uint32_t now = HAL_GetTick();

  if (a == NULL)
  {
    return -1;
  }

  delay_init();
  if (IICInit(&i2c_bus) != 0)
  {
    a->valid = false;
    return -1;
  }

  a->i2c_ins = &i2c_bus;
  a->prev_angle = 0.0f;
  a->prev_angle_ts = now;
  a->rotation_offset = 0.0f;
  a->vel_rotation_offset = 0.0f;
  a->vel_prev_angle = 0.0f;
  a->vel_prev_angle_ts = now;
  a->error_count = 0;
  a->valid = false;

  if (AS5600_ReadRawAngle(a, &angle) != 0)
  {
    AS5600_RecordReadError(a);
    CAW_LOG_ERROR("AS5600 init failed");
    return -1;
  }

  a->prev_angle = (float)angle;
  a->vel_prev_angle = (float)angle;
  AS5600_RecordReadOk(a);
  return 0;
}

uint16_t AS5600_GetRawAngle(AS5600_T *a)
{
  uint16_t raw_angle = 0;

  if (AS5600_ReadRawAngle(a, &raw_angle) != 0)
  {
    AS5600_RecordReadError(a);
    CAW_LOG_WARN("AS5600_GetRawAngle error");
    return (a == NULL) ? 0U : (uint16_t)a->prev_angle;
  }

  AS5600_RecordReadOk(a);
  return raw_angle;
}

float AS5600_GetOnceAngle(AS5600_T *a)
{
  return (float)AS5600_GetRawAngle(a) * _2PI / (float)AS5600_RESOLUTION;
}

float AS5600_GetAngle(AS5600_T *a)
{
  if (a == NULL)
  {
    return 0.0f;
  }

  return (a->rotation_offset + (a->prev_angle / (float)AS5600_RESOLUTION) * _2PI);
}

void AS5600_Update(AS5600_T *a)
{
  uint16_t raw_angle = 0;
  uint32_t now = HAL_GetTick();
  float angle_data;
  float delta;

  if (AS5600_ReadRawAngle(a, &raw_angle) != 0)
  {
    AS5600_RecordReadError(a);
    return;
  }

  angle_data = (float)raw_angle;
  delta = angle_data - a->prev_angle;
  if (fabsf(delta) > (0.8f * (float)AS5600_RESOLUTION))
  {
    a->rotation_offset += (delta > 0.0f ? -_2PI : _2PI);
  }

  a->prev_angle = angle_data;
  a->prev_angle_ts = now;
  AS5600_RecordReadOk(a);
}

float AS5600_GetVelocity(AS5600_T *a)
{
  float ts;
  float vel;

  if ((a == NULL) || (!a->valid))
  {
    return 0.0f;
  }

  ts = (float)(a->prev_angle_ts - a->vel_prev_angle_ts) * 1e-3f;
  if (ts <= 0.0f)
  {
    ts = 1e-3f;
  }

  vel = ((a->rotation_offset - a->vel_rotation_offset) +
         (a->prev_angle - a->vel_prev_angle) / (float)AS5600_RESOLUTION * _2PI) / ts;
  a->vel_prev_angle = a->prev_angle;
  a->vel_rotation_offset = a->rotation_offset;
  a->vel_prev_angle_ts = a->prev_angle_ts;
  return vel;
}

bool AS5600_IsValid(AS5600_T *a)
{
  return (a != NULL) && a->valid;
}
