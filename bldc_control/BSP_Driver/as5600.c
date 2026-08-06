#include "as5600.h"

#include <math.h>
#include <stddef.h>

#include "delay.h"
#include "main.h"

#define AS5600_TWO_PI          6.283185307179586f
#define AS5600_MAX_ERROR_COUNT 0xFFFFU

static iic_bus_t i2c_bus = {
  .IIC_SCL_PORT = AS5600_SCL_GPIO_Port,
  .IIC_SCL_PIN = AS5600_SCL_Pin,
  .IIC_SDA_PORT = AS5600_SDA_GPIO_Port,
  .IIC_SDA_PIN = AS5600_SDA_Pin,
};

static int AS5600_ReadRegister(AS5600_T *sensor, uint8_t reg, uint8_t *value)
{
  if ((sensor == NULL) || (sensor->i2c_ins == NULL) || (value == NULL))
  {
    return -1;
  }

  return (IIC_Read_Multi_Byte(sensor->i2c_ins, AS5600_RAW_ADDR, reg, 1U, value) == 0U) ? 0 : -1;
}

static int AS5600_ReadRawAngle(AS5600_T *sensor, uint16_t *raw_angle)
{
  uint8_t buffer[2] = {0U};

  if ((sensor == NULL) || (sensor->i2c_ins == NULL) || (raw_angle == NULL))
  {
    return -1;
  }

  if (IIC_Read_Multi_Byte(sensor->i2c_ins, AS5600_RAW_ADDR,
                          AS5600_RAW_ANGLE_REGISTER, 2U, buffer) != 0U)
  {
    return -1;
  }

  *raw_angle = (((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]) & AS5600_RAW_MASK;
  return 0;
}

static void AS5600_RecordReadError(AS5600_T *sensor)
{
  sensor->valid = false;
  if (sensor->error_count < AS5600_MAX_ERROR_COUNT)
  {
    ++sensor->error_count;
  }
}

static void AS5600_PublishSample(AS5600_T *sensor, uint16_t raw_angle, uint32_t now)
{
  sensor->previous_raw_angle = sensor->raw_angle;
  sensor->raw_angle = raw_angle;
  sensor->mechanical_angle = (float)raw_angle * AS5600_TWO_PI /
                             (float)AS5600_RESOLUTION;
  sensor->full_angle = sensor->rotation_offset + sensor->mechanical_angle;
  sensor->last_success_tick = now;
  sensor->error_count = 0U;
  sensor->valid = true;
}

int AS5600_Init(AS5600_T *sensor)
{
  uint8_t status = 0U;
  uint16_t raw_angle = 0U;
  uint32_t now;

  if (sensor == NULL)
  {
    return -1;
  }

  sensor->i2c_ins = &i2c_bus;
  sensor->raw_angle = 0U;
  sensor->previous_raw_angle = 0U;
  sensor->mechanical_angle = 0.0f;
  sensor->full_angle = 0.0f;
  sensor->rotation_offset = 0.0f;
  sensor->last_success_tick = 0U;
  sensor->velocity_previous_angle = 0.0f;
  sensor->velocity_previous_tick = 0U;
  sensor->error_count = 0U;
  sensor->magnet_detected = false;
  sensor->valid = false;

  delay_init();
  if (IICInit(sensor->i2c_ins) != 0U)
  {
    return -1;
  }

  if ((AS5600_ReadRegister(sensor, AS5600_STATUS_REGISTER, &status) != 0) ||
      ((status & AS5600_STATUS_MAGNET_DETECT) == 0U))
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }
  sensor->magnet_detected = true;

  if (AS5600_ReadRawAngle(sensor, &raw_angle) != 0)
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }

  now = HAL_GetTick();
  AS5600_PublishSample(sensor, raw_angle, now);
  sensor->previous_raw_angle = raw_angle;
  sensor->velocity_previous_angle = sensor->full_angle;
  sensor->velocity_previous_tick = now;
  return 0;
}

int AS5600_Update(AS5600_T *sensor)
{
  int32_t delta;
  uint16_t raw_angle;
  uint32_t now;

  if ((sensor == NULL) || (!sensor->magnet_detected))
  {
    return -1;
  }

  if (AS5600_ReadRawAngle(sensor, &raw_angle) != 0)
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }

  delta = (int32_t)raw_angle - (int32_t)sensor->raw_angle;
  if (delta > ((int32_t)AS5600_RESOLUTION / 2))
  {
    delta -= (int32_t)AS5600_RESOLUTION;
  }
  else if (delta < -((int32_t)AS5600_RESOLUTION / 2))
  {
    delta += (int32_t)AS5600_RESOLUTION;
  }

  if ((delta > AS5600_MAX_STEP_COUNTS) || (delta < -AS5600_MAX_STEP_COUNTS))
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }

  if (((int32_t)raw_angle - (int32_t)sensor->raw_angle) >
      ((int32_t)AS5600_RESOLUTION / 2))
  {
    sensor->rotation_offset -= AS5600_TWO_PI;
  }
  else if (((int32_t)raw_angle - (int32_t)sensor->raw_angle) <
           -((int32_t)AS5600_RESOLUTION / 2))
  {
    sensor->rotation_offset += AS5600_TWO_PI;
  }

  now = HAL_GetTick();
  AS5600_PublishSample(sensor, raw_angle, now);
  return 0;
}

uint16_t AS5600_GetRawAngle(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->raw_angle : 0U;
}

float AS5600_GetOnceAngle(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->mechanical_angle : 0.0f;
}

float AS5600_GetAngle(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->full_angle : 0.0f;
}

float AS5600_GetVelocity(AS5600_T *sensor)
{
  float dt;
  float velocity;

  if (!AS5600_IsValid(sensor))
  {
    return 0.0f;
  }

  dt = (float)(sensor->last_success_tick - sensor->velocity_previous_tick) * 1e-3f;
  if (dt <= 0.0f)
  {
    return 0.0f;
  }

  velocity = (sensor->full_angle - sensor->velocity_previous_angle) / dt;
  sensor->velocity_previous_angle = sensor->full_angle;
  sensor->velocity_previous_tick = sensor->last_success_tick;
  return velocity;
}

uint32_t AS5600_GetLastSuccessTick(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->last_success_tick : 0U;
}

uint16_t AS5600_GetErrorCount(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->error_count : 0U;
}

bool AS5600_IsValid(const AS5600_T *sensor)
{
  return (sensor != NULL) && sensor->valid && sensor->magnet_detected;
}

bool AS5600_IsFresh(const AS5600_T *sensor, uint32_t now, uint32_t max_age_ms)
{
  return AS5600_IsValid(sensor) &&
         ((uint32_t)(now - sensor->last_success_tick) <= max_age_ms);
}
