#include "as5600.h"
#include <math.h>
#include <stddef.h>
#include "delay.h"

#define AS5600_TWO_PI          6.283185307179586f
#define AS5600_MAX_ERROR_COUNT 0xFFFFU

static int AS5600_ReadRegister(AS5600_T *sensor, uint8_t reg, uint8_t *value)
{
  if ((sensor == NULL) || (sensor->hi2c == NULL) || (value == NULL))
  {
    return -1;
  }

  return (HAL_I2C_Mem_Read(sensor->hi2c, AS5600_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, value, 1U,
                           AS5600_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

/* CONF 为易失寄存器，每次上电写入，不使用 BURN_SETTING，不消耗 ZMCO 次数 */
static int AS5600_WriteRegisterVerified(AS5600_T *sensor, uint8_t reg,
                                        uint8_t value, uint8_t mask)
{
  uint8_t readback = 0U;

  if (HAL_I2C_Mem_Write(sensor->hi2c, AS5600_I2C_ADDR, reg,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U,
                        AS5600_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }

  delay_us(500U);
  if (AS5600_ReadRegister(sensor, reg, &readback) != 0)
  {
    return -1;
  }

  return ((readback & mask) == (value & mask)) ? 0 : -1;
}

/* 磁体状态判定：MD 必须置位，ML/MH 必须清零 */
static int AS5600_ValidateStatus(uint8_t status)
{
  if ((status & AS5600_STATUS_MAGNET_DETECT) == 0U)
  {
    return -1;
  }
  if ((status & (AS5600_STATUS_MAGNET_WEAK | AS5600_STATUS_MAGNET_STRONG)) != 0U)
  {
    return -1;
  }

  return 0;
}

/* 阻塞式读取原始角度，Init 取首个基准角度与运行期轮询共用 */
static int AS5600_ReadRawAngle(AS5600_T *sensor, uint16_t *raw_angle)
{
  uint8_t buffer[2] = {0U};

  if ((sensor == NULL) || (sensor->hi2c == NULL) || (raw_angle == NULL))
  {
    return -1;
  }

  if (HAL_I2C_Mem_Read(sensor->hi2c, AS5600_I2C_ADDR, AS5600_RAW_ANGLE_REGISTER,
                       I2C_MEMADD_SIZE_8BIT, buffer, 2U,
                       AS5600_I2C_TIMEOUT_MS) != HAL_OK)
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
  sensor->raw_angle = raw_angle;
  sensor->mechanical_angle = (float)raw_angle * AS5600_TWO_PI /
                             (float)AS5600_RESOLUTION;
  sensor->full_angle = sensor->rotation_offset + sensor->mechanical_angle;
  sensor->last_success_tick = now;
  sensor->error_count = 0U;
  sensor->step_reject_count = 0U;
  sensor->valid = true;
}

/*
 * 总线恢复。从机拉死 SDA/SCL 时 BUSY 标志不会自行清除，HAL 内部会空转 25 ms
 * 等待超时且后续每次读都立即失败，必须复位外设（SWRST 清 BUSY）才能恢复。
 */
static void AS5600_RecoverBus(AS5600_T *sensor)
{
  (void)HAL_I2C_DeInit(sensor->hi2c);
  (void)HAL_I2C_Init(sensor->hi2c);
}

int AS5600_Init(AS5600_T *sensor, I2C_HandleTypeDef *hi2c)
{
  uint8_t status = 0U;
  uint16_t raw_angle = 0U;

  if ((sensor == NULL) || (hi2c == NULL))
  {
    return -1;
  }

  sensor->hi2c = hi2c;
  sensor->raw_angle = 0U;
  sensor->mechanical_angle = 0.0f;
  sensor->full_angle = 0.0f;
  sensor->rotation_offset = 0.0f;
  sensor->last_success_tick = 0U;
  sensor->error_count = 0U;
  sensor->step_reject_count = 0U;
  sensor->magnet_detected = false;
  sensor->valid = false;

  if ((AS5600_ReadRegister(sensor, AS5600_STATUS_REGISTER, &status) != 0) ||
      (AS5600_ValidateStatus(status) != 0))
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }
  sensor->magnet_detected = true;

  /* 慢速滤波必须显式配置：上电默认 16x 的阶跃延迟对 1 kHz 修正不可接受 */
  if ((AS5600_WriteRegisterVerified(sensor, AS5600_CONF_LOW_REGISTER,
                                    AS5600_CONF_LOW_VALUE,
                                    AS5600_CONF_LOW_MASK) != 0) ||
      (AS5600_WriteRegisterVerified(sensor, AS5600_CONF_HIGH_REGISTER,
                                    AS5600_CONF_HIGH_VALUE,
                                    AS5600_CONF_HIGH_MASK) != 0))
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }

  if (AS5600_ReadRawAngle(sensor, &raw_angle) != 0)
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }

  AS5600_PublishSample(sensor, raw_angle, HAL_GetTick());
  return 0;
}

/*
 * 轮询读取并发布一次采样。由 TIM2 中断每 4 个周期（1 kHz）调用一次：
 * 100 kHz 总线下单次读约 0.5 ms，不能在 4 kHz 每周期都做，否则中断占满 CPU。
 */
int AS5600_Update(AS5600_T *sensor)
{
  uint16_t raw_angle;
  int32_t delta;
  float wrap = 0.0f;

  if ((sensor == NULL) || (sensor->hi2c == NULL) || (!sensor->magnet_detected))
  {
    return -1;
  }

  /*
   * 总线空闲时 BUSY 必须为 0；为 1 说明从机把 SDA/SCL 拉死在中途。此时不能交给
   * HAL（其内部会空转 25 ms 等 BUSY 清除），直接复位外设恢复，否则角度通道永久失效。
   */
  if (__HAL_I2C_GET_FLAG(sensor->hi2c, I2C_FLAG_BUSY) != RESET)
  {
    AS5600_RecoverBus(sensor);
    AS5600_RecordReadError(sensor);
    return -1;
  }

  if (AS5600_ReadRawAngle(sensor, &raw_angle) != 0)
  {
    AS5600_RecordReadError(sensor);
    return -1;
  }

  /* 折叠到 ±半圈得到真实步进，折叠方向同时给出跨圈量 */
  delta = (int32_t)raw_angle - (int32_t)sensor->raw_angle;
  if (delta > ((int32_t)AS5600_RESOLUTION / 2))
  {
    delta -= (int32_t)AS5600_RESOLUTION;
    wrap = -AS5600_TWO_PI;
  }
  else if (delta < -((int32_t)AS5600_RESOLUTION / 2))
  {
    delta += (int32_t)AS5600_RESOLUTION;
    wrap = AS5600_TWO_PI;
  }

  if ((delta > AS5600_MAX_STEP_COUNTS) || (delta < -AS5600_MAX_STEP_COUNTS))
  {
    AS5600_RecordReadError(sensor);
    if (sensor->step_reject_count < AS5600_MAX_STEP_REJECT)
    {
      ++sensor->step_reject_count;
      return -1;
    }
  }

  sensor->rotation_offset += wrap;
  AS5600_PublishSample(sensor, raw_angle, HAL_GetTick());
  return 0;
}

/*
 * 运行期磁体状态复检。角度通道只读 RAW_ANGLE，磁体掉落后器件仍会返回
 * 语法合法的角度值，必须由低频任务定期查 STATUS 才能发现。
 * 返回 0 正常，-1 磁体状态异常（须关断），-2 读失败。
 * 读失败与磁体异常必须分开：单次总线抖动不该关断电机，总线真死了角度通道会先
 * 失败，由 sensor_max_age_ms 兜底。
 * 与 TIM2 中断内的轮询读共用总线，调用方须保证期间不发起角度刷新。
 */
int AS5600_CheckStatus(AS5600_T *sensor)
{
  uint8_t status = 0U;

  if (sensor == NULL)
  {
    return -1;
  }

  if (AS5600_ReadRegister(sensor, AS5600_STATUS_REGISTER, &status) != 0)
  {
    AS5600_RecordReadError(sensor);
    return -2;
  }

  if (AS5600_ValidateStatus(status) != 0)
  {
    sensor->magnet_detected = false;
    sensor->valid = false;
    return -1;
  }

  return 0;
}

float AS5600_GetOnceAngle(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->mechanical_angle : 0.0f;
}

float AS5600_GetAngle(const AS5600_T *sensor)
{
  return (sensor != NULL) ? sensor->full_angle : 0.0f;
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
