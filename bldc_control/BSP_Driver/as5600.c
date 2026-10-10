#include "as5600.h"
#include "AS5600_driver.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "delay.h"
/* 排查打点：只在 Init / CheckStatus / WriteRegisterVerified（均为 while(1) 上下文）里打，
   AS5600_Update() 跑在 TIM2 中断内，禁止在它及其调用的函数里打日志。格式串只用 ASCII */
#include "log.h"

#define AS5600_TWO_PI          6.283185307179586f
#define AS5600_MAX_ERROR_COUNT 0xFFFFU

/*
 * 官方 LibDriver 句柄（本板只有一颗 AS5600，用文件级静态实例）。
 * 驱动的 iic_read/write 回调签名不含用户上下文，只能经静态变量 s_hi2c 取回 I2C 外设。
 */
static as5600_handle_t s_handle;
static I2C_HandleTypeDef *s_hi2c;

/* I2C1 已由 MX_I2C1_Init() 配置，驱动的 iic_init/iic_deinit 只需返回成功 */
static uint8_t AS5600_IicInit(void)
{
  return 0;
}

static uint8_t AS5600_IicDeinit(void)
{
  return 0;
}

/* 驱动传入 7 位从机地址（AS5600_ADDRESS=0x36），HAL 的 I2C API 需要含读写位的 8 位地址 */
static uint8_t AS5600_IicRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(s_hi2c, (uint16_t)((uint16_t)addr << 1U), reg,
                           I2C_MEMADD_SIZE_8BIT, buf, len,
                           AS5600_I2C_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static uint8_t AS5600_IicWrite(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Write(s_hi2c, (uint16_t)((uint16_t)addr << 1U), reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            AS5600_I2C_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static void AS5600_DelayMs(uint32_t ms)
{
  delay_ms(ms);
}

/*
 * 驱动在读写失败路径会回调 debug_print，而角度读路径运行在 TIM2 中断内。
 * 这里必须保持空实现：在中断里做阻塞串口打印会拖垮实时性，失败由应用层返回值判定。
 */
static void AS5600_DebugPrint(const char *const fmt, ...)
{
  (void)fmt;
}

/* CONF 为易失寄存器，每次上电写入，不使用 BURN_SETTING，不消耗 ZMCO 次数 */
static int AS5600_WriteRegisterVerified(uint8_t reg, uint8_t value, uint8_t mask)
{
  uint8_t readback = 0U;

  if (as5600_set_reg(&s_handle, reg, &value, 1U) != 0)
  {
    return -1;
  }

  delay_us(500U);
  if (as5600_get_reg(&s_handle, reg, &readback, 1U) != 0)
  {
    return -1;
  }

  if ((readback & mask) != (value & mask))
  {
    /* 写入成功但回读不符：多半是总线边沿质量差导致位错，或器件未按预期接受该位 */
    CAW_LOG_ERROR("AS5600 reg 0x%02X verify mismatch: wrote 0x%02X read 0x%02X mask 0x%02X",
                  (unsigned)reg, (unsigned)value, (unsigned)readback, (unsigned)mask);
    return -1;
  }
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
 * I2C 总线恢复（I2C 规范 3.1.16）。从机卡在字节中间拉死 SDA 时（传输被复位/EMI 打断），
 * 它在等主机的下一个 SCL 时钟，复位 MCU 外设（SWRST）对它无效，必须由主机手动补时钟
 * 把剩余位"喂"完，再给 STOP 让从机状态机回到空闲。
 * 流程：外设 DeInit 释放引脚 -> 两脚切开漏 GPIO 并释放 -> SDA 为低则最多敲 9 个 SCL 脉冲，
 * 每拍查 SDA -> 补 STOP -> 外设 Init（MspInit 重配 AF + SWRST 清 BUSY）。
 * 返回 0 SDA 已释放，-1 仍被拉死（SDA 对地短路或器件损坏）。约 100 us，可在 TIM2 中断内调用。
 */
static int AS5600_BusRecover(AS5600_T *sensor)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;
  int released;

  (void)HAL_I2C_DeInit(sensor->hi2c); /* MspDeInit 内已 HAL_GPIO_DeInit 两脚 */

  /* 先写 1 再切输出，避免切换瞬间输出 0 的毛刺；开漏 + 外部上拉，读 IDR 即总线真实电平 */
  HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AS5600_SDA_PORT, AS5600_SDA_PIN, GPIO_PIN_SET);
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = AS5600_SCL_PIN;
  HAL_GPIO_Init(AS5600_SCL_PORT, &gpio);
  gpio.Pin = AS5600_SDA_PIN;
  HAL_GPIO_Init(AS5600_SDA_PORT, &gpio);
  delay_us(AS5600_RECOVER_HALF_US);

  /* 从机卡在输出 0 位时会拉住 SDA，每个 SCL 脉冲喂走一位，SDA 一释放即停 */
  for (i = 0U; (i < AS5600_RECOVER_MAX_CLOCKS) &&
               (HAL_GPIO_ReadPin(AS5600_SDA_PORT, AS5600_SDA_PIN) == GPIO_PIN_RESET); i++)
  {
    HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_RESET);
    delay_us(AS5600_RECOVER_HALF_US);
    HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_SET);
    delay_us(AS5600_RECOVER_HALF_US);
  }
  released = (HAL_GPIO_ReadPin(AS5600_SDA_PORT, AS5600_SDA_PIN) != GPIO_PIN_RESET) ? 0 : -1;

  /* 补 STOP：SCL 低时拉低 SDA，SCL 拉高后再释放 SDA（SCL 高期间 SDA 上升沿 = STOP） */
  HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_RESET);
  delay_us(AS5600_RECOVER_HALF_US);
  HAL_GPIO_WritePin(AS5600_SDA_PORT, AS5600_SDA_PIN, GPIO_PIN_RESET);
  delay_us(AS5600_RECOVER_HALF_US);
  HAL_GPIO_WritePin(AS5600_SCL_PORT, AS5600_SCL_PIN, GPIO_PIN_SET);
  delay_us(AS5600_RECOVER_HALF_US);
  HAL_GPIO_WritePin(AS5600_SDA_PORT, AS5600_SDA_PIN, GPIO_PIN_SET);
  delay_us(AS5600_RECOVER_HALF_US);

  /* HAL_I2C_Init 内部调 MspInit 把两脚还给 I2C1 的 AF，并 SWRST 复位外设 */
  (void)HAL_I2C_Init(sensor->hi2c);
  return released;
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
  sensor->valid = false;

  /*
   * 上一轮传输被复位/EMI 打断时从机会一直拉死 SDA，且 MCU 复位不会清掉它（AS5600 没断电）。
   * 第一笔事务之前先查 SDA 电平，为低就做总线恢复，否则 HAL 每次读都要空转 25 ms 等 BUSY。
   */
  if (HAL_GPIO_ReadPin(AS5600_SDA_PORT, AS5600_SDA_PIN) == GPIO_PIN_RESET)
  {
    int recovered = AS5600_BusRecover(sensor);
    CAW_LOG_WARN("AS5600 SDA stuck low at init, bus recover %s (SDA now=%u)",
                 (recovered == 0) ? "OK" : "FAILED",
                 (unsigned)HAL_GPIO_ReadPin(AS5600_SDA_PORT, AS5600_SDA_PIN));
  }

  /* 绑定官方驱动句柄与 I2C 回调，再交给驱动做初始化（校验回调并置 inited 标志） */
  s_hi2c = hi2c;
  DRIVER_AS5600_LINK_INIT(&s_handle, as5600_handle_t);
  DRIVER_AS5600_LINK_IIC_INIT(&s_handle, AS5600_IicInit);
  DRIVER_AS5600_LINK_IIC_DEINIT(&s_handle, AS5600_IicDeinit);
  DRIVER_AS5600_LINK_IIC_READ(&s_handle, AS5600_IicRead);
  DRIVER_AS5600_LINK_IIC_WRITE(&s_handle, AS5600_IicWrite);
  DRIVER_AS5600_LINK_DELAY_MS(&s_handle, AS5600_DelayMs);
  DRIVER_AS5600_LINK_DEBUG_PRINT(&s_handle, AS5600_DebugPrint);
  if (as5600_init(&s_handle) != 0)
  {
    CAW_LOG_ERROR("AS5600 driver init FAILED: I2C err=0x%02lX state=0x%02X",
                  (unsigned long)HAL_I2C_GetError(hi2c), (unsigned)HAL_I2C_GetState(hi2c));
    AS5600_RecordReadError(sensor);
    return -1;
  }

  /*
   * STATUS 只作诊断打点，磁体检测判定已取消：MD/ML/MH 不再决定初始化成败，
   * 仅总线读失败才返回错误。失败点分开打点，否则统一返回 -1 分不清是总线没通还是 CONF 没写进去。
   * HAL ErrorCode：0x04=AF 从机不应答（地址/供电/SDA 断线），0x20=TIMEOUT 总线 BUSY
   * 卡死或等待标志超时（无上拉、线被拉低），0x01=BERR 总线错误。
   */
  if (as5600_get_status(&s_handle, &status) != 0)
  {
    CAW_LOG_ERROR("AS5600 STATUS(0x0B) read FAILED: I2C err=0x%02lX state=0x%02X",
                  (unsigned long)HAL_I2C_GetError(hi2c), (unsigned)HAL_I2C_GetState(hi2c));
    AS5600_RecordReadError(sensor);
    return -1;
  }
  CAW_LOG_INFO("AS5600 STATUS=0x%02X MD=%u ML=%u MH=%u (magnet check disabled)",
               (unsigned)status,
               (unsigned)((status & AS5600_STATUS_MAGNET_DETECT) != 0U),
               (unsigned)((status & AS5600_STATUS_MAGNET_WEAK) != 0U),
               (unsigned)((status & AS5600_STATUS_MAGNET_STRONG) != 0U));

  /* 慢速滤波必须显式配置：上电默认 16x 的阶跃延迟对 1 kHz 修正不可接受 */
  if (AS5600_WriteRegisterVerified(AS5600_CONF_LOW_REGISTER,
                                   AS5600_CONF_LOW_VALUE,
                                   AS5600_CONF_LOW_MASK) != 0)
  {
    CAW_LOG_ERROR("AS5600 CONF low(0x08) write/verify FAILED: I2C err=0x%02lX",
                  (unsigned long)HAL_I2C_GetError(hi2c));
    AS5600_RecordReadError(sensor);
    return -1;
  }
  if (AS5600_WriteRegisterVerified(AS5600_CONF_HIGH_REGISTER,
                                   AS5600_CONF_HIGH_VALUE,
                                   AS5600_CONF_HIGH_MASK) != 0)
  {
    CAW_LOG_ERROR("AS5600 CONF high(0x07) write/verify FAILED: I2C err=0x%02lX",
                  (unsigned long)HAL_I2C_GetError(hi2c));
    AS5600_RecordReadError(sensor);
    return -1;
  }

  if (as5600_get_raw_angle(&s_handle, &raw_angle) != 0)
  {
    CAW_LOG_ERROR("AS5600 RAW_ANGLE(0x0C) read FAILED: I2C err=0x%02lX",
                  (unsigned long)HAL_I2C_GetError(hi2c));
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

  if ((sensor == NULL) || (sensor->hi2c == NULL))
  {
    return -1;
  }

  /*
   * 总线空闲时 BUSY 必须为 0；为 1 说明从机把 SDA/SCL 拉死在中途。此时不能交给
   * HAL（其内部会空转 25 ms 等 BUSY 清除），直接做总线恢复（补时钟 + STOP + 外设复位），
   * 否则角度通道永久失效。恢复结果由下一拍的读是否成功体现，本拍按失败计。
   */
  if (__HAL_I2C_GET_FLAG(sensor->hi2c, I2C_FLAG_BUSY) != RESET)
  {
    (void)AS5600_BusRecover(sensor);
    AS5600_RecordReadError(sensor);
    return -1;
  }

  if (as5600_get_raw_angle(&s_handle, &raw_angle) != 0)
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
 * 运行期 STATUS 观测。磁体检测判定已取消：这里只读 STATUS 打点 MD/ML/MH 变化，
 * 不再据此关断电机。返回 0 读取正常，-2 读失败。
 * 读失败单独返回是为了让调用方知道总线状态：单次总线抖动不该关断电机，总线真死了
 * 角度通道会先失败，由 sensor_max_age_ms 兜底。
 * 与 TIM2 中断内的轮询读共用总线，调用方须保证期间不发起角度刷新。
 */
int AS5600_CheckStatus(AS5600_T *sensor)
{
  /* 只在 STATUS 字节变化时打印：调磁体气隙时能实时看到 MD/ML/MH 翻转，又不会 10 Hz 刷屏 */
  static uint16_t last_status = 0xFFFFU;
  uint8_t status = 0U;

  if (sensor == NULL)
  {
    return -1;
  }

  if (as5600_get_status(&s_handle, &status) != 0)
  {
    AS5600_RecordReadError(sensor);
    return -2;
  }

  if (status != last_status)
  {
    last_status = status;
    CAW_LOG_INFO("AS5600 STATUS=0x%02X MD=%u ML=%u MH=%u",
                 (unsigned)status,
                 (unsigned)((status & AS5600_STATUS_MAGNET_DETECT) != 0U),
                 (unsigned)((status & AS5600_STATUS_MAGNET_WEAK) != 0U),
                 (unsigned)((status & AS5600_STATUS_MAGNET_STRONG) != 0U));
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
  return (sensor != NULL) && sensor->valid;
}

bool AS5600_IsFresh(const AS5600_T *sensor, uint32_t now, uint32_t max_age_ms)
{
  return AS5600_IsValid(sensor) &&
         ((uint32_t)(now - sensor->last_success_tick) <= max_age_ms);
}
