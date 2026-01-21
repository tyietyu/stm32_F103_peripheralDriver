#include "as5600.h"
#include "hal_iic.h"
#include "log.h"
#include "main.h"
#include "delay.h"

#define abs(x)  ((x) > 0 ? (x) : -(x))
#define _2PI    6.283185307179586f
#define _PI     3.141592653589793f

iic_bus_t i2c_bus = {
    .IIC_SCL_PORT = AS5600_SCL_GPIO_Port,
    .IIC_SCL_PIN = AS5600_SCL_Pin,
    .IIC_SDA_PORT = AS5600_SDA_GPIO_Port,
    .IIC_SDA_PIN = AS5600_SDA_Pin,
};

int AS5600_Init(AS5600_T *a)
{
  IICInit(&i2c_bus);
  delay_init();
  a->i2c_ins = &i2c_bus;
  uint16_t angle = AS5600_GetRawAngle(a);
  a->prev_angle = angle;
  a->prev_angle_ts = 0;
  a->rotation_offset = 0;

  a->vel_rotation_offset = 0;
  a->vel_prev_angle = angle;
  a->vel_prev_angle_ts = 0;
  return 0;
}

uint16_t AS5600_GetRawAngle(AS5600_T *a)
{
  uint8_t buffer[2];
  if(IIC_Read_Multi_Byte(a->i2c_ins, AS5600_RAW_ADDR, AS5600_RAW_ANGLE_REGISTER, 2, buffer) != 0)
  {
    CAW_LOG_WARN("AS5600_GetRawAngle error, return prev_angle");
    return (uint16_t)a->prev_angle;
  }

  return ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
}

// 获得单圈弧度值
float AS5600_GetOnceAngle(AS5600_T *a)
{
  return AS5600_GetRawAngle(a) * (360.0 / AS5600_RESOLUTION) * _PI / 180.0;
}

// 获得累计圈数
float AS5600_GetAngle(AS5600_T *a)
{
  return (a->rotation_offset + (a->prev_angle / (float)AS5600_RESOLUTION) * _2PI);
}

void AS5600_Update(AS5600_T *a)
{
  float angle_data = AS5600_GetRawAngle(a);
  float delta = angle_data - a->prev_angle;
  a->prev_angle_ts = HAL_GetTick();
  if (abs(delta) > (0.8 * AS5600_RESOLUTION))
  {
    a->rotation_offset += (delta > 0 ? -_2PI : _2PI);
  }
  a->prev_angle = angle_data;
}

// 计算速度
float AS5600_GetVelocity(AS5600_T *a)
{
  float ts = (a->prev_angle_ts - a->vel_prev_angle_ts) * 1e-3;
  if (ts <= 0)
    ts = 1e-3f;
  float vel =  ((a->rotation_offset - a->vel_rotation_offset) + 
                (a->prev_angle - a->vel_prev_angle) / (float)AS5600_RESOLUTION * _2PI) / ts;
  a->vel_prev_angle = a->prev_angle;
  a->vel_rotation_offset = a->rotation_offset;
  a->vel_prev_angle_ts = a->prev_angle_ts;
  return vel;
}

