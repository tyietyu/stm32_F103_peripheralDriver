#ifndef __HAL_IIC_H
#define __HAL_IIC_H

#include "stm32f1xx_hal.h"

/**
 * @brief IIC总线结构体
 */
typedef struct
{
	GPIO_TypeDef * IIC_SDA_PORT;  /**< SDA引脚端口 */
	GPIO_TypeDef * IIC_SCL_PORT;  /**< SCL引脚端口 */
	uint16_t IIC_SDA_PIN;         /**< SDA引脚号 */
	uint16_t IIC_SCL_PIN;         /**< SCL引脚号 */
}iic_bus_t;

/**
 * @brief 初始化IIC总线
 * @param bus IIC总线结构体指针
 * @retval None
 */
void IICInit(iic_bus_t *bus);

/**
 * @brief IIC写入一个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址（7位地址）
 * @param reg 寄存器地址
 * @param data 写入的数据
 * @retval 0表示成功，1表示失败
 */
uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t data);

/**
 * @brief IIC写入多个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址（7位地址）
 * @param reg 寄存器地址
 * @param length 写入的字节数
 * @param data 写入数据的缓冲区
 * @retval 0表示成功，1表示失败
 */
uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data, uint8_t length);

/**
 * @brief IIC读取一个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址（7位地址）
 * @param reg 寄存器地址
 * @param data 读取数据存放指针
 * @retval 0表示成功，1表示失败
 */
uint8_t IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data);

/**
 * @brief IIC读取多个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址（7位地址）
 * @param reg 寄存器地址
 * @param length 读取的字节数
 * @param data 存储读取数据的缓冲区
 * @retval 0表示成功，1表示失败
 */
uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data, uint8_t length);

#endif
