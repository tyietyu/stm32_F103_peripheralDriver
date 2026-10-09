#ifndef __HAL_IIC_H
#define __HAL_IIC_H

#include "stm32f1xx_hal.h"

/**
 * @brief 软件 IIC 事务状态码，0 表示成功。
 */
typedef enum
{
	IIC_OK = 0U,
	IIC_ERR_ARG,			/* 入参非法 */
	IIC_ERR_ADDR_NACK,		/* 地址字节无 ACK：设备离线或未上电 */
	IIC_ERR_DATA_NACK,		/* 寄存器或数据字节无 ACK：事务被中途打断 */
	IIC_ERR_SDA_STUCK,		/* 补满恢复时钟后 SDA 仍被拉低 */
	IIC_ERR_SCL_STUCK,		/* SCL 被外部拉低，软件无法恢复 */
} iic_status_t;

typedef struct
{
	GPIO_TypeDef * IIC_SDA_PORT;
	GPIO_TypeDef * IIC_SCL_PORT;
	uint16_t IIC_SDA_PIN;
	uint16_t IIC_SCL_PIN;
	//void (*CLK_ENABLE)(void);
	iic_status_t last_error;	/* 最近一次事务的失败原因，成功时为 IIC_OK */
	uint8_t nack_streak;		/* 连续地址 NACK 次数，达阈值才触发总线恢复 */
	uint8_t bus_stuck;			/* 最近一次恢复结束时 SCL/SDA 仍未释放，用于诊断 */
	uint16_t recovery_count;	/* 累计总线恢复次数，仅用于诊断 */
}iic_bus_t;

/**
 * @brief IIC总线初始化
 * @param bus IIC总线结构体指针
 * @retval 返回0表示总线已处于空闲态，1表示引脚被拉低无法恢复
 * @note 调用方需先使能对应 GPIO 端口时钟。
 */
uint8_t IICInit(iic_bus_t *bus);

/**
 * @brief IIC总线释放
 * @param bus IIC总线结构体指针
 * @retval None
 */
uint8_t IICDeinit(iic_bus_t *bus);

/**
 * @brief IIC写入一个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param data 写入的数据
 * @retval 返回0表示成功，1表示失败
 */
uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t data);

/**
 * @brief IIC写入多个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param length 写入的字节数
 * @param buff 写入的数据缓冲区
 * @retval 返回0表示成功，1表示失败，详细原因见 bus->last_error
 */
uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t length,uint8_t buff[]);

/**
 * @brief IIC读取一个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param data 读出数据的输出指针，失败时不被修改
 * @retval 返回0表示成功，1表示失败，详细原因见 bus->last_error
 */
uint8_t IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t *data);

/**
 * @brief IIC读取多个字节
 * @param bus IIC总线结构体指针
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param length 读取的字节数，必须大于0
 * @param buff 存储读取数据的缓冲区
 * @retval 返回0表示成功，1表示失败，详细原因见 bus->last_error
 */
uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[]);



#endif
