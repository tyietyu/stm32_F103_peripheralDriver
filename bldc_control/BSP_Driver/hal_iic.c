#include "hal_iic.h"
#include "delay.h"
#include "gpio.h"

/**
 * @brief SDA线输入模式配置
 * @param None
 * @retval None
 */
void SDA_Input_Mode(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);
}

/**
 * @brief SDA线输出模式配置
 * @param None
 * @retval None
 */
void SDA_Output_Mode(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);
}

/**
 * @brief SDA线输出一个位
 * @param val 输出的数据
 * @retval None
 */
void SDA_Output(iic_bus_t *bus, uint16_t val)
{
    if (val)
    {
        bus->IIC_SDA_PORT->BSRR = bus->IIC_SDA_PIN;
    }
    else
    {
        bus->IIC_SDA_PORT->BSRR = (uint32_t)bus->IIC_SDA_PIN << 16U;
    }
}

/**
 * @brief SCL线输出一个位
 * @param val 输出的数据
 * @retval None
 */
void SCL_Output(iic_bus_t *bus, uint16_t val)
{
    if (val)
    {
        bus->IIC_SCL_PORT->BSRR = bus->IIC_SCL_PIN;
    }
    else
    {
        bus->IIC_SCL_PORT->BSRR = (uint32_t)bus->IIC_SCL_PIN << 16U;
    }
}

/**
 * @brief SDA输入一位
 * @param None
 * @retval GPIO读入一位
 */
uint8_t SDA_Input(iic_bus_t *bus)
{
    if (HAL_GPIO_ReadPin(bus->IIC_SDA_PORT, bus->IIC_SDA_PIN) == GPIO_PIN_SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief IIC起始信号
 * @param None
 * @retval None
 */
void IICStart(iic_bus_t *bus)
{
    SDA_Output(bus, 1);
    delay_us(2);
    SCL_Output(bus, 1);
    delay_us(1);
    SDA_Output(bus, 0);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(1);
}

/**
 * @brief IIC停止信号
 * @param None
 * @retval None
 */
void IICStop(iic_bus_t *bus)
{
    SCL_Output(bus, 0);
    delay_us(2);
    SDA_Output(bus, 0);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SDA_Output(bus, 1);
    delay_us(1);
}

/**
 * @brief  IIC等待确认信号
 * @param None
 * @retval None
 */
unsigned char IICWaitAck(iic_bus_t *bus)
{
    unsigned short cErrTime = 5;
    SDA_Input_Mode(bus);
    SCL_Output(bus, 1);
    while (SDA_Input(bus))
    {
        cErrTime--;
        delay_us(1);
        if (0 == cErrTime)
        {
            SDA_Output_Mode(bus);
            IICStop(bus);
            return ERROR;
        }
    }
    SDA_Output_Mode(bus);
    SCL_Output(bus, 0);
    delay_us(2);
    return SUCCESS;
}

/**
 * @brief IIC发送确认信号
 * @param None
 * @retval None
 */
void IICSendAck(iic_bus_t *bus)
{
    SDA_Output(bus, 0);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(1);
}

/**
 * @brief IIC发送非确认信号
 * @param None
 * @retval None
 */
void IICSendNotAck(iic_bus_t *bus)
{
    SDA_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(2);
}

/**
 * @brief IIC发送一个字节
 * @param cSendByte 需要发送的字节
 * @retval None
 */
void IICSendByte(iic_bus_t *bus, unsigned char cSendByte)
{
    unsigned char i = 8;
    while (i--)
    {
        SCL_Output(bus, 0);
        delay_us(2);
        SDA_Output(bus, cSendByte & 0x80);
        delay_us(1);
        cSendByte += cSendByte;
        delay_us(1);
        SCL_Output(bus, 1);
        delay_us(1);
    }
    SCL_Output(bus, 0);
    delay_us(2);
}

/**
 * @brief IIC接收一个字节
 * @param None
 * @retval 接收到的字节
 */
unsigned char IICReceiveByte(iic_bus_t *bus)
{
    unsigned char i = 8;
    unsigned char cR_Byte = 0;
    SDA_Input_Mode(bus);
    while (i--)
    {
        cR_Byte += cR_Byte;
        SCL_Output(bus, 0);
        delay_us(2);
        SCL_Output(bus, 1);
        delay_us(1);
        cR_Byte |= SDA_Input(bus);
    }
    SCL_Output(bus, 0);
    SDA_Output_Mode(bus);
    return cR_Byte;
}

/**
 * @brief IIC写入一个字节
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param data 写入的数据
 * @retval 返回0表示成功，1表示失败
 */
uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t data)
{
    IICStart(bus);
    IICSendByte(bus, daddr << 1);

    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }

    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }
    IICSendByte(bus, data);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }
    IICStop(bus);
    delay_us(1);

    return 0;
}

/**
 * @brief IIC写入多个字节
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param length 写入的字节数
 * @param buff 写入的数据缓冲区
 * @retval 返回0表示成功，1表示失败
 */
uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
{
    unsigned char i;
    IICStart(bus);

    IICSendByte(bus, daddr << 1);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }
    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }
    for (i = 0; i < length; i++)
    {
        IICSendByte(bus, buff[i]);
        if (IICWaitAck(bus))
        {
            IICStop(bus);
            return 1;
        }
    }
    IICStop(bus);
    delay_us(1);
    return 0;
}

/**
 * @brief IIC读取一个字节
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @retval 返回读取到的字节
 */
unsigned char IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg)
{
    unsigned char dat;
    IICStart(bus);
    IICSendByte(bus, daddr << 1);
    IICWaitAck(bus);
    IICSendByte(bus, reg);
    IICWaitAck(bus);

    IICStart(bus);
    IICSendByte(bus, (daddr << 1) + 1);
    IICWaitAck(bus);
    dat = IICReceiveByte(bus);
    IICSendNotAck(bus);
    IICStop(bus);
    return dat;
}

/**
 * @brief IIC读取多个字节
 * @param daddr 设备地址
 * @param reg 寄存器地址
 * @param length 读取的字节数
 * @param buff 存储读取数据的缓冲区
 * @retval 返回0表示成功，1表示失败
 */
uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
{
    unsigned char i;
    IICStart(bus);
    IICSendByte(bus, daddr << 1);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }
    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }

    IICStart(bus);
    IICSendByte(bus, (daddr << 1) + 1);
    if (IICWaitAck(bus))
    {
        IICStop(bus);
        return 1;
    }
    for (i = 0; i < length; i++)
    {
        buff[i] = IICReceiveByte(bus);
        if (i < length - 1)
        {
            IICSendAck(bus);
        }
    }
    IICSendNotAck(bus);
    IICStop(bus);
    return 0;
}

/**
 * @brief IIC总线初始化
 * @param bus IIC总线结构体指针
 * @retval None
 */
uint8_t IICInit(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    if (bus == NULL)
    {
        return 1U;
    }

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = bus->IIC_SCL_PIN;
    HAL_GPIO_Init(bus->IIC_SCL_PORT, &GPIO_InitStructure);

    /* 开漏输出写 1 表示释放总线，I2C 空闲态必须为高。 */
    SDA_Output(bus, 1U);
    SCL_Output(bus, 1U);
    delay_us(5U);
    if ((SDA_Input(bus) == 0U) ||
        (HAL_GPIO_ReadPin(bus->IIC_SCL_PORT, bus->IIC_SCL_PIN) == GPIO_PIN_RESET))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief IIC总线释放
 * @param bus IIC总线结构体指针
 * @retval None
 */
uint8_t IICDeinit(iic_bus_t *bus)
{
    HAL_GPIO_DeInit(bus->IIC_SDA_PORT, bus->IIC_SDA_PIN);
    HAL_GPIO_DeInit(bus->IIC_SCL_PORT, bus->IIC_SCL_PIN);
	return 0;
}

