#include "hal_iic.h"

static void delay_us(uint32_t us)
{
    uint32_t current_value = 0;
    uint32_t change_number = 0;

    uint32_t reload = SysTick -> LOAD;
    uint32_t first_value = SysTick ->VAL;
    uint32_t nus_number = us * ((reload + 1) / 1000);
    while (1)
    {
        current_value = SysTick ->VAL;
        if (current_value != first_value)
        {
            if (current_value < first_value)
            {
                change_number += first_value - current_value;       
            }
            else
            {
                change_number += reload - current_value + first_value;
            }

            first_value = current_value;
            if (change_number >= nus_number)
            {
                break;
            }
        }
    }
}

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
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
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
        bus->IIC_SDA_PORT->BSRR |= bus->IIC_SDA_PIN;
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
        bus->IIC_SCL_PORT->BSRR |= bus->IIC_SCL_PIN;
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
    SCL_Output(bus, 1);
    delay_us(10);
    SDA_Output(bus, 0);
    delay_us(10);
    SCL_Output(bus, 0);
    delay_us(10);
}

/**
 * @brief IIC停止信号
 * @param None
 * @retval None
 */
void IICStop(iic_bus_t *bus)
{
    SCL_Output(bus, 0);
    delay_us(10);
    SDA_Output(bus, 0);
    delay_us(10);
    SCL_Output(bus, 1);
    delay_us(10);
    SDA_Output(bus, 1);
    delay_us(10);
}

/**
 * @brief  IIC等待确认信号
 * @param None
 * @retval None
 */
unsigned char IICWaitAck(iic_bus_t *bus)
{
    SDA_Input_Mode(bus);
    delay_us(10);
    SCL_Output(bus, 1);
    delay_us(10);
    SCL_Output(bus, 0);
    delay_us(10);
    SDA_Output_Mode(bus);
    return 0;
}

/**
 * @brief IIC发送确认信号
 * @param None
 * @retval None
 */
void IICSendAck(iic_bus_t *bus)
{
    SDA_Output(bus, 0);
    delay_us(10);
    SCL_Output(bus, 1);
    delay_us(10);
    SCL_Output(bus, 0);
    delay_us(10);
}

/**
 * @brief IIC发送非确认信号
 * @param None
 * @retval None
 */
void IICSendNotAck(iic_bus_t *bus)
{
    SDA_Output(bus, 1);
    delay_us(10);
    SCL_Output(bus, 1);
    delay_us(10);
    SCL_Output(bus, 0);
    delay_us(10);
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
        SDA_Output(bus, (cSendByte & 0x80) >> 7);
        cSendByte <<= 1;
        delay_us(10);
        SCL_Output(bus, 1);
        delay_us(10);
        SCL_Output(bus, 0);
        delay_us(10);
    }
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
        cR_Byte <<= 1;
        delay_us(10);
        SCL_Output(bus, 1);
        if(SDA_Input(bus)) cR_Byte |= 0x01;
        delay_us(10);
        SCL_Output(bus, 0);
    }

    SDA_Output_Mode(bus);
    SDA_Output(bus, 1);
    delay_us(10);
    SCL_Output(bus, 1);
    delay_us(10);
    SCL_Output(bus, 0);
    delay_us(10);

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
    IICWaitAck(bus);
    IICSendByte(bus, reg);
    IICWaitAck(bus);
    IICSendByte(bus, data);
    IICWaitAck(bus);
    IICStop(bus);
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
    IICStop(bus);
    delay_us(10);

    IICStart(bus);
    IICSendByte(bus, (daddr << 1) | 0x01); 
    IICWaitAck(bus);
    dat = IICReceiveByte(bus);
    IICStop(bus);
    
    return dat;
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
    IICWaitAck(bus);

    IICSendByte(bus, reg);
    IICWaitAck(bus);
    for (i = 0; i < length; i++)
    {
        IICSendByte(bus, buff[i]);
        IICWaitAck(bus);
    }
    IICStop(bus);
    delay_us(10);
    return 0;
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
    IICWaitAck(bus);
    IICSendByte(bus, reg);
    IICWaitAck(bus);
    IICStop(bus);
    delay_us(10);

    IICStart(bus);
    IICSendByte(bus, (daddr << 1) | 0x01);
    IICWaitAck(bus);
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

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = bus->IIC_SCL_PIN;
    HAL_GPIO_Init(bus->IIC_SCL_PORT, &GPIO_InitStructure);
		
	return 0;
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

