#include "hal_iic.h"

static void delay_us(uint32_t us)
{
    uint32_t start_value;
    uint32_t current_value;
    uint32_t reload;
    uint32_t elapsed_ticks = 0;
    uint32_t systick_freq;
    uint32_t required_ticks;
    uint32_t max_wait_loops;
    uint32_t loop_counter = 0;

    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0)
    {
        return;
    }

    reload = SysTick->LOAD;
    if (reload == 0)
    {
        return;
    }

    extern uint32_t SystemCoreClock;
    if (SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk)
    {
        systick_freq = SystemCoreClock;
    }
    else
    {
        systick_freq = SystemCoreClock / 8;
    }

    uint64_t ticks64 = ((uint64_t)us * systick_freq) / 1000000UL;
    if (ticks64 > 0xFFFFFFFFUL)
    {
        required_ticks = 0xFFFFFFFFUL;
    }
    else
    {
        required_ticks = (uint32_t)ticks64;
    }

    max_wait_loops = required_ticks * 4 + 1000;
    start_value = SysTick->VAL;
    while (elapsed_ticks < required_ticks)
    {
        current_value = SysTick->VAL;
        if (current_value <= start_value)
        {
            elapsed_ticks += start_value - current_value;
        }
        else
        {
            elapsed_ticks += start_value + (reload - current_value) + 1;
        }
        start_value = current_value;
        loop_counter++;
        if (loop_counter > max_wait_loops)
        {
            break;
        }
    }
}

static void SDA_Input_Mode(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);
}

static void SDA_Output_Mode(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);
}

static void SDA_Output(iic_bus_t *bus, uint16_t val)
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

static void SCL_Output(iic_bus_t *bus, uint16_t val)
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


static uint8_t SDA_Input(iic_bus_t *bus)
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


static void IICStart(iic_bus_t *bus)
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

static void IICStop(iic_bus_t *bus)
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

static unsigned char IICWaitAck(iic_bus_t *bus)
{
    unsigned short cErrTime = 200;
    SDA_Input_Mode(bus);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
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

static void IICSendAck(iic_bus_t *bus)
{
    SDA_Output(bus, 0);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(1);
}

static void IICSendNotAck(iic_bus_t *bus)
{
    SDA_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(2);
}

static void IICSendByte(iic_bus_t *bus, unsigned char cSendByte)
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

static unsigned char IICReceiveByte(iic_bus_t *bus)
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

uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t data)
{
    IICStart(bus);
    IICSendByte(bus, daddr << 1);

    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICSendByte(bus, data);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICStop(bus);
    delay_us(1);

    return 0;
}

uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data, uint8_t length)
{
    unsigned char i;
    IICStart(bus);

    IICSendByte(bus, daddr << 1);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    for (i = 0; i < length; i++)
    {
        IICSendByte(bus, data[i]);
        if (IICWaitAck(bus))
        {
            return 1;
        }
    }
    IICStop(bus);
    delay_us(1);
    return 0;
}

uint8_t IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data)
{
    IICStart(bus);
    IICSendByte(bus, daddr << 1);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICStart(bus);
    IICSendByte(bus, (daddr << 1) + 1);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    *data = IICReceiveByte(bus);
    IICSendNotAck(bus);
    IICStop(bus);
    return 0;
}

uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data, uint8_t length)
{
    unsigned char i;
    IICStart(bus);
    IICSendByte(bus, daddr << 1);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICSendByte(bus, reg);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    IICStart(bus);
    IICSendByte(bus, (daddr << 1) + 1);
    if (IICWaitAck(bus))
    {
        return 1;
    }

    for (i = 0; i < length; i++)
    {
        data[i] = IICReceiveByte(bus);
        if (i < length - 1)
        {
            IICSendAck(bus);
        }
    }
    IICSendNotAck(bus);
    IICStop(bus);
    return 0;
}

void IICInit(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = bus->IIC_SCL_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(bus->IIC_SCL_PORT, &GPIO_InitStructure);

    SDA_Output(bus, 1);
    SCL_Output(bus, 1);
}

