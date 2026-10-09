#include "hal_iic.h"
#include "sys_udelay.h"
#include "gpio.h"

#define delay_us(us)	SYS_uDelay(us)

#define IIC_T_HIGH_US			4U		/* SCL 高电平保持 */
#define IIC_T_LOW_US			5U		/* SCL 低电平保持 */
#define IIC_T_SU_US				1U		/* SCL 下降到 SDA 允许变化的间隔 */
#define IIC_T_SAMPLE_US			(IIC_T_HIGH_US / 2U)				/* SCL 上升到采样点 */
#define IIC_T_HOLD_US			(IIC_T_HIGH_US - IIC_T_SAMPLE_US)	/* 采样点到 SCL 下降 */

#define IIC_T_RECOVER_US		5U		/* 恢复阶段半周期，比正常时序更宽以留足余量 */
#define IIC_SCL_WAIT_US			100U	/* 释放 SCL 后等待其到达高电平的超时 */
#define IIC_RECOVER_CLOCKS		9U		/* 一个字节 8 位数据加 1 位 ACK */
#define IIC_NACK_RECOVER_THRESHOLD	3U	/* 连续地址 NACK 达此次数才做总线恢复 */

static void IICOnTransferError(iic_bus_t *bus, iic_status_t err);
static iic_status_t IIC_WriteFrame(iic_bus_t *bus, uint8_t daddr, uint8_t reg,  uint8_t length, const uint8_t *buff);
static iic_status_t IIC_ReadFrame(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t *buff);


static void SDA_Output(iic_bus_t *bus, uint16_t val)
{
    if (val)
    {
        bus->IIC_SDA_PORT->BSRR = (uint32_t)bus->IIC_SDA_PIN;
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
        bus->IIC_SCL_PORT->BSRR = (uint32_t)bus->IIC_SCL_PIN;
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

static uint8_t SCL_Input(iic_bus_t *bus)
{
    if (HAL_GPIO_ReadPin(bus->IIC_SCL_PORT, bus->IIC_SCL_PIN) == GPIO_PIN_SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t SCL_ReleaseAndWait(iic_bus_t *bus)
{
    uint32_t wait = IIC_SCL_WAIT_US;

    SCL_Output(bus, 1);
    while (SCL_Input(bus) == 0U)
    {
        if (wait == 0U)
        {
            return 1U;
        }
        --wait;
        delay_us(1);
    }

    return 0U;
}

static iic_status_t IICStart(iic_bus_t *bus)
{
    SDA_Output(bus, 1);
    delay_us(IIC_T_SU_US);

    if (SCL_ReleaseAndWait(bus) != 0U)
    {
        return IIC_ERR_SCL_STUCK;
    }
    delay_us(IIC_T_LOW_US);

    SDA_Output(bus, 0);
    delay_us(IIC_T_HIGH_US);

    SCL_Output(bus, 0);
    delay_us(IIC_T_LOW_US);

    return IIC_OK;
}

static void IICStop(iic_bus_t *bus)
{
    SCL_Output(bus, 0);
    delay_us(IIC_T_SU_US);
    SDA_Output(bus, 0);
    delay_us(IIC_T_LOW_US);
    SCL_Output(bus, 1);
    delay_us(IIC_T_LOW_US);
    SDA_Output(bus, 1);
    delay_us(IIC_T_LOW_US);
}

static iic_status_t IICWaitAck(iic_bus_t *bus)
{
    uint8_t ack;

    SDA_Output(bus, 1);
    delay_us(IIC_T_LOW_US);

    if (SCL_ReleaseAndWait(bus) != 0U)
    {
        return IIC_ERR_SCL_STUCK;
    }

    delay_us(IIC_T_SAMPLE_US);
    ack = (SDA_Input(bus) == 0U) ? 1U : 0U;
    delay_us(IIC_T_HOLD_US);
    SCL_Output(bus, 0);
    delay_us(IIC_T_LOW_US);

    return (ack != 0U) ? IIC_OK : IIC_ERR_DATA_NACK;
}

static void IICSendAck(iic_bus_t *bus)
{
    SDA_Output(bus, 0);
    delay_us(IIC_T_LOW_US);
    SCL_Output(bus, 1);
    delay_us(IIC_T_HIGH_US);
    SCL_Output(bus, 0);
    delay_us(IIC_T_LOW_US);
}

static void IICSendNotAck(iic_bus_t *bus)
{
    SDA_Output(bus, 1);
    delay_us(IIC_T_LOW_US);
    SCL_Output(bus, 1);
    delay_us(IIC_T_HIGH_US);
    SCL_Output(bus, 0);
    delay_us(IIC_T_LOW_US);
}

static void IICSendByte(iic_bus_t *bus, unsigned char cSendByte)
{
    unsigned char i = 8U;
    while (i--)
    {
        SCL_Output(bus, 0);
        delay_us(IIC_T_SU_US);
        SDA_Output(bus, (uint16_t)(cSendByte & 0x80U));
        cSendByte = (unsigned char)(cSendByte << 1);
        delay_us(IIC_T_LOW_US);
        SCL_Output(bus, 1);
        delay_us(IIC_T_HIGH_US);
    }
    SCL_Output(bus, 0);
    delay_us(IIC_T_SU_US);
}

static unsigned char IICReceiveByte(iic_bus_t *bus)
{
    unsigned char i = 8U;
    unsigned char cR_Byte = 0U;

    SDA_Output(bus, 1);
    while (i--)
    {
        cR_Byte = (unsigned char)(cR_Byte << 1);
        SCL_Output(bus, 0);
        delay_us(IIC_T_LOW_US);
        SCL_Output(bus, 1);
        delay_us(IIC_T_SAMPLE_US);
        cR_Byte |= SDA_Input(bus);
        delay_us(IIC_T_HOLD_US);
    }
    SCL_Output(bus, 0);
    delay_us(IIC_T_SU_US);

    return cR_Byte;
}

static iic_status_t IICBusRecover(iic_bus_t *bus)
{
    uint8_t i;

    if (bus == NULL)
    {
        return IIC_ERR_ARG;
    }

    bus->recovery_count++;
    SDA_Output(bus, 1);
    delay_us(IIC_T_RECOVER_US);
    if (SCL_ReleaseAndWait(bus) != 0U)
    {
        bus->bus_stuck = 1U;
        return IIC_ERR_SCL_STUCK;
    }
    delay_us(IIC_T_RECOVER_US);

    for (i = 0U; i < IIC_RECOVER_CLOCKS; ++i)
    {
        if (SDA_Input(bus) != 0U)
        {
            break;
        }

        SCL_Output(bus, 0);
        delay_us(IIC_T_RECOVER_US);
        if (SCL_ReleaseAndWait(bus) != 0U)
        {
            bus->bus_stuck = 1U;
            return IIC_ERR_SCL_STUCK;
        }
        delay_us(IIC_T_RECOVER_US);
    }

    SCL_Output(bus, 0);
    delay_us(IIC_T_RECOVER_US);
    SDA_Output(bus, 0);
    delay_us(IIC_T_RECOVER_US);
    SCL_Output(bus, 1);
    delay_us(IIC_T_RECOVER_US);
    SDA_Output(bus, 1);
    delay_us(IIC_T_RECOVER_US);

    if (SCL_Input(bus) == 0U)
    {
        bus->bus_stuck = 1U;
        return IIC_ERR_SCL_STUCK;
    }
    if (SDA_Input(bus) == 0U)
    {
        bus->bus_stuck = 1U;
        return IIC_ERR_SDA_STUCK;
    }

    bus->bus_stuck = 0U;
    return IIC_OK;
}

static iic_status_t IICBusIdleOrRecover(iic_bus_t *bus)
{
    if ((SCL_Input(bus) != 0U) && (SDA_Input(bus) != 0U))
    {
        return IIC_OK;
    }

    return IICBusRecover(bus);
}

static void IICOnTransferError(iic_bus_t *bus, iic_status_t err)
{
    iic_status_t recover_result;
    uint8_t need_recover;

    bus->last_error = err;

    if ((err == IIC_ERR_SDA_STUCK) || (err == IIC_ERR_SCL_STUCK))
    {
        need_recover = 1U;
    }
    else if (err == IIC_ERR_ADDR_NACK)
    {
        if (bus->nack_streak < IIC_NACK_RECOVER_THRESHOLD)
        {
            bus->nack_streak++;
            return;
        }
        bus->nack_streak = 0U;
        need_recover = 1U;
    }
    else
    {
        need_recover = 0U;
    }

    if (need_recover == 0U)
    {
        return;
    }

    recover_result = IICBusRecover(bus);
    if (recover_result != IIC_OK)
    {
        bus->last_error = recover_result;
    }
}

static iic_status_t IIC_WriteFrame(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, const uint8_t *buff)
{
    iic_status_t status;
    uint8_t i;

    status = IICStart(bus);
    if (status != IIC_OK)
    {
        return status;
    }

    IICSendByte(bus, (unsigned char)(daddr << 1));
    status = IICWaitAck(bus);
    if (status != IIC_OK)
    {
        return (status == IIC_ERR_DATA_NACK) ? IIC_ERR_ADDR_NACK : status;
    }

    IICSendByte(bus, reg);
    status = IICWaitAck(bus);
    if (status != IIC_OK)
    {
        return status;
    }

    for (i = 0U; i < length; ++i)
    {
        IICSendByte(bus, buff[i]);
        status = IICWaitAck(bus);
        if (status != IIC_OK)
        {
            return status;
        }
    }

    return IIC_OK;
}

static iic_status_t IIC_ReadFrame(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t *buff)
{
    iic_status_t status;
    uint8_t i;

    status = IICStart(bus);
    if (status != IIC_OK)
    {
        return status;
    }

    IICSendByte(bus, (unsigned char)(daddr << 1));
    status = IICWaitAck(bus);
    if (status != IIC_OK)
    {
        return (status == IIC_ERR_DATA_NACK) ? IIC_ERR_ADDR_NACK : status;
    }

    IICSendByte(bus, reg);
    status = IICWaitAck(bus);
    if (status != IIC_OK)
    {
        return status;
    }

    status = IICStart(bus);
    if (status != IIC_OK)
    {
        return status;
    }

    IICSendByte(bus, (unsigned char)((daddr << 1) | 0x01U));
    status = IICWaitAck(bus);
    if (status != IIC_OK)
    {
        return (status == IIC_ERR_DATA_NACK) ? IIC_ERR_ADDR_NACK : status;
    }

    for (i = 0U; i < length; ++i)
    {
        buff[i] = IICReceiveByte(bus);
        if ((uint16_t)(i + 1U) < (uint16_t)length)
        {
            IICSendAck(bus);
        }
    }
    IICSendNotAck(bus);

    return IIC_OK;
}

uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t data)
{
    return IIC_Write_Multi_Byte(bus, daddr, reg, 1U, &data);
}

uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
{
    iic_status_t status;

    if ((bus == NULL) || ((length > 0U) && (buff == NULL)))
    {
        if (bus != NULL)
        {
            bus->last_error = IIC_ERR_ARG;
        }
        return 1U;
    }

    status = IICBusIdleOrRecover(bus);
    if (status != IIC_OK)
    {
        bus->last_error = status;
        return 1U;
    }

    status = IIC_WriteFrame(bus, daddr, reg, length, buff);
    IICStop(bus);

    if (status != IIC_OK)
    {
        IICOnTransferError(bus, status);
        return 1U;
    }

    bus->last_error = IIC_OK;
    bus->nack_streak = 0U;
    return 0U;
}

uint8_t IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t *data)
{
    return IIC_Read_Multi_Byte(bus, daddr, reg, 1U, data);
}

uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
{
    iic_status_t status;

    if ((bus == NULL) || (length == 0U) || (buff == NULL))
    {
        if (bus != NULL)
        {
            bus->last_error = IIC_ERR_ARG;
        }
        return 1U;
    }

    status = IICBusIdleOrRecover(bus);
    if (status != IIC_OK)
    {
        bus->last_error = status;
        return 1U;
    }

    status = IIC_ReadFrame(bus, daddr, reg, length, buff);
    IICStop(bus);

    if (status != IIC_OK)
    {
        IICOnTransferError(bus, status);
        return 1U;
    }

    bus->last_error = IIC_OK;
    bus->nack_streak = 0U;
    return 0U;
}

uint8_t IICInit(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    iic_status_t status;

    if (bus == NULL)
    {
        return 1U;
    }

    SDA_Output(bus, 1);
    SCL_Output(bus, 1);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = bus->IIC_SCL_PIN;
    HAL_GPIO_Init(bus->IIC_SCL_PORT, &GPIO_InitStructure);

    bus->nack_streak = 0U;
    bus->bus_stuck = 0U;
    bus->recovery_count = 0U;

    status = IICBusRecover(bus);
    bus->last_error = status;

    return (status == IIC_OK) ? 0U : 1U;
}

uint8_t IICDeinit(iic_bus_t *bus)
{
    HAL_GPIO_DeInit(bus->IIC_SDA_PORT, bus->IIC_SDA_PIN);
    HAL_GPIO_DeInit(bus->IIC_SCL_PORT, bus->IIC_SCL_PIN);
	return 0;
}

