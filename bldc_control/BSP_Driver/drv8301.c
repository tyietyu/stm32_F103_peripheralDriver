/**
 * @file    drv8301.c
 * @brief   DRV8301 three-phase gate driver implementation
 * @note    Based on Texas Instruments DRV8301 Datasheet
 */

#include "drv8301.h"
#include "main.h"

#define DRV8301_CS_LOW() \
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define DRV8301_CS_HIGH() \
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

#define DRV8301_SPI_TIMEOUT     100U

/* Datasheet tSU_SCS/tHD_SCS are 50 ns; use at least 100 ns and verify on-board. */
#define DRV8301_CS_DELAY_CYCLES ((SystemCoreClock / 10000000U) + 1U)
#define DRV8301_CS_DELAY()      DRV8301_DelayCoreCycles(DRV8301_CS_DELAY_CYCLES)

#if (DRV8301_SHUNT_GAIN == 10U)
#define DRV8301_CTRL2_GAIN_CONFIG DRV8301_CR2_GAIN_10
#elif (DRV8301_SHUNT_GAIN == 20U)
#define DRV8301_CTRL2_GAIN_CONFIG DRV8301_CR2_GAIN_20
#elif (DRV8301_SHUNT_GAIN == 40U)
#define DRV8301_CTRL2_GAIN_CONFIG DRV8301_CR2_GAIN_40
#elif (DRV8301_SHUNT_GAIN == 80U)
#define DRV8301_CTRL2_GAIN_CONFIG DRV8301_CR2_GAIN_80
#else
#error "Unsupported DRV8301 shunt amplifier gain"
#endif

/* Fixed configuration for the current motor driver board. */
#define DRV8301_CTRL1_CONFIG \
    ((uint16_t)(DRV8301_CR1_GATE_CURRENT_1_7A | \
                DRV8301_CR1_GATE_RESET_NORMAL | \
                DRV8301_CR1_PWM_MODE_6PWM | \
                DRV8301_CR1_OC_MODE_LATCH_SD | \
                DRV8301_OC_ADJ_SET_0_250V))

#define DRV8301_CTRL2_CONFIG \
    ((uint16_t)(DRV8301_CR2_OCTW_MODE_OT_OC | \
                DRV8301_CTRL2_GAIN_CONFIG | \
                DRV8301_CR2_DC_CAL_CH1_NORMAL | \
                DRV8301_CR2_DC_CAL_CH2_NORMAL | \
                DRV8301_CR2_OC_TOFF_CYCLE))

static SPI_HandleTypeDef *drv8301_hspi = NULL;

static void DRV8301_DelayCoreCycles(uint32_t cycles)
{
    while (cycles > 0U)
    {
        __NOP();
        --cycles;
    }
}

static DRV8301_Result_t DRV8301_WaitSpiIdle(void)
{
    uint32_t tick_start;

    tick_start = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(drv8301_hspi, SPI_FLAG_BSY) != RESET)
    {
        if ((HAL_GetTick() - tick_start) >= DRV8301_SPI_TIMEOUT)
        {
            return DRV8301_ERROR_SPI;
        }
    }

    return DRV8301_OK;
}

static DRV8301_Result_t DRV8301_Transfer(uint16_t tx_data, uint16_t *rx_data)
{
    HAL_StatusTypeDef status;
    DRV8301_Result_t result;

    if ((drv8301_hspi == NULL) || (rx_data == NULL))
    {
        return DRV8301_ERROR_PARAM;
    }
    if (drv8301_hspi->Init.DataSize != SPI_DATASIZE_16BIT)
    {
        return DRV8301_ERROR_PARAM;
    }

    DRV8301_CS_LOW();
    DRV8301_CS_DELAY();

    status = HAL_SPI_TransmitReceive(drv8301_hspi, (uint8_t *)&tx_data, (uint8_t *)rx_data, 1U, DRV8301_SPI_TIMEOUT);
    result = (status == HAL_OK) ? DRV8301_WaitSpiIdle() : DRV8301_ERROR_SPI;

    DRV8301_CS_DELAY();
    DRV8301_CS_HIGH();

    return result;
}

static DRV8301_Result_t DRV8301_CheckReadResponse(uint16_t response, uint8_t expected_addr)
{
    if (DRV8301_IS_FRAME_ERROR(response) != 0U)
    {
        return DRV8301_ERROR_FRAME;
    }
    if (DRV8301_GET_ADDR(response) != (expected_addr & 0x0FU))
    {
        return DRV8301_ERROR_ADDRESS;
    }

    return DRV8301_OK;
}

static DRV8301_Result_t DRV8301_ReadReg(uint8_t addr, uint16_t *data)
{
    uint16_t command;
    uint16_t response;
    DRV8301_Result_t result;

    if ((data == NULL) || (addr > DRV8301_REG_CTRL2))
    {
        return DRV8301_ERROR_PARAM;
    }

    command = DRV8301_READ_CMD(addr);
    response = 0U;

    /* The first frame sends the command; the second returns its response. */
    result = DRV8301_Transfer(command, &response);
    if (result != DRV8301_OK)
    {
        return result;
    }

    DRV8301_CS_DELAY();
    result = DRV8301_Transfer(command, &response);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_CheckReadResponse(response, addr);
    if (result != DRV8301_OK)
    {
        return result;
    }

    *data = DRV8301_GET_DATA(response);
    return DRV8301_OK;
}

static DRV8301_Result_t DRV8301_WriteReg(uint8_t addr, uint16_t data)
{
    uint16_t command;
    uint16_t response;

    if ((addr != DRV8301_REG_CTRL1) && (addr != DRV8301_REG_CTRL2))
    {
        return DRV8301_ERROR_PARAM;
    }

    command = DRV8301_WRITE_CMD(addr, data);
    response = 0U;

    /* SDO belongs to the previous frame; verify the write by reading it back. */
    return DRV8301_Transfer(command, &response);
}

static DRV8301_Result_t DRV8301_WriteRegVerified(uint8_t addr, uint16_t data)
{
    uint16_t readback;
    DRV8301_Result_t result;

    result = DRV8301_WriteReg(addr, data);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadReg(addr, &readback);
    if (result != DRV8301_OK)
    {
        return result;
    }
    if (readback != (data & DRV8301_DATA_MASK))
    {
        return DRV8301_ERROR_VERIFY;
    }

    return DRV8301_OK;
}

static DRV8301_Result_t DRV8301_ClearFaults(void)
{
    uint16_t ctrl1;
    uint16_t status1;
    uint16_t status2;
    DRV8301_Result_t result;

    result = DRV8301_ReadReg(DRV8301_REG_STATUS1, &status1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadReg(DRV8301_REG_STATUS2, &status2);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadReg(DRV8301_REG_CTRL1, &ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    ctrl1 |= DRV8301_CR1_GATE_RESET_LATCHED;
    result = DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    HAL_Delay(1U);

    ctrl1 &= (uint16_t)~DRV8301_CR1_GATE_RESET_Msk;
    result = DRV8301_WriteRegVerified(DRV8301_REG_CTRL1, ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadReg(DRV8301_REG_STATUS1, &status1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    return ((status1 & DRV8301_SR1_FAULT) == 0U) ? DRV8301_OK : DRV8301_ERROR_FAULT;
}

static DRV8301_Result_t DRV8301_CheckStatus2(uint16_t status2)
{
    uint8_t device_id;

    device_id = (uint8_t)((status2 & DRV8301_SR2_DEVICE_ID_Msk) >> DRV8301_SR2_DEVICE_ID_Pos);
    if (device_id != DRV8301_DEVICE_ID_VALUE)
    {
        return DRV8301_ERROR_DEVICE_ID;
    }
    if ((status2 & DRV8301_SR2_GVDD_OV) != 0U)
    {
        return DRV8301_ERROR_FAULT;
    }

    return DRV8301_OK;
}

DRV8301_Result_t DRV8301_Init(SPI_HandleTypeDef *hspi)
{
    uint16_t status1;
    uint16_t status2;
    DRV8301_Result_t result;

    if ((hspi == NULL) || (hspi->Init.DataSize != SPI_DATASIZE_16BIT))
    {
        return DRV8301_ERROR_PARAM;
    }

    drv8301_hspi = hspi;
    DRV8301_CS_HIGH();

    /* Datasheet tSPI_READY is 5 ms typical and 10 ms maximum. */
    HAL_Delay(10U);

    result = DRV8301_ReadReg(DRV8301_REG_STATUS2, &status2);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_CheckStatus2(status2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ClearFaults();
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_WriteRegVerified(DRV8301_REG_CTRL1, DRV8301_CTRL1_CONFIG);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_WriteRegVerified(DRV8301_REG_CTRL2, DRV8301_CTRL2_CONFIG);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadReg(DRV8301_REG_STATUS1, &status1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadReg(DRV8301_REG_STATUS2, &status2);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_CheckStatus2(status2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    return ((status1 & DRV8301_SR1_FAULT) == 0U) ? DRV8301_OK : DRV8301_ERROR_FAULT;
}

DRV8301_Result_t DRV8301_ReadStatus1(uint16_t *status)
{
    return DRV8301_ReadReg(DRV8301_REG_STATUS1, status);
}

DRV8301_Result_t DRV8301_ReadStatus(uint16_t *status1, uint16_t *status2)
{
    DRV8301_Result_t result;

    if ((status1 == NULL) || (status2 == NULL))
    {
        return DRV8301_ERROR_PARAM;
    }

    result = DRV8301_ReadReg(DRV8301_REG_STATUS1, status1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadReg(DRV8301_REG_STATUS2, status2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    return DRV8301_CheckStatus2(*status2);
}
