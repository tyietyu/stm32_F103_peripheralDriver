/**
 * @file    drv8301.c
 * @brief   DRV8301 Three-Phase Gate Driver Driver Implementation
 * @note    Based on Texas Instruments DRV8301 Datasheet
 */

#include "drv8301.h"
#include "spi.h"
#include "main.h"

/*============================================================================*/
/*                           Private Defines                                   */
/*============================================================================*/

/* CS Pin Control */
#define DRV8301_CS_LOW()    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define DRV8301_CS_HIGH()   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

/* SPI Timeout */
#define DRV8301_SPI_TIMEOUT     100U

/* Datasheet tSU_SCS/tHD_SCS are 50 ns; use at least 100 ns and verify on-board. */
#define DRV8301_CS_DELAY_CYCLES ((SystemCoreClock / 10000000U) + 1U)
#define DRV8301_CS_DELAY()      DRV8301_DelayCoreCycles(DRV8301_CS_DELAY_CYCLES)

/*============================================================================*/
/*                           Private Variables                                 */
/*============================================================================*/

static SPI_HandleTypeDef *drv8301_hspi = NULL;
static DRV8301_Handle_t drv8301_handle = {0};

/*============================================================================*/
/*                           Private Functions                                 */
/*============================================================================*/

static void DRV8301_DelayCoreCycles(uint32_t cycles)
{
    while (cycles > 0U)
    {
        __NOP();
        --cycles;
    }
}

static DRV8301_Result_t DRV8301_RecordResult(DRV8301_Result_t result)
{
    drv8301_handle.last_result = result;
    if ((result == DRV8301_ERROR_SPI) || (result == DRV8301_ERROR_FRAME) ||
        (result == DRV8301_ERROR_ADDRESS))
    {
        drv8301_handle.communication_error = 1U;
    }

    return result;
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

/**
 * @brief  SPI transmit and receive one 16-bit frame
 * @param  tx_data: data to transmit
 * @param  rx_data: received data
 * @return DRV8301 result
 */
static DRV8301_Result_t DRV8301_SPI_TransmitReceive(uint16_t tx_data, uint16_t *rx_data)
{
    HAL_StatusTypeDef status;
    DRV8301_Result_t result;

    if ((drv8301_hspi == NULL) || (rx_data == NULL))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    if (drv8301_hspi->Init.DataSize != SPI_DATASIZE_16BIT)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    DRV8301_CS_LOW();
    DRV8301_CS_DELAY();

    status = HAL_SPI_TransmitReceive(drv8301_hspi, (uint8_t *)&tx_data, (uint8_t *)rx_data, 1U, DRV8301_SPI_TIMEOUT);
    result = (status == HAL_OK) ? DRV8301_WaitSpiIdle() : DRV8301_ERROR_SPI;

    DRV8301_CS_DELAY();
    DRV8301_CS_HIGH();

    return DRV8301_RecordResult(result);
}

static DRV8301_Result_t DRV8301_CheckReadResponse(uint16_t response, uint8_t expected_addr)
{
    if (DRV8301_IS_FRAME_ERROR(response))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_FRAME);
    }

    if (DRV8301_GET_ADDR(response) != (expected_addr & 0x0FU))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_ADDRESS);
    }

    return DRV8301_RecordResult(DRV8301_OK);
}

static DRV8301_Result_t DRV8301_ReadRegChecked(uint8_t addr, uint16_t *data)
{
    uint16_t cmd;
    uint16_t rx_data;
    DRV8301_Result_t result;

    if ((data == NULL) || (addr > DRV8301_REG_CTRL2))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    cmd = DRV8301_READ_CMD(addr);
    rx_data = 0U;

    /* First read sends command, returns previous data */
    result = DRV8301_SPI_TransmitReceive(cmd, &rx_data);
    if (result != DRV8301_OK)
    {
        return result;
    }

    /* Meet tHI_SCS before starting the response frame. */
    DRV8301_CS_DELAY();

    /* Second read returns actual register data */
    result = DRV8301_SPI_TransmitReceive(cmd, &rx_data);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_CheckReadResponse(rx_data, addr);
    if (result != DRV8301_OK)
    {
        return result;
    }

    *data = DRV8301_GET_DATA(rx_data);
    return DRV8301_RecordResult(DRV8301_OK);
}

static DRV8301_Result_t DRV8301_WriteRegChecked(uint8_t addr, uint16_t data)
{
    uint16_t cmd;
    uint16_t rx_data;

    if ((addr != DRV8301_REG_CTRL1) && (addr != DRV8301_REG_CTRL2))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    cmd = DRV8301_WRITE_CMD(addr, data);
    rx_data = 0U;

    /* SDO belongs to the previous frame; current write is verified separately. */
    return DRV8301_SPI_TransmitReceive(cmd, &rx_data);
}

static DRV8301_Result_t DRV8301_WriteRegVerified(uint8_t addr, uint16_t data)
{
    uint16_t readback;
    DRV8301_Result_t result;

    result = DRV8301_WriteRegChecked(addr, data);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadRegChecked(addr, &readback);
    if (result != DRV8301_OK)
    {
        return result;
    }

    if (readback != (data & DRV8301_DATA_MASK))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_VERIFY);
    }

    return DRV8301_RecordResult(DRV8301_OK);
}

static void DRV8301_AppendFaultString(char *buf, uint16_t buf_size, const char *fault)
{
    size_t used;

    if ((buf == NULL) || (fault == NULL) || (buf_size == 0U))
    {
        return;
    }

    used = strlen(buf);
    if (used >= (size_t)(buf_size - 1U))
    {
        return;
    }

    strncat(buf, fault, (size_t)buf_size - used - 1U);
}

/*============================================================================*/
/*                           Public Functions                                  */
/*============================================================================*/

/**
 * @brief  Read DRV8301 register
 * @param  addr: register address (0x00-0x03)
 * @param  data: register value output (11-bit data)
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ReadReg(uint8_t addr, uint16_t *data)
{
    return DRV8301_ReadRegChecked(addr, data);
}

/**
 * @brief  Write DRV8301 register
 * @param  addr: register address (0x02-0x03, only control registers are writable)
 * @param  data: data to write (11-bit)
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_WriteReg(uint8_t addr, uint16_t data)
{
    return DRV8301_WriteRegVerified(addr, data);
}

/**
 * @brief  Read Status Register 1
 * @param  status: Status Register 1 output
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ReadStatus1(uint16_t *status)
{
    DRV8301_Result_t result;

    if (status == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadRegChecked(DRV8301_REG_STATUS1, status);
    if (result == DRV8301_OK)
    {
        drv8301_handle.status1 = *status;
    }

    return result;
}

/**
 * @brief  Read Status Register 2
 * @param  status: Status Register 2 output
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ReadStatus2(uint16_t *status)
{
    DRV8301_Result_t result;

    if (status == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadRegChecked(DRV8301_REG_STATUS2, status);
    if (result == DRV8301_OK)
    {
        drv8301_handle.status2 = *status;
    }

    return result;
}

/**
 * @brief  Read Control Register 1
 * @param  ctrl: Control Register 1 output
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ReadCtrl1(uint16_t *ctrl)
{
    DRV8301_Result_t result;

    if (ctrl == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadRegChecked(DRV8301_REG_CTRL1, ctrl);
    if (result == DRV8301_OK)
    {
        drv8301_handle.ctrl1 = *ctrl;
    }

    return result;
}

/**
 * @brief  Read Control Register 2
 * @param  ctrl: Control Register 2 output
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ReadCtrl2(uint16_t *ctrl)
{
    DRV8301_Result_t result;

    if (ctrl == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadRegChecked(DRV8301_REG_CTRL2, ctrl);
    if (result == DRV8301_OK)
    {
        drv8301_handle.ctrl2 = *ctrl;
    }

    return result;
}

/**
 * @brief  Check if DRV8301 has fault
 * @param  has_fault: 1 if fault is active, otherwise 0
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_HasFault(uint8_t *has_fault)
{
    uint16_t status;
    DRV8301_Result_t result;

    if (has_fault == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadStatus1(&status);
    if (result != DRV8301_OK)
    {
        return result;
    }

    *has_fault = ((status & DRV8301_SR1_FAULT) != 0U) ? 1U : 0U;
    return DRV8301_OK;
}

/**
 * @brief  Clear fault flags by reading status registers
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ClearFaults(void)
{
    uint16_t status1;
    uint16_t status2;
    uint16_t ctrl1;
    DRV8301_Result_t result;

    /* Reading status registers clears the fault flags */
    result = DRV8301_ReadStatus1(&status1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadStatus2(&status2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    /* Also perform gate reset if needed */
    result = DRV8301_ReadCtrl1(&ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl1 |= DRV8301_CR1_GATE_RESET_LATCHED;
    result = DRV8301_WriteRegChecked(DRV8301_REG_CTRL1, ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    HAL_Delay(1);

    /* Clear gate reset bit */
    ctrl1 &= ~DRV8301_CR1_GATE_RESET_Msk;
    result = DRV8301_WriteRegVerified(DRV8301_REG_CTRL1, ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadStatus1(&status1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    return ((status1 & DRV8301_SR1_FAULT) == 0U)
               ? DRV8301_RecordResult(DRV8301_OK)
               : DRV8301_RecordResult(DRV8301_ERROR_FAULT);
}

/**
 * @brief  Set gate drive current
 * @param  current: gate current setting
 *         - DRV8301_CR1_GATE_CURRENT_1_7A
 *         - DRV8301_CR1_GATE_CURRENT_0_7A
 *         - DRV8301_CR1_GATE_CURRENT_0_25A
 */
DRV8301_Result_t DRV8301_SetGateCurrent(uint16_t current)
{
    uint16_t ctrl1;
    DRV8301_Result_t result;

    if ((current != DRV8301_CR1_GATE_CURRENT_1_7A) &&
        (current != DRV8301_CR1_GATE_CURRENT_0_7A) &&
        (current != DRV8301_CR1_GATE_CURRENT_0_25A))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl1(&ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl1 &= ~DRV8301_CR1_GATE_CURRENT_Msk;
    ctrl1 |= (current & DRV8301_CR1_GATE_CURRENT_Msk);
    return DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set PWM mode
 * @param  mode: PWM mode
 *         - DRV8301_CR1_PWM_MODE_6PWM
 *         - DRV8301_CR1_PWM_MODE_3PWM
 */
DRV8301_Result_t DRV8301_SetPWMMode(uint16_t mode)
{
    uint16_t ctrl1;
    DRV8301_Result_t result;

    if ((mode != DRV8301_CR1_PWM_MODE_6PWM) &&
        (mode != DRV8301_CR1_PWM_MODE_3PWM))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl1(&ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl1 &= ~DRV8301_CR1_PWM_MODE_Msk;
    ctrl1 |= (mode & DRV8301_CR1_PWM_MODE_Msk);
    return DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set overcurrent mode
 * @param  mode: OC mode
 *         - DRV8301_CR1_OC_MODE_LIMIT
 *         - DRV8301_CR1_OC_MODE_LATCH_SD
 *         - DRV8301_CR1_OC_MODE_REPORT
 *         - DRV8301_CR1_OC_MODE_DISABLE
 */
DRV8301_Result_t DRV8301_SetOCMode(uint16_t mode)
{
    uint16_t ctrl1;
    DRV8301_Result_t result;

    if ((mode != DRV8301_CR1_OC_MODE_LIMIT) &&
        (mode != DRV8301_CR1_OC_MODE_LATCH_SD) &&
        (mode != DRV8301_CR1_OC_MODE_REPORT) &&
        (mode != DRV8301_CR1_OC_MODE_DISABLE))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl1(&ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl1 &= ~DRV8301_CR1_OC_MODE_Msk;
    ctrl1 |= (mode & DRV8301_CR1_OC_MODE_Msk);
    return DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set overcurrent threshold
 * @param  threshold: OC threshold value (0x00-0x1F)
 */
DRV8301_Result_t DRV8301_SetOCThreshold(uint8_t threshold)
{
    uint16_t ctrl1;
    DRV8301_Result_t result;

    if (threshold > DRV8301_CR1_OC_ADJ_SET_Msk)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl1(&ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl1 &= ~DRV8301_CR1_OC_ADJ_SET_Msk;
    ctrl1 |= (threshold & DRV8301_CR1_OC_ADJ_SET_Msk);
    return DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set shunt amplifier gain
 * @param  gain: amplifier gain
 *         - DRV8301_CR2_GAIN_10
 *         - DRV8301_CR2_GAIN_20
 *         - DRV8301_CR2_GAIN_40
 *         - DRV8301_CR2_GAIN_80
 */
DRV8301_Result_t DRV8301_SetGain(uint16_t gain)
{
    uint16_t ctrl2;
    DRV8301_Result_t result;

    if ((gain != DRV8301_CR2_GAIN_10) && (gain != DRV8301_CR2_GAIN_20) &&
        (gain != DRV8301_CR2_GAIN_40) && (gain != DRV8301_CR2_GAIN_80))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl2(&ctrl2);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl2 &= ~DRV8301_CR2_GAIN_Msk;
    ctrl2 |= (gain & DRV8301_CR2_GAIN_Msk);
    return DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
}

/**
 * @brief  Enable/Disable DC calibration mode
 * @param  enable: 1 = enable calibration, 0 = normal mode
 */
DRV8301_Result_t DRV8301_SetDCCalMode(uint8_t enable)
{
    uint16_t ctrl2;
    DRV8301_Result_t result;

    if (enable > 1U)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl2(&ctrl2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    if (enable != 0U)
    {
        ctrl2 |= DRV8301_CR2_DC_CAL_CH1_CAL | DRV8301_CR2_DC_CAL_CH2_CAL;
    }
    else
    {
        ctrl2 &= ~(DRV8301_CR2_DC_CAL_CH1_Msk | DRV8301_CR2_DC_CAL_CH2_Msk);
    }

    return DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
}

/**
 * @brief  Set OCTW (overcurrent/overtemperature warning) mode
 * @param  mode: OCTW mode
 *         - DRV8301_CR2_OCTW_MODE_OT_OC
 *         - DRV8301_CR2_OCTW_MODE_OT_ONLY
 *         - DRV8301_CR2_OCTW_MODE_OC_ONLY
 */
DRV8301_Result_t DRV8301_SetOCTWMode(uint16_t mode)
{
    uint16_t ctrl2;
    DRV8301_Result_t result;

    if ((mode != DRV8301_CR2_OCTW_MODE_OT_OC) &&
        (mode != DRV8301_CR2_OCTW_MODE_OT_ONLY) &&
        (mode != DRV8301_CR2_OCTW_MODE_OC_ONLY))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl2(&ctrl2);
    if (result != DRV8301_OK)
    {
        return result;
    }
    ctrl2 &= ~DRV8301_CR2_OCTW_MODE_Msk;
    ctrl2 |= (mode & DRV8301_CR2_OCTW_MODE_Msk);
    return DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
}

/**
 * @brief  Get device ID
 * @param  device_id: Device ID output
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_GetDeviceID(uint8_t *device_id)
{
    uint16_t status2;
    DRV8301_Result_t result;

    if (device_id == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadStatus2(&status2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    *device_id = (uint8_t)((status2 & DRV8301_SR2_DEVICE_ID_Msk) >>
                           DRV8301_SR2_DEVICE_ID_Pos);
    return DRV8301_OK;
}

/**
 * @brief  Get current amplifier gain value
 * @param  gain_value: gain value output (10.0, 20.0, 40.0, or 80.0)
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_GetGainValue(float *gain_value)
{
    uint16_t ctrl2;
    uint8_t gain_setting;
    DRV8301_Result_t result;

    if (gain_value == NULL)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    result = DRV8301_ReadCtrl2(&ctrl2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    gain_setting = (uint8_t)((ctrl2 & DRV8301_CR2_GAIN_Msk) >>
                             DRV8301_CR2_GAIN_Pos);

    switch (gain_setting)
    {
        case 0U: *gain_value = DRV8301_GAIN_VALUE_10; break;
        case 1U: *gain_value = DRV8301_GAIN_VALUE_20; break;
        case 2U: *gain_value = DRV8301_GAIN_VALUE_40; break;
        case 3U: *gain_value = DRV8301_GAIN_VALUE_80; break;
        default: return DRV8301_RecordResult(DRV8301_ERROR_VERIFY);
    }

    return DRV8301_OK;
}

/**
 * @brief  Get DRV8301 handle for accessing register structures
 * @return pointer to DRV8301 handle
 */
DRV8301_Handle_t* DRV8301_GetHandle(void)
{
    return &drv8301_handle;
}

/**
 * @brief  Print fault status (for debugging)
 * @param  status1: Status Register 1 value
 */
void DRV8301_GetFaultString(uint16_t status1, char *buf, uint16_t buf_size)
{
    if ((buf == NULL) || (buf_size == 0U))
    {
        return;
    }

    buf[0] = '\0';

    if (status1 & DRV8301_SR1_FAULT)
    {
        if (status1 & DRV8301_SR1_GVDD_UV)
            DRV8301_AppendFaultString(buf, buf_size, "GVDD_UV ");
        if (status1 & DRV8301_SR1_PVDD_UV)
            DRV8301_AppendFaultString(buf, buf_size, "PVDD_UV ");
        if (status1 & DRV8301_SR1_OTSD)
            DRV8301_AppendFaultString(buf, buf_size, "OTSD ");
        if (status1 & DRV8301_SR1_OTW)
            DRV8301_AppendFaultString(buf, buf_size, "OTW ");
        if (status1 & DRV8301_SR1_FETHA_OC)
            DRV8301_AppendFaultString(buf, buf_size, "FETHA_OC ");
        if (status1 & DRV8301_SR1_FETLA_OC)
            DRV8301_AppendFaultString(buf, buf_size, "FETLA_OC ");
        if (status1 & DRV8301_SR1_FETHB_OC)
            DRV8301_AppendFaultString(buf, buf_size, "FETHB_OC ");
        if (status1 & DRV8301_SR1_FETLB_OC)
            DRV8301_AppendFaultString(buf, buf_size, "FETLB_OC ");
        if (status1 & DRV8301_SR1_FETHC_OC)
            DRV8301_AppendFaultString(buf, buf_size, "FETHC_OC ");
        if (status1 & DRV8301_SR1_FETLC_OC)
            DRV8301_AppendFaultString(buf, buf_size, "FETLC_OC ");
        if (buf[0] == '\0')
            DRV8301_AppendFaultString(buf, buf_size, "FAULT ");
    }
    else
    {
        strncpy(buf, "No Fault", buf_size - 1);
        buf[buf_size - 1] = '\0';
    }
}

/**
 * @brief  Read all registers and update handle
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_UpdateAll(void)
{
    uint16_t value;
    DRV8301_Result_t result;

    result = DRV8301_ReadStatus1(&value);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadStatus2(&value);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadCtrl1(&value);
    if (result != DRV8301_OK)
    {
        return result;
    }

    return DRV8301_ReadCtrl2(&value);
}

/**
 * @brief  Configure DRV8301 with full custom settings
 * @param  hspi: SPI handle pointer
 * @param  gate_current: gate drive current setting
 * @param  pwm_mode: PWM mode setting
 * @param  oc_mode: overcurrent mode setting
 * @param  oc_threshold: overcurrent threshold setting
 * @param  gain: shunt amplifier gain setting
 * @param  octw_mode: overcurrent/overtemperature warning mode
 * @param  oc_toff: overcurrent off-time mode
 * @return DRV8301 result
 */
DRV8301_Result_t DRV8301_ConfigInit(SPI_HandleTypeDef *hspi,
                                    uint16_t gate_current,
                                    uint16_t pwm_mode,
                                    uint16_t oc_mode,
                                    uint8_t oc_threshold,
                                    uint16_t gain,
                                    uint16_t octw_mode,
                                    uint16_t oc_toff)
{
    uint16_t status1;
    uint16_t status2;
    uint16_t ctrl1;
    uint16_t ctrl2;
    uint16_t read_ctrl1;
    uint16_t read_ctrl2;
    uint8_t device_id;
    DRV8301_Result_t result;

    if ((hspi == NULL) || (hspi->Init.DataSize != SPI_DATASIZE_16BIT))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    if (((gate_current != DRV8301_CR1_GATE_CURRENT_1_7A) &&
         (gate_current != DRV8301_CR1_GATE_CURRENT_0_7A) &&
         (gate_current != DRV8301_CR1_GATE_CURRENT_0_25A)) ||
        ((pwm_mode != DRV8301_CR1_PWM_MODE_6PWM) &&
         (pwm_mode != DRV8301_CR1_PWM_MODE_3PWM)) ||
        ((oc_mode != DRV8301_CR1_OC_MODE_LIMIT) &&
         (oc_mode != DRV8301_CR1_OC_MODE_LATCH_SD) &&
         (oc_mode != DRV8301_CR1_OC_MODE_REPORT) &&
         (oc_mode != DRV8301_CR1_OC_MODE_DISABLE)) ||
        (oc_threshold > DRV8301_CR1_OC_ADJ_SET_Msk) ||
        ((gain != DRV8301_CR2_GAIN_10) && (gain != DRV8301_CR2_GAIN_20) &&
         (gain != DRV8301_CR2_GAIN_40) && (gain != DRV8301_CR2_GAIN_80)) ||
        ((octw_mode != DRV8301_CR2_OCTW_MODE_OT_OC) &&
         (octw_mode != DRV8301_CR2_OCTW_MODE_OT_ONLY) &&
         (octw_mode != DRV8301_CR2_OCTW_MODE_OC_ONLY)) ||
        ((oc_toff != DRV8301_CR2_OC_TOFF_CYCLE) &&
         (oc_toff != DRV8301_CR2_OC_TOFF_OFFTIME)))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_PARAM);
    }

    memset(&drv8301_handle, 0, sizeof(drv8301_handle));
    drv8301_hspi = hspi;

    DRV8301_CS_HIGH();
    /* Datasheet tSPI_READY is 5 ms typical and 10 ms maximum. */
    HAL_Delay(10);

    result = DRV8301_GetDeviceID(&device_id);
    if (result != DRV8301_OK)
    {
        return result;
    }
    if (device_id != DRV8301_DEVICE_ID_VALUE)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_DEVICE_ID);
    }

    result = DRV8301_ReadStatus1(&status1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadStatus2(&status2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ClearFaults();
    if (result != DRV8301_OK)
    {
        return result;
    }
    HAL_Delay(1);

    ctrl1 = 0U;
    ctrl1 |= (gate_current & DRV8301_CR1_GATE_CURRENT_Msk);
    ctrl1 |= DRV8301_CR1_GATE_RESET_NORMAL;
    ctrl1 |= (pwm_mode & DRV8301_CR1_PWM_MODE_Msk);
    ctrl1 |= (oc_mode & DRV8301_CR1_OC_MODE_Msk);
    ctrl1 |= (oc_threshold & DRV8301_CR1_OC_ADJ_SET_Msk);

    result = DRV8301_WriteRegVerified(DRV8301_REG_CTRL1, ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }

    ctrl2 = 0U;
    ctrl2 |= (octw_mode & DRV8301_CR2_OCTW_MODE_Msk);
    ctrl2 |= (gain & DRV8301_CR2_GAIN_Msk);
    ctrl2 |= DRV8301_CR2_DC_CAL_CH1_NORMAL;
    ctrl2 |= DRV8301_CR2_DC_CAL_CH2_NORMAL;
    ctrl2 |= (oc_toff & DRV8301_CR2_OC_TOFF_Msk);

    result = DRV8301_WriteRegVerified(DRV8301_REG_CTRL2, ctrl2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    result = DRV8301_ReadCtrl1(&read_ctrl1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadCtrl2(&read_ctrl2);
    if (result != DRV8301_OK)
    {
        return result;
    }

    if ((read_ctrl1 != ctrl1) || (read_ctrl2 != ctrl2))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_VERIFY);
    }

    result = DRV8301_GetDeviceID(&device_id);
    if (result != DRV8301_OK)
    {
        return result;
    }
    if (device_id != DRV8301_DEVICE_ID_VALUE)
    {
        return DRV8301_RecordResult(DRV8301_ERROR_DEVICE_ID);
    }

    result = DRV8301_ReadStatus1(&status1);
    if (result != DRV8301_OK)
    {
        return result;
    }
    result = DRV8301_ReadStatus2(&status2);
    if (result != DRV8301_OK)
    {
        return result;
    }
    if (((status1 & DRV8301_SR1_FAULT) != 0U) ||
        ((status2 & DRV8301_SR2_GVDD_OV) != 0U))
    {
        return DRV8301_RecordResult(DRV8301_ERROR_FAULT);
    }

    return DRV8301_RecordResult(DRV8301_OK);
}
