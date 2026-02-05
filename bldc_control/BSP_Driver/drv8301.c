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
#define DRV8301_SPI_TIMEOUT     100

/* Delay for CS timing (minimum 400ns) */
#define DRV8301_CS_DELAY()      do { __NOP(); __NOP(); __NOP(); __NOP(); } while(0)

/*============================================================================*/
/*                           Private Variables                                 */
/*============================================================================*/

static SPI_HandleTypeDef *drv8301_hspi = NULL;
static DRV8301_Handle_t drv8301_handle = {0};

/*============================================================================*/
/*                           Private Functions                                 */
/*============================================================================*/

/**
 * @brief  SPI transmit and receive 16-bit data
 * @param  tx_data: data to transmit
 * @return received data
 */
static uint16_t DRV8301_SPI_TransmitReceive(uint16_t tx_data)
{
    uint16_t rx_data = 0;

    DRV8301_CS_LOW();
    DRV8301_CS_DELAY();

    HAL_SPI_TransmitReceive(drv8301_hspi, (uint8_t *)&tx_data, (uint8_t *)&rx_data, 1, DRV8301_SPI_TIMEOUT);

    DRV8301_CS_DELAY();
    DRV8301_CS_HIGH();

    return rx_data;
}

/*============================================================================*/
/*                           Public Functions                                  */
/*============================================================================*/

/**
 * @brief  Read DRV8301 register
 * @param  addr: register address (0x00-0x03)
 * @return register value (11-bit data)
 */
uint16_t DRV8301_ReadReg(uint8_t addr)
{
    uint16_t cmd = DRV8301_READ_CMD(addr);

    /* First read sends command, returns previous data */
    DRV8301_SPI_TransmitReceive(cmd);

    /* Short delay between transactions */
    DRV8301_CS_DELAY();

    /* Second read returns actual register data */
    uint16_t rx_data = DRV8301_SPI_TransmitReceive(cmd);

    return DRV8301_GET_DATA(rx_data);
}

/**
 * @brief  Write DRV8301 register
 * @param  addr: register address (0x02-0x03, only control registers are writable)
 * @param  data: data to write (11-bit)
 */
void DRV8301_WriteReg(uint8_t addr, uint16_t data)
{
    uint16_t cmd = DRV8301_WRITE_CMD(addr, data);
    DRV8301_SPI_TransmitReceive(cmd);
}

/**
 * @brief  Read Status Register 1
 * @return Status Register 1 value
 */
uint16_t DRV8301_ReadStatus1(void)
{
    uint16_t status = DRV8301_ReadReg(DRV8301_REG_STATUS1);

    /* Update handle */
    drv8301_handle.status1 = *(DRV8301_StatusReg1_t *)&status;

    return status;
}

/**
 * @brief  Read Status Register 2
 * @return Status Register 2 value
 */
uint16_t DRV8301_ReadStatus2(void)
{
    uint16_t status = DRV8301_ReadReg(DRV8301_REG_STATUS2);

    /* Update handle */
    drv8301_handle.status2 = *(DRV8301_StatusReg2_t *)&status;

    return status;
}

/**
 * @brief  Read Control Register 1
 * @return Control Register 1 value
 */
uint16_t DRV8301_ReadCtrl1(void)
{
    uint16_t ctrl = DRV8301_ReadReg(DRV8301_REG_CTRL1);

    /* Update handle */
    drv8301_handle.ctrl1 = *(DRV8301_CtrlReg1_t *)&ctrl;

    return ctrl;
}

/**
 * @brief  Read Control Register 2
 * @return Control Register 2 value
 */
uint16_t DRV8301_ReadCtrl2(void)
{
    uint16_t ctrl = DRV8301_ReadReg(DRV8301_REG_CTRL2);

    /* Update handle */
    drv8301_handle.ctrl2 = *(DRV8301_CtrlReg2_t *)&ctrl;

    return ctrl;
}

/**
 * @brief  Check if DRV8301 has fault
 * @return 1: has fault, 0: no fault
 */
uint8_t DRV8301_HasFault(void)
{
    uint16_t status = DRV8301_ReadStatus1();
    return (status & DRV8301_SR1_FAULT) ? 1 : 0;
}

/**
 * @brief  Clear fault flags by reading status registers
 */
void DRV8301_ClearFaults(void)
{
    /* Reading status registers clears the fault flags */
    DRV8301_ReadStatus1();
    DRV8301_ReadStatus2();

    /* Also perform gate reset if needed */
    uint16_t ctrl1 = DRV8301_ReadCtrl1();
    ctrl1 |= DRV8301_CR1_GATE_RESET_LATCHED;
    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);

    HAL_Delay(1);

    /* Clear gate reset bit */
    ctrl1 &= ~DRV8301_CR1_GATE_RESET_Msk;
    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set gate drive current
 * @param  current: gate current setting
 *         - DRV8301_CR1_GATE_CURRENT_1_7A
 *         - DRV8301_CR1_GATE_CURRENT_0_7A
 *         - DRV8301_CR1_GATE_CURRENT_0_25A
 */
void DRV8301_SetGateCurrent(uint16_t current)
{
    uint16_t ctrl1 = DRV8301_ReadCtrl1();
    ctrl1 &= ~DRV8301_CR1_GATE_CURRENT_Msk;
    ctrl1 |= (current & DRV8301_CR1_GATE_CURRENT_Msk);
    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set PWM mode
 * @param  mode: PWM mode
 *         - DRV8301_CR1_PWM_MODE_6PWM
 *         - DRV8301_CR1_PWM_MODE_3PWM
 */
void DRV8301_SetPWMMode(uint16_t mode)
{
    uint16_t ctrl1 = DRV8301_ReadCtrl1();
    ctrl1 &= ~DRV8301_CR1_PWM_MODE_Msk;
    ctrl1 |= (mode & DRV8301_CR1_PWM_MODE_Msk);
    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set overcurrent mode
 * @param  mode: OC mode
 *         - DRV8301_CR1_OC_MODE_LIMIT
 *         - DRV8301_CR1_OC_MODE_LATCH_SD
 *         - DRV8301_CR1_OC_MODE_REPORT
 *         - DRV8301_CR1_OC_MODE_DISABLE
 */
void DRV8301_SetOCMode(uint16_t mode)
{
    uint16_t ctrl1 = DRV8301_ReadCtrl1();
    ctrl1 &= ~DRV8301_CR1_OC_MODE_Msk;
    ctrl1 |= (mode & DRV8301_CR1_OC_MODE_Msk);
    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set overcurrent threshold
 * @param  threshold: OC threshold value (0x00-0x1F)
 */
void DRV8301_SetOCThreshold(uint8_t threshold)
{
    uint16_t ctrl1 = DRV8301_ReadCtrl1();
    ctrl1 &= ~DRV8301_CR1_OC_ADJ_SET_Msk;
    ctrl1 |= (threshold & DRV8301_CR1_OC_ADJ_SET_Msk);
    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
}

/**
 * @brief  Set shunt amplifier gain
 * @param  gain: amplifier gain
 *         - DRV8301_CR2_GAIN_10
 *         - DRV8301_CR2_GAIN_20
 *         - DRV8301_CR2_GAIN_40
 *         - DRV8301_CR2_GAIN_80
 */
void DRV8301_SetGain(uint16_t gain)
{
    uint16_t ctrl2 = DRV8301_ReadCtrl2();
    ctrl2 &= ~DRV8301_CR2_GAIN_Msk;
    ctrl2 |= (gain & DRV8301_CR2_GAIN_Msk);
    DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
}

/**
 * @brief  Enable/Disable DC calibration mode
 * @param  enable: 1 = enable calibration, 0 = normal mode
 */
void DRV8301_SetDCCalMode(uint8_t enable)
{
    uint16_t ctrl2 = DRV8301_ReadCtrl2();

    if (enable)
    {
        ctrl2 |= DRV8301_CR2_DC_CAL_CH1_CAL | DRV8301_CR2_DC_CAL_CH2_CAL;
    }
    else
    {
        ctrl2 &= ~(DRV8301_CR2_DC_CAL_CH1_Msk | DRV8301_CR2_DC_CAL_CH2_Msk);
    }

    DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
}

/**
 * @brief  Set OCTW (overcurrent/overtemperature warning) mode
 * @param  mode: OCTW mode
 *         - DRV8301_CR2_OCTW_MODE_OT_OC
 *         - DRV8301_CR2_OCTW_MODE_OT_ONLY
 *         - DRV8301_CR2_OCTW_MODE_OC_ONLY
 */
void DRV8301_SetOCTWMode(uint16_t mode)
{
    uint16_t ctrl2 = DRV8301_ReadCtrl2();
    ctrl2 &= ~DRV8301_CR2_OCTW_MODE_Msk;
    ctrl2 |= (mode & DRV8301_CR2_OCTW_MODE_Msk);
    DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
}

/**
 * @brief  Get device ID
 * @return device ID (should be 0x01 for DRV8301)
 */
uint8_t DRV8301_GetDeviceID(void)
{
    uint16_t status2 = DRV8301_ReadStatus2();
    return (status2 & DRV8301_SR2_DEVICE_ID_Msk);
}

/**
 * @brief  Get current amplifier gain value
 * @return gain value (10.0, 20.0, 40.0, or 80.0)
 */
float DRV8301_GetGainValue(void)
{
    uint16_t ctrl2 = DRV8301_ReadCtrl2();
    uint8_t gain_setting = (ctrl2 & DRV8301_CR2_GAIN_Msk) >> DRV8301_CR2_GAIN_Pos;

    switch (gain_setting)
    {
        case 0: return DRV8301_GAIN_VALUE_10;
        case 1: return DRV8301_GAIN_VALUE_20;
        case 2: return DRV8301_GAIN_VALUE_40;
        case 3: return DRV8301_GAIN_VALUE_80;
        default: return DRV8301_GAIN_VALUE_10;
    }
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
    buf[0] = '\0';

    if (status1 & DRV8301_SR1_FAULT)
    {
        if (status1 & DRV8301_SR1_GVDD_UV)
            strncat(buf, "GVDD_UV ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_PVDD_UV)
            strncat(buf, "PVDD_UV ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_OTSD)
            strncat(buf, "OTSD ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_OTW)
            strncat(buf, "OTW ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_FETHA_OC)
            strncat(buf, "FETHA_OC ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_FETLA_OC)
            strncat(buf, "FETLA_OC ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_FETHB_OC)
            strncat(buf, "FETHB_OC ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_FETLB_OC)
            strncat(buf, "FETLB_OC ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_FETHC_OC)
            strncat(buf, "FETHC_OC ", buf_size - strlen(buf) - 1);
        if (status1 & DRV8301_SR1_FETLC_OC)
            strncat(buf, "FETLC_OC ", buf_size - strlen(buf) - 1);
    }
    else
    {
        strncpy(buf, "No Fault", buf_size - 1);
    }
}

/**
 * @brief  Read all registers and update handle
 */
void DRV8301_UpdateAll(void)
{
    DRV8301_ReadStatus1();
    DRV8301_ReadStatus2();
    DRV8301_ReadCtrl1();
    DRV8301_ReadCtrl2();
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
 * @return 0: success, -1: failed
 */
int DRV8301_ConfigInit(SPI_HandleTypeDef *hspi, uint16_t gate_current, uint16_t pwm_mode,
                           uint16_t oc_mode, uint8_t oc_threshold, uint16_t gain,
                           uint16_t octw_mode, uint16_t oc_toff)
{
    if (hspi == NULL)
    {
        return -1;
    }

    drv8301_hspi = hspi;

    /* Ensure CS is high initially */
    DRV8301_CS_HIGH();
    HAL_Delay(10);

    /* Clear any existing faults */
    DRV8301_ClearFaults();
    HAL_Delay(1);

    /* Configure Control Register 1 */
    uint16_t ctrl1 = 0;
    ctrl1 |= (gate_current & DRV8301_CR1_GATE_CURRENT_Msk);
    ctrl1 |= DRV8301_CR1_GATE_RESET_NORMAL;
    ctrl1 |= (pwm_mode & DRV8301_CR1_PWM_MODE_Msk);
    ctrl1 |= (oc_mode & DRV8301_CR1_OC_MODE_Msk);
    ctrl1 |= (oc_threshold & DRV8301_CR1_OC_ADJ_SET_Msk);

    DRV8301_WriteReg(DRV8301_REG_CTRL1, ctrl1);
    HAL_Delay(1);

    /* Configure Control Register 2 */
    uint16_t ctrl2 = 0;
    ctrl2 |= (octw_mode & DRV8301_CR2_OCTW_MODE_Msk);
    ctrl2 |= (gain & DRV8301_CR2_GAIN_Msk);
    ctrl2 |= DRV8301_CR2_DC_CAL_CH1_NORMAL;
    ctrl2 |= DRV8301_CR2_DC_CAL_CH2_NORMAL;
    ctrl2 |= (oc_toff & DRV8301_CR2_OC_TOFF_Msk);

    DRV8301_WriteReg(DRV8301_REG_CTRL2, ctrl2);
    HAL_Delay(1);

    /* Verify configuration */
    uint16_t read_ctrl1 = DRV8301_ReadCtrl1();
    uint16_t read_ctrl2 = DRV8301_ReadCtrl2();

    if ((read_ctrl1 != ctrl1) || (read_ctrl2 != ctrl2))
    {
        return -1;
    }

    return 0;
}
