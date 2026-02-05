/**
 * @file    drv8301.h
 * @brief   DRV8301 Three-Phase Gate Driver Register Definitions
 * @note    Based on Texas Instruments DRV8301 Datasheet (SLVSCJ4)
 *          https://www.ti.com/lit/ds/symlink/drv8301.pdf
 */

#ifndef _DRV8301_H_
#define _DRV8301_H_

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/*                           SPI Frame Format                                  */
/*============================================================================*/
/* SPI 16-bit frame structure:
 * Bit 15:    R/W (0 = Write, 1 = Read)
 * Bit 14-11: Address (4 bits)
 * Bit 10-0:  Data (11 bits)
 */

#define DRV8301_SPI_READ            (1U << 15)
#define DRV8301_SPI_WRITE           (0U << 15)
#define DRV8301_ADDR_SHIFT          11
#define DRV8301_ADDR_MASK           (0x0F << DRV8301_ADDR_SHIFT)
#define DRV8301_DATA_MASK           0x07FF

/*============================================================================*/
/*                           Register Addresses                                */
/*============================================================================*/

#define DRV8301_REG_STATUS1         0x00    /* Status Register 1 (Read Only) */
#define DRV8301_REG_STATUS2         0x01    /* Status Register 2 (Read Only) */
#define DRV8301_REG_CTRL1           0x02    /* Control Register 1 */
#define DRV8301_REG_CTRL2           0x03    /* Control Register 2 */

/*============================================================================*/
/*                    Status Register 1 (0x00) - Read Only                     */
/*============================================================================*/
/* Bit definitions for fault and warning flags */

#define DRV8301_SR1_FAULT_Pos       10
#define DRV8301_SR1_FAULT_Msk       (0x01U << DRV8301_SR1_FAULT_Pos)
#define DRV8301_SR1_FAULT           DRV8301_SR1_FAULT_Msk   /* Fault indicator (OR of all faults) */

#define DRV8301_SR1_GVDD_UV_Pos     9
#define DRV8301_SR1_GVDD_UV_Msk     (0x01U << DRV8301_SR1_GVDD_UV_Pos)
#define DRV8301_SR1_GVDD_UV         DRV8301_SR1_GVDD_UV_Msk /* Gate driver undervoltage fault */

#define DRV8301_SR1_PVDD_UV_Pos     8
#define DRV8301_SR1_PVDD_UV_Msk     (0x01U << DRV8301_SR1_PVDD_UV_Pos)
#define DRV8301_SR1_PVDD_UV         DRV8301_SR1_PVDD_UV_Msk /* Charge pump undervoltage fault */

#define DRV8301_SR1_OTSD_Pos        7
#define DRV8301_SR1_OTSD_Msk        (0x01U << DRV8301_SR1_OTSD_Pos)
#define DRV8301_SR1_OTSD            DRV8301_SR1_OTSD_Msk    /* Overtemperature shutdown */

#define DRV8301_SR1_OTW_Pos         6
#define DRV8301_SR1_OTW_Msk         (0x01U << DRV8301_SR1_OTW_Pos)
#define DRV8301_SR1_OTW             DRV8301_SR1_OTW_Msk     /* Overtemperature warning */

#define DRV8301_SR1_FETHA_OC_Pos    5
#define DRV8301_SR1_FETHA_OC_Msk    (0x01U << DRV8301_SR1_FETHA_OC_Pos)
#define DRV8301_SR1_FETHA_OC        DRV8301_SR1_FETHA_OC_Msk /* FET high side A overcurrent */

#define DRV8301_SR1_FETLA_OC_Pos    4
#define DRV8301_SR1_FETLA_OC_Msk    (0x01U << DRV8301_SR1_FETLA_OC_Pos)
#define DRV8301_SR1_FETLA_OC        DRV8301_SR1_FETLA_OC_Msk /* FET low side A overcurrent */

#define DRV8301_SR1_FETHB_OC_Pos    3
#define DRV8301_SR1_FETHB_OC_Msk    (0x01U << DRV8301_SR1_FETHB_OC_Pos)
#define DRV8301_SR1_FETHB_OC        DRV8301_SR1_FETHB_OC_Msk /* FET high side B overcurrent */

#define DRV8301_SR1_FETLB_OC_Pos    2
#define DRV8301_SR1_FETLB_OC_Msk    (0x01U << DRV8301_SR1_FETLB_OC_Pos)
#define DRV8301_SR1_FETLB_OC        DRV8301_SR1_FETLB_OC_Msk /* FET low side B overcurrent */

#define DRV8301_SR1_FETHC_OC_Pos    1
#define DRV8301_SR1_FETHC_OC_Msk    (0x01U << DRV8301_SR1_FETHC_OC_Pos)
#define DRV8301_SR1_FETHC_OC        DRV8301_SR1_FETHC_OC_Msk /* FET high side C overcurrent */

#define DRV8301_SR1_FETLC_OC_Pos    0
#define DRV8301_SR1_FETLC_OC_Msk    (0x01U << DRV8301_SR1_FETLC_OC_Pos)
#define DRV8301_SR1_FETLC_OC        DRV8301_SR1_FETLC_OC_Msk /* FET low side C overcurrent */

/*============================================================================*/
/*                    Status Register 2 (0x01) - Read Only                     */
/*============================================================================*/

#define DRV8301_SR2_GVDD_OV_Pos     7
#define DRV8301_SR2_GVDD_OV_Msk     (0x01U << DRV8301_SR2_GVDD_OV_Pos)
#define DRV8301_SR2_GVDD_OV         DRV8301_SR2_GVDD_OV_Msk /* Gate driver overvoltage */

#define DRV8301_SR2_DEVICE_ID_Pos   0
#define DRV8301_SR2_DEVICE_ID_Msk   (0x0FU << DRV8301_SR2_DEVICE_ID_Pos)
#define DRV8301_SR2_DEVICE_ID       DRV8301_SR2_DEVICE_ID_Msk /* Device ID (should be 0x01) */

#define DRV8301_DEVICE_ID_VALUE     0x01    /* Expected Device ID */

/*============================================================================*/
/*                       Control Register 1 (0x02)                             */
/*============================================================================*/

/* Gate Drive Peak Current (GATE_CURRENT) - Bits 10:9 */
#define DRV8301_CR1_GATE_CURRENT_Pos    9
#define DRV8301_CR1_GATE_CURRENT_Msk    (0x03U << DRV8301_CR1_GATE_CURRENT_Pos)
#define DRV8301_CR1_GATE_CURRENT_1_7A   (0x00U << DRV8301_CR1_GATE_CURRENT_Pos) /* 1.7A peak */
#define DRV8301_CR1_GATE_CURRENT_0_7A   (0x01U << DRV8301_CR1_GATE_CURRENT_Pos) /* 0.7A peak */
#define DRV8301_CR1_GATE_CURRENT_0_25A  (0x02U << DRV8301_CR1_GATE_CURRENT_Pos) /* 0.25A peak */

/* Gate Reset - Bit 8 */
#define DRV8301_CR1_GATE_RESET_Pos      8
#define DRV8301_CR1_GATE_RESET_Msk      (0x01U << DRV8301_CR1_GATE_RESET_Pos)
#define DRV8301_CR1_GATE_RESET_NORMAL   (0x00U << DRV8301_CR1_GATE_RESET_Pos)   /* Normal operation */
#define DRV8301_CR1_GATE_RESET_LATCHED  (0x01U << DRV8301_CR1_GATE_RESET_Pos)   /* Reset gate driver latched faults */

/* PWM Mode - Bit 7 */
#define DRV8301_CR1_PWM_MODE_Pos        7
#define DRV8301_CR1_PWM_MODE_Msk        (0x01U << DRV8301_CR1_PWM_MODE_Pos)
#define DRV8301_CR1_PWM_MODE_6PWM       (0x00U << DRV8301_CR1_PWM_MODE_Pos)     /* 6 PWM inputs */
#define DRV8301_CR1_PWM_MODE_3PWM       (0x01U << DRV8301_CR1_PWM_MODE_Pos)     /* 3 PWM inputs */

/* Overcurrent Mode (OC_MODE) - Bits 6:5 */
#define DRV8301_CR1_OC_MODE_Pos         5
#define DRV8301_CR1_OC_MODE_Msk         (0x03U << DRV8301_CR1_OC_MODE_Pos)
#define DRV8301_CR1_OC_MODE_LIMIT       (0x00U << DRV8301_CR1_OC_MODE_Pos)      /* Current limit */
#define DRV8301_CR1_OC_MODE_LATCH_SD    (0x01U << DRV8301_CR1_OC_MODE_Pos)      /* OC latch shutdown */
#define DRV8301_CR1_OC_MODE_REPORT      (0x02U << DRV8301_CR1_OC_MODE_Pos)      /* Report only */
#define DRV8301_CR1_OC_MODE_DISABLE     (0x03U << DRV8301_CR1_OC_MODE_Pos)      /* OC disabled */

/* Overcurrent Adjustment (OC_ADJ_SET) - Bits 4:0 */
#define DRV8301_CR1_OC_ADJ_SET_Pos      0
#define DRV8301_CR1_OC_ADJ_SET_Msk      (0x1FU << DRV8301_CR1_OC_ADJ_SET_Pos)

/* OC_ADJ_SET Values - VDS threshold for overcurrent */
#define DRV8301_OC_ADJ_SET_0_060V       0x00    /* 0.060V */
#define DRV8301_OC_ADJ_SET_0_068V       0x01    /* 0.068V */
#define DRV8301_OC_ADJ_SET_0_076V       0x02    /* 0.076V */
#define DRV8301_OC_ADJ_SET_0_086V       0x03    /* 0.086V */
#define DRV8301_OC_ADJ_SET_0_097V       0x04    /* 0.097V */
#define DRV8301_OC_ADJ_SET_0_109V       0x05    /* 0.109V */
#define DRV8301_OC_ADJ_SET_0_123V       0x06    /* 0.123V */
#define DRV8301_OC_ADJ_SET_0_138V       0x07    /* 0.138V */
#define DRV8301_OC_ADJ_SET_0_155V       0x08    /* 0.155V */
#define DRV8301_OC_ADJ_SET_0_175V       0x09    /* 0.175V */
#define DRV8301_OC_ADJ_SET_0_197V       0x0A    /* 0.197V */
#define DRV8301_OC_ADJ_SET_0_222V       0x0B    /* 0.222V */
#define DRV8301_OC_ADJ_SET_0_250V       0x0C    /* 0.250V */
#define DRV8301_OC_ADJ_SET_0_282V       0x0D    /* 0.282V */
#define DRV8301_OC_ADJ_SET_0_317V       0x0E    /* 0.317V */
#define DRV8301_OC_ADJ_SET_0_358V       0x0F    /* 0.358V */
#define DRV8301_OC_ADJ_SET_0_403V       0x10    /* 0.403V */
#define DRV8301_OC_ADJ_SET_0_454V       0x11    /* 0.454V */
#define DRV8301_OC_ADJ_SET_0_511V       0x12    /* 0.511V */
#define DRV8301_OC_ADJ_SET_0_576V       0x13    /* 0.576V */
#define DRV8301_OC_ADJ_SET_0_648V       0x14    /* 0.648V */
#define DRV8301_OC_ADJ_SET_0_730V       0x15    /* 0.730V */
#define DRV8301_OC_ADJ_SET_0_822V       0x16    /* 0.822V */
#define DRV8301_OC_ADJ_SET_0_926V       0x17    /* 0.926V */
#define DRV8301_OC_ADJ_SET_1_043V       0x18    /* 1.043V */
#define DRV8301_OC_ADJ_SET_1_175V       0x19    /* 1.175V */
#define DRV8301_OC_ADJ_SET_1_324V       0x1A    /* 1.324V */
#define DRV8301_OC_ADJ_SET_1_491V       0x1B    /* 1.491V */
#define DRV8301_OC_ADJ_SET_1_679V       0x1C    /* 1.679V */
#define DRV8301_OC_ADJ_SET_1_892V       0x1D    /* 1.892V */
#define DRV8301_OC_ADJ_SET_2_131V       0x1E    /* 2.131V */
#define DRV8301_OC_ADJ_SET_2_400V       0x1F    /* 2.400V */

/* Control Register 1 Default Value */
#define DRV8301_CR1_DEFAULT             0x0000

/*============================================================================*/
/*                       Control Register 2 (0x03)                             */
/*============================================================================*/

/* OCTW Mode - Bits 10:9 */
#define DRV8301_CR2_OCTW_MODE_Pos       9
#define DRV8301_CR2_OCTW_MODE_Msk       (0x03U << DRV8301_CR2_OCTW_MODE_Pos)
#define DRV8301_CR2_OCTW_MODE_OT_OC     (0x00U << DRV8301_CR2_OCTW_MODE_Pos)    /* Report OT and OC */
#define DRV8301_CR2_OCTW_MODE_OT_ONLY   (0x01U << DRV8301_CR2_OCTW_MODE_Pos)    /* Report OT only */
#define DRV8301_CR2_OCTW_MODE_OC_ONLY   (0x02U << DRV8301_CR2_OCTW_MODE_Pos)    /* Report OC only */

/* Shunt Amplifier Gain (GAIN) - Bits 8:7 */
#define DRV8301_CR2_GAIN_Pos            7
#define DRV8301_CR2_GAIN_Msk            (0x03U << DRV8301_CR2_GAIN_Pos)
#define DRV8301_CR2_GAIN_10             (0x00U << DRV8301_CR2_GAIN_Pos)         /* Gain = 10 V/V */
#define DRV8301_CR2_GAIN_20             (0x01U << DRV8301_CR2_GAIN_Pos)         /* Gain = 20 V/V */
#define DRV8301_CR2_GAIN_40             (0x02U << DRV8301_CR2_GAIN_Pos)         /* Gain = 40 V/V */
#define DRV8301_CR2_GAIN_80             (0x03U << DRV8301_CR2_GAIN_Pos)         /* Gain = 80 V/V */

/* DC Calibration CH1 (DC_CAL_CH1) - Bit 6 */
#define DRV8301_CR2_DC_CAL_CH1_Pos      6
#define DRV8301_CR2_DC_CAL_CH1_Msk      (0x01U << DRV8301_CR2_DC_CAL_CH1_Pos)
#define DRV8301_CR2_DC_CAL_CH1_NORMAL   (0x00U << DRV8301_CR2_DC_CAL_CH1_Pos)   /* Normal operation */
#define DRV8301_CR2_DC_CAL_CH1_CAL      (0x01U << DRV8301_CR2_DC_CAL_CH1_Pos)   /* Short inputs for offset cal */

/* DC Calibration CH2 (DC_CAL_CH2) - Bit 5 */
#define DRV8301_CR2_DC_CAL_CH2_Pos      5
#define DRV8301_CR2_DC_CAL_CH2_Msk      (0x01U << DRV8301_CR2_DC_CAL_CH2_Pos)
#define DRV8301_CR2_DC_CAL_CH2_NORMAL   (0x00U << DRV8301_CR2_DC_CAL_CH2_Pos)   /* Normal operation */
#define DRV8301_CR2_DC_CAL_CH2_CAL      (0x01U << DRV8301_CR2_DC_CAL_CH2_Pos)   /* Short inputs for offset cal */

/* OC_TOFF - Bit 4 */
#define DRV8301_CR2_OC_TOFF_Pos         4
#define DRV8301_CR2_OC_TOFF_Msk         (0x01U << DRV8301_CR2_OC_TOFF_Pos)
#define DRV8301_CR2_OC_TOFF_CYCLE       (0x00U << DRV8301_CR2_OC_TOFF_Pos)      /* Cycle by cycle */
#define DRV8301_CR2_OC_TOFF_OFFTIME     (0x01U << DRV8301_CR2_OC_TOFF_Pos)      /* Off-time control */

/* Control Register 2 Default Value */
#define DRV8301_CR2_DEFAULT             0x0000

/*============================================================================*/
/*                         SPI Helper Macros                                   */
/*============================================================================*/

/* Build SPI write command */
#define DRV8301_WRITE_CMD(addr, data) \
    (DRV8301_SPI_WRITE | (((addr) & 0x0F) << DRV8301_ADDR_SHIFT) | ((data) & DRV8301_DATA_MASK))

/* Build SPI read command */
#define DRV8301_READ_CMD(addr) \
    (DRV8301_SPI_READ | (((addr) & 0x0F) << DRV8301_ADDR_SHIFT))

/* Extract data from SPI response */
#define DRV8301_GET_DATA(response) \
    ((response) & DRV8301_DATA_MASK)

/* Extract address from SPI response */
#define DRV8301_GET_ADDR(response) \
    (((response) >> DRV8301_ADDR_SHIFT) & 0x0F)

/* Check frame error bit (Bit 15 of SDO indicates frame error) */
#define DRV8301_IS_FRAME_ERROR(response) \
    (((response) >> 15) & 0x01)

/*============================================================================*/
/*                         Type Definitions                                    */
/*============================================================================*/

/* Status Register 1 bit field structure */
typedef struct {
    uint16_t FETLC_OC  : 1;     /* Bit 0:  FET Low side C overcurrent */
    uint16_t FETHC_OC  : 1;     /* Bit 1:  FET High side C overcurrent */
    uint16_t FETLB_OC  : 1;     /* Bit 2:  FET Low side B overcurrent */
    uint16_t FETHB_OC  : 1;     /* Bit 3:  FET High side B overcurrent */
    uint16_t FETLA_OC  : 1;     /* Bit 4:  FET Low side A overcurrent */
    uint16_t FETHA_OC  : 1;     /* Bit 5:  FET High side A overcurrent */
    uint16_t OTW       : 1;     /* Bit 6:  Overtemperature warning */
    uint16_t OTSD      : 1;     /* Bit 7:  Overtemperature shutdown */
    uint16_t PVDD_UV   : 1;     /* Bit 8:  Charge pump undervoltage */
    uint16_t GVDD_UV   : 1;     /* Bit 9:  Gate driver undervoltage */
    uint16_t FAULT     : 1;     /* Bit 10: Fault indicator */
    uint16_t Reserved  : 5;     /* Bit 15-11: Reserved */
} DRV8301_StatusReg1_t;

/* Status Register 2 bit field structure */
typedef struct {
    uint16_t DEVICE_ID : 4;     /* Bit 3-0: Device ID */
    uint16_t Reserved1 : 3;     /* Bit 6-4: Reserved */
    uint16_t GVDD_OV   : 1;     /* Bit 7:   Gate driver overvoltage */
    uint16_t Reserved2 : 8;     /* Bit 15-8: Reserved */
} DRV8301_StatusReg2_t;

/* Control Register 1 bit field structure */
typedef struct {
    uint16_t OC_ADJ_SET   : 5;  /* Bit 4-0:  Overcurrent adjustment */
    uint16_t OC_MODE      : 2;  /* Bit 6-5:  Overcurrent mode */
    uint16_t PWM_MODE     : 1;  /* Bit 7:    PWM mode (0=6PWM, 1=3PWM) */
    uint16_t GATE_RESET   : 1;  /* Bit 8:    Gate reset */
    uint16_t GATE_CURRENT : 2;  /* Bit 10-9: Gate drive current */
    uint16_t Reserved     : 5;  /* Bit 15-11: Reserved */
} DRV8301_CtrlReg1_t;

/* Control Register 2 bit field structure */
typedef struct {
    uint16_t Reserved1    : 4;  /* Bit 3-0:  Reserved */
    uint16_t OC_TOFF      : 1;  /* Bit 4:    OC off-time mode */
    uint16_t DC_CAL_CH2   : 1;  /* Bit 5:    DC calibration CH2 */
    uint16_t DC_CAL_CH1   : 1;  /* Bit 6:    DC calibration CH1 */
    uint16_t GAIN         : 2;  /* Bit 8-7:  Shunt amplifier gain */
    uint16_t OCTW_MODE    : 2;  /* Bit 10-9: OC/OT warning mode */
    uint16_t Reserved2    : 5;  /* Bit 15-11: Reserved */
} DRV8301_CtrlReg2_t;

/* DRV8301 handle structure */
typedef struct {
    DRV8301_StatusReg1_t status1;
    DRV8301_StatusReg2_t status2;
    DRV8301_CtrlReg1_t   ctrl1;
    DRV8301_CtrlReg2_t   ctrl2;
} DRV8301_Handle_t;

/*============================================================================*/
/*                      Gain Value Lookup                                      */
/*============================================================================*/

/* Shunt amplifier gain values for current calculation */
#define DRV8301_GAIN_VALUE_10   10.0f
#define DRV8301_GAIN_VALUE_20   20.0f
#define DRV8301_GAIN_VALUE_40   40.0f
#define DRV8301_GAIN_VALUE_80   80.0f

/*============================================================================*/
/*                         Function Prototypes                                 */
/*============================================================================*/

/* Initialization */

int DRV8301_ConfigInit(SPI_HandleTypeDef *hspi, uint16_t gate_current, uint16_t pwm_mode,
                           uint16_t oc_mode, uint8_t oc_threshold, uint16_t gain,
                           uint16_t octw_mode, uint16_t oc_toff);

/* Register Read/Write */
uint16_t DRV8301_ReadReg(uint8_t addr);
void DRV8301_WriteReg(uint8_t addr, uint16_t data);

/* Status Register Access */
uint16_t DRV8301_ReadStatus1(void);
uint16_t DRV8301_ReadStatus2(void);
uint16_t DRV8301_ReadCtrl1(void);
uint16_t DRV8301_ReadCtrl2(void);

/* Fault Management */
uint8_t DRV8301_HasFault(void);
void DRV8301_ClearFaults(void);
void DRV8301_GetFaultString(uint16_t status1, char *buf, uint16_t buf_size);

/* Control Register 1 Configuration */
void DRV8301_SetGateCurrent(uint16_t current);
void DRV8301_SetPWMMode(uint16_t mode);
void DRV8301_SetOCMode(uint16_t mode);
void DRV8301_SetOCThreshold(uint8_t threshold);

/* Control Register 2 Configuration */
void DRV8301_SetGain(uint16_t gain);
void DRV8301_SetDCCalMode(uint8_t enable);
void DRV8301_SetOCTWMode(uint16_t mode);

/* Utility Functions */
uint8_t DRV8301_GetDeviceID(void);
float DRV8301_GetGainValue(void);
DRV8301_Handle_t* DRV8301_GetHandle(void);
void DRV8301_UpdateAll(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRV8301_H_ */
