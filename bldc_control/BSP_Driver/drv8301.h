/**
 * @file    drv8301.h
 * @brief   DRV8301 Three-Phase Gate Driver Register Definitions
 * @note    Based on Texas Instruments DRV8301 Datasheet (SLVSCJ4)
 *          https://www.ti.com/lit/ds/symlink/drv8301.pdf
 */

#ifndef _DRV8301_H_
#define _DRV8301_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/*                        Board-Level Configuration                            */
/*============================================================================*/
/*
 * 电流采样标定（依据 SCH_Motor.pdf）：三相 shunt 均为 10 mΩ。C 相不走 DRV8301
 * 内部放大器，而是外接 LM2904（R45=1k / R47=10k，增益 10 V/V），因此内部增益
 * 必须同样取 10 V/V，三相才能共用一套标定系数。
 * 0.010 Ω x 10 V/V = 0.1 V/A，3.3 V 量程叠加 REF/2 偏置后约 ±16.5 A。
 */
#ifndef DRV8301_SHUNT_GAIN
#define DRV8301_SHUNT_GAIN          10U
#endif
#define DRV8301_SHUNT_RESISTANCE    0.010f
#define DRV8301_VOLTS_PER_AMP       (DRV8301_SHUNT_RESISTANCE * (float)DRV8301_SHUNT_GAIN)

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
#define DRV8301_SR2_DEVICE_ID       DRV8301_SR2_DEVICE_ID_Msk /* Device ID */

/*
 * 手册 Table 10 只定义了 Device ID[3:0] 字段，未给出期望值，因此驱动不做校验，
 * 由上层从 DRV8301_ReadStatus() 返回的 status2 中自行读取判断。
 */

/*============================================================================*/
/*                       Control Register 1 (0x02)                             */
/*============================================================================*/

/* Gate Drive Peak Current (GATE_CURRENT) - Bits 1:0 */
#define DRV8301_CR1_GATE_CURRENT_Pos    0
#define DRV8301_CR1_GATE_CURRENT_Msk    (0x03U << DRV8301_CR1_GATE_CURRENT_Pos)
#define DRV8301_CR1_GATE_CURRENT_1_7A   (0x00U << DRV8301_CR1_GATE_CURRENT_Pos) /* 1.7A peak */
#define DRV8301_CR1_GATE_CURRENT_0_7A   (0x01U << DRV8301_CR1_GATE_CURRENT_Pos) /* 0.7A peak */
#define DRV8301_CR1_GATE_CURRENT_0_25A  (0x02U << DRV8301_CR1_GATE_CURRENT_Pos) /* 0.25A peak */

/* Gate Reset - Bit 2 */
#define DRV8301_CR1_GATE_RESET_Pos      2
#define DRV8301_CR1_GATE_RESET_Msk      (0x01U << DRV8301_CR1_GATE_RESET_Pos)
#define DRV8301_CR1_GATE_RESET_NORMAL   (0x00U << DRV8301_CR1_GATE_RESET_Pos)   /* Normal operation */
#define DRV8301_CR1_GATE_RESET_LATCHED  (0x01U << DRV8301_CR1_GATE_RESET_Pos)   /* Reset gate driver latched faults */

/* PWM Mode - Bit 3 */
#define DRV8301_CR1_PWM_MODE_Pos        3
#define DRV8301_CR1_PWM_MODE_Msk        (0x01U << DRV8301_CR1_PWM_MODE_Pos)
#define DRV8301_CR1_PWM_MODE_6PWM       (0x00U << DRV8301_CR1_PWM_MODE_Pos)     /* 6 PWM inputs */
#define DRV8301_CR1_PWM_MODE_3PWM       (0x01U << DRV8301_CR1_PWM_MODE_Pos)     /* 3 PWM inputs */

/* Overcurrent Mode (OC_MODE) - Bits 5:4 */
#define DRV8301_CR1_OC_MODE_Pos         4
#define DRV8301_CR1_OC_MODE_Msk         (0x03U << DRV8301_CR1_OC_MODE_Pos)
#define DRV8301_CR1_OC_MODE_LIMIT       (0x00U << DRV8301_CR1_OC_MODE_Pos)      /* Current limit */
#define DRV8301_CR1_OC_MODE_LATCH_SD    (0x01U << DRV8301_CR1_OC_MODE_Pos)      /* OC latch shutdown */
#define DRV8301_CR1_OC_MODE_REPORT      (0x02U << DRV8301_CR1_OC_MODE_Pos)      /* Report only */
#define DRV8301_CR1_OC_MODE_DISABLE     (0x03U << DRV8301_CR1_OC_MODE_Pos)      /* OC disabled */

/* Overcurrent Adjustment (OC_ADJ_SET) - Bits 10:6 */
#define DRV8301_CR1_OC_ADJ_SET_Pos      6
#define DRV8301_CR1_OC_ADJ_SET_Msk      (0x1FU << DRV8301_CR1_OC_ADJ_SET_Pos)

/* OC_ADJ_SET Values - VDS threshold for overcurrent */
#define DRV8301_OC_ADJ_SET_0_060V       (0x00U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.060V */
#define DRV8301_OC_ADJ_SET_0_068V       (0x01U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.068V */
#define DRV8301_OC_ADJ_SET_0_076V       (0x02U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.076V */
#define DRV8301_OC_ADJ_SET_0_086V       (0x03U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.086V */
#define DRV8301_OC_ADJ_SET_0_097V       (0x04U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.097V */
#define DRV8301_OC_ADJ_SET_0_109V       (0x05U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.109V */
#define DRV8301_OC_ADJ_SET_0_123V       (0x06U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.123V */
#define DRV8301_OC_ADJ_SET_0_138V       (0x07U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.138V */
#define DRV8301_OC_ADJ_SET_0_155V       (0x08U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.155V */
#define DRV8301_OC_ADJ_SET_0_175V       (0x09U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.175V */
#define DRV8301_OC_ADJ_SET_0_197V       (0x0AU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.197V */
#define DRV8301_OC_ADJ_SET_0_222V       (0x0BU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.222V */
#define DRV8301_OC_ADJ_SET_0_250V       (0x0CU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.250V */
#define DRV8301_OC_ADJ_SET_0_282V       (0x0DU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.282V */
#define DRV8301_OC_ADJ_SET_0_317V       (0x0EU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.317V */
#define DRV8301_OC_ADJ_SET_0_358V       (0x0FU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.358V */
#define DRV8301_OC_ADJ_SET_0_403V       (0x10U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.403V */
#define DRV8301_OC_ADJ_SET_0_454V       (0x11U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.454V */
#define DRV8301_OC_ADJ_SET_0_511V       (0x12U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.511V */
#define DRV8301_OC_ADJ_SET_0_576V       (0x13U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.576V */
#define DRV8301_OC_ADJ_SET_0_648V       (0x14U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.648V */
#define DRV8301_OC_ADJ_SET_0_730V       (0x15U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.730V */
#define DRV8301_OC_ADJ_SET_0_822V       (0x16U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.822V */
#define DRV8301_OC_ADJ_SET_0_926V       (0x17U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 0.926V */
#define DRV8301_OC_ADJ_SET_1_043V       (0x18U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 1.043V */
#define DRV8301_OC_ADJ_SET_1_175V       (0x19U << DRV8301_CR1_OC_ADJ_SET_Pos) /* 1.175V */
#define DRV8301_OC_ADJ_SET_1_324V       (0x1AU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 1.324V */
#define DRV8301_OC_ADJ_SET_1_491V       (0x1BU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 1.491V */
#define DRV8301_OC_ADJ_SET_1_679V       (0x1CU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 1.679V */
#define DRV8301_OC_ADJ_SET_1_892V       (0x1DU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 1.892V */
#define DRV8301_OC_ADJ_SET_2_131V       (0x1EU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 2.131V */
#define DRV8301_OC_ADJ_SET_2_400V       (0x1FU << DRV8301_CR1_OC_ADJ_SET_Pos) /* 2.400V */

/* Control Register 1 Default Value */
#define DRV8301_CR1_DEFAULT             0x0000

/*============================================================================*/
/*                       Control Register 2 (0x03)                             */
/*============================================================================*/

/* OCTW Mode - Bits 1:0 */
#define DRV8301_CR2_OCTW_MODE_Pos       0
#define DRV8301_CR2_OCTW_MODE_Msk       (0x03U << DRV8301_CR2_OCTW_MODE_Pos)
#define DRV8301_CR2_OCTW_MODE_OT_OC     (0x00U << DRV8301_CR2_OCTW_MODE_Pos)    /* Report OT and OC */
#define DRV8301_CR2_OCTW_MODE_OT_ONLY   (0x01U << DRV8301_CR2_OCTW_MODE_Pos)    /* Report OT only */
#define DRV8301_CR2_OCTW_MODE_OC_ONLY   (0x02U << DRV8301_CR2_OCTW_MODE_Pos)    /* Report OC only */

/* Shunt Amplifier Gain (GAIN) - Bits 3:2 */
#define DRV8301_CR2_GAIN_Pos            2
#define DRV8301_CR2_GAIN_Msk            (0x03U << DRV8301_CR2_GAIN_Pos)
#define DRV8301_CR2_GAIN_10             (0x00U << DRV8301_CR2_GAIN_Pos)         /* Gain = 10 V/V */
#define DRV8301_CR2_GAIN_20             (0x01U << DRV8301_CR2_GAIN_Pos)         /* Gain = 20 V/V */
#define DRV8301_CR2_GAIN_40             (0x02U << DRV8301_CR2_GAIN_Pos)         /* Gain = 40 V/V */
#define DRV8301_CR2_GAIN_80             (0x03U << DRV8301_CR2_GAIN_Pos)         /* Gain = 80 V/V */

/* DC Calibration CH1 (DC_CAL_CH1) - Bit 4 */
#define DRV8301_CR2_DC_CAL_CH1_Pos      4
#define DRV8301_CR2_DC_CAL_CH1_Msk      (0x01U << DRV8301_CR2_DC_CAL_CH1_Pos)
#define DRV8301_CR2_DC_CAL_CH1_NORMAL   (0x00U << DRV8301_CR2_DC_CAL_CH1_Pos)   /* Normal operation */
#define DRV8301_CR2_DC_CAL_CH1_CAL      (0x01U << DRV8301_CR2_DC_CAL_CH1_Pos)   /* Short inputs for offset cal */

/* DC Calibration CH2 (DC_CAL_CH2) - Bit 5 */
#define DRV8301_CR2_DC_CAL_CH2_Pos      5
#define DRV8301_CR2_DC_CAL_CH2_Msk      (0x01U << DRV8301_CR2_DC_CAL_CH2_Pos)
#define DRV8301_CR2_DC_CAL_CH2_NORMAL   (0x00U << DRV8301_CR2_DC_CAL_CH2_Pos)   /* Normal operation */
#define DRV8301_CR2_DC_CAL_CH2_CAL      (0x01U << DRV8301_CR2_DC_CAL_CH2_Pos)   /* Short inputs for offset cal */

/* OC_TOFF - Bit 6 */
#define DRV8301_CR2_OC_TOFF_Pos         6
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

typedef enum {
    DRV8301_OK = 0,
    DRV8301_ERROR_PARAM = -1,
    DRV8301_ERROR_SPI = -2,
    DRV8301_ERROR_FRAME = -3,
    DRV8301_ERROR_ADDRESS = -4,
    DRV8301_ERROR_VERIFY = -5,
    DRV8301_ERROR_FAULT = -6
} DRV8301_Result_t;

/*============================================================================*/
/*                         Function Prototypes                                 */
/*============================================================================*/

/**
 * @brief Enable the device (EN_GATE high) and apply the fixed board configuration.
 * @param hspi SPI handle configured for 16-bit transfers.
 * @return DRV8301 operation result.
 * @note  EN_GATE 拉低时器件休眠且 SPI 完全不响应，因此必须由本函数先使能。
 */
DRV8301_Result_t DRV8301_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief Pull EN_GATE low to shut down the gate driver immediately.
 * @note  本板 nFAULT/nOCTW 未接 MCU，这是唯一的软件级硬关断手段，急停必须调用。
 */
void DRV8301_Shutdown(void);

/**
 * @brief Read both status registers for low-rate diagnostics.
 */
DRV8301_Result_t DRV8301_ReadStatus(uint16_t *status1, uint16_t *status2);

#ifdef __cplusplus
}
#endif

#endif /* _DRV8301_H_ */
