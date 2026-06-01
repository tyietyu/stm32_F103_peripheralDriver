#include "spi_flash.h"
#include "main.h"
#include <string.h>

#define W25Q_CMD_WRITE_ENABLE       0x06U
#define W25Q_CMD_READ_STATUS1       0x05U
#define W25Q_CMD_PAGE_PROGRAM       0x02U
#define W25Q_CMD_READ_DATA          0x03U
#define W25Q_CMD_SECTOR_ERASE_4K    0x20U
#define W25Q_CMD_JEDEC_ID           0x9FU
#define W25Q_STATUS1_BUSY           0x01U
#define SPI_FLASH_TIMEOUT_TICKS     1000000UL

#ifndef SPI1_CS_GPIO_Port
#define SPI1_CS_GPIO_Port GPIOA
#endif

#ifndef SPI1_CS_Pin
#define SPI1_CS_Pin GPIO_PIN_4
#endif

static uint8_t s_spi_flash_inited;

static int spi_flash_is_supported_id(uint32_t id)
{
    switch (id) {
        case 0xEF4015UL: /* W25Q16 */
        case 0xEF4016UL: /* W25Q32 */
        case 0xEF4017UL: /* W25Q64 */
        case 0xEF4018UL: /* W25Q128 */
        case 0xC84015UL: /* GD25Q16 */
        case 0xC84016UL: /* GD25Q32 */
        case 0xC84017UL: /* GD25Q64 */
        case 0xC84018UL: /* GD25Q128 */
            return 1;
        default:
            return 0;
    }
}

static void spi_flash_cs_low(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static void spi_flash_cs_high(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

static int spi_flash_wait_flag(uint32_t flag, uint32_t set)
{
    uint32_t timeout = SPI_FLASH_TIMEOUT_TICKS;

    while (timeout-- > 0U) {
        if (set != 0U) {
            if ((SPI1->SR & flag) != 0U) {
                return SPI_FLASH_OK;
            }
        } else {
            if ((SPI1->SR & flag) == 0U) {
                return SPI_FLASH_OK;
            }
        }
    }

    return SPI_FLASH_ERR_TIMEOUT;
}

static int spi_flash_transfer_byte(uint8_t tx, uint8_t *rx)
{
    uint8_t dummy;

    if (spi_flash_wait_flag(SPI_SR_TXE, 1U) != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_TIMEOUT;
    }

    *(__IO uint8_t *)&SPI1->DR = tx;

    if (spi_flash_wait_flag(SPI_SR_RXNE, 1U) != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_TIMEOUT;
    }

    dummy = *(__IO uint8_t *)&SPI1->DR;
    if (rx != 0) {
        *rx = dummy;
    }

    return SPI_FLASH_OK;
}

static int spi_flash_wait_not_busy(void)
{
    uint8_t status;
    uint32_t timeout = SPI_FLASH_TIMEOUT_TICKS;

    do {
        spi_flash_cs_low();
        if (spi_flash_transfer_byte(W25Q_CMD_READ_STATUS1, 0) != SPI_FLASH_OK ||
            spi_flash_transfer_byte(0xFFU, &status) != SPI_FLASH_OK) {
            spi_flash_cs_high();
            return SPI_FLASH_ERR_IO;
        }
        spi_flash_cs_high();

        if ((status & W25Q_STATUS1_BUSY) == 0U) {
            return SPI_FLASH_OK;
        }
    } while (timeout-- > 0U);

    return SPI_FLASH_ERR_TIMEOUT;
}

static int spi_flash_write_enable(void)
{
    int ret;

    spi_flash_cs_low();
    ret = spi_flash_transfer_byte(W25Q_CMD_WRITE_ENABLE, 0);
    spi_flash_cs_high();

    return ret;
}

static int spi_flash_check_range(uint32_t addr, uint32_t len)
{
    if (len == 0U) {
        return SPI_FLASH_OK;
    }

    if ((addr >= OTA_EXT_FLASH_SIZE_BYTES) ||
        (len > OTA_EXT_FLASH_SIZE_BYTES) ||
        ((addr + len) > OTA_EXT_FLASH_SIZE_BYTES) ||
        ((addr + len) < addr)) {
        return SPI_FLASH_ERR_RANGE;
    }

    return SPI_FLASH_OK;
}

static void spi_flash_hw_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SPI1_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1;
    SPI1->CR2 = 0U;
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint32_t spi_flash_read_id(void)
{
    uint8_t id[3] = {0U, 0U, 0U};

    spi_flash_hw_init();
    spi_flash_cs_low();
    (void)spi_flash_transfer_byte(W25Q_CMD_JEDEC_ID, 0);
    (void)spi_flash_transfer_byte(0xFFU, &id[0]);
    (void)spi_flash_transfer_byte(0xFFU, &id[1]);
    (void)spi_flash_transfer_byte(0xFFU, &id[2]);
    spi_flash_cs_high();

    return ((uint32_t)id[0] << 16U) | ((uint32_t)id[1] << 8U) | id[2];
}

int spi_flash_init(void)
{
    uint32_t id;

    spi_flash_hw_init();
    id = spi_flash_read_id();
    if (spi_flash_is_supported_id(id) == 0) {
        return SPI_FLASH_ERR_ID;
    }

    s_spi_flash_inited = 1U;
    return SPI_FLASH_OK;
}

int spi_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t i;

    if ((buf == 0) && (len != 0U)) {
        return SPI_FLASH_ERR_PARAM;
    }
    if (spi_flash_check_range(addr, len) != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_RANGE;
    }
    if (s_spi_flash_inited == 0U) {
        if (spi_flash_init() != SPI_FLASH_OK) {
            return SPI_FLASH_ERR_ID;
        }
    }

    if (spi_flash_wait_not_busy() != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_TIMEOUT;
    }

    spi_flash_cs_low();
    if (spi_flash_transfer_byte(W25Q_CMD_READ_DATA, 0) != SPI_FLASH_OK ||
        spi_flash_transfer_byte((uint8_t)(addr >> 16U), 0) != SPI_FLASH_OK ||
        spi_flash_transfer_byte((uint8_t)(addr >> 8U), 0) != SPI_FLASH_OK ||
        spi_flash_transfer_byte((uint8_t)addr, 0) != SPI_FLASH_OK) {
        spi_flash_cs_high();
        return SPI_FLASH_ERR_IO;
    }

    for (i = 0U; i < len; i++) {
        if (spi_flash_transfer_byte(0xFFU, &buf[i]) != SPI_FLASH_OK) {
            spi_flash_cs_high();
            return SPI_FLASH_ERR_IO;
        }
    }
    spi_flash_cs_high();

    return SPI_FLASH_OK;
}

int spi_flash_erase_sector(uint32_t addr)
{
    uint32_t sector_addr = addr & ~(OTA_EXT_FLASH_SECTOR_SIZE - 1UL);

    if (spi_flash_check_range(sector_addr, OTA_EXT_FLASH_SECTOR_SIZE) != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_RANGE;
    }
    if (s_spi_flash_inited == 0U) {
        if (spi_flash_init() != SPI_FLASH_OK) {
            return SPI_FLASH_ERR_ID;
        }
    }
    if (spi_flash_wait_not_busy() != SPI_FLASH_OK ||
        spi_flash_write_enable() != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_TIMEOUT;
    }

    spi_flash_cs_low();
    if (spi_flash_transfer_byte(W25Q_CMD_SECTOR_ERASE_4K, 0) != SPI_FLASH_OK ||
        spi_flash_transfer_byte((uint8_t)(sector_addr >> 16U), 0) != SPI_FLASH_OK ||
        spi_flash_transfer_byte((uint8_t)(sector_addr >> 8U), 0) != SPI_FLASH_OK ||
        spi_flash_transfer_byte((uint8_t)sector_addr, 0) != SPI_FLASH_OK) {
        spi_flash_cs_high();
        return SPI_FLASH_ERR_IO;
    }
    spi_flash_cs_high();

    return spi_flash_wait_not_busy();
}

int spi_flash_erase_range(uint32_t addr, uint32_t len)
{
    uint32_t end;
    uint32_t cur;
    int ret;

    if (len == 0U) {
        return SPI_FLASH_OK;
    }
    if (spi_flash_check_range(addr, len) != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_RANGE;
    }

    cur = addr & ~(OTA_EXT_FLASH_SECTOR_SIZE - 1UL);
    end = addr + len;
    while (cur < end) {
        ret = spi_flash_erase_sector(cur);
        if (ret != SPI_FLASH_OK) {
            return ret;
        }
        cur += OTA_EXT_FLASH_SECTOR_SIZE;
    }

    return SPI_FLASH_OK;
}

int spi_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint8_t verify_buf[32];
    uint32_t written = 0U;
    uint32_t verify_offset;

    if ((buf == 0) && (len != 0U)) {
        return SPI_FLASH_ERR_PARAM;
    }
    if (spi_flash_check_range(addr, len) != SPI_FLASH_OK) {
        return SPI_FLASH_ERR_RANGE;
    }
    if (s_spi_flash_inited == 0U) {
        if (spi_flash_init() != SPI_FLASH_OK) {
            return SPI_FLASH_ERR_ID;
        }
    }

    while (written < len) {
        uint32_t page_left = OTA_EXT_FLASH_PAGE_SIZE - ((addr + written) % OTA_EXT_FLASH_PAGE_SIZE);
        uint32_t chunk = (len - written > page_left) ? page_left : (len - written);
        uint32_t i;
        uint32_t wr_addr = addr + written;

        if (spi_flash_wait_not_busy() != SPI_FLASH_OK ||
            spi_flash_write_enable() != SPI_FLASH_OK) {
            return SPI_FLASH_ERR_TIMEOUT;
        }

        spi_flash_cs_low();
        if (spi_flash_transfer_byte(W25Q_CMD_PAGE_PROGRAM, 0) != SPI_FLASH_OK ||
            spi_flash_transfer_byte((uint8_t)(wr_addr >> 16U), 0) != SPI_FLASH_OK ||
            spi_flash_transfer_byte((uint8_t)(wr_addr >> 8U), 0) != SPI_FLASH_OK ||
            spi_flash_transfer_byte((uint8_t)wr_addr, 0) != SPI_FLASH_OK) {
            spi_flash_cs_high();
            return SPI_FLASH_ERR_IO;
        }

        for (i = 0U; i < chunk; i++) {
            if (spi_flash_transfer_byte(buf[written + i], 0) != SPI_FLASH_OK) {
                spi_flash_cs_high();
                return SPI_FLASH_ERR_IO;
            }
        }
        spi_flash_cs_high();

        if (spi_flash_wait_not_busy() != SPI_FLASH_OK) {
            return SPI_FLASH_ERR_TIMEOUT;
        }
        written += chunk;
    }

    verify_offset = 0U;
    while (verify_offset < len) {
        uint32_t chunk = ((len - verify_offset) > sizeof(verify_buf)) ? sizeof(verify_buf) : (len - verify_offset);

        if (spi_flash_read(addr + verify_offset, verify_buf, chunk) != SPI_FLASH_OK) {
            return SPI_FLASH_ERR_IO;
        }
        if (memcmp(verify_buf, buf + verify_offset, chunk) != 0) {
            return SPI_FLASH_ERR_VERIFY;
        }
        verify_offset += chunk;
    }

    return SPI_FLASH_OK;
}
