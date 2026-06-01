#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "ota_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SPI_FLASH_OK = 0,
    SPI_FLASH_ERR_PARAM = -1,
    SPI_FLASH_ERR_TIMEOUT = -2,
    SPI_FLASH_ERR_IO = -3,
    SPI_FLASH_ERR_ID = -4,
    SPI_FLASH_ERR_RANGE = -5,
    SPI_FLASH_ERR_VERIFY = -6
} spi_flash_status_t;

int spi_flash_init(void);
int spi_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int spi_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);
int spi_flash_erase_sector(uint32_t addr);
int spi_flash_erase_range(uint32_t addr, uint32_t len);
uint32_t spi_flash_read_id(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */
