#ifndef __BOOT_FLASH_H
#define __BOOT_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int BOOT_Flash_Erase_App(void);
int BOOT_Flash_Write(uint32_t addr, const uint8_t *data, uint32_t len);
int BOOT_Flash_Verify(uint32_t addr, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_FLASH_H */
