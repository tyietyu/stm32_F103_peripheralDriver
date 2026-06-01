#include "boot_flash.h"
#include "main.h"
#include "ota_types.h"
#include <string.h>

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 0x400U
#endif

int BOOT_Flash_Erase_App(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0U;

    HAL_FLASH_Unlock();
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = OTA_APP_START_ADDR;
    erase.NbPages = OTA_APP_SIZE_BYTES / FLASH_PAGE_SIZE;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return OTA_ERR_BURN_FAIL;
    }
    HAL_FLASH_Lock();

    return OTA_OK;
}

int BOOT_Flash_Write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t offset;

    if ((data == 0) || (len == 0U)) {
        return OTA_ERR_BURN_FAIL;
    }
    if ((addr < OTA_APP_START_ADDR) ||
        ((addr + len) > OTA_APP_END_ADDR) ||
        ((addr + len) < addr)) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }

    HAL_FLASH_Unlock();
    for (offset = 0U; offset < len; offset += 2U) {
        uint16_t halfword = data[offset];

        if ((offset + 1U) < len) {
            halfword |= (uint16_t)((uint16_t)data[offset + 1U] << 8U);
        } else {
            halfword |= 0xFF00U;
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + offset, halfword) != HAL_OK) {
            HAL_FLASH_Lock();
            return OTA_ERR_BURN_FAIL;
        }
    }
    HAL_FLASH_Lock();

    return BOOT_Flash_Verify(addr, data, len);
}

int BOOT_Flash_Verify(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if ((data == 0) || (len == 0U)) {
        return OTA_ERR_BURN_FAIL;
    }
    if (memcmp((const void *)addr, data, len) != 0) {
        return OTA_ERR_BURN_FAIL;
    }

    return OTA_OK;
}
