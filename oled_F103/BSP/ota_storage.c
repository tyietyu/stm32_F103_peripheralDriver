#include "ota_storage.h"
#include "crc32.h"
#include "spi_flash.h"
#include <string.h>

static int ota_storage_read_metadata_at(uint32_t addr, ota_metadata_t *metadata)
{
    if (metadata == 0) {
        return OTA_ERR_METADATA;
    }

    if (spi_flash_read(addr, (uint8_t *)metadata, sizeof(*metadata)) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_Metadata_Is_Valid(metadata);
}

static int ota_storage_write_metadata_at(uint32_t addr, const ota_metadata_t *metadata)
{
    ota_metadata_t check;

    if (spi_flash_erase_sector(addr) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }
    if (spi_flash_write(addr, (const uint8_t *)metadata, sizeof(*metadata)) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }
    if (spi_flash_read(addr, (uint8_t *)&check, sizeof(check)) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }
    if ((memcmp(&check, metadata, sizeof(check)) != 0) || (OTA_Metadata_Is_Valid(&check) != OTA_OK)) {
        return OTA_ERR_METADATA;
    }

    return OTA_OK;
}

static int ota_storage_load_latest_metadata(ota_metadata_t *metadata, uint32_t *addr)
{
    ota_metadata_t meta_a;
    ota_metadata_t meta_b;
    int valid_a;
    int valid_b;

    if (metadata == 0) {
        return OTA_ERR_METADATA;
    }

    valid_a = ota_storage_read_metadata_at(OTA_EXT_METADATA_A_ADDR, &meta_a);
    valid_b = ota_storage_read_metadata_at(OTA_EXT_METADATA_B_ADDR, &meta_b);

    if ((valid_a == OTA_OK) && (valid_b == OTA_OK)) {
        if (meta_b.sequence > meta_a.sequence) {
            memcpy(metadata, &meta_b, sizeof(*metadata));
            if (addr != 0) {
                *addr = OTA_EXT_METADATA_B_ADDR;
            }
        } else {
            memcpy(metadata, &meta_a, sizeof(*metadata));
            if (addr != 0) {
                *addr = OTA_EXT_METADATA_A_ADDR;
            }
        }
        return OTA_OK;
    }
    if (valid_a == OTA_OK) {
        memcpy(metadata, &meta_a, sizeof(*metadata));
        if (addr != 0) {
            *addr = OTA_EXT_METADATA_A_ADDR;
        }
        return OTA_OK;
    }
    if (valid_b == OTA_OK) {
        memcpy(metadata, &meta_b, sizeof(*metadata));
        if (addr != 0) {
            *addr = OTA_EXT_METADATA_B_ADDR;
        }
        return OTA_OK;
    }

    memset(metadata, 0, sizeof(*metadata));
    return OTA_ERR_METADATA;
}

void OTA_Metadata_Fill_Crc(ota_metadata_t *metadata)
{
    if (metadata == 0) {
        return;
    }

    metadata->magic = OTA_METADATA_MAGIC;
    metadata->struct_version = OTA_METADATA_VERSION;
    metadata->struct_size = (uint16_t)sizeof(ota_metadata_t);
    metadata->metadata_crc32 = 0UL;
    metadata->metadata_crc32 = CRC32_Calc((const uint8_t *)metadata, sizeof(ota_metadata_t));
}

int OTA_Metadata_Is_Valid(const ota_metadata_t *metadata)
{
    ota_metadata_t temp;
    uint32_t crc;

    if (metadata == 0) {
        return OTA_ERR_METADATA;
    }
    if ((metadata->magic != OTA_METADATA_MAGIC) ||
        (metadata->struct_version != OTA_METADATA_VERSION) ||
        (metadata->struct_size != sizeof(ota_metadata_t))) {
        return OTA_ERR_METADATA;
    }

    memcpy(&temp, metadata, sizeof(temp));
    crc = temp.metadata_crc32;
    temp.metadata_crc32 = 0UL;
    if (CRC32_Calc((const uint8_t *)&temp, sizeof(temp)) != crc) {
        return OTA_ERR_METADATA;
    }

    return OTA_OK;
}

int OTA_Storage_Init(void)
{
    if (spi_flash_init() != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_OK;
}

int OTA_Storage_Load_Metadata(ota_metadata_t *metadata)
{
    if (metadata == 0) {
        return OTA_ERR_METADATA;
    }

    return ota_storage_load_latest_metadata(metadata, 0);
}

int OTA_Storage_Save_Metadata(const ota_metadata_t *metadata)
{
    ota_metadata_t current;
    ota_metadata_t next;
    uint32_t current_addr = OTA_EXT_METADATA_B_ADDR;
    uint32_t next_addr;
    int has_current;

    if (metadata == 0) {
        return OTA_ERR_METADATA;
    }

    has_current = ota_storage_load_latest_metadata(&current, &current_addr);
    memcpy(&next, metadata, sizeof(next));
    if (has_current == OTA_OK) {
        if (next.sequence <= current.sequence) {
            next.sequence = current.sequence + 1UL;
        }
    } else if (next.sequence == 0UL) {
        next.sequence = 1UL;
    }

    OTA_Metadata_Fill_Crc(&next);
    next_addr = (current_addr == OTA_EXT_METADATA_A_ADDR) ? OTA_EXT_METADATA_B_ADDR : OTA_EXT_METADATA_A_ADDR;

    return ota_storage_write_metadata_at(next_addr, &next);
}

int OTA_Storage_Clear_Slot(uint32_t package_size)
{
    uint32_t erase_len;

    if ((package_size == 0U) || (package_size > OTA_PACKAGE_MAX_SIZE)) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }

    erase_len = ((package_size + OTA_EXT_FLASH_SECTOR_SIZE - 1UL) / OTA_EXT_FLASH_SECTOR_SIZE) *
                OTA_EXT_FLASH_SECTOR_SIZE;
    if (erase_len > OTA_EXT_FIRMWARE_SLOT_SIZE) {
        erase_len = OTA_EXT_FIRMWARE_SLOT_SIZE;
    }

    if (spi_flash_erase_range(OTA_EXT_FIRMWARE_SLOT_ADDR, erase_len) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_Storage_Clear_Bitmap();
}

int OTA_Storage_Write_Block(uint32_t offset, const uint8_t *data, uint32_t len)
{
    if ((data == 0) || (len == 0U)) {
        return OTA_ERR_DOWNLOAD_FAIL;
    }
    if ((offset >= OTA_EXT_FIRMWARE_SLOT_SIZE) ||
        (len > OTA_EXT_FIRMWARE_SLOT_SIZE) ||
        ((offset + len) > OTA_EXT_FIRMWARE_SLOT_SIZE) ||
        ((offset + len) < offset)) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }

    if (spi_flash_write(OTA_EXT_FIRMWARE_SLOT_ADDR + offset, data, len) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_OK;
}

int OTA_Storage_Read_Image(uint32_t offset, uint8_t *data, uint32_t len)
{
    if ((data == 0) && (len != 0U)) {
        return OTA_ERR_DOWNLOAD_FAIL;
    }
    if ((offset >= OTA_EXT_FIRMWARE_SLOT_SIZE) ||
        (len > OTA_EXT_FIRMWARE_SLOT_SIZE) ||
        ((offset + len) > OTA_EXT_FIRMWARE_SLOT_SIZE) ||
        ((offset + len) < offset)) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }

    if (spi_flash_read(OTA_EXT_FIRMWARE_SLOT_ADDR + offset, data, len) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_OK;
}

int OTA_Storage_Clear_Bitmap(void)
{
    if (spi_flash_erase_sector(OTA_EXT_BITMAP_ADDR) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_OK;
}

int OTA_Storage_Set_Block_Done(uint32_t block_index)
{
    uint32_t byte_addr = OTA_EXT_BITMAP_ADDR + (block_index / 8UL);
    uint8_t bit = (uint8_t)(1U << (block_index % 8UL));
    uint8_t value;

    if ((block_index / 8UL) >= OTA_EXT_BITMAP_SIZE) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }
    if (spi_flash_read(byte_addr, &value, 1U) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    value = (uint8_t)(value & (uint8_t)~bit);
    if (spi_flash_write(byte_addr, &value, 1U) != SPI_FLASH_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    return OTA_OK;
}

int OTA_Storage_Is_Block_Done(uint32_t block_index)
{
    uint32_t byte_addr = OTA_EXT_BITMAP_ADDR + (block_index / 8UL);
    uint8_t bit = (uint8_t)(1U << (block_index % 8UL));
    uint8_t value;

    if ((block_index / 8UL) >= OTA_EXT_BITMAP_SIZE) {
        return 0;
    }
    if (spi_flash_read(byte_addr, &value, 1U) != SPI_FLASH_OK) {
        return 0;
    }

    return ((value & bit) == 0U) ? 1 : 0;
}

uint32_t OTA_Storage_Calc_Total_Blocks(uint32_t package_size)
{
    if (package_size == 0U) {
        return 0U;
    }

    return (package_size + OTA_BLOCK_SIZE - 1UL) / OTA_BLOCK_SIZE;
}
