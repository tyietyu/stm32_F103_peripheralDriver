#ifndef __OTA_TYPES_H
#define __OTA_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OTA_IMAGE_MAGIC              0x4F544131UL  /* "OTA1" */
#define OTA_METADATA_MAGIC           0x4F544D31UL  /* "OTM1" */
#define OTA_IMAGE_HEADER_VERSION     1U
#define OTA_METADATA_VERSION         1U

#define OTA_FLASH_BASE_ADDR          0x08000000UL
#define OTA_FLASH_SIZE_BYTES         (64UL * 1024UL)
#define OTA_BOOTLOADER_START_ADDR    0x08000000UL
#define OTA_BOOTLOADER_SIZE_BYTES    (16UL * 1024UL)
#define OTA_APP_START_ADDR           0x08004000UL
#define OTA_APP_SIZE_BYTES           (44UL * 1024UL)
#define OTA_APP_END_ADDR             (OTA_APP_START_ADDR + OTA_APP_SIZE_BYTES)
#define OTA_PARAM_START_ADDR         0x0800F000UL
#define OTA_PARAM_SIZE_BYTES         (4UL * 1024UL)
#define OTA_SRAM_START_ADDR          0x20000000UL
#define OTA_SRAM_END_ADDR            0x20005000UL

#define OTA_EXT_FLASH_SIZE_BYTES     (8UL * 1024UL * 1024UL)
#define OTA_EXT_FLASH_SECTOR_SIZE    4096UL
#define OTA_EXT_FLASH_PAGE_SIZE      256UL
#define OTA_EXT_METADATA_A_ADDR      0x000000UL
#define OTA_EXT_METADATA_B_ADDR      0x001000UL
#define OTA_EXT_FIRMWARE_SLOT_ADDR   0x002000UL
#define OTA_PACKAGE_MAX_SIZE         (sizeof(ota_image_header_t) + OTA_APP_SIZE_BYTES)
#define OTA_EXT_FIRMWARE_SLOT_SIZE   (((OTA_PACKAGE_MAX_SIZE + OTA_EXT_FLASH_SECTOR_SIZE - 1UL) / OTA_EXT_FLASH_SECTOR_SIZE) * OTA_EXT_FLASH_SECTOR_SIZE)
#define OTA_EXT_BITMAP_ADDR          (OTA_EXT_FIRMWARE_SLOT_ADDR + OTA_EXT_FIRMWARE_SLOT_SIZE)
#define OTA_EXT_BITMAP_SIZE          OTA_EXT_FLASH_SECTOR_SIZE
#define OTA_BLOCK_SIZE               512UL

typedef enum {
    OTA_SLOT_EMPTY = 0,
    OTA_SLOT_DOWNLOADING = 1,
    OTA_SLOT_READY = 2,
    OTA_SLOT_UPDATING = 3,
    OTA_SLOT_PENDING_CONFIRM = 4,
    OTA_SLOT_CONFIRMED = 5,
    OTA_SLOT_FAILED = 6
} ota_slot_state_t;

typedef enum {
    OTA_OK = 0,
    OTA_ERR_UPGRADE_FAIL = -1,
    OTA_ERR_DOWNLOAD_FAIL = -2,
    OTA_ERR_VERIFY_FAIL = -3,
    OTA_ERR_BURN_FAIL = -4,
    OTA_ERR_NO_STORAGE = -5,
    OTA_ERR_INVALID_MSG = -6,
    OTA_ERR_SIZE_TOO_LARGE = -7,
    OTA_ERR_HW_MISMATCH = -8,
    OTA_ERR_VERSION_ROLLBACK = -9,
    OTA_ERR_BOOT_PROGRAM_FAIL = -10,
    OTA_ERR_METADATA = -11,
    OTA_ERR_NOT_CONFIRMED = -12
} ota_error_t;

typedef struct {
    uint32_t magic;
    uint16_t header_version;
    uint16_t header_size;
    uint32_t image_size;
    uint32_t image_crc32;
    uint8_t  image_sha256[32];
    uint32_t app_start_addr;
    uint32_t hw_version;
    uint32_t sw_version;
    uint32_t flags;
    uint32_t header_crc32;
} ota_image_header_t;

typedef struct {
    uint32_t magic;
    uint16_t struct_version;
    uint16_t struct_size;
    uint32_t sequence;
    uint32_t state;
    uint32_t image_size;
    uint32_t image_crc32;
    uint8_t  image_sha256[32];
    uint32_t app_start_addr;
    uint32_t sw_version;
    uint32_t downloaded_size;
    uint32_t block_size;
    uint32_t total_blocks;
    uint32_t failed_reason;
    uint32_t metadata_crc32;
} ota_metadata_t;

typedef struct {
    ota_image_header_t header;
    uint32_t package_size;
} ota_verified_image_t;

#ifdef __cplusplus
}
#endif

#endif /* __OTA_TYPES_H */
