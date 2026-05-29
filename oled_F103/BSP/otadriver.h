#ifndef __OTADRIVER_H
#define __OTADRIVER_H

#include "main.h"
#include <stdint.h>

/*
 * STM32F103C8T6 has only 64KB Flash. The default build keeps OTA metadata
 * parsing and cloud reporting enabled, but disables internal Flash staging so
 * OTA traffic can never overwrite the OLED application or EEPROM pages.
 *
 * To enable a real download path later, provide a valid staging area, flag
 * address, and bootloader design before setting OTA_ENABLE_INTERNAL_FLASH_STAGING.
 */
#ifndef OTA_ENABLE_INTERNAL_FLASH_STAGING
#define OTA_ENABLE_INTERNAL_FLASH_STAGING 0U
#endif

#define OTA_FLASH_BASE_ADDR     0x08000000UL
#define OTA_FLASH_SIZE_BYTES    (64UL * 1024UL)
#define OTA_EE_RESERVED_SIZE    (2UL * 1024UL)
#define OTA_APP_MAX_END_ADDR    (OTA_FLASH_BASE_ADDR + OTA_FLASH_SIZE_BYTES - OTA_EE_RESERVED_SIZE)

#ifndef OTA_STORAGE_START_ADDR
#define OTA_STORAGE_START_ADDR  0UL
#endif

#ifndef OTA_STAGING_SIZE_BYTES
#define OTA_STAGING_SIZE_BYTES  0UL
#endif

#define OTA_STORAGE_END_ADDR    (OTA_STORAGE_START_ADDR + OTA_STAGING_SIZE_BYTES)

#ifndef OTA_FLAG_ADDR
#define OTA_FLAG_ADDR           0UL
#endif

#define OTA_BLOCK_SIZE          512U
#define OTA_RX_TIMEOUT          5000U
#define OTA_MAX_RETRY           5U

#define OTA_ERR_UPGRADE_FAIL    -1
#define OTA_ERR_DOWNLOAD_FAIL   -2
#define OTA_ERR_VERIFY_FAIL     -3
#define OTA_ERR_BURN_FAIL       -4
#define OTA_ERR_NO_STORAGE      -5
#define OTA_ERR_INVALID_MSG     -6
#define OTA_ERR_SIZE_TOO_LARGE  -7

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_INIT,
    OTA_STATE_NEGOTIATING,
    OTA_STATE_DOWNLOADING,
    OTA_STATE_FINISHING,
    OTA_STATE_ERROR,
    OTA_STATE_REBOOTING
} OTA_State_t;

typedef struct {
    OTA_State_t state;

    uint32_t total_size;
    uint32_t stream_id;
    uint32_t current_offset;
    char     target_version[33];
    char     expected_sign[65];
    char     sign_method[16];

    uint32_t last_req_tick;
    uint8_t  retry_count;
    uint8_t  file_id;
} OTA_Context_t;

typedef struct {
    uint32_t magic_flag;
    uint32_t firmware_len;
    uint8_t  version[32];
    uint32_t crc32;
    uint8_t  reserved[64];
} OTA_Flash_Info_t;

void OTA_Init(void);
void OTA_Loop(void);
void OTA_Process_MQTT_Msg(const char *topic, uint8_t *payload, uint16_t len);
void OTA_Request_Firmware(void);

#endif /* __OTADRIVER_H */
