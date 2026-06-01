#ifndef __OTADRIVER_H
#define __OTADRIVER_H

#include "main.h"
#include "ota_types.h"
#include <stdint.h>

#ifndef OTA_RX_TIMEOUT
#define OTA_RX_TIMEOUT          5000U
#endif

#ifndef OTA_MAX_RETRY
#define OTA_MAX_RETRY           5U
#endif

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

    uint32_t total_size;       /* 云端下发的完整 OTA 包长度：ota_image_header_t + APP bin */
    uint32_t stream_id;
    uint32_t current_offset;
    uint32_t total_blocks;
    char     target_version[33];
    char     expected_sign[65];
    char     sign_method[16];

    uint32_t last_req_tick;
    uint8_t  retry_count;
    uint8_t  file_id;
} OTA_Context_t;

void OTA_Init(void);
void OTA_Loop(void);
void OTA_Process_MQTT_Msg(const char *topic, uint8_t *payload, uint16_t len);
void OTA_Request_Firmware(void);
int OTA_Confirm_Image(void);

#endif /* __OTADRIVER_H */
