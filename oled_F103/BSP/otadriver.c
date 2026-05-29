#include "otadriver.h"
#include "esp8266.h"
#include "core_json.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if OTA_ENABLE_INTERNAL_FLASH_STAGING
#include "flash.h"
#include "md5.h"
#endif

OTA_Context_t g_ota_ctx;
extern esp8266_config_t esp8266_config;

static void OTA_Report_Progress(int percent, const char *desc);
static void OTA_Report_Version(const char *version);
static int OTA_Parse_Upgrade_Notify(char *json, uint16_t len);
static void OTA_Handle_Error(int code, const char *reason);
static uint8_t OTA_Has_Valid_Storage(uint32_t firmware_size);
static void OTA_Copy_Json_Value(char *dst, size_t dst_size, const char *src, size_t src_len);
static int OTA_Case_Equal(const char *a, const char *b);

#if OTA_ENABLE_INTERNAL_FLASH_STAGING
static void OTA_Send_Block_Request(void);
static int OTA_Process_Download_Reply(uint8_t *data, uint16_t len);
static void OTA_Finish_Download(void);
static void bin2hex(const unsigned char *bin, char *out);
static uint8_t aligned_buffer[OTA_BLOCK_SIZE + 16U];
#endif

void OTA_Init(void)
{
    char cmd[192];

    memset(&g_ota_ctx, 0, sizeof(g_ota_ctx));
    g_ota_ctx.state = OTA_STATE_INIT;

    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", esp8266_config.ota.download_info_sub);
    (void)ESP8266_send_at_cmd((uint8_t *)cmd, (unsigned char)strlen(cmd), "OK");

#if OTA_ENABLE_INTERNAL_FLASH_STAGING
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", esp8266_config.ota.device_download_file_reply);
    (void)ESP8266_send_at_cmd((uint8_t *)cmd, (unsigned char)strlen(cmd), "OK");
#endif

    OTA_Report_Version("1.0.0");
    g_ota_ctx.state = OTA_STATE_IDLE;
    printf("[OTA] Init success. internal staging=%u\r\n", OTA_ENABLE_INTERNAL_FLASH_STAGING);

    OTA_Request_Firmware();
}

void OTA_Loop(void)
{
#if OTA_ENABLE_INTERNAL_FLASH_STAGING
    if (g_ota_ctx.state == OTA_STATE_DOWNLOADING) {
        if (HAL_GetTick() - g_ota_ctx.last_req_tick > OTA_RX_TIMEOUT) {
            if (g_ota_ctx.retry_count < OTA_MAX_RETRY) {
                printf("[OTA] Timeout offset %lu. Retry %u/%u\r\n",
                       (unsigned long)g_ota_ctx.current_offset,
                       (unsigned int)(g_ota_ctx.retry_count + 1U),
                       (unsigned int)OTA_MAX_RETRY);
                g_ota_ctx.retry_count++;
                OTA_Send_Block_Request();
            } else {
                OTA_Handle_Error(OTA_ERR_DOWNLOAD_FAIL, "Timeout");
            }
        }
    }
#endif
}

void OTA_Process_MQTT_Msg(const char *topic, uint8_t *payload, uint16_t len)
{
    if ((topic == NULL) || (payload == NULL) || (len == 0U)) {
        OTA_Handle_Error(OTA_ERR_INVALID_MSG, "Invalid MQTT message");
        return;
    }

    if (strstr(topic, "/ota/device/upgrade") != NULL) {
        printf("[OTA] Upgrade notification received\r\n");

        if (OTA_Parse_Upgrade_Notify((char *)payload, len) != 0) {
            OTA_Handle_Error(OTA_ERR_INVALID_MSG, "Bad upgrade JSON");
            return;
        }

        printf("[OTA] Upgrade size=%lu version=%s sign=%s\r\n",
               (unsigned long)g_ota_ctx.total_size,
               g_ota_ctx.target_version,
               g_ota_ctx.expected_sign);

        if (!OTA_Has_Valid_Storage(g_ota_ctx.total_size)) {
            OTA_Handle_Error(OTA_ERR_NO_STORAGE, "No safe staging storage on STM32F103C8T6");
            return;
        }

#if OTA_ENABLE_INTERNAL_FLASH_STAGING
        g_ota_ctx.state = OTA_STATE_DOWNLOADING;
        g_ota_ctx.current_offset = 0U;
        g_ota_ctx.retry_count = 0U;
        g_ota_ctx.file_id = 1U;
        OTA_Send_Block_Request();
#else
        OTA_Handle_Error(OTA_ERR_NO_STORAGE, "Internal Flash staging disabled");
#endif
    }
#if OTA_ENABLE_INTERNAL_FLASH_STAGING
    else if (strstr(topic, "thing/file/download_reply") != NULL) {
        if (g_ota_ctx.state != OTA_STATE_DOWNLOADING) {
            return;
        }

        if (OTA_Process_Download_Reply(payload, len) == 0) {
            if (g_ota_ctx.current_offset >= g_ota_ctx.total_size) {
                OTA_Finish_Download();
            } else {
                g_ota_ctx.retry_count = 0U;
                OTA_Send_Block_Request();
            }
        }
    }
#endif
}

void OTA_Request_Firmware(void)
{
    char json_req[128];

    snprintf(json_req, sizeof(json_req),
             "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{\"module\":\"default\"}}",
             (unsigned long)HAL_GetTick());
    (void)ESP8266_send_msg(esp8266_config.ota.device_active_info_pub, "%s", json_req);
    printf("[OTA] Requested firmware metadata\r\n");
}

static int OTA_Parse_Upgrade_Notify(char *json, uint16_t len)
{
    JSONStatus_t result;
    char *val;
    size_t val_len;
    char temp[16];

    if ((json == NULL) || (len == 0U)) {
        return -1;
    }

    result = JSON_Validate(json, len);
    if (result != JSONSuccess) {
        return -1;
    }

    result = JSON_Search(json, len, "data.size", sizeof("data.size") - 1U, &val, &val_len);
    if ((result != JSONSuccess) || (val_len >= sizeof(temp))) {
        return -1;
    }
    OTA_Copy_Json_Value(temp, sizeof(temp), val, val_len);
    g_ota_ctx.total_size = (uint32_t)strtoul(temp, NULL, 10);
    if (g_ota_ctx.total_size == 0U) {
        return -1;
    }

    result = JSON_Search(json, len, "data.streamId", sizeof("data.streamId") - 1U, &val, &val_len);
    if ((result != JSONSuccess) || (val_len >= sizeof(temp))) {
        return -1;
    }
    OTA_Copy_Json_Value(temp, sizeof(temp), val, val_len);
    g_ota_ctx.stream_id = (uint32_t)strtoul(temp, NULL, 10);

    result = JSON_Search(json, len, "data.version", sizeof("data.version") - 1U, &val, &val_len);
    if (result == JSONSuccess) {
        OTA_Copy_Json_Value(g_ota_ctx.target_version, sizeof(g_ota_ctx.target_version), val, val_len);
    } else {
        g_ota_ctx.target_version[0] = '\0';
    }

    result = JSON_Search(json, len, "data.signMethod", sizeof("data.signMethod") - 1U, &val, &val_len);
    if (result == JSONSuccess) {
        OTA_Copy_Json_Value(g_ota_ctx.sign_method, sizeof(g_ota_ctx.sign_method), val, val_len);
        if (!OTA_Case_Equal(g_ota_ctx.sign_method, "MD5")) {
            printf("[OTA] Unsupported sign method: %s\r\n", g_ota_ctx.sign_method);
            return -1;
        }
    } else {
        strcpy(g_ota_ctx.sign_method, "MD5");
    }

    result = JSON_Search(json, len, "data.sign", sizeof("data.sign") - 1U, &val, &val_len);
    if (result != JSONSuccess) {
        result = JSON_Search(json, len, "data.md5", sizeof("data.md5") - 1U, &val, &val_len);
    }

    if ((result != JSONSuccess) || (val_len != 32U)) {
        printf("[OTA] Missing or invalid MD5 signature\r\n");
        return -1;
    }
    OTA_Copy_Json_Value(g_ota_ctx.expected_sign, sizeof(g_ota_ctx.expected_sign), val, val_len);

    return 0;
}

static uint8_t OTA_Has_Valid_Storage(uint32_t firmware_size)
{
#if OTA_ENABLE_INTERNAL_FLASH_STAGING
    if ((OTA_STORAGE_START_ADDR < OTA_FLASH_BASE_ADDR) ||
        (OTA_STORAGE_START_ADDR >= OTA_APP_MAX_END_ADDR) ||
        (OTA_STORAGE_END_ADDR > OTA_APP_MAX_END_ADDR) ||
        (OTA_FLAG_ADDR < OTA_FLASH_BASE_ADDR) ||
        (OTA_FLAG_ADDR >= OTA_APP_MAX_END_ADDR)) {
        return 0U;
    }

    return ((firmware_size > 0U) && (firmware_size <= OTA_STAGING_SIZE_BYTES)) ? 1U : 0U;
#else
    (void)firmware_size;
    return 0U;
#endif
}

static void OTA_Report_Progress(int percent, const char *desc)
{
    char json_msg[128];

    snprintf(json_msg, sizeof(json_msg),
             "{\"id\":\"%lu\",\"params\":{\"step\":\"%d\",\"desc\":\"%s\",\"module\":\"default\"}}",
             (unsigned long)HAL_GetTick(),
             percent,
             desc ? desc : "downloading");
    (void)ESP8266_send_msg(esp8266_config.ota.device_report_progress_pub, "%s", json_msg);
}

static void OTA_Report_Version(const char *version)
{
    char json_msg[96];

    snprintf(json_msg, sizeof(json_msg),
             "{\"id\":\"1\",\"params\":{\"version\":\"%s\"}}",
             version ? version : "unknown");
    (void)ESP8266_send_msg(esp8266_config.ota.upload_info_pub, "%s", json_msg);
}

static void OTA_Handle_Error(int code, const char *reason)
{
    printf("[OTA] Error %d: %s\r\n", code, reason ? reason : "unknown");
    OTA_Report_Progress(code, reason ? reason : "error");
    g_ota_ctx.state = OTA_STATE_ERROR;
}

static void OTA_Copy_Json_Value(char *dst, size_t dst_size, const char *src, size_t src_len)
{
    size_t copy_len;

    if ((dst == NULL) || (dst_size == 0U)) {
        return;
    }

    dst[0] = '\0';
    if (src == NULL) {
        return;
    }

    copy_len = src_len;
    if ((copy_len >= 2U) && (src[0] == '"') && (src[copy_len - 1U] == '"')) {
        src++;
        copy_len -= 2U;
    }

    if (copy_len >= dst_size) {
        copy_len = dst_size - 1U;
    }

    memcpy(dst, src, copy_len);
    dst[copy_len] = '\0';
}

static int OTA_Case_Equal(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0')) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return 0;
        }
        a++;
        b++;
    }

    return (*a == '\0') && (*b == '\0');
}

#if OTA_ENABLE_INTERNAL_FLASH_STAGING
static void OTA_Send_Block_Request(void)
{
    uint32_t req_size = OTA_BLOCK_SIZE;
    char json_buf[256];

    if (g_ota_ctx.current_offset >= g_ota_ctx.total_size) {
        return;
    }

    if ((g_ota_ctx.total_size - g_ota_ctx.current_offset) < OTA_BLOCK_SIZE) {
        req_size = g_ota_ctx.total_size - g_ota_ctx.current_offset;
    }

    snprintf(json_buf, sizeof(json_buf),
             "{\"id\":\"%lu\",\"params\":{\"fileInfo\":{\"streamId\":%lu,\"fileId\":%u},\"fileBlock\":{\"size\":%lu,\"offset\":%lu},\"module\":\"default\"}}",
             (unsigned long)((g_ota_ctx.current_offset / OTA_BLOCK_SIZE) + 1U),
             (unsigned long)g_ota_ctx.stream_id,
             (unsigned int)g_ota_ctx.file_id,
             (unsigned long)req_size,
             (unsigned long)g_ota_ctx.current_offset);
    (void)ESP8266_send_msg(esp8266_config.ota.device_download_file, "%s", json_buf);
    g_ota_ctx.last_req_tick = HAL_GetTick();
}

static int OTA_Process_Download_Reply(uint8_t *data, uint16_t len)
{
    uint16_t json_len;
    uint8_t *bin_start;
    uint16_t bin_len;
    uint32_t next_offset;

    if ((data == NULL) || (len < 2U)) {
        return -1;
    }

    json_len = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    if (len < (uint16_t)(2U + json_len)) {
        printf("[OTA] Download reply too short. json_len=%u\r\n", json_len);
        return -1;
    }

    bin_start = data + 2U + json_len;
    bin_len = (uint16_t)(len - (2U + json_len));
    next_offset = g_ota_ctx.current_offset + bin_len;

    if ((bin_len == 0U) || (next_offset > g_ota_ctx.total_size) ||
        (next_offset > OTA_STAGING_SIZE_BYTES)) {
        return -1;
    }

    memset(aligned_buffer, 0xFF, sizeof(aligned_buffer));
    memcpy(aligned_buffer, bin_start, bin_len);
    iap_write_flash(OTA_STORAGE_START_ADDR + g_ota_ctx.current_offset,
                    (uint16_t *)aligned_buffer,
                    (uint16_t)((bin_len + 1U) / 2U));

    g_ota_ctx.current_offset = next_offset;
    OTA_Report_Progress((int)((g_ota_ctx.current_offset * 100U) / g_ota_ctx.total_size), NULL);
    return 0;
}

static void OTA_Finish_Download(void)
{
    MD5_CTX md5_ctx;
    uint8_t buf[256];
    uint32_t offset = 0U;
    uint32_t remain = g_ota_ctx.total_size;
    unsigned char digest[16];
    char calc_sign[33];
    OTA_Flash_Info_t info;
    uint16_t len_in_halfwords;

    g_ota_ctx.state = OTA_STATE_FINISHING;
    MD5Init(&md5_ctx);

    while (remain > 0U) {
        uint32_t chunk_size = (remain > sizeof(buf)) ? sizeof(buf) : remain;
        iap_read_flash(OTA_STORAGE_START_ADDR + offset, (uint16_t *)buf, (uint16_t)((chunk_size + 1U) / 2U));
        MD5Update(&md5_ctx, buf, chunk_size);
        offset += chunk_size;
        remain -= chunk_size;
    }

    MD5Final(&md5_ctx, digest);
    bin2hex(digest, calc_sign);

    if (strncmp(calc_sign, g_ota_ctx.expected_sign, 32U) != 0) {
        OTA_Handle_Error(OTA_ERR_VERIFY_FAIL, "MD5 mismatch");
        return;
    }

    memset(&info, 0, sizeof(info));
    info.magic_flag = 0xAA55A55AUL;
    info.firmware_len = g_ota_ctx.total_size;
    strncpy((char *)info.version, g_ota_ctx.target_version, sizeof(info.version));

    len_in_halfwords = (uint16_t)((sizeof(OTA_Flash_Info_t) + 1U) / 2U);
    iap_write_flash(OTA_FLAG_ADDR, (uint16_t *)&info, len_in_halfwords);

    OTA_Report_Progress(100, "success");
    HAL_Delay(100U);
    HAL_NVIC_SystemReset();
}

static void bin2hex(const unsigned char *bin, char *out)
{
    static const char hex[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 16; i++) {
        out[i * 2] = hex[(bin[i] >> 4) & 0x0F];
        out[(i * 2) + 1] = hex[bin[i] & 0x0F];
    }
    out[32] = '\0';
}
#endif
