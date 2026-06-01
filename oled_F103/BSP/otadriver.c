#include "otadriver.h"
#include "core_json.h"
#include "esp8266.h"
#include "md5.h"
#include "ota_storage.h"
#include "ota_verify.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OTA_APP_VERSION_TEXT "1.0.0"

OTA_Context_t g_ota_ctx;
extern esp8266_config_t esp8266_config;

static uint8_t s_download_buf[OTA_BLOCK_SIZE + 16U];

static void OTA_Report_Progress(int percent, const char *desc);
static void OTA_Report_Version(const char *version);
static int OTA_Parse_Upgrade_Notify(char *json, uint16_t len);
static void OTA_Handle_Error(int code, const char *reason);
static uint8_t OTA_Has_Valid_Storage(uint32_t package_size);
static void OTA_Copy_Json_Value(char *dst, size_t dst_size, const char *src, size_t src_len);
static int OTA_Case_Equal(const char *a, const char *b);
static void OTA_Send_Block_Request(void);
static int OTA_Process_Download_Reply(uint8_t *data, uint16_t len);
static void OTA_Finish_Download(void);
static int OTA_Prepare_Download(void);
static uint32_t OTA_Find_Next_Missing_Block(void);
static int OTA_Check_Package_MD5(uint32_t package_size, const char *expected_sign);
static void OTA_Bin2Hex(const unsigned char *bin, char *out);

void OTA_Init(void)
{
    char cmd[192];
    ota_metadata_t metadata;

    memset(&g_ota_ctx, 0, sizeof(g_ota_ctx));
    g_ota_ctx.state = OTA_STATE_INIT;

    (void)OTA_Storage_Init();

    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", esp8266_config.ota.download_info_sub);
    (void)ESP8266_send_at_cmd((uint8_t *)cmd, (unsigned char)strlen(cmd), "OK");

    /* 固件块响应必须订阅 thing/file/download_reply，原工程字段命名和取值相反，这里按 topic 实际语义使用。 */
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", esp8266_config.ota.device_download_file);
    (void)ESP8266_send_at_cmd((uint8_t *)cmd, (unsigned char)strlen(cmd), "OK");

    OTA_Report_Version(OTA_APP_VERSION_TEXT);

    if (OTA_Storage_Load_Metadata(&metadata) == OTA_OK) {
        if (metadata.state == OTA_SLOT_PENDING_CONFIRM) {
            (void)OTA_Confirm_Image();
        } else if (metadata.state == OTA_SLOT_FAILED) {
            OTA_Report_Progress((int)metadata.failed_reason, "bootloader failed");
        }
    }

    g_ota_ctx.state = OTA_STATE_IDLE;
    printf("[OTA] Init success. external W25Q64 staging enabled\r\n");

    OTA_Request_Firmware();
}

void OTA_Loop(void)
{
    if (g_ota_ctx.state == OTA_STATE_DOWNLOADING) {
        if ((HAL_GetTick() - g_ota_ctx.last_req_tick) > OTA_RX_TIMEOUT) {
            if (g_ota_ctx.retry_count < OTA_MAX_RETRY) {
                printf("[OTA] Timeout offset %lu. Retry %u/%u\r\n",
                       (unsigned long)g_ota_ctx.current_offset,
                       (unsigned int)(g_ota_ctx.retry_count + 1U),
                       (unsigned int)OTA_MAX_RETRY);
                g_ota_ctx.retry_count++;
                OTA_Send_Block_Request();
            } else {
                OTA_Handle_Error(OTA_ERR_DOWNLOAD_FAIL, "download timeout");
            }
        }
    }
}

void OTA_Process_MQTT_Msg(const char *topic, uint8_t *payload, uint16_t len)
{
    if ((topic == NULL) || (payload == NULL) || (len == 0U)) {
        OTA_Handle_Error(OTA_ERR_INVALID_MSG, "invalid MQTT message");
        return;
    }

    if (strstr(topic, "/ota/device/upgrade") != NULL) {
        printf("[OTA] Upgrade notification received\r\n");

        if (OTA_Parse_Upgrade_Notify((char *)payload, len) != 0) {
            OTA_Handle_Error(OTA_ERR_INVALID_MSG, "bad upgrade JSON");
            return;
        }

        printf("[OTA] Upgrade package size=%lu version=%s sign=%s\r\n",
               (unsigned long)g_ota_ctx.total_size,
               g_ota_ctx.target_version,
               g_ota_ctx.expected_sign);

        if (!OTA_Has_Valid_Storage(g_ota_ctx.total_size)) {
            OTA_Handle_Error(OTA_ERR_NO_STORAGE, "external flash unavailable or package too large");
            return;
        }

        if (OTA_Prepare_Download() != OTA_OK) {
            OTA_Handle_Error(OTA_ERR_NO_STORAGE, "prepare staging failed");
            return;
        }

        g_ota_ctx.state = OTA_STATE_DOWNLOADING;
        g_ota_ctx.retry_count = 0U;
        g_ota_ctx.file_id = 1U;
        OTA_Send_Block_Request();
    } else if (strstr(topic, "thing/file/download_reply") != NULL) {
        if (g_ota_ctx.state != OTA_STATE_DOWNLOADING) {
            return;
        }

        if (OTA_Process_Download_Reply(payload, len) == 0) {
            if (OTA_Find_Next_Missing_Block() >= g_ota_ctx.total_blocks) {
                OTA_Finish_Download();
            } else {
                g_ota_ctx.retry_count = 0U;
                OTA_Send_Block_Request();
            }
        }
    }
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

int OTA_Confirm_Image(void)
{
    ota_metadata_t metadata;

    if (OTA_Storage_Load_Metadata(&metadata) != OTA_OK) {
        return OTA_ERR_METADATA;
    }
    if (metadata.state != OTA_SLOT_PENDING_CONFIRM) {
        return OTA_OK;
    }

    metadata.state = OTA_SLOT_CONFIRMED;
    metadata.failed_reason = OTA_OK;
    if (OTA_Storage_Save_Metadata(&metadata) != OTA_OK) {
        return OTA_ERR_METADATA;
    }

    OTA_Report_Progress(100, "confirmed");
    printf("[OTA] New image confirmed\r\n");
    return OTA_OK;
}

static int OTA_Parse_Upgrade_Notify(char *json, uint16_t len)
{
    JSONStatus_t result;
    char *val;
    size_t val_len;
    char temp[24];

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
    if (g_ota_ctx.total_size < sizeof(ota_image_header_t)) {
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
    } else {
        g_ota_ctx.sign_method[0] = '\0';
    }

    result = JSON_Search(json, len, "data.sign", sizeof("data.sign") - 1U, &val, &val_len);
    if (result != JSONSuccess) {
        result = JSON_Search(json, len, "data.md5", sizeof("data.md5") - 1U, &val, &val_len);
    }

    if (result == JSONSuccess) {
        if (val_len >= sizeof(g_ota_ctx.expected_sign)) {
            return -1;
        }
        OTA_Copy_Json_Value(g_ota_ctx.expected_sign, sizeof(g_ota_ctx.expected_sign), val, val_len);
        if ((g_ota_ctx.sign_method[0] != '\0') &&
            !OTA_Case_Equal(g_ota_ctx.sign_method, "MD5")) {
            printf("[OTA] Unsupported sign method: %s\r\n", g_ota_ctx.sign_method);
            return -1;
        }
    } else {
        g_ota_ctx.expected_sign[0] = '\0';
    }

    return 0;
}

static uint8_t OTA_Has_Valid_Storage(uint32_t package_size)
{
    if ((package_size < sizeof(ota_image_header_t)) || (package_size > OTA_PACKAGE_MAX_SIZE)) {
        return 0U;
    }

    return (OTA_Storage_Init() == OTA_OK) ? 1U : 0U;
}

static int OTA_Prepare_Download(void)
{
    ota_metadata_t metadata;
    ota_metadata_t old_metadata;
    uint8_t resume = 0U;

    if (OTA_Storage_Load_Metadata(&old_metadata) == OTA_OK) {
        if ((old_metadata.state == OTA_SLOT_DOWNLOADING) &&
            (old_metadata.image_size == g_ota_ctx.total_size) &&
            (old_metadata.block_size == OTA_BLOCK_SIZE)) {
            resume = 1U;
        }
    }

    if (resume == 0U) {
        int ret = OTA_Storage_Clear_Slot(g_ota_ctx.total_size);
        if (ret != OTA_OK) {
            return ret;
        }
    }

    memset(&metadata, 0, sizeof(metadata));
    metadata.state = OTA_SLOT_DOWNLOADING;
    metadata.image_size = g_ota_ctx.total_size;
    metadata.downloaded_size = (resume != 0U) ? old_metadata.downloaded_size : 0U;
    metadata.block_size = OTA_BLOCK_SIZE;
    metadata.total_blocks = OTA_Storage_Calc_Total_Blocks(g_ota_ctx.total_size);
    metadata.app_start_addr = OTA_APP_START_ADDR;

    g_ota_ctx.total_blocks = metadata.total_blocks;
    g_ota_ctx.current_offset = OTA_Find_Next_Missing_Block() * OTA_BLOCK_SIZE;

    return OTA_Storage_Save_Metadata(&metadata);
}

static uint32_t OTA_Find_Next_Missing_Block(void)
{
    uint32_t i;

    for (i = 0U; i < g_ota_ctx.total_blocks; i++) {
        if (OTA_Storage_Is_Block_Done(i) == 0) {
            return i;
        }
    }

    return g_ota_ctx.total_blocks;
}

static void OTA_Send_Block_Request(void)
{
    uint32_t block_index;
    uint32_t req_size;
    char json_buf[256];

    block_index = OTA_Find_Next_Missing_Block();
    if (block_index >= g_ota_ctx.total_blocks) {
        return;
    }

    g_ota_ctx.current_offset = block_index * OTA_BLOCK_SIZE;
    req_size = OTA_BLOCK_SIZE;
    if ((g_ota_ctx.total_size - g_ota_ctx.current_offset) < OTA_BLOCK_SIZE) {
        req_size = g_ota_ctx.total_size - g_ota_ctx.current_offset;
    }

    snprintf(json_buf, sizeof(json_buf),
             "{\"id\":\"%lu\",\"params\":{\"fileInfo\":{\"streamId\":%lu,\"fileId\":%u},\"fileBlock\":{\"size\":%lu,\"offset\":%lu},\"module\":\"default\"}}",
             (unsigned long)(block_index + 1U),
             (unsigned long)g_ota_ctx.stream_id,
             (unsigned int)g_ota_ctx.file_id,
             (unsigned long)req_size,
             (unsigned long)g_ota_ctx.current_offset);

    /* 请求必须发布到 thing/file/download；esp8266_config 字段名沿用旧工程，实际取值在 device_download_file_reply。 */
    (void)ESP8266_send_msg(esp8266_config.ota.device_download_file_reply, "%s", json_buf);
    g_ota_ctx.last_req_tick = HAL_GetTick();
}

static int OTA_Process_Download_Reply(uint8_t *data, uint16_t len)
{
    ota_metadata_t metadata;
    uint16_t json_len;
    uint8_t *bin_start;
    uint16_t bin_len;
    uint32_t next_offset;
    uint32_t block_index;

    if ((data == NULL) || (len < 2U)) {
        return -1;
    }

    json_len = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    if (len < (uint16_t)(2U + json_len)) {
        printf("[OTA] Download reply too short. json_len=%u\r\n", json_len);
        return -1;
    }

    bin_start = data + 2U + json_len;
    bin_len = (uint16_t)(len - (2U + json_len));
    next_offset = g_ota_ctx.current_offset + bin_len;

    if ((bin_len == 0U) ||
        (bin_len > OTA_BLOCK_SIZE) ||
        (next_offset > g_ota_ctx.total_size) ||
        (next_offset < g_ota_ctx.current_offset)) {
        return -1;
    }

    memset(s_download_buf, 0xFF, sizeof(s_download_buf));
    memcpy(s_download_buf, bin_start, bin_len);
    if (OTA_Storage_Write_Block(g_ota_ctx.current_offset, s_download_buf, bin_len) != OTA_OK) {
        OTA_Handle_Error(OTA_ERR_DOWNLOAD_FAIL, "write staging failed");
        return -1;
    }

    block_index = g_ota_ctx.current_offset / OTA_BLOCK_SIZE;
    if (OTA_Storage_Set_Block_Done(block_index) != OTA_OK) {
        OTA_Handle_Error(OTA_ERR_DOWNLOAD_FAIL, "write bitmap failed");
        return -1;
    }

    if (OTA_Storage_Load_Metadata(&metadata) == OTA_OK) {
        metadata.downloaded_size += bin_len;
        if (metadata.downloaded_size > g_ota_ctx.total_size) {
            metadata.downloaded_size = g_ota_ctx.total_size;
        }
        metadata.state = OTA_SLOT_DOWNLOADING;
        metadata.failed_reason = OTA_OK;
        (void)OTA_Storage_Save_Metadata(&metadata);
    }

    g_ota_ctx.current_offset = next_offset;
    OTA_Report_Progress((int)((g_ota_ctx.current_offset * 100UL) / g_ota_ctx.total_size), NULL);
    return 0;
}

static void OTA_Finish_Download(void)
{
    ota_verified_image_t verified;
    ota_metadata_t metadata;
    int ret;

    g_ota_ctx.state = OTA_STATE_FINISHING;

    if (g_ota_ctx.expected_sign[0] != '\0') {
        ret = OTA_Check_Package_MD5(g_ota_ctx.total_size, g_ota_ctx.expected_sign);
        if (ret != OTA_OK) {
            OTA_Handle_Error(OTA_ERR_VERIFY_FAIL, "MD5 mismatch");
            return;
        }
    }

    ret = OTA_Verify_Stored_Image(&verified);
    if (ret != OTA_OK) {
        OTA_Handle_Error(ret, "image verify failed");
        return;
    }
    if (verified.package_size > g_ota_ctx.total_size) {
        OTA_Handle_Error(OTA_ERR_VERIFY_FAIL, "bad package size");
        return;
    }

    memset(&metadata, 0, sizeof(metadata));
    metadata.state = OTA_SLOT_READY;
    metadata.image_size = verified.header.image_size;
    metadata.image_crc32 = verified.header.image_crc32;
    memcpy(metadata.image_sha256, verified.header.image_sha256, sizeof(metadata.image_sha256));
    metadata.app_start_addr = verified.header.app_start_addr;
    metadata.sw_version = verified.header.sw_version;
    metadata.downloaded_size = verified.package_size;
    metadata.block_size = OTA_BLOCK_SIZE;
    metadata.total_blocks = OTA_Storage_Calc_Total_Blocks(verified.package_size);
    metadata.failed_reason = OTA_OK;

    if (OTA_Storage_Save_Metadata(&metadata) != OTA_OK) {
        OTA_Handle_Error(OTA_ERR_METADATA, "save ready metadata failed");
        return;
    }

    OTA_Report_Progress(100, "ready");
    g_ota_ctx.state = OTA_STATE_REBOOTING;
    HAL_Delay(100U);
    HAL_NVIC_SystemReset();
}

static int OTA_Check_Package_MD5(uint32_t package_size, const char *expected_sign)
{
    MD5_CTX md5_ctx;
    uint8_t buf[256];
    uint32_t offset = 0U;
    uint32_t remain = package_size;
    unsigned char digest[16];
    char calc_sign[33];

    if ((expected_sign == NULL) || (strlen(expected_sign) != 32U)) {
        return OTA_ERR_VERIFY_FAIL;
    }

    MD5Init(&md5_ctx);
    while (remain > 0U) {
        uint32_t chunk_size = (remain > sizeof(buf)) ? sizeof(buf) : remain;
        if (OTA_Storage_Read_Image(offset, buf, chunk_size) != OTA_OK) {
            return OTA_ERR_NO_STORAGE;
        }
        MD5Update(&md5_ctx, buf, chunk_size);
        offset += chunk_size;
        remain -= chunk_size;
    }

    MD5Final(&md5_ctx, digest);
    OTA_Bin2Hex(digest, calc_sign);

    return (OTA_Case_Equal(calc_sign, expected_sign) != 0) ? OTA_OK : OTA_ERR_VERIFY_FAIL;
}

static void OTA_Report_Progress(int percent, const char *desc)
{
    char json_msg[160];

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
    ota_metadata_t metadata;

    printf("[OTA] Error %d: %s\r\n", code, reason ? reason : "unknown");
    OTA_Report_Progress(code, reason ? reason : "error");
    g_ota_ctx.state = OTA_STATE_ERROR;

    if (OTA_Storage_Load_Metadata(&metadata) != OTA_OK) {
        memset(&metadata, 0, sizeof(metadata));
    }
    metadata.state = OTA_SLOT_FAILED;
    metadata.failed_reason = (uint32_t)code;
    metadata.block_size = OTA_BLOCK_SIZE;
    metadata.total_blocks = g_ota_ctx.total_blocks;
    metadata.downloaded_size = g_ota_ctx.current_offset;
    metadata.app_start_addr = OTA_APP_START_ADDR;
    (void)OTA_Storage_Save_Metadata(&metadata);
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
    if ((a == NULL) || (b == NULL)) {
        return 0;
    }

    while ((*a != '\0') && (*b != '\0')) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return 0;
        }
        a++;
        b++;
    }

    return (*a == '\0') && (*b == '\0');
}

static void OTA_Bin2Hex(const unsigned char *bin, char *out)
{
    static const char hex[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 16; i++) {
        out[i * 2] = hex[(bin[i] >> 4) & 0x0F];
        out[(i * 2) + 1] = hex[bin[i] & 0x0F];
    }
    out[32] = '\0';
}
