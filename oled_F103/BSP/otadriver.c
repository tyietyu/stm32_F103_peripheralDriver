#include "otadriver.h"
#include "esp8266.h"
#include "flash.h"
#include "core_json.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "md5.h"

OTA_Context_t g_ota_ctx;
extern esp8266_config_t esp8266_config;

static void OTA_Send_Block_Request(void);
static void OTA_Report_Progress(int percent, const char *desc);
static void OTA_Report_Version(const char *version);
static void OTA_Finish_Download(void);
static void OTA_Handle_Error(const char *reason);
static int  OTA_Parse_Upgrade_Notify(const char *json, uint16_t len);
static int  OTA_Process_Download_Reply(uint8_t *data, uint16_t len);
static void bin2hex(const unsigned char *bin, char *out);


void OTA_Init(void)
{
    memset(&g_ota_ctx, 0, sizeof(OTA_Context_t));
    g_ota_ctx.state = OTA_STATE_INIT;
    
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", esp8266_config.ota.download_info_sub);
    ESP8266_send_at_cmd((uint8_t*)cmd, strlen(cmd), "OK");
    
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", esp8266_config.ota.device_download_file_reply);
    ESP8266_send_at_cmd((uint8_t*)cmd, strlen(cmd), "OK");

    OTA_Report_Version("1.0.0");
    g_ota_ctx.state = OTA_STATE_IDLE;
    printf("[OTA] Init Success. Waiting for upgrade...\r\n");

    OTA_Request_Firmware();
}

void OTA_Loop(void)
{
    if (g_ota_ctx.state == OTA_STATE_DOWNLOADING) {
        if (HAL_GetTick() - g_ota_ctx.last_req_tick > OTA_RX_TIMEOUT) 
        {
            if (g_ota_ctx.retry_count < OTA_MAX_RETRY) 
            {
                printf("[OTA] Timeout waiting for offset %d. Retrying (%d/%d)...\r\n", 
                       g_ota_ctx.current_offset, g_ota_ctx.retry_count + 1, OTA_MAX_RETRY);
                g_ota_ctx.retry_count++;
                OTA_Send_Block_Request(); 
            } else {
                OTA_Report_Progress(OTA_ERR_DOWNLOAD_FAIL, "Timeout");
                OTA_Handle_Error("Max retries reached");
            }
        }
    }
}

void OTA_Process_MQTT_Msg(const char *topic, uint8_t *payload, uint16_t len)
{
    // 1. 处理云端下发的升级通知
    if (strstr(topic, "/ota/device/upgrade")) 
    {
        printf("[OTA] Received Upgrade Notification\r\n");
        if (OTA_Parse_Upgrade_Notify((const char *)payload, len) == 0) 
        {
            printf("[OTA] Starting Download. Size: %d bytes\r\n", g_ota_ctx.total_size);
            g_ota_ctx.state = OTA_STATE_DOWNLOADING;
            g_ota_ctx.current_offset = 0;
            g_ota_ctx.retry_count = 0;
            g_ota_ctx.file_id = 1; 
            OTA_Send_Block_Request(); 
        }
    }
    // 2. 处理文件数据块响应
    else if (strstr(topic, "thing/file/download_reply")) 
    {
        if (g_ota_ctx.state == OTA_STATE_DOWNLOADING) 
        {
            if (OTA_Process_Download_Reply(payload, len) == 0) 
            {
                if (g_ota_ctx.current_offset >= g_ota_ctx.total_size) 
                {
                    OTA_Finish_Download();
                } 
                else 
                {
                    g_ota_ctx.retry_count = 0;
                    OTA_Send_Block_Request();
                }
            }
        }
    }
}

void OTA_Request_Firmware(void)
{
    char json_req[128];
    snprintf(json_req, sizeof(json_req), "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{\"module\":\"default\"}}", HAL_GetTick());
    ESP8266_send_msg(esp8266_config.ota.device_active_info_pub, "%s", json_req);
    printf("[OTA] Requested firmware update info...\r\n");
}

/**
 * @brief 发送分块下载请求
 * @doc   阿里云 Topic: /sys/${pk}/${dn}/thing/file/download
 */
static void OTA_Send_Block_Request(void)
{
    // 计算本次请求大小
    uint32_t req_size = OTA_BLOCK_SIZE;
    if (g_ota_ctx.total_size - g_ota_ctx.current_offset < OTA_BLOCK_SIZE) 
    {
        req_size = g_ota_ctx.total_size - g_ota_ctx.current_offset;
    }

    char json_buf[256];
    snprintf(json_buf, sizeof(json_buf), 
        "{\"id\":\"%d\",\"params\":{\"fileInfo\":{\"streamId\":%d,\"fileId\":%d},\"fileBlock\":{\"size\":%d,\"offset\":%d},\"module\":\"default\"}}",
        (g_ota_ctx.current_offset / OTA_BLOCK_SIZE) + 1, // ID 自增即可
        g_ota_ctx.stream_id,
        g_ota_ctx.file_id,
        req_size,
        g_ota_ctx.current_offset
    );
    ESP8266_send_msg(esp8266_config.ota.device_download_file, "%s", json_buf);
    g_ota_ctx.last_req_tick = HAL_GetTick();
}

/**
 * @brief 解析文件下载回复 (关键函数)
 * @doc   数据格式: [2字节 JSON 长度] + [JSON 字符串] + [二进制文件流]
 */
uint8_t aligned_buffer[OTA_BLOCK_SIZE + 16];
static int OTA_Process_Download_Reply(uint8_t *data, uint16_t len)
{
    if (len < 2) return -1;
    // 1. 读取 JSON 部分长度 (大端序: 高位在前，低位在后)
    uint16_t json_len = (uint16_t)((data[0] << 8) | data[1]);
    // 校验总长度
    if (len < 2 + json_len) {
        printf("[OTA] Error: Data too short (Header says JSON is %d bytes)\r\n", json_len);
        return -1;
    }
    // 2. 定位二进制数据起始位置
    uint8_t *bin_start = data + 2 + json_len;
    uint16_t bin_len = len - (2 + json_len);
    if (bin_len == 0) 
    {
        printf("[OTA] Warn: Reply contains no binary data.\r\n");
        // 可能是服务端报错，建议解析 JSON 里的 code 字段查看原因
        return -1;
    }

    // 3. 写入 Flash 内存
    memset(aligned_buffer, 0, sizeof(aligned_buffer));
    memcpy(aligned_buffer, bin_start, bin_len);
    iap_write_flash(OTA_STORAGE_START_ADDR + g_ota_ctx.current_offset, (uint16_t*)aligned_buffer, (bin_len + 1) / 2);

    // 4. 更新进度
    g_ota_ctx.current_offset += bin_len;
    
    // 5. 每 10% 或 完成时上报一次进度
    uint32_t percent = (g_ota_ctx.current_offset * 100) / g_ota_ctx.total_size;
    static uint32_t last_percent = 0;
    if (percent - last_percent >= 10 || percent == 100) 
    {
        OTA_Report_Progress(percent, NULL);
        last_percent = percent;
        printf("[OTA] Progress: %d%%\r\n", percent);
    }
    return 0;
}

/**
 * @brief 解析升级通知消息
 */
static int OTA_Parse_Upgrade_Notify(const char *json, uint16_t len)
{
    JSONStatus_t result;
    char *val;
    size_t val_len;

    // 1. 获取固件大小 (data.size)
    result = JSON_Search((char*)json, len, "data.size", 9, &val, &val_len);
    if (result == JSONSuccess) 
    {
        char temp[16] = {0};
        if(val_len < 15) memcpy(temp, val, val_len);
        g_ota_ctx.total_size = atoi(temp);
    } else return -1;

    // 2. 获取 streamId (data.streamId) - 必须保存，用于后续请求
    result = JSON_Search((char*)json, len, "data.streamId", 13, &val, &val_len);
    if (result == JSONSuccess) 
    {
        char temp[16] = {0};
        if(val_len < 15) memcpy(temp, val, val_len);
        g_ota_ctx.stream_id = atoi(temp);
    } else return -1;

    // 3. 获取目标版本号
    result = JSON_Search((char*)json, len, "data.version", 12, &val, &val_len);
    if (result == JSONSuccess) 
    {
        memset(g_ota_ctx.target_version, 0, sizeof(g_ota_ctx.target_version));
        if(val_len < 32) memcpy(g_ota_ctx.target_version, val, val_len);
    }

    // 4. 获取签名方法 (data.signMethod) - 必须为 "MD5"
    result = JSON_Search((char*)json, len, "data.signMethod", 15, &val, &val_len);
    if (result == JSONSuccess) 
    {
        char method[16] = {0};
        if(val_len < 15) memcpy(method, val, val_len);
        if (strcasecmp(method, "MD5") != 0) 
        {
            printf("[OTA] Error: Unsupported sign method: %s. Only MD5 is supported.\r\n", method);
            return -1; 
        }
    }

    result = JSON_Search((char*)json, len, "data.sign", 9, &val, &val_len);
    if (result == JSONSuccess) 
    {
        memset(g_ota_ctx.expected_sign, 0, sizeof(g_ota_ctx.expected_sign));
        if(val_len < 64) memcpy(g_ota_ctx.expected_sign, val, val_len);
        printf("[OTA] Expected Sign: %s\r\n", g_ota_ctx.expected_sign);
    } 
    else 
    {
        /* 如果没有 sign，尝试找 md5 字段  */
        result = JSON_Search((char*)json, len, "data.md5", 8, &val, &val_len);
        if (result == JSONSuccess) 
        {
            memset(g_ota_ctx.expected_sign, 0, sizeof(g_ota_ctx.expected_sign));
            if(val_len < 64) memcpy(g_ota_ctx.expected_sign, val, val_len);
        } 
        else 
        {
            printf("[OTA] Error: No signature found.\r\n");
            return -1; // 安全起见，无签名不升级
        }
    }
    return 0;
}

static void OTA_Report_Progress(int percent, const char *desc)
{
    char json_msg[128];
    snprintf(json_msg, sizeof(json_msg), "{\"id\":\"%d\",\"params\":{\"step\":\"%d\",\"desc\":\"%s\",\"module\":\"default\"}}",
            HAL_GetTick(), percent, desc ? desc : "downloading");
    ESP8266_send_msg(esp8266_config.ota.device_report_progress_pub, "%s", json_msg);
}

static void OTA_Report_Version(const char *version)
{
    char json_msg[128];
    snprintf(json_msg, sizeof(json_msg), 
             "{\"id\":\"1\",\"params\":{\"version\":\"%s\"}}", version);
    ESP8266_send_msg(esp8266_config.ota.upload_info_pub, "%s", json_msg);
}

static void OTA_Finish_Download(void)
{
    printf("[OTA] Download Complete. Verifying...\r\n");
    g_ota_ctx.state = OTA_STATE_FINISHING;

    MD5_CTX md5_ctx;
    MD5Init(&md5_ctx);
    uint8_t buf[256]; 
    uint32_t offset = 0;
    uint32_t remain = g_ota_ctx.total_size;

    while(remain > 0)
    {
        uint32_t chunk_size = (remain > sizeof(buf)) ? sizeof(buf) : remain;
        /* 从 Flash 回读固件 (注意 iap_read_flash 的参数单位是半字/2字节) */
        iap_read_flash(OTA_STORAGE_START_ADDR + offset, (uint16_t*)buf, (chunk_size + 1) / 2);
        MD5Update(&md5_ctx, buf, chunk_size);
        offset += chunk_size;
        remain -= chunk_size;
    }

    unsigned char digest[16];
    char calc_sign[33];
    MD5Final(&md5_ctx, digest);
    bin2hex(digest, calc_sign);

    printf("[OTA] Calc MD5: %s\r\n", calc_sign);
    printf("[OTA] Cloud MD5: %s\r\n", g_ota_ctx.expected_sign);

    /* 忽略大小写对比 */
    if(strncasecmp(calc_sign, g_ota_ctx.expected_sign, 32) != 0)
    {
        OTA_Report_Progress(OTA_ERR_VERIFY_FAIL, "MD5 Mismatch");
        OTA_Handle_Error("MD5 Check Failed!");
        return;
    }

    // 写入 OTA 完成标志位到 Flash 参数区
    // Bootloader 启动时检查此标志，决定是否搬运 OTA_STORAGE_START_ADDR 的数据覆盖 APP
    OTA_Flash_Info_t info;
    memset(&info, 0, sizeof(info));

    info.magic_flag = 0xAA55A55A;
    info.firmware_len = g_ota_ctx.total_size;
    strncpy((char*)info.version, g_ota_ctx.target_version, 32);

    uint16_t len_in_halfwords = (sizeof(OTA_Flash_Info_t) + 1) / 2;
    iap_write_flash(OTA_FLAG_ADDR, (uint16_t*)&info, len_in_halfwords);

    OTA_Report_Progress(100, "success");
    printf("[OTA] Rebooting system...\r\n");
    HAL_Delay(100); // 等待 MQTT 发送完成

    // 4. 系统重启
    printf("[OTA] Rebooting system...\r\n");
    HAL_NVIC_SystemReset();
}

static void OTA_Handle_Error(const char *reason)
{
    printf("[OTA] Error: %s\r\n", reason);
    OTA_Report_Progress(-1, reason); // -1 代表失败
    g_ota_ctx.state = OTA_STATE_ERROR;
}

static void bin2hex(const unsigned char *bin, char *out)
{
    const char *hex = "0123456789abcdef";
    for(int i=0; i<16; i++) 
    {
        out[i*2] = hex[(bin[i] >> 4) & 0xF];
        out[i*2+1] = hex[bin[i] & 0xF];
    }
    out[32] = 0;
}

