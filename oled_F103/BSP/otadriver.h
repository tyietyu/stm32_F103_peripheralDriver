#ifndef __OTADRIVER_H
#define __OTADRIVER_H

#include "main.h"
#include <stdint.h>

/* ================= 用户配置区 ================= */
/* 固件存储起始地址 (必须对齐扇区，且不覆盖 Bootloader 和当前 APP) */
#define OTA_STORAGE_START_ADDR  0x08008000  //32K
#define OTA_FLAG_ADDR  0x0800FC00
/* 单次请求的分块大小  */
#define OTA_BLOCK_SIZE          512        
/* 接收超时时间 (毫秒) */
#define OTA_RX_TIMEOUT          5000        
/* 最大重试次数 */
#define OTA_MAX_RETRY           5           

#define OTA_ERR_UPGRADE_FAIL    -1
#define OTA_ERR_DOWNLOAD_FAIL   -2
#define OTA_ERR_VERIFY_FAIL     -3
#define OTA_ERR_BURN_FAIL       -4

/* ================= 状态机定义 ================= */
typedef enum {
    OTA_STATE_IDLE = 0,        // 空闲状态
    OTA_STATE_INIT,            // 初始化中
    OTA_STATE_NEGOTIATING,     // 协商中 (收到升级通知，准备请求)
    OTA_STATE_DOWNLOADING,     // 下载中 (循环请求数据块)
    OTA_STATE_FINISHING,       // 下载完成 (校验、写入标志位)
    OTA_STATE_ERROR,           // 错误状态
    OTA_STATE_REBOOTING        // 准备重启
} OTA_State_t;

/* OTA 运行时上下文结构体 */
typedef struct {
    OTA_State_t state;           // 当前状态

    /* 固件信息 */
    uint32_t total_size;         // 固件总大小 (Bytes)
    uint32_t stream_id;          // 阿里云下发的升级流 ID
    uint32_t current_offset;     // 当前已写入 Flash 的偏移量
    char     target_version[33]; // 目标版本号
    char     expected_sign[65];  // 存储云端下发的签名 (Hex字符串)
    char     sign_method[16];	 // 签名方法 (MD5 或 SHA256)

    /* 运行控制 */
    uint32_t last_req_tick;      // 上次请求的时间戳 (用于超时判断)
    uint8_t  retry_count;        // 当前块重试次数
    uint8_t  file_id;            // 文件 ID (固定为 1)
} OTA_Context_t;

/* Flash 标志位结构体 (用于 Bootloader 读取，需与 Bootloader 定义一致) */
typedef struct {
    uint32_t magic_flag;         // 魔法数，例如 0xAABB1122
    uint32_t firmware_len;       // 固件长度
    uint8_t  version[32];        // 版本号
    uint32_t crc32;              // (可选) CRC校验值
    uint8_t  reserved[64];
} OTA_Flash_Info_t;

/* 外部可调用函数 */

/**
 * @brief OTA 初始化
 * @note  订阅相关 Topic，并上报当前版本号
 */
void OTA_Init(void);

/**
 * @brief OTA 主循环
 * @note  需要在 main 循环中持续调用，处理超时和状态流转
 */
void OTA_Loop(void);

/**
 * @brief MQTT 消息处理回调
 * @note  当 ESP8266 收到 MQTT 消息时调用此函数
 * @param topic   主题字符串
 * @param payload 消息体指针
 * @param len     消息体长度
 */
void OTA_Process_MQTT_Msg(const char *topic, uint8_t *payload, uint16_t len);

/**
 * @brief 请求固件信息
 * @note  向云端请求当前固件版本、大小等信息
 */
void OTA_Request_Firmware(void);

#endif /* __OTADRIVER_H */

