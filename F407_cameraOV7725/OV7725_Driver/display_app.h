#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H

#include "main.h"

/* 协议常量 */
#define FRAME_HEADER_SYNC1      0xAA
#define FRAME_HEADER_SYNC2      0x55
#define FRAME_HEADER_END1       0xA5
#define FRAME_HEADER_END2       0x5A
#define FRAME_HEADER_SIZE       14      /* 同步字(2) + 帧号(2) + 总包数(2) + 每包大小(2) +包序号(2) + 数据长度(2) + 结束字(2) */

#define PACKET_HEAD_SIZE        4       /* 包头大小: 包序号(2) + 数据长度(2) */
#define PACKET_DATA_SIZE        2044    /* 每包中图像数据的大小 */
#pragma pack(1)
typedef struct 
{
    uint16_t frame_head;        /* 帧头同步字: (SYNC1 << 8) | SYNC2 */
    uint16_t frame_tail;        /* 帧尾结束字: (END1 << 8) | END2 */
    uint16_t frame_header_size; /* 帧头大小 */
    uint16_t packet_data_size;  /* 数据包大小 */
    uint32_t frame_count;       /* 帧计数 */
} OV7725_DisplayApp_Handle_t;
#pragma pack()

extern OV7725_DisplayApp_Handle_t OV7725_DisplayApp;

uint8_t OV7725_Setup_Config(void);
void OV7725_Process_Image(void);

#endif // DISPLAY_APP_H
