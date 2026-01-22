#include "display_app.h"
#include "log.h"
#include "ov7725.h"
#include "usbd_cdc_if.h"


extern OV7725_Handle_t OV7725_Camera;
extern volatile OV7725_Capture_State_t capture_state;
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t usb_tx_buffer[2][PACKET_DATA_SIZE];  /* 双缓冲区 */

OV7725_DisplayApp_Handle_t OV7725_DisplayApp = {
    .frame_head = (FRAME_HEADER_SYNC1 << 8) | FRAME_HEADER_SYNC2,
    .frame_tail = (FRAME_HEADER_END1 << 8) | FRAME_HEADER_END2,
    .frame_header_size = FRAME_HEADER_SIZE,
    .packet_data_size = PACKET_DATA_SIZE,
    .frame_count = 0,
};

/**
 * @brief 等待 USB CDC 发送缓冲区空闲
 */
static uint8_t USB_Wait_Tx_Complete(uint32_t timeout_ms)
{
  uint32_t start_time = HAL_GetTick();
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  
  if (hcdc == NULL)
  {
    CAW_LOG_ERROR("USB CDC Handle is NULL");
    return HAL_ERROR;
  }

  while (hcdc->TxState != 0)
  {
    if (HAL_GetTick() - start_time > timeout_ms)
    {
      CAW_LOG_ERROR("USB Tx Data Timeout");
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

/**
 * @brief 发送帧头 (14字节)
 * @note 帧头格式: 同步字(2) + 帧号(2) + 总包数(2) + 每包大小(2) + 包序号(2) + 数据长度(2) + 结束字(2)
 * @return HAL_OK成功, HAL_ERROR失败
 */
static uint8_t USB_Send_Frame_Header(uint16_t frame_idx, uint16_t total_packets, uint16_t packet_size)
{
  uint8_t header[FRAME_HEADER_SIZE];
  
  /* 同步字 (2B) */
  header[0] = OV7725_DisplayApp.frame_head >> 8;
  header[1] = OV7725_DisplayApp.frame_head & 0xFF;
  /* 帧号 (2B) */
  header[2] = (uint8_t)(frame_idx & 0xFF);
  header[3] = (uint8_t)((frame_idx >> 8) & 0xFF);
  /* 总包数 (2B) */
  header[4] = (uint8_t)(total_packets & 0xFF);
  header[5] = (uint8_t)((total_packets >> 8) & 0xFF);
  /* 每包大小 (2B) */
  header[6] = (uint8_t)(packet_size & 0xFF);
  header[7] = (uint8_t)((packet_size >> 8) & 0xFF);
  /* 包序号 (2B) - 帧头中为0 */
  header[8] = 0x00;
  header[9] = 0x00;
  /* 数据长度 (2B) - 帧头中为0 */
  header[10] = 0x00;
  header[11] = 0x00;
  /* 结束字 (2B) */
  header[12] = OV7725_DisplayApp.frame_tail >> 8;
  header[13] = OV7725_DisplayApp.frame_tail & 0xFF;

  if (USB_Wait_Tx_Complete(500) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if (CDC_Transmit_FS(header, OV7725_DisplayApp.frame_header_size) != USBD_OK)
  {
    CAW_LOG_ERROR("USB sending frame header failed");
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief 发送数据包（包头+数据）
 * @return HAL_OK成功, HAL_ERROR失败
 */
static uint8_t USB_Send_Data_Packet(uint8_t *data, uint16_t data_len, uint16_t packet_idx)
{
  /* 包头: 包序号(2B) + 数据长度(2B) = 4字节 */
  uint8_t pkt_header[4];
  
  pkt_header[0] = (uint8_t)(packet_idx & 0xFF);
  pkt_header[1] = (uint8_t)((packet_idx >> 8) & 0xFF);
  pkt_header[2] = (uint8_t)(data_len & 0xFF);
  pkt_header[3] = (uint8_t)((data_len >> 8) & 0xFF);
  
  if (USB_Wait_Tx_Complete(500) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if (CDC_Transmit_FS(pkt_header, 4) != USBD_OK)
  {
    CAW_LOG_ERROR("USB sending packet header failed");
    return HAL_ERROR;
  }
  
  if (USB_Wait_Tx_Complete(500) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if (CDC_Transmit_FS(data, data_len) != USBD_OK)
  {
    CAW_LOG_ERROR("USB sending packet data failed");
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief 通过USB CDC分块发送一帧图像数据（带包序号验证）
 * @note 使用双缓冲：当一个缓冲区正在USB发送时，另一个缓冲区同时读取数据
 *       每个数据包带有包序号，上位机可验证完整性
 */
static void OV7725_Send_Frame_USB(void)
{
  uint32_t total_bytes = (OV7725_Camera.image_width * OV7725_Camera.image_height) * 2;
  uint32_t bytes_per_chunk = OV7725_DisplayApp.packet_data_size;
  uint16_t total_packets = (total_bytes + bytes_per_chunk - 1) / bytes_per_chunk;
  uint32_t bytes_sent = 0;
  uint32_t bytes_read = 0;
  uint16_t packet_idx = 0;
  uint8_t write_idx = 0;
  uint8_t send_idx = 0;
  uint8_t send_ok = 1;

  /* 只有在 CAPTURE_DONE 状态才能开始发送 */
  if (capture_state != CAPTURE_DONE) return;
  
  /* 检查USB是否已连接 */
  if (ov7725_Frame_Read_Start() != OV7725_OK) return;
  
  CAW_LOG_INFO("Frame %d sending, %d packets, USB connected", OV7725_DisplayApp.frame_count, total_packets);

  /* 发送帧头（包含总包数和每包大小） */
  if (USB_Send_Frame_Header(OV7725_DisplayApp.frame_count, total_packets, bytes_per_chunk) != HAL_OK)
  {
    ov7725_Frame_Read_End();
    CAW_LOG_ERROR("USB sending frame header failed");
    return;
  }

  /* 预填充第一个缓冲区 */
  uint16_t first_chunk_bytes = (total_bytes > bytes_per_chunk) ? bytes_per_chunk : total_bytes;
  uint16_t first_chunk_pixels = first_chunk_bytes / 2;
  ov7725_Frame_Read_Chunk(usb_tx_buffer[write_idx], first_chunk_pixels);
  bytes_read += first_chunk_bytes;

  while (bytes_sent < total_bytes && send_ok)
  {
    send_idx = write_idx;
    write_idx = 1 - write_idx;
    uint16_t current_bytes = (total_bytes - bytes_sent) > bytes_per_chunk ? bytes_per_chunk : (total_bytes - bytes_sent);

    /* 发送带包序号的数据包 */
    if (USB_Send_Data_Packet(usb_tx_buffer[send_idx], current_bytes, packet_idx) != HAL_OK)
    {
      send_ok = 0;
      break;
    }
    packet_idx++;

    /* 在 USB 正在发送的同时，读取下一块数据到另一个缓冲区 (异步并行) */
    if (bytes_read < total_bytes)
    {
      uint16_t next_bytes = (total_bytes - bytes_read) > bytes_per_chunk ? bytes_per_chunk : (total_bytes - bytes_read);
      uint16_t next_pixels = next_bytes / 2;
      ov7725_Frame_Read_Chunk(usb_tx_buffer[write_idx], next_pixels);
      bytes_read += next_bytes;
    }
    bytes_sent += current_bytes;
  }

  if (send_ok)
  {
    CAW_LOG_DEBUG("Frame %d sent OK, %d packets", OV7725_DisplayApp.frame_count, packet_idx);
    OV7725_DisplayApp.frame_count++;
  }
  ov7725_Frame_Read_End();
}

/**
 * @brief 处理一帧图像的读取与 USB 传输
 * @note OV7725_Send_Frame_USB 内部已调用 ov7725_Frame_Read_End()
 *       发送完成后状态重置为 IDLE，允许 VSYNC 触发新采集
 */
void OV7725_Process_Image(void)
{
  if (ov7725_Frame_Available())
  {
    OV7725_Send_Frame_USB();
    CAW_LOG_DEBUG("Frame send ok, capturing next image...");
  }
}

/**
 * @brief OV7725 初始化与配置
 */
uint8_t OV7725_Setup_Config(void)
{
  if (ov7725_Init() != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Hardware Init Failed!");
    return OV7725_ERROR;
  }
    
  /* 配置为 QVGA (320x240) RGB565 格式 */
  if (ov7725_Config(OV7725_QVGA_WIDTH_MAX, OV7725_QVGA_HEIGHT_MAX,
                    OV7725_OUTPUT_MODE_QVGA,
                    OV7725_LIGHT_MODE_AUTO,
                    OV7725_COLOR_SATURATION_4,
                    OV7725_BRIGHTNESS_4,
                    OV7725_CONTRAST_4,
                    OV7725_SPECIAL_EFFECT_NORMAL) != OV7725_OK)
  {
    CAW_LOG_ERROR("OV7725 Logic Config Failed!");
    return OV7725_ERROR;
  }
  
  CAW_LOG_INFO("OV7725 Ready! Resolution: %dx%d", OV7725_Camera.image_width, OV7725_Camera.image_height);
  CAW_LOG_INFO("Camera MID: %04X, PID: %04X", OV7725_Camera.mid, OV7725_Camera.pid);
  return OV7725_OK;
}
