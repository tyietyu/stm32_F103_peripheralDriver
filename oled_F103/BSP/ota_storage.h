#ifndef __OTA_STORAGE_H
#define __OTA_STORAGE_H

#include "ota_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int OTA_Storage_Init(void);
int OTA_Storage_Load_Metadata(ota_metadata_t *metadata);
int OTA_Storage_Save_Metadata(const ota_metadata_t *metadata);
int OTA_Storage_Clear_Slot(uint32_t package_size);
int OTA_Storage_Write_Block(uint32_t offset, const uint8_t *data, uint32_t len);
int OTA_Storage_Read_Image(uint32_t offset, uint8_t *data, uint32_t len);
int OTA_Storage_Set_Block_Done(uint32_t block_index);
int OTA_Storage_Is_Block_Done(uint32_t block_index);
int OTA_Storage_Clear_Bitmap(void);
uint32_t OTA_Storage_Calc_Total_Blocks(uint32_t package_size);
void OTA_Metadata_Fill_Crc(ota_metadata_t *metadata);
int OTA_Metadata_Is_Valid(const ota_metadata_t *metadata);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_STORAGE_H */
