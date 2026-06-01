#ifndef __OTA_VERIFY_H
#define __OTA_VERIFY_H

#include "ota_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int OTA_Verify_Header(const ota_image_header_t *header);
int OTA_Verify_Stored_Image(ota_verified_image_t *image);
int OTA_Verify_App_Vector(uint32_t app_addr);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_VERIFY_H */
