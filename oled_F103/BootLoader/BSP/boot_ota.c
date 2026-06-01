#include "boot_ota.h"
#include "boot_flash.h"
#include "ota_storage.h"
#include "ota_types.h"
#include "ota_verify.h"
#include <string.h>

static int BOOT_OTA_Set_State(ota_slot_state_t state, int failed_reason)
{
    ota_metadata_t metadata;

    if (OTA_Storage_Load_Metadata(&metadata) != OTA_OK) {
        memset(&metadata, 0, sizeof(metadata));
        metadata.block_size = OTA_BLOCK_SIZE;
        metadata.app_start_addr = OTA_APP_START_ADDR;
    }

    metadata.state = (uint32_t)state;
    metadata.failed_reason = (uint32_t)failed_reason;
    return OTA_Storage_Save_Metadata(&metadata);
}

static int BOOT_OTA_Program_App(const ota_verified_image_t *image)
{
    uint8_t buf[256];
    uint32_t src_offset;
    uint32_t dst_addr;
    uint32_t remain;
    int ret;

    if (image == 0) {
        return OTA_ERR_VERIFY_FAIL;
    }

    ret = BOOT_Flash_Erase_App();
    if (ret != OTA_OK) {
        return ret;
    }

    src_offset = image->header.header_size;
    dst_addr = OTA_APP_START_ADDR;
    remain = image->header.image_size;

    while (remain > 0U) {
        uint32_t chunk = (remain > sizeof(buf)) ? sizeof(buf) : remain;

        ret = OTA_Storage_Read_Image(src_offset, buf, chunk);
        if (ret != OTA_OK) {
            return ret;
        }

        ret = BOOT_Flash_Write(dst_addr, buf, chunk);
        if (ret != OTA_OK) {
            return ret;
        }

        src_offset += chunk;
        dst_addr += chunk;
        remain -= chunk;
    }

    return OTA_Verify_App_Vector(OTA_APP_START_ADDR);
}

int BOOT_OTA_Check(void)
{
    ota_metadata_t metadata;
    ota_verified_image_t image;
    int ret;

    if (OTA_Storage_Init() != OTA_OK) {
        return OTA_ERR_NO_STORAGE;
    }

    if (OTA_Storage_Load_Metadata(&metadata) != OTA_OK) {
        return OTA_OK;
    }

    if (metadata.state == OTA_SLOT_PENDING_CONFIRM) {
        (void)BOOT_OTA_Set_State(OTA_SLOT_FAILED, OTA_ERR_NOT_CONFIRMED);
        return OTA_ERR_NOT_CONFIRMED;
    }

    if ((metadata.state != OTA_SLOT_READY) && (metadata.state != OTA_SLOT_UPDATING)) {
        return OTA_OK;
    }

    ret = BOOT_OTA_Set_State(OTA_SLOT_UPDATING, OTA_OK);
    if (ret != OTA_OK) {
        return ret;
    }

    ret = OTA_Verify_Stored_Image(&image);
    if (ret != OTA_OK) {
        (void)BOOT_OTA_Set_State(OTA_SLOT_FAILED, ret);
        return ret;
    }

    ret = BOOT_OTA_Program_App(&image);
    if (ret != OTA_OK) {
        (void)BOOT_OTA_Set_State(OTA_SLOT_FAILED, OTA_ERR_BOOT_PROGRAM_FAIL);
        return ret;
    }

    ret = BOOT_OTA_Set_State(OTA_SLOT_PENDING_CONFIRM, OTA_OK);
    if (ret != OTA_OK) {
        return ret;
    }

    return OTA_OK;
}
