#include "ota_verify.h"
#include "crc32.h"
#include "ota_storage.h"
#include <string.h>

int OTA_Verify_Header(const ota_image_header_t *header)
{
    ota_image_header_t temp;
    uint32_t header_crc;

    if (header == 0) {
        return OTA_ERR_VERIFY_FAIL;
    }
    if ((header->magic != OTA_IMAGE_MAGIC) ||
        (header->header_version != OTA_IMAGE_HEADER_VERSION) ||
        (header->header_size < sizeof(ota_image_header_t)) ||
        (header->header_size > OTA_BLOCK_SIZE)) {
        return OTA_ERR_VERIFY_FAIL;
    }
    if ((header->image_size == 0U) || (header->image_size > OTA_APP_SIZE_BYTES)) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }
    if (header->app_start_addr != OTA_APP_START_ADDR) {
        return OTA_ERR_HW_MISMATCH;
    }

    memcpy(&temp, header, sizeof(temp));
    header_crc = temp.header_crc32;
    temp.header_crc32 = 0UL;
    if (CRC32_Calc((const uint8_t *)&temp, sizeof(temp)) != header_crc) {
        return OTA_ERR_VERIFY_FAIL;
    }

    return OTA_OK;
}

int OTA_Verify_Stored_Image(ota_verified_image_t *image)
{
    ota_image_header_t header;
    uint8_t buf[256];
    uint32_t crc;
    uint32_t offset;
    uint32_t remain;
    uint32_t app_sp;
    uint32_t app_reset;
    int ret;

    ret = OTA_Storage_Read_Image(0U, (uint8_t *)&header, sizeof(header));
    if (ret != OTA_OK) {
        return ret;
    }

    ret = OTA_Verify_Header(&header);
    if (ret != OTA_OK) {
        return ret;
    }
    if (((uint32_t)header.header_size + header.image_size) < header.image_size) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }
    if (((uint32_t)header.header_size + header.image_size) > OTA_PACKAGE_MAX_SIZE) {
        return OTA_ERR_SIZE_TOO_LARGE;
    }

    offset = header.header_size;
    remain = header.image_size;
    crc = CRC32_InitValue();

    while (remain > 0U) {
        uint32_t chunk = (remain > sizeof(buf)) ? sizeof(buf) : remain;

        ret = OTA_Storage_Read_Image(offset, buf, chunk);
        if (ret != OTA_OK) {
            return ret;
        }
        crc = CRC32_Update(crc, buf, chunk);
        offset += chunk;
        remain -= chunk;
    }

    if (CRC32_Final(crc) != header.image_crc32) {
        return OTA_ERR_VERIFY_FAIL;
    }

    ret = OTA_Storage_Read_Image(header.header_size, buf, 8U);
    if (ret != OTA_OK) {
        return ret;
    }
    app_sp = ((uint32_t)buf[0] |
              ((uint32_t)buf[1] << 8U) |
              ((uint32_t)buf[2] << 16U) |
              ((uint32_t)buf[3] << 24U));
    app_reset = ((uint32_t)buf[4] |
                 ((uint32_t)buf[5] << 8U) |
                 ((uint32_t)buf[6] << 16U) |
                 ((uint32_t)buf[7] << 24U));
    if ((app_sp < OTA_SRAM_START_ADDR) || (app_sp > OTA_SRAM_END_ADDR)) {
        return OTA_ERR_VERIFY_FAIL;
    }
    if ((app_reset < OTA_APP_START_ADDR) ||
        (app_reset >= OTA_APP_END_ADDR) ||
        ((app_reset & 1UL) == 0UL)) {
        return OTA_ERR_VERIFY_FAIL;
    }

    if (image != 0) {
        memcpy(&image->header, &header, sizeof(header));
        image->package_size = header.header_size + header.image_size;
    }

    return OTA_OK;
}

int OTA_Verify_App_Vector(uint32_t app_addr)
{
    uint32_t app_sp;
    uint32_t app_reset;

    if ((app_addr < OTA_APP_START_ADDR) || (app_addr >= OTA_APP_END_ADDR)) {
        return OTA_ERR_VERIFY_FAIL;
    }

    app_sp = *(volatile uint32_t *)app_addr;
    app_reset = *(volatile uint32_t *)(app_addr + 4U);

    if ((app_sp < OTA_SRAM_START_ADDR) || (app_sp > OTA_SRAM_END_ADDR)) {
        return OTA_ERR_VERIFY_FAIL;
    }
    if ((app_reset < OTA_APP_START_ADDR) || (app_reset >= OTA_APP_END_ADDR) || ((app_reset & 1UL) == 0UL)) {
        return OTA_ERR_VERIFY_FAIL;
    }

    return OTA_OK;
}
