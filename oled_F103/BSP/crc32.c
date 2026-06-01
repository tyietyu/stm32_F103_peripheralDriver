#include "crc32.h"

uint32_t CRC32_InitValue(void)
{
    return 0xFFFFFFFFUL;
}

uint32_t CRC32_Update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t bit;

    if ((data == 0) && (len != 0U)) {
        return crc;
    }

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

uint32_t CRC32_Final(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t CRC32_Calc(const uint8_t *data, uint32_t len)
{
    return CRC32_Final(CRC32_Update(CRC32_InitValue(), data, len));
}
