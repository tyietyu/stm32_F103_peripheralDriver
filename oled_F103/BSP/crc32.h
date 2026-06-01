#ifndef __CRC32_H
#define __CRC32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t CRC32_InitValue(void);
uint32_t CRC32_Update(uint32_t crc, const uint8_t *data, uint32_t len);
uint32_t CRC32_Final(uint32_t crc);
uint32_t CRC32_Calc(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRC32_H */
