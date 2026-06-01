#ifndef __BOOT_JUMP_H
#define __BOOT_JUMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int BOOT_Is_Valid_App(uint32_t app_addr);
void BOOT_Jump_To_App(uint32_t app_addr);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_JUMP_H */
