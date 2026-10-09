#ifndef _SYS_UDELAY_H
#define _SYS_UDELAY_H

#include <stdint.h>

/**
 * @brief Initialize the uDelay function
 * 
 */
void SYS_uDelay_Init(void);

/**
 * @brief Delay for a specified number of microseconds
 * 
 * @param delay_us The number of microseconds to delay
 */
void SYS_uDelay(uint32_t delay_us);

#endif

