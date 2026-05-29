#ifndef _FLASH_H
#define _FLASH_H

#include "main.h"


#define __IO volatile
typedef __IO uint16_t vuint16_t;

#define STM32_FLASH_SIZE 64U // STM32F103C8T6 Flash size, unit: KB
#if STM32_FLASH_SIZE < 256   // 设置扇区大小
#define STM_SECTOR_SIZE 1024 // 1K字节
#else
#define STM_SECTOR_SIZE 2048 // 2K字节
#endif

#define STM32_FLASH_BASE 0x08000000                             // STM32 FLASH的起始地址
#define FLASH_SAVE_ADDR (STM32_FLASH_BASE + STM_SECTOR_SIZE * 60U) // Keep away from the last 2 EEPROM pages
#define STM32_FLASH_WREN 1                                      // 使能FLASH写入(0，不是能;1，使能)
#define FLASH_WAITETIME 50000                                   // FLASH等待超时时间

uint8_t STMFLASH_GetStatus(void);                                                   // 获得状态
uint8_t STMFLASH_WaitDone(uint16_t time);                                           // 等待操作结束
uint8_t STMFLASH_ErasePage(uint32_t paddr);                                         // 擦除页
uint8_t STMFLASH_WriteHalfWord(uint32_t faddr, uint16_t dat);                       // 写入半字
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);                                     // 读出半字
void STMFLASH_WriteLenByte(uint32_t WriteAddr, uint32_t DataToWrite, uint16_t Len); // 指定地址开始写入指定长度的数据
uint32_t STMFLASH_ReadLenByte(uint32_t ReadAddr, uint16_t Len);                     // 指定地址开始读取指定长度数据
void iap_write_flash(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);    // 从指定地址开始写入指定长度的数据
void iap_read_flash(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);       // 从指定地址开始读出指定长度的数据

#endif // !_FLASH_H
