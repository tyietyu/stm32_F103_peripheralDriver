#ifndef _EE_H_
#define _EE_H_

/***********************************************************************************************************

  Author:     Nima Askari
  Github:     https://www.github.com/NimaLTD
  LinkedIn:   https://www.linkedin.com/in/nimaltd
  Youtube:    https://www.youtube.com/@nimaltd
  Instagram:  https://instagram.com/github.NimaLTD

  Version:    3.0.0

  History:

              3.0.0
              - Rewrite again
              - Support STM32CubeMx Packet installer

***********************************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "main.h"

typedef struct
{
  uint8_t                *DataPointer;
  uint32_t               Size;
  uint32_t               Offset;

} EE_HandleTypeDef;


bool      EE_Init(EE_HandleTypeDef *pHandle, void *StoragePointer, uint32_t Size, uint32_t Offset);
uint32_t  EE_Capacity(void);
bool      EE_Format(void); 
void      EE_Read(EE_HandleTypeDef *pHandle);
bool      EE_Write(EE_HandleTypeDef *pHandle);

#define EE_ERASE_PAGE_ADDRESS               0
#define EE_ERASE_PAGE_NUMBER                1
#define EE_ERASE_SECTOR_NUMBER              2

#define USE_BOOTLOADER                      0

#ifdef  STM32F1

#if    defined FLASH_BANK_2
#define  EE_BANK_SELECT                     FLASH_BANK_2
#elif  defined FLASH_BANK_1
#define EE_BANK_SELECT                      FLASH_BANK_1
#endif

//configure region page numbers
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         0x400
#endif

#define FLASH_SIZE                          ((((uint32_t)(*((uint16_t *)FLASHSIZE_BASE)) & (0xFFFFU))) * 1024)
#define USER_EE_SIZE                        (2 * FLASH_PAGE_SIZE) // Reserve 2 pages for EEPROM

#ifdef USE_BOOTLOADER
    /*****************************************************************************************************
     * 配置 1: 有 Bootloader 的内存布局 (STM32F1)
     *****************************************************************************************************/
    #define BTLD_PAGE_NUM						20
    #define BTLD_ADDR							  FLASH_BASE
    #define BTLD_SIZE							  (BTLD_PAGE_NUM * FLASH_PAGE_SIZE)

    #define USER_EE_ADDR						(BTLD_ADDR + BTLD_SIZE)

    #define APP_ADDR							  (USER_EE_ADDR + USER_EE_SIZE)
    #define APP_SIZE							  (FLASH_SIZE - BTLD_SIZE - USER_EE_SIZE)
    #define APP_ERASE_PAGE_NUM			((FLASH_SIZE - BTLD_SIZE - USER_EE_SIZE) / FLASH_PAGE_SIZE)
#else
    /*****************************************************************************************************
     * 配置 2: 无 Bootloader 的内存布局 (STM32F1)
     *****************************************************************************************************/
    #define APP_ADDR							  FLASH_BASE
    #define APP_SIZE							  (FLASH_SIZE - USER_EE_SIZE)

    #define USER_EE_ADDR						(FLASH_BASE + FLASH_SIZE - USER_EE_SIZE)
    #define APP_ERASE_PAGE_NUM			(APP_SIZE / FLASH_PAGE_SIZE)

#endif // USE_BOOTLOADER

#define EE_ERASE                            EE_ERASE_PAGE_ADDRESS
#ifndef EE_ADDRESS
#define EE_ADDRESS                          USER_EE_ADDR
#endif
#endif // STM32F1

#ifdef  STM32F4

#if    defined FLASH_BANK_2
#define  EE_BANK_SELECT                     FLASH_BANK_2
#elif  defined FLASH_BANK_1
#define EE_BANK_SELECT                      FLASH_BANK_1
#endif

/**
 * @brief for custom, eeprom only in FLASH_SECTOR_4, total 64Kb
 * 
 */
#define FLASH_SIZE                          ((((uint32_t)(*((uint16_t *)FLASHSIZE_BASE)) & (0xFFFFU))) * 1024)

#ifdef USE_BOOTLOADER
    /*****************************************************************************************************
     * 配置 1: 有 Bootloader 的内存布局 (STM32F4)
     *****************************************************************************************************/
    #define BTLD_ADDR							FLASH_BASE
    #define BTLD_SIZE							(4 * 16 * 1024)     // 假设Bootloader占用 64KB (Sector 0-3)

    #define USER_EE_ADDR					(BTLD_ADDR + BTLD_SIZE) // EEPROM区域紧跟在Bootloader之后
    #define USER_EE_SIZE          (1 * 64 * 1024)         // EEPROM大小为 64KB
    #define EE_PAGE_SECTOR				FLASH_SECTOR_4          // EEPROM 使用 Sector 4

    #define APP_ADDR							(BTLD_ADDR + BTLD_SIZE + USER_EE_SIZE) // APP 在EEPROM之后
    #define APP_SIZE							(FLASH_SIZE - BTLD_SIZE - USER_EE_SIZE)

#else
    /*****************************************************************************************************
     * 配置 2: 无 Bootloader 的内存布局 (STM32F4)
     *****************************************************************************************************/
    #define USER_EE_SIZE            (1 * 128 * 1024) // 使用最后一个扇区，通常为128KB

    #define APP_ADDR						    FLASH_BASE
    #define APP_SIZE							  (FLASH_SIZE - USER_EE_SIZE)

    #define USER_EE_ADDR						(FLASH_BASE + FLASH_SIZE - USER_EE_SIZE) 
    
    // 重要：需要根据MCU型号和Flash大小，查阅数据手册来确定最后一个扇区的编号。
    #define EE_PAGE_SECTOR						FLASH_SECTOR_11  
#endif // USE_BOOTLOADER

#define EE_ERASE                            EE_ERASE_SECTOR_NUMBER

#ifndef EE_ADDRESS
#define EE_ADDRESS                          USER_EE_ADDR
#endif
#endif // STM32F4





#ifndef EE_SIZE
#if (EE_ERASE == EE_ERASE_PAGE_NUMBER) || (EE_ERASE == EE_ERASE_PAGE_ADDRESS)
#define EE_SIZE                             FLASH_PAGE_SIZE
#elif (EE_ERASE == EE_ERASE_SECTOR_NUMBER)
#define EE_SIZE                             FLASH_SECTOR_SIZE
#endif
#endif


#if 0
#if    defined FLASH_BANK_2
#define  EE_BANK_SELECT                     FLASH_BANK_2
#elif  defined FLASH_BANK_1
#define EE_BANK_SELECT                      FLASH_BANK_1
#endif

/**
 * @brief for custom, 
 * bootloader in sector 0-3,  64Kb;
 * eeprom in sector 4;
 * APP in sector 5->
 */
#ifndef EE_PAGE_SECTOR
#if (EE_BANK_SELECT ==  FLASH_BANK_2)
// #define EE_PAGE_SECTOR                      ((FLASH_SIZE / EE_SIZE / 2) - 1)
#define EE_PAGE_SECTOR                      (FLASH_SECTOR_4)
#else
// #define EE_PAGE_SECTOR                      ((FLASH_SIZE / EE_SIZE) - 1)
#define EE_PAGE_SECTOR                      (FLASH_SECTOR_4)
#endif
#endif

#ifndef EE_ADDRESS
#if (EE_BANK_SELECT !=  FLASH_BANK_2)
/**
 * @brief for custom
 * 
 */
//#define EE_ADDRESS                          (FLASH_BASE + EE_SIZE * EE_PAGE_SECTOR)
#define EE_ADDRESS                          (FLASH_BASE + (16 * 4) * 1024)       // sector 0-3 per sector 16Kb, and sector 4 64Kb as eeprom room 

#else

#define EE_ADDRESS                          (FLASH_BASE + (16 * 4) * 1024)       // sector 0-3 per sector 16Kb, and sector 4 64Kb as eeprom room 
// #define EE_ADDRESS                          (FLASH_BASE + EE_SIZE * (EE_PAGE_SECTOR * 2 + 1))
#endif
#endif

#endif

#ifndef EE_ERASE
#error "Not Supported MCU!"
#endif

#ifdef __cplusplus
}
#endif
#endif
