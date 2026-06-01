#include "boot_jump.h"
#include "main.h"
#include "ota_types.h"
#include "ota_verify.h"

typedef void (*boot_app_entry_t)(void);

int BOOT_Is_Valid_App(uint32_t app_addr)
{
    return (OTA_Verify_App_Vector(app_addr) == OTA_OK) ? 1 : 0;
}

void BOOT_Jump_To_App(uint32_t app_addr)
{
    uint32_t app_sp;
    uint32_t app_reset;
    uint32_t i;

    if (BOOT_Is_Valid_App(app_addr) == 0) {
        return;
    }

    app_sp = *(volatile uint32_t *)app_addr;
    app_reset = *(volatile uint32_t *)(app_addr + 4U);

    __disable_irq();
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    for (i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    HAL_RCC_DeInit();
    HAL_DeInit();

    SCB->VTOR = app_addr;
    __set_MSP(app_sp);
    ((boot_app_entry_t)app_reset)();
}
