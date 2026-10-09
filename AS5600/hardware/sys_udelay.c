#include "sys_udelay.h"
#include "main.h"

void SYS_uDelay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void SYS_uDelay(uint32_t delay_us)
{
    uint32_t start;
    uint32_t ticks;
    uint32_t guard;

    /* JTAG 探针在"下载→复位运行→断开"收尾时可能清掉 DEMCR.TRCENA（发生在
     * 本模块 Init 之后），CYCCNT 随之冻结；本函数常在关中断临界区内被调用，
     * 死等冻结的计数器会导致整机僵死。故每次进入先确保计数器使能。 */
    if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) ||
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    start = DWT->CYCCNT;
    ticks = (SystemCoreClock / 1000000U) * delay_us;

    /* 兜底上限：每次循环至少 2 个周期，guard 耗尽时实际等待已不少于
     * 8×ticks 周期，延时下限仍有保证；即使计数器中途被外部冻结也不会死等。 */
    guard = ticks * 4U;
    while (((uint32_t)(DWT->CYCCNT - start) < ticks) && (guard != 0U))
    {
        --guard;
        __NOP();
    }
}
