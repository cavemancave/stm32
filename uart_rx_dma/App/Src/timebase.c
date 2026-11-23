/**
 * @file    timebase.c
 * @brief   DWT CYCCNT 初始化与换算。
 */
#include "timebase.h"
#include "main.h"

void timebase_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t timebase_cycles(void)
{
    return DWT->CYCCNT;
}

uint32_t timebase_cycles_to_us(uint32_t cycles)
{
    uint32_t per_us = SystemCoreClock / 1000000U;
    if (per_us == 0U) {
        return 0U;
    }
    return cycles / per_us;
}
