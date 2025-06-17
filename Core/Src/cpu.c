/*
 * CPU.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include <cpu.h>
#include "stm32h7xx_hal.h"

bool CPU_CheckTimingConstraints(void)
{
    static uint32_t lastCall = 0;
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - lastCall;
    lastCall = now;

    /*
     * Expect this function to be called at least every 5 ms by the main loop.
     * If the time delta grows beyond that we report a failure which can be used
     * by the caller as a CPU load indicator.
     */
    return dt <= 5;
}

void CPU_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
