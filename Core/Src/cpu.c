/*
 * CPU.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include <cpu.h>
#include "stm32h7xx_hal.h"

static uint32_t cpuLastCallMs = 0u;
static uint32_t cpuLastDtMs = 0u;
static uint32_t cpuMaxDtMs = 0u;
static uint32_t cpuMissedDeadlines = 0u;
static uint32_t cpuWindowStartMs = 0u;
static uint32_t cpuWindowCount = 0u;
static uint16_t cpuLoopRateHz = 0u;

bool CPU_CheckTimingConstraints(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt;

    if (cpuLastCallMs == 0u) {
        cpuLastCallMs = now;
        cpuWindowStartMs = now;
        cpuLastDtMs = 0u;
        return true;
    }

    dt = now - cpuLastCallMs;
    cpuLastCallMs = now;
    cpuLastDtMs = dt;
    if (dt > cpuMaxDtMs) {
        cpuMaxDtMs = dt;
    }
    cpuWindowCount++;
    if ((now - cpuWindowStartMs) >= 1000u) {
        cpuLoopRateHz = (uint16_t)cpuWindowCount;
        cpuWindowCount = 0u;
        cpuWindowStartMs = now;
    }

    /*
     * Expect this function to be called at least every 5 ms by the main loop.
     * If the time delta grows beyond that we report a failure which can be used
     * by the caller as a CPU load indicator.
     */
    if (dt > 5u) {
        cpuMissedDeadlines++;
        return false;
    }
    return true;
}

void CPU_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint16_t CPU_GetLoopRateHz(void)
{
    return cpuLoopRateHz;
}

uint32_t CPU_GetLastLoopDtMs(void)
{
    return cpuLastDtMs;
}

uint32_t CPU_GetMaxLoopDtMs(void)
{
    return cpuMaxDtMs;
}

uint32_t CPU_GetMissedDeadlineCount(void)
{
    return cpuMissedDeadlines;
}
