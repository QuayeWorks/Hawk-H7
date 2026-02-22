/*
 * CPU.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef CPU_H
#define CPU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  Verifies that critical loops are meeting timing constraints
 *         (e.g. IMU loop ≥1 kHz, attitude loop ≥200 Hz).
 * @return true if all timing constraints are currently met.
 */
bool CPU_CheckTimingConstraints(void);

/// Enable the CPU cycle counter for precise timing.
void CPU_EnableCycleCounter(void);
uint16_t CPU_GetLoopRateHz(void);
uint32_t CPU_GetLastLoopDtMs(void);
uint32_t CPU_GetMaxLoopDtMs(void);
uint32_t CPU_GetMissedDeadlineCount(void);

#endif // CPU_H
