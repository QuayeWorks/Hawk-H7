/*
 * CPU.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef CPU_H
#define CPU_H

#include <stdbool.h>

/**
 * @brief  Verifies that critical loops are meeting timing constraints
 *         (e.g. IMU loop ≥1 kHz, attitude loop ≥200 Hz).
 * @return true if all timing constraints are currently met.
 */
bool CPU_CheckTimingConstraints(void);

/// Enable the CPU cycle counter for precise timing.
void CPU_EnableCycleCounter(void);

#endif // CPU_H
