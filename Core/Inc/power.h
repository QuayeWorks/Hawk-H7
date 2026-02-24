/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

// ================== power.h ==================
#ifndef POWER_H
#define POWER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

// Initialize ADC for battery voltage and current sensing
void Power_Init(void);

// Read total pack voltage in volts
float Power_GetPackVoltage(ADC_HandleTypeDef *hadcVoltage);

// Read per-cell voltage assuming known cell count
float Power_GetCellVoltage(void);

// Read current draw in amperes
float Power_GetCurrentDraw(void);

// Check if pack voltage is above a threshold (e.g., 3.5 V per cell)
bool Power_IsVoltageOK(float thresholdPerCell, uint8_t cellCount);

// Control buck converter enables; buckID from 0 to 3 and state 0 (off) or 1 (on)
void Power_SetBuckEnable(uint8_t buckID, uint8_t state);
#endif // POWER_H
