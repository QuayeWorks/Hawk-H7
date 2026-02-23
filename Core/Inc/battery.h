/*
 * battery.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/*
 * Battery & Power Management Driver
 *
 * - Call Battery_Init(&hi2c, &hadc, shunt_ohm) once at startup.
 * - Regularly call Battery_Tick(now_ms) to integrate capacity.
 * - Use Battery_ReadPackVoltage() and Battery_ReadCurrent() to get live readings.
 * - Use Battery_GetRemaining_mAh() or Battery_GetRemainingPercent() for capacity.
 * - Use Battery_GetCellVoltage() or Battery_GetPerCellVoltage() for per‐cell checks.
 */

/// Initialize battery monitoring.
///   • hi2c: handle for I2C to INA219.
///   • hadc: ADC handle for battery voltage measurement (e.g. PH2 channel).
///   • shuntOhm: value of the external shunt resistor in ohms.
void       Battery_Init(I2C_HandleTypeDef *hi2c,
                        ADC_HandleTypeDef *hadc,
                        float shuntOhm);

/// Must be called periodically (e.g. in your 1 kHz tick or main loop).
///   • now_ms: HAL_GetTick() current time in ms.
void       Battery_Tick(uint32_t now_ms);

/// Read the pack voltage (V), accounting for the hardware voltage divider from settings.
float      Battery_ReadPackVoltage(void);

/// Read total current (A) summed across detected INA219 sensors.
float      Battery_ReadCurrent(void);

/// Read per-cell voltage (V).
float      Battery_ReadPerCellVoltage(void);

/// Get cumulative consumed capacity (mAh) since init (or reset).
float      Battery_GetConsumed_mAh(void);

/// Get remaining capacity (mAh) (total pack capacity – consumed).
float      Battery_GetRemaining_mAh(void);

/// Get remaining capacity as a percentage (0–100%).
float      Battery_GetRemainingPercent(void);
uint32_t   Battery_GetI2CErrorCount(void);
uint32_t   Battery_GetADCTimeoutCount(void);
uint32_t   Battery_GetCurrentSampleCount(void);
uint32_t   Battery_GetVoltageSampleCount(void);
uint32_t   Battery_GetLastCurrentMs(void);
uint32_t   Battery_GetLastVoltageMs(void);
uint8_t    Battery_GetI2CAddress7bit(void);
HAL_StatusTypeDef Battery_GetLastI2CStatus(void);
HAL_StatusTypeDef Battery_GetLastADCStatus(void);

// Multi-INA219 helpers (for per-rail telemetry/diagnostics).
uint8_t    Battery_GetINA219Count(void);
uint8_t    Battery_GetINA219Address7bitAt(uint8_t index);
float      Battery_ReadCurrentAt(uint8_t index);
float      Battery_GetLastCurrentAt(uint8_t index);
HAL_StatusTypeDef Battery_GetLastI2CStatusAt(uint8_t index);

#endif // BATTERY_H
