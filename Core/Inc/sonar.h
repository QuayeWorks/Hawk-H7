/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

#ifndef SONAR_H
#define SONAR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
/*
 * Sonar Driver (HC-SR04 style)
 *
 * - Call Sonar_Init() once at startup.
 * - In HAL_GPIO_EXTI_Callback for each echo pin, call Sonar_EchoCallback(index, state).
 * - Call Sonar_TriggerAll() periodically (e.g. every 50ms) to send trigger pulses.
 * - Use Sonar_ReadDistance(index) to get the last measured distance (in meters).
 *   A return value < 0 indicates no echo was received (sensor missing or out
 *   of range).
 *
 *   Echo pins:   PF6, PF7, PF8
 *   Trigger pins: PF10, PF11, PF12
 */

#define SONAR_COUNT 3

// Initialize the sonar driver (configures trigger pins if needed)
void Sonar_Init(void);

// Must be called from HAL_GPIO_EXTI_Callback for PF6/7/8 (echo pins):
void Sonar_EchoCallback(uint8_t index, GPIO_PinState state);

// Trigger all sonars simultaneously (send 10µs pulse on each TRIG pin)
void Sonar_TriggerAll(void);

// Read the last computed distance (meters) for sonar index [0..SONAR_COUNT-1]
float Sonar_ReadDistance(uint8_t index);

#endif // SONAR_H
