/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

// Initialize GPS driver on the given UART handle (uses Rx IT).
// Must be called once after HAL and clocks are up.
void GPS_Init(UART_HandleTypeDef *huart);

// Called internally when a full NMEA line is received.
// You do not normally call this yourself.
void GPS_ProcessSentence(const char *sentence);

// Called periodically (e.g. in main loop or 1kHz tick) to handle any
// time-based tasks (drift, timeouts). Can be a no-op if unused.
void GPS_Update(void);

/// Returns true if the current position has not drifted by more than X meters
/// over the last holdSeconds, false otherwise.
bool GPS_CheckDriftOK(uint16_t holdSeconds);


// Accessors for flight-state checks:
bool   GPS_HasFix(void);
uint8_t GPS_GetSatCount(void);
float  GPS_GetHDOP(void);
double GPS_GetLatitude(void);
double GPS_GetLongitude(void);
float  GPS_GetAltitude(void);
uint32_t GPS_GetUpdateCount(void);
uint32_t GPS_GetLastUpdateMs(void);
uint16_t GPS_GetSentenceRateHz(void);
uint32_t GPS_GetDroppedByteCount(void);
uint16_t GPS_GetRxRingLevel(void);

#endif // GPS_H
