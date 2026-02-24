/*
 * telemetry.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "attitude.h"   // for AttitudeAngles

/**
 * @brief  Send a bitmask of health/failsafe flags to the ground station.
 */
void Telemetry_SendHealth(uint32_t stateMask);

/**
 * @brief  Send the current attitude (roll, pitch, yaw) to the ground station.
 */
void Telemetry_SendAttitude(AttitudeAngles angles);

/**
 * @brief Send a raw text line over telemetry UART.
 *        Appends "\r\n" if not already present.
 * @return true on successful UART transmit.
 */
bool Telemetry_SendRawLine(const char *line);

/**
 * @brief Telemetry TX diagnostics for debug menu/status.
 */
uint32_t Telemetry_GetTxOkCount(void);
uint32_t Telemetry_GetTxFailCount(void);
uint32_t Telemetry_GetLastTxMs(void);
uint16_t Telemetry_GetLastTxLen(void);
int32_t  Telemetry_GetLastHalStatusCode(void);
uint32_t Telemetry_GetLastUartError(void);


#endif // TELEMETRY_H
