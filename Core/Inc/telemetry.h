/*
 * telemetry.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "attitude.h"   // for AttitudeAngles

/**
 * @brief  Send a bitmask of health/failsafe flags to the ground station.
 */
void Telemetry_SendHealth(uint32_t stateMask);

/**
 * @brief  Send the current attitude (roll, pitch, yaw) to the ground station.
 */
void Telemetry_SendAttitude(AttitudeAngles angles);



#endif // TELEMETRY_H
