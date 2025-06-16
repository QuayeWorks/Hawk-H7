/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

// ================== flight_state.h ==================
#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdint.h>
#include <stdbool.h>

/*
   We’re going to keep a 32‐bit mask called fsState.  Each bit represents
   either a “health” condition or a “state” flag.  We’ll reserve lower bits
   for health, and a few upper bits for composite state (Ready, Armed, etc).
*/

// Health bits (bits 0–7).  These must all be set before we become “Ready”:
#define FS_HEALTH_IMU_OK_BIT          (1u << 0)   // 0: IMU biases within tolerances
#define FS_HEALTH_COMPASS_OK_BIT      (1u << 1)   // 1: Magnetometer passes Mahalanobis test
#define FS_HEALTH_BARO_OK_BIT         (1u << 2)   // 2: Baro pressure within ±10hPa
#define FS_HEALTH_SONAR_OK_BIT        (1u << 3)   // 3: Sonar reading is plausible (if used)
#define FS_HEALTH_GPS_OK_BIT          (1u << 4)   // 4: GPS fix ≥ min satellites, HDOP ≤ threshold
#define FS_HEALTH_RC_OK_BIT           (1u << 5)   // 5: RSSI ok, no stale/jittery channels, throttle low
#define FS_HEALTH_BATT_OK_BIT         (1u << 6)   // 6: Per‐cell voltage > min and under‐load‐measured OK
#define FS_HEALTH_EKF_OK_BIT          (1u << 7)   // 7: EKF innovations all within gates / covariance OK

// Composite masks for convenience:
#define FS_ALL_HEALTH_BITS  (FS_HEALTH_IMU_OK_BIT   \
                           |FS_HEALTH_COMPASS_OK_BIT \
                           |FS_HEALTH_BARO_OK_BIT    \
                           |FS_HEALTH_SONAR_OK_BIT   \
                           |FS_HEALTH_GPS_OK_BIT     \
                           |FS_HEALTH_RC_OK_BIT      \
                           |FS_HEALTH_BATT_OK_BIT    \
                           |FS_HEALTH_EKF_OK_BIT)

// “State” bits (≥ bit 8).  These are set only when all pre‐conditions are satisfied.
#define FS_READY_BIT                 (1u << 8)   // 8: All health bits are OK → ready to arm
#define FS_ARMED_BIT                 (1u << 9)   // 9: Pilots pulled sticks to arm while READY=1
#define FS_BATTERY_OK_BIT            (1u << 10)  // 10: Battery measured as “OK under load”

/*
  In the future, you can add more “Mode” bits here, e.g. GPS‐Hold, AltHold, RTL flagged, etc.
*/

// The single 32‐bit container for all health+state flags (defined in .c)
extern uint32_t fsState;

// ——— Public API ———
// Set/clear individual health bits (called from main sensor checks)
void FlightState_SetHealth(uint32_t bit);
void FlightState_ClearHealth(uint32_t bit);

// Set/clear the “Ready” and “Armed” bits (usually only FlightState_Update()
// or FlightState_Arm/Disarm will do this, not user code directly):
void FlightState_SetReady(void);
void FlightState_ClearReady(void);
void FlightState_Arm(void);
void FlightState_Disarm(void);

// Battery “OK under load” bit.  (Main battery loop should set/clear this.)
void FlightState_SetBatteryOK(void);
void FlightState_ClearBatteryOK(void);

// Composite query functions
bool FlightState_IsReady(void);        // true if (fsState & FS_READY_BIT) != 0
bool FlightState_IsArmed(void);        // true if (fsState & FS_ARMED_BIT) != 0
bool FlightState_IsBatteryOK(void);    // true if (fsState & FS_BATTERY_OK_BIT) != 0
bool FlightState_AllHealthOK(void);    // true if (fsState & FS_ALL_HEALTH_BITS) == FS_ALL_HEALTH_BITS
uint32_t FlightState_GetStateMask(void);

// Called periodically (or whenever a health bit changes) to auto‐toggle READY/ARMED:
void FlightState_Update(void);

// Immediately begins an automatic landing sequence.
// Stub implementation; replace with your actual RTL/Land logic.
void CommenceAutoLand(void);
void CommenceAltHold(void);

#endif // FLIGHT_STATE_H
