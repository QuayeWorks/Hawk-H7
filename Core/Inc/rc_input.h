/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdint.h>
#include <stdbool.h>

/*
 * RC PPM Input Driver
 *
 * - Call RC_Input_Init() once at startup (EXTI and timer must already be configured).
 * - In HAL_GPIO_EXTI_Callback, call RC_Input_EXTI_Callback() on your PPM EXTI pin.
 * - Call RC_Input_Update() periodically (e.g. in your main loop or 1kHz tick) to track stale channels.
 * - Use RC_GetChannel(i) to fetch the last-measured pulse width (µs).
 * - Use RC_GetRSSI() and RC_AllChannelsStable() for link health checks.
 */

#define RC_MAX_CHANNELS    8
#define RC_SYNC_PULSE_US  3000   // gap > this indicates frame sync

// Initialize PPM input state. Should be called once after EXTI for PPM pin is enabled.
void RC_Input_Init(void);

// Must be called from HAL_GPIO_EXTI_Callback when the PPM EXTI pin toggles.
void RC_Input_EXTI_Callback(void);

// Should be called at least once per millisecond to update stale detection
void RC_Input_Update(uint32_t now_ms);

// Get the last pulse width (µs) for channel [0..RC_MAX_CHANNELS-1]
uint16_t RC_GetChannel(uint8_t chan);

// Returns true if no channel has gone stale for > stale_time_ms
bool RC_ChannelsAreStale(uint32_t stale_time_ms);

// Returns nonzero RSSI reading if your receiver provides one; else returns 0
uint16_t RC_GetRSSI(void);

// Returns true if all channels have been updated within the last 200µs of jitter threshold
bool RC_AllChannelsStable(void);

/// @brief  Returns true if the RC PPM channels have been lost for at least `seconds` seconds.
/// @param  seconds  How many seconds of lost signal to consider as link‐loss.
/// @return true if the link is lost, false otherwise.
bool RC_LinkLostForSeconds(uint16_t seconds);

#endif // RC_INPUT_H
