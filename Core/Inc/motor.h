/*
 * motor.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

// ================== motor_control.h ==================
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>    // at the very top of the file

// Initialize motor PWM outputs on TIM1 (motors 1–4) and TIM3 (motors 5–8).
// Must be called once at startup, after HAL and timer inits.
void Motor_Init(void);

// Set PWM pulse width in microseconds on motor [0..7].
// If index >= Settings_GetMotorCount(), it does nothing.
void Motor_SetPWM(uint8_t index, uint16_t pulse_us);

// Set the same PWM pulse to all motors that are enabled.
void Motor_SetAllPWM(uint16_t pulse_us);

// Map an RC throttle PWM (1000–2000 µs) to ESC PWM endpoints from settings.
// Returns a pulse in the range [motor_pwm_min_us, motor_pwm_max_us].
uint16_t Motor_ThrottleToPWM(uint16_t throttle_ppm);

/// @brief  Stub: return true if ESC i responded to the spin‐up pulse.
/// @note   Replace this with a real RPM or current‐draw check if you have the hardware.
bool Motor_EscResponds(uint8_t index);

/// @brief  (Stub) Auto-detect motor spin direction by pulsing each ESC and sensing back-EMF or RPM.
/// @note   Replace with real sensor feedback code when available.
void Motor_AutoDetectOrientation(void);

#endif // MOTOR_CONTROL_H
