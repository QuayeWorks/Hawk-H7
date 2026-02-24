/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

// ================== buzzer.h ==================
// Non‐blocking buzzer driver using TIM PWM
// -----------------------------------------
// Usage:
//   • Call Buzzer_Init(&htimX, TIM_CHANNEL_Y, GPIOx, GPIO_PIN_Z) once at startup.
//   • Call Buzzer_PlayTone(TONE_ARMED) or any other tone ID from anywhere.
//   • In your main loop (or better, in your 1 kHz systick handler), call Buzzer_Tick()
//       to advance the buzzer state machine.
//   • Call Buzzer_Stop() to silence immediately.
// -----------------------------------------

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

// All the tones we'll use in the flight controller
typedef enum {
    TONE_NONE = 0,

    // System states
    TONE_READY,      // power-on & pre-arm OK
    TONE_ARMED,      // armed
    TONE_DISARMED,   // disarmed

    // Errors / warnings
    TONE_ERROR_IMU,
    TONE_ERROR_COMPASS,
    TONE_ERROR_BARO,
    TONE_ERROR_SONAR,
    TONE_ERROR_GPS,
    TONE_ERROR_RC,
    TONE_ERROR_BATTERY,
    TONE_ERROR_EKF,
    TONE_ERROR_CPU,
    TONE_ERROR_MOTOR,

    TONE_WARN_BATT_LOW,
    TONE_WARN_BATT_CRIT,
    TONE_WARN_ATTITUDE,
    TONE_WARN_GEOFENCE,
    TONE_WARN_RCLINK,
    TONE_WARN_THERMAL,

    TONE_COUNT
} Buzzer_ToneID;

// Buzzer configuration struct
typedef struct {
    TIM_HandleTypeDef *htim;   // timer handle for PWM
    uint32_t           channel; // TIM_CHANNEL_x
    GPIO_TypeDef      *port;   // optional GPIO port for manual toggle (if PWM not used)
    uint16_t           pin;    // optional GPIO pin
} Buzzer_Config;

// Initialize the buzzer driver.
//   • htim + channel must be configured for PWM mode at some base frequency (see note).
//   • If you prefer GPIO toggle, set htim = NULL and specify port+pin.
void     Buzzer_Init(Buzzer_Config *cfg);

// Play a named tone (overrides any current tone).
// Plays one “pattern” sequence (see buzzer.c for durations).
void     Buzzer_PlayTone(Buzzer_ToneID tone);

// Immediately stop any tone output
void     Buzzer_Stop(void);

// Call periodically (best at 1 kHz) to run the buzzer state machine.
// For example, call in your SysTick handler or main loop.
void     Buzzer_Tick(void);

#endif // BUZZER_H
