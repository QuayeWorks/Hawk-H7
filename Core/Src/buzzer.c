/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

// ================== buzzer.c ==================
#include "buzzer.h"
#include <string.h>

// Max steps in a tone pattern
#define MAX_BUZZ_STEPS    8

// One step: frequency in Hz and duration in ms
typedef struct {
    uint32_t freq_hz;
    uint16_t duration_ms;
} BuzzStep;

// Each tone is a sequence of steps, terminated by {0,0}
static const BuzzStep tonePatterns[TONE_COUNT][MAX_BUZZ_STEPS] = {
    [TONE_NONE] = {
        {    0,    0 }
    },

    // System states
    [TONE_READY] = {  // triple short beeps at 1000 Hz
        {1000, 100}, {   0, 50},
        {1000, 100}, {   0, 50},
        {1000, 100}, {   0, 0}
    },
    [TONE_ARMED] = {  // two short beeps at 1200 Hz
        {1200, 100}, {   0, 50},
        {1200, 100}, {   0, 0}
    },
    [TONE_DISARMED] = { // single short beep at 800 Hz
        { 800, 100}, {   0, 0}
    },

    // Errors (single long tone)
    [TONE_ERROR_IMU]      = {{ 500, 500}, {   0,   0}},
    [TONE_ERROR_COMPASS]  = {{ 550, 500}, {   0,   0}},
    [TONE_ERROR_BARO]     = {{ 600, 500}, {   0,   0}},
    [TONE_ERROR_SONAR]    = {{ 650, 500}, {   0,   0}},
    [TONE_ERROR_GPS]      = {{ 700, 500}, {   0,   0}},
    [TONE_ERROR_RC]       = {{ 750, 500}, {   0,   0}},
    [TONE_ERROR_BATTERY]  = {{ 800, 500}, {   0,   0}},
    [TONE_ERROR_EKF]      = {{ 850, 500}, {   0,   0}},
    [TONE_ERROR_CPU]      = {{ 900, 500}, {   0,   0}},
    [TONE_ERROR_MOTOR]    = {{ 950, 500}, {   0,   0}},

    // Warnings (two medium‐length beeps)
    [TONE_WARN_BATT_LOW]   = {{1000, 200}, {   0, 50}, {1000, 200}, {0,0}},
    [TONE_WARN_BATT_CRIT]  = {{ 500, 400}, {   0, 50}, { 500, 400}, {0,0}},
    [TONE_WARN_ATTITUDE]   = {{ 600, 200}, {   0, 50}, { 600, 200}, {0,0}},
    [TONE_WARN_GEOFENCE]   = {{ 700, 200}, {   0, 50}, { 700, 200}, {0,0}},
    [TONE_WARN_RCLINK]     = {{ 800, 200}, {   0, 50}, { 800, 200}, {0,0}},
    [TONE_WARN_THERMAL]    = {{ 900, 200}, {   0, 50}, { 900, 200}, {0,0}},
};

// Internal state
static Buzzer_Config buzzerCfg;
static bool          buzzerActive;
static const BuzzStep *currentPattern;
static uint8_t       currentStep;
static uint32_t      stepTimeLeft;  // in ms

// Helper to start a PWM @ given frequency
static void start_pwm(uint32_t freq_hz) {
    if (buzzerCfg.htim != NULL && buzzerCfg.htim->Instance != NULL) {
        // Assuming timer clock = 100 MHz. Adjust prescaler so that
        // ARR+1 = timer_clock / freq_hz.
        uint32_t timerCLK = HAL_RCC_GetPCLK1Freq();  // or PCLK2 depending on TIM
        uint32_t period = (timerCLK / freq_hz) - 1;
        __HAL_TIM_SET_AUTORELOAD(buzzerCfg.htim, period);
        __HAL_TIM_SET_COMPARE(buzzerCfg.htim, buzzerCfg.channel, period/2);
        HAL_TIM_PWM_Start(buzzerCfg.htim, buzzerCfg.channel);
    } else {
        // Fallback: manual GPIO toggle in Tick()
    }
}

// Helper to stop PWM
static void stop_pwm(void) {
    if (buzzerCfg.htim != NULL && buzzerCfg.htim->Instance != NULL) {
        HAL_TIM_PWM_Stop(buzzerCfg.htim, buzzerCfg.channel);
    }
    if (buzzerCfg.port != NULL) {
        HAL_GPIO_WritePin(buzzerCfg.port, buzzerCfg.pin, GPIO_PIN_RESET);
    }
}

// Initialize the buzzer driver
void Buzzer_Init(Buzzer_Config *cfg) {
    memcpy(&buzzerCfg, cfg, sizeof(buzzerCfg));
    buzzerActive    = false;
    currentPattern  = tonePatterns[TONE_NONE];
    currentStep     = 0;
    stepTimeLeft    = 0;
    stop_pwm();
}

// Play a named tone (resets pattern and starts it)
void Buzzer_PlayTone(Buzzer_ToneID tone) {
    if (tone >= TONE_COUNT) tone = TONE_NONE;
    currentPattern = tonePatterns[tone];
    currentStep    = 0;
    buzzerActive   = true;
    // Immediately start the first step:
    if (currentPattern[0].freq_hz) {
        start_pwm(currentPattern[0].freq_hz);
    } else {
        stop_pwm();
    }
    stepTimeLeft = currentPattern[0].duration_ms;
}

// Immediately silence buzzer
void Buzzer_Stop(void) {
    buzzerActive = false;
    stop_pwm();
}

// Must be called ~1 kHz (every 1 ms) to advance the pattern
void Buzzer_Tick(void) {
    if (!buzzerActive) return;
    if (stepTimeLeft > 0) {
        --stepTimeLeft;
        return;
    }
    // Advance to next step
    ++currentStep;
    if (currentPattern[currentStep].duration_ms == 0) {
        // End of pattern
        buzzerActive = false;
        stop_pwm();
        return;
    }
    // Start next step
    if (currentPattern[currentStep].freq_hz) {
        start_pwm(currentPattern[currentStep].freq_hz);
    } else {
        stop_pwm();
    }
    stepTimeLeft = currentPattern[currentStep].duration_ms;
}
