/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

#include "rc_input.h"
#include "stm32h7xx_hal.h"
#include <string.h>    // for memset
#include <string.h>    // for memset
#include <stdlib.h>    // for abs()

static uint32_t lastSYNC;
static uint8_t  ppmChannel;
static uint32_t riseTime;            // timestamp of last rising edge (µs)
static uint16_t channelWidths[RC_MAX_CHANNELS];
static uint32_t channelTimestamps[RC_MAX_CHANNELS]; // in ms for stale check
static uint16_t rssiValue;

// Helper to get microseconds since boot (uses DWT cycle counter)
static inline uint32_t micros(void) {
    // Ensure DWT is enabled elsewhere; here we assume DWT->CYCCNT increments
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}

void RC_Input_Init(void) {
    memset(channelWidths, 0, sizeof(channelWidths));
    memset(channelTimestamps, 0, sizeof(channelTimestamps));
    ppmChannel  = 0;
    lastSYNC    = 0;
    riseTime    = 0;
    rssiValue   = 0;
    // If your receiver has RSSI on an ADC channel, start ADC here
    // HAL_ADC_Start_DMA(&hadc1, &rssiValue, 1);
}

void RC_Input_EXTI_Callback(void) {
    uint32_t now = micros();
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9); // Adjust port/pin as needed

    if (state == GPIO_PIN_SET) {
        // Rising edge: record time
        riseTime = now;
    } else {
        // Falling edge: compute pulse width
        uint32_t width = now - riseTime;
        if (width > RC_SYNC_PULSE_US) {
            // Sync pause → reset channel index
            ppmChannel = 0;
        } else {
            if (ppmChannel < RC_MAX_CHANNELS) {
                channelWidths[ppmChannel]     = (uint16_t)width;
                channelTimestamps[ppmChannel] = HAL_GetTick();
                ppmChannel++;
            }
        }
    }
}

void RC_Input_Update(uint32_t now_ms) {
    (void)now_ms;
    // Optionally read RSSI from ADC or UART RSSI pin here
    // rssiValue = HAL_ADC_GetValue(&hadc1) >> 4; // 0..4095 → 0..255
}

uint16_t RC_GetChannel(uint8_t chan) {
    if (chan < RC_MAX_CHANNELS) {
        return channelWidths[chan];
    }
    return 0;
}

bool RC_ChannelsAreStale(uint32_t stale_time_ms) {
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        if ((now - channelTimestamps[i]) > stale_time_ms) {
            return true;
        }
    }
    return false;
}

uint16_t RC_GetRSSI(void) {
    return rssiValue;
}

bool RC_AllChannelsStable(void) {
    // If any channel width jumps > 500µs between calls, treat as unstable
    static uint16_t lastWidths[RC_MAX_CHANNELS] = {0};
    bool stable = true;
    for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        if (abs((int)channelWidths[i] - (int)lastWidths[i]) > 500) {
            stable = false;
        }
        lastWidths[i] = channelWidths[i];
    }
    return stable;
}

bool RC_LinkLostForSeconds(uint16_t seconds)
{
    uint32_t now = HAL_GetTick();
    uint32_t threshold = (uint32_t)seconds * 1000u;
    for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        if ((now - channelTimestamps[i]) < threshold) {
            return false;
        }
    }
    return true;
}
