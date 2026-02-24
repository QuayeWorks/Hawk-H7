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
#include "main.h"
#include "settings.h"
#include "stm32h7xx_hal.h"
#include <string.h>    // for memset
#include <stdlib.h>    // for abs()

#define RC_MIN_PULSE_US                 750u
#define RC_MAX_PULSE_US                 2250u
#define RC_MIN_FRAME_CHANNELS           4u
#define RC_PPM_LOSS_TIMEOUT_MS          250u
#define RC_PWM_SINGLE_FRESH_MS          80u

static volatile uint8_t  ppmChannel;
static volatile uint32_t sampleEdgeUs; // timestamp of last sampled edge (us)
static volatile uint16_t channelWidths[RC_MAX_CHANNELS];
static volatile uint32_t channelTimestamps[RC_MAX_CHANNELS]; // in ms for stale check
static volatile uint16_t rssiValue;
static volatile uint32_t frameCount;
static volatile uint32_t lastFrameMs;

static volatile uint32_t edgeCount;
static volatile uint32_t syncCount;
static volatile uint32_t goodPulseCount;
static volatile uint32_t rejectCount;
static volatile uint16_t lastIntervalUs;
static volatile uint16_t lastPwmHighUs;
static volatile uint32_t lastEdgeMs;
static volatile uint8_t sampledHighEdge;
static volatile RC_SignalType signalType;

static volatile uint8_t sampleOnHighEdge;
static volatile uint32_t pwmRiseUs;
static volatile uint8_t pwmRiseSeen;
static volatile uint16_t frameScratch[RC_MAX_CHANNELS];

// Helper to get microseconds since boot (uses DWT cycle counter)
static inline uint32_t micros(void)
{
    // DWT->CYCCNT increments at the core clock frequency. Using SystemCoreClock
    // avoids errors if HCLK differs from the CPU speed.
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static inline uint16_t clamp_u16(uint32_t v)
{
    return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
}

static void rc_commit_frame(uint32_t now_ms)
{
    uint8_t i;

    if (ppmChannel < RC_MIN_FRAME_CHANNELS) {
        ppmChannel = 0u;
        return;
    }

    for (i = 0u; i < ppmChannel && i < RC_MAX_CHANNELS; i++) {
        channelWidths[i] = frameScratch[i];
        channelTimestamps[i] = now_ms;
    }

    frameCount++;
    lastFrameMs = now_ms;
    signalType = RC_SIGNAL_PPM;
    ppmChannel = 0u;
}

void RC_Input_Init(void)
{
    memset((void*)channelWidths, 0, sizeof(channelWidths));
    memset((void*)channelTimestamps, 0, sizeof(channelTimestamps));
    memset((void*)frameScratch, 0, sizeof(frameScratch));

    ppmChannel = 0u;
    sampleEdgeUs = 0u;
    rssiValue = 0u;
    frameCount = 0u;
    lastFrameMs = 0u;

    edgeCount = 0u;
    syncCount = 0u;
    goodPulseCount = 0u;
    rejectCount = 0u;
    lastIntervalUs = 0u;
    lastPwmHighUs = 0u;
    lastEdgeMs = 0u;
    sampledHighEdge = 1u;
    signalType = RC_SIGNAL_NONE;

    sampleOnHighEdge = Settings_GetRCInvertPWM() ? 0u : 1u;
    pwmRiseUs = 0u;
    pwmRiseSeen = 0u;

    // If your receiver has RSSI on an ADC channel, start ADC here
    // HAL_ADC_Start_DMA(&hadc1, &rssiValue, 1);
}

void RC_Input_EXTI_Callback(void)
{
    uint32_t now_us = micros();
    uint32_t now_ms = HAL_GetTick();
    GPIO_PinState level = HAL_GPIO_ReadPin(PPM_GPIO_Port, PPM_Pin);
    uint8_t pinHigh = (level == GPIO_PIN_SET) ? 1u : 0u;

    edgeCount++;
    lastEdgeMs = now_ms;
    sampledHighEdge = pinHigh;

    // Optional single-channel PWM fallback detector (for wiring/config mistakes).
    if (pinHigh != 0u) {
        pwmRiseUs = now_us;
        pwmRiseSeen = 1u;
    } else if (pwmRiseSeen != 0u) {
        uint32_t high_us = now_us - pwmRiseUs;
        pwmRiseSeen = 0u;
        if (high_us >= RC_MIN_PULSE_US && high_us <= RC_MAX_PULSE_US) {
            lastPwmHighUs = clamp_u16(high_us);
            // Only publish PWM fallback when no recent valid PPM frame exists.
            if ((frameCount == 0u) || ((now_ms - lastFrameMs) > RC_PPM_LOSS_TIMEOUT_MS)) {
                channelWidths[0] = (uint16_t)high_us;
                channelTimestamps[0] = now_ms;
                signalType = RC_SIGNAL_PWM1;
            }
        }
    }

    if ((sampleOnHighEdge != 0u) != (pinHigh != 0u)) {
        return;
    }

    if (sampleEdgeUs == 0u) {
        sampleEdgeUs = now_us;
        return;
    }

    {
        uint32_t width = now_us - sampleEdgeUs;
        sampleEdgeUs = now_us;
        lastIntervalUs = clamp_u16(width);

        if (width > RC_SYNC_PULSE_US) {
            syncCount++;
            rc_commit_frame(now_ms);
            return;
        }

        if (width < RC_MIN_PULSE_US || width > RC_MAX_PULSE_US) {
            rejectCount++;
            if ((frameCount == 0u) || ((now_ms - lastFrameMs) > RC_PPM_LOSS_TIMEOUT_MS)) {
                signalType = RC_SIGNAL_NOISY;
            }
            return;
        }

        goodPulseCount++;

        if (ppmChannel < RC_MAX_CHANNELS) {
            frameScratch[ppmChannel++] = (uint16_t)width;
        } else {
            // Receiver may output > RC_MAX_CHANNELS channels.
            // Keep the first RC_MAX_CHANNELS and ignore extras until next sync.
            // No action needed here.
        }
    }
}

void RC_Input_Update(uint32_t now_ms)
{
    if ((frameCount > 0u) && ((now_ms - lastFrameMs) <= RC_PPM_LOSS_TIMEOUT_MS)) {
        signalType = RC_SIGNAL_PPM;
    } else if ((channelTimestamps[0] > 0u) &&
               ((now_ms - channelTimestamps[0]) <= RC_PWM_SINGLE_FRESH_MS)) {
        signalType = RC_SIGNAL_PWM1;
    } else if ((lastEdgeMs > 0u) && ((now_ms - lastEdgeMs) <= RC_PWM_SINGLE_FRESH_MS)) {
        signalType = RC_SIGNAL_NOISY;
    } else {
        signalType = RC_SIGNAL_NONE;
    }

    // Optionally read RSSI from ADC or UART RSSI pin here
    // rssiValue = HAL_ADC_GetValue(&hadc1) >> 4; // 0..4095 -> 0..255
}

uint16_t RC_GetChannel(uint8_t chan)
{
    if (chan < RC_MAX_CHANNELS) {
        return channelWidths[chan];
    }
    return 0u;
}

bool RC_ChannelsAreStale(uint32_t stale_time_ms)
{
    uint32_t now = HAL_GetTick();

    if (frameCount > 0u) {
        return ((now - lastFrameMs) > stale_time_ms);
    }

    // Fallback before first valid PPM frame: consider channel 0 freshness.
    return ((now - channelTimestamps[0]) > stale_time_ms);
}

uint16_t RC_GetRSSI(void)
{
    return rssiValue;
}

bool RC_AllChannelsStable(void)
{
    // If any channel width jumps > 500us between calls, treat as unstable.
    static uint16_t lastWidths[RC_MAX_CHANNELS] = {0u};
    bool stable = true;
    uint8_t i;

    for (i = 0u; i < RC_MAX_CHANNELS; i++) {
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

    if (frameCount > 0u) {
        return ((now - lastFrameMs) >= threshold);
    }

    return ((now - channelTimestamps[0]) >= threshold);
}

uint32_t RC_GetFrameCount(void)
{
    return frameCount;
}

uint32_t RC_GetLastFrameMs(void)
{
    return lastFrameMs;
}

void RC_GetDiagnostics(RC_Diagnostics *out)
{
    if (out == NULL) {
        return;
    }

    __disable_irq();
    out->edgeCount = edgeCount;
    out->syncCount = syncCount;
    out->goodPulseCount = goodPulseCount;
    out->rejectCount = rejectCount;
    out->lastIntervalUs = lastIntervalUs;
    out->lastPwmHighUs = lastPwmHighUs;
    out->lastEdgeMs = lastEdgeMs;
    out->sampledHighEdge = sampledHighEdge;
    out->signalType = signalType;
    __enable_irq();
}
