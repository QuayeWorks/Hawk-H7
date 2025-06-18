#include "sonar.h"
#include "stm32h7xx_hal.h"
#include "main.h"  // for TRIGx pin definitions

#define TRIG_PULSE_US   10   // 10 µs pulse
#define MAX_DISTANCE_M  6.0f // 6 m max


// Map of trigger GPIO pins per sonar
static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} trigPins[SONAR_COUNT] = {
    { TRIG1_GPIO_Port, TRIG1_Pin }, // PF10
    { TRIG2_GPIO_Port, TRIG2_Pin }, // PF11
    { TRIG3_GPIO_Port, TRIG3_Pin }  // PF12
};

static uint32_t riseTime[SONAR_COUNT];
static float    lastDistance[SONAR_COUNT];

void Sonar_Init(void) {
    // Configure trigger pins as output (CubeMX should set them up)
    for (int i = 0; i < SONAR_COUNT; i++) {
        HAL_GPIO_WritePin(trigPins[i].port, trigPins[i].pin, GPIO_PIN_RESET);
    }
    // Clear last distances
    for (int i = 0; i < SONAR_COUNT; i++) {
        lastDistance[i] = MAX_DISTANCE_M;
    }
}

void Sonar_TriggerAll(void) {
    // Send 10µs high on all trigger pins
    for (int i = 0; i < SONAR_COUNT; i++) {
        HAL_GPIO_WritePin(trigPins[i].port, trigPins[i].pin, GPIO_PIN_SET);
    }
    // Busy‐wait 10µs
    uint32_t start = DWT->CYCCNT;
    uint32_t delayTicks = (HAL_RCC_GetHCLKFreq()/1000000) * TRIG_PULSE_US;
    while ((DWT->CYCCNT - start) < delayTicks);
    for (int i = 0; i < SONAR_COUNT; i++) {
        HAL_GPIO_WritePin(trigPins[i].port, trigPins[i].pin, GPIO_PIN_RESET);
    }
}

void Sonar_EchoCallback(uint8_t index, GPIO_PinState state) {
    if (index >= SONAR_COUNT) return;
    uint32_t now = DWT->CYCCNT;  // raw cycle counter
    if (state == GPIO_PIN_SET) {
        // Rising edge: record start
        riseTime[index] = now;
    } else {
        // Falling edge: compute pulse width and distance
        uint32_t width = now - riseTime[index];
        /*
         * Convert cycle width to seconds. DWT->CYCCNT increments at the
         * SystemCoreClock frequency, so dividing by this yields seconds.
         * Distance is then (time * speed_of_sound)/2.
         */
        float time_s = (float)width / (float)SystemCoreClock;
        float dist = (time_s * 343.0f) / 2.0f;
        if (dist > MAX_DISTANCE_M) dist = MAX_DISTANCE_M;
        lastDistance[index] = dist;
    }
}

float Sonar_ReadDistance(uint8_t index) {
    if (index < SONAR_COUNT) {
        return lastDistance[index];
    }
    return MAX_DISTANCE_M;
}
