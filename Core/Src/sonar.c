#include "sonar.h"
#include "stm32h7xx_hal.h"

#define TRIG_PULSE_US   10   // 10 µs pulse
#define MAX_DISTANCE_M  6.0f // 6 m max


// Map of trigger GPIO pins per sonar
static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} trigPins[SONAR_COUNT] = {
    { GPIOG, GPIO_PIN_6 }, // SG6
    { GPIOG, GPIO_PIN_7 }, // SG7
    { GPIOG, GPIO_PIN_8 }  // SG8
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
    uint32_t now = DWT->CYCCNT;  // microsecond resolution
    if (state == GPIO_PIN_SET) {
        // Rising edge: record start
        riseTime[index] = now;
    } else {
        // Falling edge: compute pulse width and distance
        uint32_t width = now - riseTime[index];
        // speed of sound ~343 m/s => 0.0343 cm/µs => distance = (width * 0.0343) / 2
        float dist = (width * 0.000343f) / 2.0f;
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
