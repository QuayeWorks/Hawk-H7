#include "sonar.h"
#include "stm32h7xx_hal.h"
#include "main.h"  // for TRIGx pin definitions

#define TRIG_PULSE_US    10   // 10 µs pulse
// Maximum measurable distance (meters)
#define MAX_DISTANCE_M   4.0f // 4 m max
#define SONAR_TIMEOUT_MS 40   // ~6 m round trip


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
static uint32_t trigTick[SONAR_COUNT];
static bool     gotEcho[SONAR_COUNT];
static float    lastDistance[SONAR_COUNT];

void Sonar_Init(void) {
    // Configure trigger pins as output (CubeMX should set them up)
    for (int i = 0; i < SONAR_COUNT; i++) {
        HAL_GPIO_WritePin(trigPins[i].port, trigPins[i].pin, GPIO_PIN_RESET);
    }
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < SONAR_COUNT; i++) {
        trigTick[i]    = now;
        gotEcho[i]     = false;
        lastDistance[i] = -1.0f; // no reading yet
    }
}

void Sonar_TriggerAll(void) {
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < SONAR_COUNT; i++) {
        // If previous trigger produced no echo within the timeout,
        // mark reading invalid
        if (!gotEcho[i] && (now - trigTick[i]) >= SONAR_TIMEOUT_MS) {
            lastDistance[i] = -1.0f;
        }
        // Prepare for this cycle
        gotEcho[i]  = false;
        trigTick[i] = now;
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
        gotEcho[index]     = true;
        lastDistance[index] = dist;
    }
}

float Sonar_ReadDistance(uint8_t index) {
    if (index < SONAR_COUNT) {
        float d = lastDistance[index];
        // When no echo was received, distance is negative. Report zero
        // to indicate an invalid or missing reading.
        if (d < 0.0f) {
            return 0.0f;
        }
        return d;
    }
    return 0.0f;
}
