/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

// ================== power.c ==================
#include "power.h"
#include "flight_state.h"
#include "settings.h"

static ADC_HandleTypeDef *hadcVolt;
//static ADC_HandleTypeDef *hadcCurr;

// Calibration constants (adjust for your divider and shunt)
#define VOLTAGE_DIVIDER_RATIO 11.0f
#define ADC_REF_VOLTAGE       3.3f
#define ADC_MAX_VALUE         4095.0f
#define SHUNT_RESISTANCE      0.01f  // ohms
#define CURRENT_AMP_PER_COUNT  (ADC_REF_VOLTAGE / SHUNT_RESISTANCE / ADC_MAX_VALUE)

void Power_Init(void) {

	/* Initially Enable I2C for BMP388 using dedicated GPIO pin (ensure these match your schematic) */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Enable BMP388 I2C

	/* Initially Enable sensors using dedicated GPIO pins (ensure these match your schematic) */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Enable MPU6050
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);  // Enable BMP388
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);  // Enable QMC5883L
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // Enable HC-05

	/* Initially Disable the buck converters using dedicated GPIO pins (ensure these match your schematic) */
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);  // buckID 0: OFF
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); // buckID 1: OFF
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET); // buckID 2: OFF
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET); // buckID 3: OFF

}

float Power_GetPackVoltage(ADC_HandleTypeDef *hadcVoltage) {
    // Start ADC in interrupt or polling mode as desired
	HAL_ADC_Start(hadcVolt);
	HAL_ADC_PollForConversion(hadcVolt, HAL_MAX_DELAY);
	uint32_t raw = HAL_ADC_GetValue(hadcVolt);
	HAL_ADC_Stop(hadcVolt);

	// convert the single ADC sample to volts using the divider
	return ((float)raw) * Settings_GetBatteryVoltDivider();
}

float Power_GetCellVoltage(void) {
    uint8_t cells = Settings_GetBatteryCellCount();
    float packV = Power_GetPackVoltage(hadcVolt);
    return packV / (float)cells;
}

/*float Power_GetCurrentDraw(void) {
    HAL_ADC_PollForConversion(hadcCurr, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(hadcCurr);
    float amps = raw * CURRENT_AMP_PER_COUNT;
    return amps;
}*/

bool Power_IsVoltageOK(float thresholdPerCell, uint8_t cellCount) {
    float cellV = Power_GetCellVoltage();
    bool ok = (cellV >= thresholdPerCell);
    if (ok) {
        FlightState_SetBatteryOK();
    } else {
        FlightState_SetBatteryOK();
        FlightState_SetHealth(FS_HEALTH_BATT_OK_BIT);
    }
    return ok;
}

void Power_SetBuckEnable(uint8_t buckID, uint8_t state) {
    // Example mapping:
    // buckID 0 => PI0, 1 => PG13, 2 => PG14, 3 => PG15
    GPIO_TypeDef* port;
    uint16_t pin;
    switch(buckID) {
        case 0: port = GPIOI; pin = GPIO_PIN_0; break;
        case 1: port = GPIOG; pin = GPIO_PIN_13; break;
        case 2: port = GPIOG; pin = GPIO_PIN_14; break;
        case 3: port = GPIOG; pin = GPIO_PIN_15; break;
        default: return;
    }
    HAL_GPIO_WritePin(port, pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}
