/*
 * battery.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#include "battery.h"
#include "settings.h"
#include <string.h>

// INA219 I²C address (7-bit) from cubeMX config or default
#ifndef INA219_I2C_ADDR
#define INA219_I2C_ADDR (0x40 << 1)
#endif

// INA219 registers
#define INA219_REG_SHUNTVOLTAGE  0x01
#define INA219_REG_BUSVOLTAGE    0x02

static I2C_HandleTypeDef *hi2cBatt;
static ADC_HandleTypeDef *hadcBatt = NULL;
static float shuntResistance;
static float voltageDivider;
static uint8_t cellCount;
static float totalCapacity_mAh;

static float consumed_mAh = 0.0f;
static uint32_t lastTickMs = 0;

#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_VALUE   4095.0f

/// Raw reading helpers
static bool INA219_ReadRegister(uint8_t reg, uint16_t *out) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2cBatt, INA219_I2C_ADDR, reg, 1, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

void Battery_Init(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, float shuntOhm) {
    hi2cBatt        = hi2c;
    hadcBatt        = hadc;
    shuntResistance = shuntOhm;
    voltageDivider  = Settings_GetBatteryVoltDivider();
    cellCount       = Settings_GetBatteryCellCount();
    totalCapacity_mAh = (float)Settings_GetBatteryLogmAh();
    consumed_mAh    = 0.0f;
    lastTickMs      = HAL_GetTick();
}

void Battery_Tick(uint32_t now_ms) {
    // integrate capacity: ΔmAh = I_A * (dt_ms / 3600)
    uint32_t dt = now_ms - lastTickMs;
    lastTickMs = now_ms;
    float currentA = Battery_ReadCurrent();
    // consumed_mAh += currentA * dt / 3600.0f;
    consumed_mAh += currentA * ((float)dt / 3600.0f);
    // clamp
    if (consumed_mAh < 0.0f) consumed_mAh = 0.0f;
    if (consumed_mAh > totalCapacity_mAh) consumed_mAh = totalCapacity_mAh;
}

float Battery_ReadPackVoltage(void) {
    if (hadcBatt == NULL) {
        return 0.0f;
    }

    HAL_ADC_Start(hadcBatt);
    HAL_ADC_PollForConversion(hadcBatt, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(hadcBatt);
    HAL_ADC_Stop(hadcBatt);

    float volts = ((float)raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    return volts * voltageDivider;
}

float Battery_ReadCurrent(void) {
    uint16_t raw;
    if (!INA219_ReadRegister(INA219_REG_SHUNTVOLTAGE, &raw)) {
        return 0.0f;
    }
    // raw is a signed 16-bit value in two's complement
    int16_t sraw = (int16_t)raw;
    // LSB = 10 µV per datasheet
    float vshunt = sraw * 0.00001f;  // V
    // current = Vshunt / Rshunt
    return vshunt / shuntResistance; // A
}

float Battery_ReadPerCellVoltage(void) {
    float packV = Battery_ReadPackVoltage();
    if (cellCount > 0) {
        return packV / (float)cellCount;
    }
    return packV;
}

float Battery_GetConsumed_mAh(void) {
    return consumed_mAh;
}

float Battery_GetRemaining_mAh(void) {
    return totalCapacity_mAh - consumed_mAh;
}

float Battery_GetRemainingPercent(void) {
    if (totalCapacity_mAh <= 0.0f) {
        return 0.0f;
    }
    return 100.0f * (1.0f - consumed_mAh / totalCapacity_mAh);
}
