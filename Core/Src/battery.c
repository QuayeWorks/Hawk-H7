/*
 * battery.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#include "battery.h"
#include "settings.h"
#include <string.h>

#define BATTERY_I2C_TIMEOUT_MS 5u
#define BATTERY_ADC_TIMEOUT_MS 2u

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
static bool battery_initialized = false;

static float consumed_mAh = 0.0f;
static uint32_t lastTickMs = 0;
static uint32_t battery_i2c_error_count = 0u;
static uint32_t battery_adc_timeout_count = 0u;
static uint32_t battery_current_sample_count = 0u;
static uint32_t battery_voltage_sample_count = 0u;
static uint32_t battery_last_current_ms = 0u;
static uint32_t battery_last_voltage_ms = 0u;
static HAL_StatusTypeDef battery_last_i2c_status = HAL_OK;
static HAL_StatusTypeDef battery_last_adc_status = HAL_OK;

#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_VALUE   4095.0f

/// Raw reading helpers
static bool INA219_ReadRegister(uint8_t reg, uint16_t *out) {
    uint8_t buf[2];
    if (hi2cBatt == NULL || out == NULL) {
        battery_last_i2c_status = HAL_ERROR;
        return false;
    }
    battery_last_i2c_status = HAL_I2C_Mem_Read(hi2cBatt, INA219_I2C_ADDR, reg, 1, buf, 2, BATTERY_I2C_TIMEOUT_MS);
    if (battery_last_i2c_status != HAL_OK) {
        battery_i2c_error_count++;
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
    battery_initialized = (hi2cBatt != NULL);
    battery_i2c_error_count = 0u;
    battery_adc_timeout_count = 0u;
    battery_current_sample_count = 0u;
    battery_voltage_sample_count = 0u;
    battery_last_current_ms = 0u;
    battery_last_voltage_ms = 0u;
    battery_last_i2c_status = HAL_OK;
    battery_last_adc_status = HAL_OK;
}

void Battery_Tick(uint32_t now_ms) {
    if (!battery_initialized || hi2cBatt == NULL) {
        return;
    }

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

    battery_last_adc_status = HAL_ADC_Start(hadcBatt);
    if (battery_last_adc_status != HAL_OK) {
        return 0.0f;
    }
    battery_last_adc_status = HAL_ADC_PollForConversion(hadcBatt, BATTERY_ADC_TIMEOUT_MS);
    if (battery_last_adc_status != HAL_OK) {
        battery_adc_timeout_count++;
        HAL_ADC_Stop(hadcBatt);
        return 0.0f;
    }
    uint32_t raw = HAL_ADC_GetValue(hadcBatt);
    HAL_ADC_Stop(hadcBatt);
    battery_voltage_sample_count++;
    battery_last_voltage_ms = HAL_GetTick();

    float volts = ((float)raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    return volts * voltageDivider;
}

float Battery_ReadCurrent(void) {
    if (!battery_initialized || hi2cBatt == NULL || shuntResistance <= 0.0f) {
        return 0.0f;
    }

    uint16_t raw;
    if (!INA219_ReadRegister(INA219_REG_SHUNTVOLTAGE, &raw)) {
        return 0.0f;
    }
    // raw is a signed 16-bit value in two's complement
    int16_t sraw = (int16_t)raw;
    // LSB = 10 µV per datasheet
    float vshunt = sraw * 0.00001f;  // V
    // current = Vshunt / Rshunt
    battery_current_sample_count++;
    battery_last_current_ms = HAL_GetTick();
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

uint32_t Battery_GetI2CErrorCount(void)
{
    return battery_i2c_error_count;
}

uint32_t Battery_GetADCTimeoutCount(void)
{
    return battery_adc_timeout_count;
}

uint32_t Battery_GetCurrentSampleCount(void)
{
    return battery_current_sample_count;
}

uint32_t Battery_GetVoltageSampleCount(void)
{
    return battery_voltage_sample_count;
}

uint32_t Battery_GetLastCurrentMs(void)
{
    return battery_last_current_ms;
}

uint32_t Battery_GetLastVoltageMs(void)
{
    return battery_last_voltage_ms;
}

HAL_StatusTypeDef Battery_GetLastI2CStatus(void)
{
    return battery_last_i2c_status;
}

HAL_StatusTypeDef Battery_GetLastADCStatus(void)
{
    return battery_last_adc_status;
}
