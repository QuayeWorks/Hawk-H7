/*
 * battery.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#include "battery.h"
#include "settings.h"
#include <string.h>

#define BATTERY_I2C_TIMEOUT_MS   5u
#define BATTERY_ADC_TIMEOUT_MS   2u
#define BATTERY_MAX_INA219       4u

// INA219 registers.
#define INA219_REG_SHUNTVOLTAGE  0x01u

#define INA219_ADDR_MIN          0x40u
#define INA219_ADDR_MAX          0x4Fu

#define ADC_REF_VOLTAGE          3.3f
#define ADC_MAX_VALUE            4095.0f

typedef struct {
    bool present;
    uint8_t addr7;
    uint16_t addr8;
    float last_current_a;
    uint32_t read_ok_count;
    uint32_t read_fail_count;
    uint32_t last_read_ms;
    HAL_StatusTypeDef last_status;
} Ina219Node;

static I2C_HandleTypeDef *hi2cBatt;
static ADC_HandleTypeDef *hadcBatt = NULL;
static float shuntResistance;
static float voltageDivider;
static uint8_t cellCount;
static float totalCapacity_mAh;
static bool battery_initialized = false;

static Ina219Node inaNodes[BATTERY_MAX_INA219];
static uint8_t inaNodeCount = 0u;
static uint8_t preferredInaAddr7 = INA219_ADDR_MIN;

static float consumed_mAh = 0.0f;
static uint32_t lastTickMs = 0u;
static uint32_t battery_i2c_error_count = 0u;
static uint32_t battery_adc_timeout_count = 0u;
static uint32_t battery_current_sample_count = 0u;
static uint32_t battery_voltage_sample_count = 0u;
static uint32_t battery_last_current_ms = 0u;
static uint32_t battery_last_voltage_ms = 0u;
static HAL_StatusTypeDef battery_last_i2c_status = HAL_OK;
static HAL_StatusTypeDef battery_last_adc_status = HAL_OK;

static bool INA219_ReadRegisterAddr(uint16_t addr8, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];

    if (hi2cBatt == NULL || out == NULL) {
        battery_last_i2c_status = HAL_ERROR;
        return false;
    }

    battery_last_i2c_status = HAL_I2C_Mem_Read(hi2cBatt, addr8, reg, I2C_MEMADD_SIZE_8BIT,
                                               buf, 2u, BATTERY_I2C_TIMEOUT_MS);
    if (battery_last_i2c_status != HAL_OK) {
        battery_i2c_error_count++;
        return false;
    }

    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

static void INA219_AddNode(uint8_t addr7)
{
    if (inaNodeCount >= BATTERY_MAX_INA219) {
        return;
    }

    inaNodes[inaNodeCount].present = true;
    inaNodes[inaNodeCount].addr7 = addr7;
    inaNodes[inaNodeCount].addr8 = (uint16_t)(addr7 << 1);
    inaNodes[inaNodeCount].last_current_a = 0.0f;
    inaNodes[inaNodeCount].read_ok_count = 0u;
    inaNodes[inaNodeCount].read_fail_count = 0u;
    inaNodes[inaNodeCount].last_read_ms = 0u;
    inaNodes[inaNodeCount].last_status = HAL_OK;
    inaNodeCount++;
}

static void INA219_ScanBus(void)
{
    uint8_t addr7;

    inaNodeCount = 0u;
    memset(inaNodes, 0, sizeof(inaNodes));

    if (hi2cBatt == NULL) {
        return;
    }

    // Prefer user-configured address first.
    if (preferredInaAddr7 >= INA219_ADDR_MIN && preferredInaAddr7 <= INA219_ADDR_MAX) {
        battery_last_i2c_status = HAL_I2C_IsDeviceReady(hi2cBatt, (uint16_t)(preferredInaAddr7 << 1),
                                                        2u, BATTERY_I2C_TIMEOUT_MS);
        if (battery_last_i2c_status == HAL_OK) {
            INA219_AddNode(preferredInaAddr7);
        }
    }

    for (addr7 = INA219_ADDR_MIN; addr7 <= INA219_ADDR_MAX && inaNodeCount < BATTERY_MAX_INA219; addr7++) {
        if (addr7 == preferredInaAddr7) {
            continue;
        }

        battery_last_i2c_status = HAL_I2C_IsDeviceReady(hi2cBatt, (uint16_t)(addr7 << 1),
                                                        2u, BATTERY_I2C_TIMEOUT_MS);
        if (battery_last_i2c_status == HAL_OK) {
            INA219_AddNode(addr7);
        }
    }

    if (inaNodeCount == 0u) {
        battery_i2c_error_count++;
    }
}

static bool INA219_ReadCurrentAtIndex(uint8_t index, float *outCurrentA)
{
    uint16_t raw;
    int16_t sraw;

    if (index >= inaNodeCount || outCurrentA == NULL ||
        shuntResistance <= 0.0f || hi2cBatt == NULL) {
        return false;
    }

    if (!INA219_ReadRegisterAddr(inaNodes[index].addr8, INA219_REG_SHUNTVOLTAGE, &raw)) {
        inaNodes[index].last_status = battery_last_i2c_status;
        inaNodes[index].read_fail_count++;
        return false;
    }

    sraw = (int16_t)raw;
    *outCurrentA = ((float)sraw * 0.00001f) / shuntResistance; // 10uV LSB.

    inaNodes[index].last_current_a = *outCurrentA;
    inaNodes[index].last_status = HAL_OK;
    inaNodes[index].read_ok_count++;
    inaNodes[index].last_read_ms = HAL_GetTick();
    return true;
}

void Battery_Init(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, float shuntOhm)
{
    uint8_t inaAddr7 = Settings_GetINA219I2CAddress();

    hi2cBatt = hi2c;
    hadcBatt = hadc;
    shuntResistance = shuntOhm;
    voltageDivider = Settings_GetBatteryVoltDivider();
    cellCount = Settings_GetBatteryCellCount();
    totalCapacity_mAh = (float)Settings_GetBatteryLogmAh();
    consumed_mAh = 0.0f;
    lastTickMs = HAL_GetTick();
    battery_i2c_error_count = 0u;
    battery_adc_timeout_count = 0u;
    battery_current_sample_count = 0u;
    battery_voltage_sample_count = 0u;
    battery_last_current_ms = 0u;
    battery_last_voltage_ms = 0u;
    battery_last_i2c_status = HAL_OK;
    battery_last_adc_status = HAL_OK;

    if (inaAddr7 < INA219_ADDR_MIN || inaAddr7 > INA219_ADDR_MAX) {
        inaAddr7 = INA219_ADDR_MIN;
    }
    preferredInaAddr7 = inaAddr7;
    INA219_ScanBus();

    battery_initialized = (hi2cBatt != NULL) || (hadcBatt != NULL);
}

void Battery_Tick(uint32_t now_ms)
{
    uint32_t dt;
    float currentA;

    if (!battery_initialized) {
        return;
    }

    dt = now_ms - lastTickMs;
    lastTickMs = now_ms;

    currentA = Battery_ReadCurrent();
    consumed_mAh += currentA * ((float)dt / 3600.0f);

    if (consumed_mAh < 0.0f) {
        consumed_mAh = 0.0f;
    }
    if (consumed_mAh > totalCapacity_mAh) {
        consumed_mAh = totalCapacity_mAh;
    }
}

float Battery_ReadPackVoltage(void)
{
    uint32_t raw;
    float volts;

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

    raw = HAL_ADC_GetValue(hadcBatt);
    HAL_ADC_Stop(hadcBatt);

    battery_voltage_sample_count++;
    battery_last_voltage_ms = HAL_GetTick();
    volts = ((float)raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    return volts * voltageDivider;
}

float Battery_ReadCurrent(void)
{
    float totalCurrent = 0.0f;
    float sample = 0.0f;
    bool anyOk = false;
    uint8_t i;

    if (!battery_initialized || hi2cBatt == NULL || shuntResistance <= 0.0f) {
        return 0.0f;
    }

    if (inaNodeCount == 0u) {
        INA219_ScanBus();
        if (inaNodeCount == 0u) {
            return 0.0f;
        }
    }

    for (i = 0u; i < inaNodeCount; i++) {
        if (INA219_ReadCurrentAtIndex(i, &sample)) {
            totalCurrent += sample;
            anyOk = true;
        }
    }

    if (anyOk) {
        battery_current_sample_count++;
        battery_last_current_ms = HAL_GetTick();
        battery_last_i2c_status = HAL_OK;
        return totalCurrent;
    }

    return 0.0f;
}

float Battery_ReadCurrentAt(uint8_t index)
{
    float current = 0.0f;

    if (INA219_ReadCurrentAtIndex(index, &current)) {
        battery_current_sample_count++;
        battery_last_current_ms = HAL_GetTick();
        battery_last_i2c_status = HAL_OK;
        return current;
    }

    return 0.0f;
}

float Battery_GetLastCurrentAt(uint8_t index)
{
    if (index >= inaNodeCount) {
        return 0.0f;
    }
    return inaNodes[index].last_current_a;
}

float Battery_ReadPerCellVoltage(void)
{
    float packV = Battery_ReadPackVoltage();
    if (cellCount > 0u) {
        return packV / (float)cellCount;
    }
    return packV;
}

float Battery_GetConsumed_mAh(void)
{
    return consumed_mAh;
}

float Battery_GetRemaining_mAh(void)
{
    return totalCapacity_mAh - consumed_mAh;
}

float Battery_GetRemainingPercent(void)
{
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

uint8_t Battery_GetI2CAddress7bit(void)
{
    if (inaNodeCount > 0u) {
        return inaNodes[0].addr7;
    }
    return preferredInaAddr7;
}

HAL_StatusTypeDef Battery_GetLastI2CStatus(void)
{
    return battery_last_i2c_status;
}

HAL_StatusTypeDef Battery_GetLastADCStatus(void)
{
    return battery_last_adc_status;
}

uint8_t Battery_GetINA219Count(void)
{
    return inaNodeCount;
}

uint8_t Battery_GetINA219Address7bitAt(uint8_t index)
{
    if (index >= inaNodeCount) {
        return 0u;
    }
    return inaNodes[index].addr7;
}

HAL_StatusTypeDef Battery_GetLastI2CStatusAt(uint8_t index)
{
    if (index >= inaNodeCount) {
        return HAL_ERROR;
    }
    return inaNodes[index].last_status;
}
