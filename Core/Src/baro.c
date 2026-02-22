#include "baro.h"
#include "settings.h"
#include <math.h>
#include <string.h>

#define BARO_I2C_TIMEOUT_MS      5u
#define BARO_VZ_IIR_ALPHA        0.85f

#define BMP280_REG_CALIB_START   0x88u
#define BMP280_REG_CHIP_ID       0xD0u
#define BMP280_REG_RESET         0xE0u
#define BMP280_REG_STATUS        0xF3u
#define BMP280_REG_CTRL_MEAS     0xF4u
#define BMP280_REG_CONFIG        0xF5u
#define BMP280_REG_PRESS_MSB     0xF7u

#define BMP280_CHIP_ID           0x58u
#define BME280_CHIP_ID           0x60u
#define BMP280_RESET_VALUE       0xB6u

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} Bmp280Calib;

static I2C_HandleTypeDef *baro_i2c = NULL;
static uint8_t baro_addr_8bit = 0;
static bool baro_initialized = false;
static bool baro_healthy = false;
static bool baro_has_sample = false;
static uint32_t baro_last_update_ms = 0;
static uint8_t baro_chip_id = 0u;
static bool baro_calibration_ok = false;
static uint32_t baro_i2c_fail_count = 0u;
static uint32_t baro_update_ok_count = 0u;
static HAL_StatusTypeDef baro_last_hal_status = HAL_OK;

static Bmp280Calib cal;
static int32_t t_fine = 0;

static float pressure_hpa = 0.0f;
static float temperature_c = 0.0f;
static float altitude_m = 0.0f;
static float climb_rate_mps = 0.0f;

static bool read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;

    if (baro_i2c == NULL || baro_addr_8bit == 0u || buf == NULL || len == 0u) {
        baro_last_hal_status = HAL_ERROR;
        return false;
    }
    st = HAL_I2C_Mem_Read(baro_i2c, baro_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT,
                          buf, len, BARO_I2C_TIMEOUT_MS);
    baro_last_hal_status = st;
    if (st != HAL_OK) {
        baro_i2c_fail_count++;
        return false;
    }
    return true;
}

static bool write_reg(uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef st;

    if (baro_i2c == NULL || baro_addr_8bit == 0u) {
        baro_last_hal_status = HAL_ERROR;
        return false;
    }
    st = HAL_I2C_Mem_Write(baro_i2c, baro_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, BARO_I2C_TIMEOUT_MS);
    baro_last_hal_status = st;
    if (st != HAL_OK) {
        baro_i2c_fail_count++;
        return false;
    }
    return true;
}

static bool read_calibration(void)
{
    uint8_t buf[24];

    if (!read_reg(BMP280_REG_CALIB_START, buf, sizeof(buf))) {
        baro_calibration_ok = false;
        return false;
    }

    cal.dig_T1 = (uint16_t)((buf[1]  << 8) | buf[0]);
    cal.dig_T2 = (int16_t) ((buf[3]  << 8) | buf[2]);
    cal.dig_T3 = (int16_t) ((buf[5]  << 8) | buf[4]);
    cal.dig_P1 = (uint16_t)((buf[7]  << 8) | buf[6]);
    cal.dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
    cal.dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
    cal.dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
    cal.dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
    cal.dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
    cal.dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
    cal.dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
    cal.dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);

    baro_calibration_ok = (cal.dig_P1 != 0u);
    return baro_calibration_ok;
}

static bool probe_chip(uint8_t addr_7bit)
{
    uint8_t chip_id = 0;
    baro_addr_8bit = (uint8_t)(addr_7bit << 1);
    if (!read_reg(BMP280_REG_CHIP_ID, &chip_id, 1)) {
        return false;
    }
    if ((chip_id == BMP280_CHIP_ID) || (chip_id == BME280_CHIP_ID)) {
        baro_chip_id = chip_id;
        return true;
    }
    return false;
}

void Baro_Init(I2C_HandleTypeDef *i2c, uint8_t i2c_addr_7bit)
{
    uint8_t reset = BMP280_RESET_VALUE;
    uint8_t ctrl_meas;
    uint8_t config;
    uint8_t status = 0;

    baro_i2c = i2c;
    baro_initialized = false;
    baro_healthy = false;
    baro_has_sample = false;
    baro_last_update_ms = 0;
    baro_chip_id = 0u;
    baro_calibration_ok = false;
    baro_i2c_fail_count = 0u;
    baro_update_ok_count = 0u;
    baro_last_hal_status = HAL_OK;
    pressure_hpa = 0.0f;
    temperature_c = 0.0f;
    altitude_m = 0.0f;
    climb_rate_mps = 0.0f;

    if (baro_i2c == NULL) {
        return;
    }

    if (i2c_addr_7bit != 0u) {
        if (!probe_chip(i2c_addr_7bit)) {
            return;
        }
    } else {
        if (!probe_chip(BARO_BMP280_ADDR_PRIMARY) &&
            !probe_chip(BARO_BMP280_ADDR_SECONDARY)) {
            return;
        }
    }

    // Soft reset and wait until NVM copy is done.
    (void)write_reg(BMP280_REG_RESET, reset);
    HAL_Delay(3);
    if (read_reg(BMP280_REG_STATUS, &status, 1)) {
        uint32_t t0 = HAL_GetTick();
        while ((status & 0x01u) != 0u && (HAL_GetTick() - t0) < 20u) {
            HAL_Delay(1);
            if (!read_reg(BMP280_REG_STATUS, &status, 1)) {
                break;
            }
        }
    }

    if (!read_calibration()) {
        return;
    }

    // Temp oversampling x2, pressure oversampling x16, normal mode.
    ctrl_meas = (uint8_t)((0x02u << 5) | (0x05u << 2) | 0x03u);
    // Standby 62.5ms, IIR filter x16.
    config = (uint8_t)((0x02u << 5) | (0x04u << 2));

    if (!write_reg(BMP280_REG_CTRL_MEAS, ctrl_meas)) {
        return;
    }
    if (!write_reg(BMP280_REG_CONFIG, config)) {
        return;
    }

    baro_initialized = true;
}

bool Baro_Update(uint32_t now_ms)
{
    uint8_t raw[6];
    int32_t adc_p;
    int32_t adc_t;
    int32_t var1, var2;
    int64_t pvar1, pvar2, p;
    float sea_level_hpa;
    float raw_alt;
    float dt_s;

    if (!baro_initialized) {
        baro_healthy = false;
        return false;
    }

    if (!read_reg(BMP280_REG_PRESS_MSB, raw, sizeof(raw))) {
        baro_healthy = false;
        return false;
    }

    adc_p = (int32_t)((((uint32_t)raw[0]) << 12) |
                      (((uint32_t)raw[1]) << 4)  |
                      (((uint32_t)raw[2]) >> 4));
    adc_t = (int32_t)((((uint32_t)raw[3]) << 12) |
                      (((uint32_t)raw[4]) << 4)  |
                      (((uint32_t)raw[5]) >> 4));

    if (adc_p <= 0 || adc_t <= 0) {
        baro_healthy = false;
        return false;
    }

    // Temperature compensation (BMP280 datasheet).
    var1 = ((((adc_t >> 3) - ((int32_t)cal.dig_T1 << 1))) * ((int32_t)cal.dig_T2)) >> 11;
    var2 = (((((adc_t >> 4) - ((int32_t)cal.dig_T1)) *
              ((adc_t >> 4) - ((int32_t)cal.dig_T1))) >> 12) *
            ((int32_t)cal.dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    // Pressure compensation (BMP280 datasheet).
    pvar1 = ((int64_t)t_fine) - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)cal.dig_P6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)cal.dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)cal.dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)cal.dig_P3) >> 8) +
            ((pvar1 * (int64_t)cal.dig_P2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1) * ((int64_t)cal.dig_P1)) >> 33;

    if (pvar1 == 0) {
        baro_healthy = false;
        return false;
    }

    p = 1048576 - adc_p;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (((int64_t)cal.dig_P8) * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)cal.dig_P7) << 4);

    // p is Q24.8 Pa
    pressure_hpa = (float)p / 25600.0f;
    if (pressure_hpa < 300.0f || pressure_hpa > 1100.0f) {
        baro_healthy = false;
        return false;
    }

    sea_level_hpa = Settings_GetBaroPressureOffsetHpa();
    if (sea_level_hpa < 800.0f || sea_level_hpa > 1200.0f) {
        sea_level_hpa = 1013.25f;
    }

    raw_alt = 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.19029495f));
    raw_alt -= Settings_GetBaroAltOffsetM();

    if (!baro_has_sample) {
        altitude_m = raw_alt;
        climb_rate_mps = 0.0f;
    } else {
        float raw_vz = 0.0f;
        float prev_alt = altitude_m;
        altitude_m = 0.80f * altitude_m + 0.20f * raw_alt;
        dt_s = (float)(now_ms - baro_last_update_ms) * 0.001f;
        if (dt_s > 0.0001f) {
            raw_vz = (altitude_m - prev_alt) / dt_s;
            climb_rate_mps = BARO_VZ_IIR_ALPHA * climb_rate_mps +
                             (1.0f - BARO_VZ_IIR_ALPHA) * raw_vz;
        }
    }

    baro_has_sample = true;
    baro_last_update_ms = now_ms;
    baro_healthy = true;
    baro_update_ok_count++;
    return true;
}

bool Baro_IsHealthy(void)
{
    return baro_healthy;
}

bool Baro_HasSample(void)
{
    return baro_has_sample;
}

bool Baro_IsIdentityOK(void)
{
    return (baro_chip_id == BMP280_CHIP_ID) || (baro_chip_id == BME280_CHIP_ID);
}

float Baro_GetPressureHpa(void)
{
    return pressure_hpa;
}

float Baro_GetTemperatureC(void)
{
    return temperature_c;
}

float Baro_GetAltitudeMeters(void)
{
    return altitude_m;
}

float Baro_GetClimbRateMps(void)
{
    return climb_rate_mps;
}

uint8_t Baro_GetChipId(void)
{
    return baro_chip_id;
}

uint32_t Baro_GetLastUpdateMs(void)
{
    return baro_last_update_ms;
}

bool Baro_IsCalibrationLoaded(void)
{
    return baro_calibration_ok;
}

uint32_t Baro_GetI2CFailCount(void)
{
    return baro_i2c_fail_count;
}

uint32_t Baro_GetUpdateOkCount(void)
{
    return baro_update_ok_count;
}

HAL_StatusTypeDef Baro_GetLastHalStatus(void)
{
    return baro_last_hal_status;
}
