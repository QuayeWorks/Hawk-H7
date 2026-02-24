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

#define BMP3_CHIP_ID             0x50u
#define BMP3_REG_CHIP_ID         0x00u
#define BMP3_REG_STATUS          0x03u
#define BMP3_REG_DATA            0x04u
#define BMP3_REG_PWR_CTRL        0x1Bu
#define BMP3_REG_OSR             0x1Cu
#define BMP3_REG_ODR             0x1Du
#define BMP3_REG_CONFIG          0x1Fu
#define BMP3_REG_CALIB_DATA      0x31u
#define BMP3_REG_CMD             0x7Eu
#define BMP3_SOFT_RESET          0xB6u
#define BMP3_CMD_RDY_MSK         0x10u

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

typedef struct {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
} Bmp3Calib;

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

static Bmp280Calib bmp280_cal;
static Bmp3Calib bmp3_cal;
static int32_t bmp280_t_fine = 0;
static bool baro_is_bmp3 = false;

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

static bool update_altitude_solution(float new_pressure_hpa, uint32_t now_ms)
{
    float sea_level_hpa;
    float raw_alt;

    pressure_hpa = new_pressure_hpa;
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
        float dt_s = (float)(now_ms - baro_last_update_ms) * 0.001f;

        altitude_m = 0.80f * altitude_m + 0.20f * raw_alt;
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

static bool read_calibration_bmp280(void)
{
    uint8_t buf[24];

    if (!read_reg(BMP280_REG_CALIB_START, buf, sizeof(buf))) {
        baro_calibration_ok = false;
        return false;
    }

    bmp280_cal.dig_T1 = (uint16_t)((buf[1]  << 8) | buf[0]);
    bmp280_cal.dig_T2 = (int16_t) ((buf[3]  << 8) | buf[2]);
    bmp280_cal.dig_T3 = (int16_t) ((buf[5]  << 8) | buf[4]);
    bmp280_cal.dig_P1 = (uint16_t)((buf[7]  << 8) | buf[6]);
    bmp280_cal.dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
    bmp280_cal.dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
    bmp280_cal.dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
    bmp280_cal.dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
    bmp280_cal.dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
    bmp280_cal.dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
    bmp280_cal.dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
    bmp280_cal.dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);

    baro_calibration_ok = (bmp280_cal.dig_P1 != 0u);
    return baro_calibration_ok;
}

static bool read_calibration_bmp3(void)
{
    uint8_t buf[21];
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;

    if (!read_reg(BMP3_REG_CALIB_DATA, buf, sizeof(buf))) {
        baro_calibration_ok = false;
        return false;
    }

    par_t1 = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    par_t2 = (uint16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    par_t3 = (int8_t)buf[4];
    par_p1 = (int16_t)(((uint16_t)buf[6] << 8) | buf[5]);
    par_p2 = (int16_t)(((uint16_t)buf[8] << 8) | buf[7]);
    par_p3 = (int8_t)buf[9];
    par_p4 = (int8_t)buf[10];
    par_p5 = (uint16_t)(((uint16_t)buf[12] << 8) | buf[11]);
    par_p6 = (uint16_t)(((uint16_t)buf[14] << 8) | buf[13]);
    par_p7 = (int8_t)buf[15];
    par_p8 = (int8_t)buf[16];
    par_p9 = (int16_t)(((uint16_t)buf[18] << 8) | buf[17]);
    par_p10 = (int8_t)buf[19];
    par_p11 = (int8_t)buf[20];

    bmp3_cal.par_t1 = ((double)par_t1) / 0.00390625;
    bmp3_cal.par_t2 = ((double)par_t2) / 1073741824.0;
    bmp3_cal.par_t3 = ((double)par_t3) / 281474976710656.0;
    bmp3_cal.par_p1 = ((double)(par_p1 - 16384)) / 1048576.0;
    bmp3_cal.par_p2 = ((double)(par_p2 - 16384)) / 536870912.0;
    bmp3_cal.par_p3 = ((double)par_p3) / 4294967296.0;
    bmp3_cal.par_p4 = ((double)par_p4) / 137438953472.0;
    bmp3_cal.par_p5 = ((double)par_p5) / 0.125;
    bmp3_cal.par_p6 = ((double)par_p6) / 64.0;
    bmp3_cal.par_p7 = ((double)par_p7) / 256.0;
    bmp3_cal.par_p8 = ((double)par_p8) / 32768.0;
    bmp3_cal.par_p9 = ((double)par_p9) / 281474976710656.0;
    bmp3_cal.par_p10 = ((double)par_p10) / 281474976710656.0;
    bmp3_cal.par_p11 = ((double)par_p11) / 36893488147419103232.0;
    bmp3_cal.t_lin = 0.0;

    baro_calibration_ok = (par_t1 != 0u && par_t2 != 0u);
    return baro_calibration_ok;
}

static bool probe_chip(uint8_t addr_7bit)
{
    uint8_t chip_id = 0u;

    baro_addr_8bit = (uint8_t)(addr_7bit << 1);
    if (!read_reg(BMP3_REG_CHIP_ID, &chip_id, 1u)) {
        return false;
    }

    if ((chip_id == BMP280_CHIP_ID) || (chip_id == BME280_CHIP_ID) ||
        (chip_id == BMP3_CHIP_ID)) {
        baro_chip_id = chip_id;
        return true;
    }

    return false;
}

static bool init_bmp280_path(void)
{
    uint8_t status = 0u;
    uint8_t ctrl_meas;
    uint8_t config;

    (void)write_reg(BMP280_REG_RESET, BMP280_RESET_VALUE);
    HAL_Delay(3);
    if (read_reg(BMP280_REG_STATUS, &status, 1u)) {
        uint32_t t0 = HAL_GetTick();
        while ((status & 0x01u) != 0u && (HAL_GetTick() - t0) < 20u) {
            HAL_Delay(1);
            if (!read_reg(BMP280_REG_STATUS, &status, 1u)) {
                break;
            }
        }
    }

    if (!read_calibration_bmp280()) {
        return false;
    }

    ctrl_meas = (uint8_t)((0x02u << 5) | (0x05u << 2) | 0x03u);
    config = (uint8_t)((0x02u << 5) | (0x04u << 2));

    if (!write_reg(BMP280_REG_CTRL_MEAS, ctrl_meas)) {
        return false;
    }
    if (!write_reg(BMP280_REG_CONFIG, config)) {
        return false;
    }

    return true;
}

static bool init_bmp3_path(void)
{
    uint8_t status = 0u;
    uint8_t osr = (uint8_t)((0x01u << 3) | 0x03u);
    uint8_t odr = 0x03u;
    uint8_t config = (uint8_t)(0x02u << 1);
    uint8_t pwr_ctrl = 0x33u;
    uint32_t t0;

    if (!write_reg(BMP3_REG_CMD, BMP3_SOFT_RESET)) {
        return false;
    }
    HAL_Delay(3);

    t0 = HAL_GetTick();
    do {
        if (!read_reg(BMP3_REG_STATUS, &status, 1u)) {
            break;
        }
        if ((status & BMP3_CMD_RDY_MSK) != 0u) {
            break;
        }
        HAL_Delay(1);
    } while ((HAL_GetTick() - t0) < 20u);

    if (!read_calibration_bmp3()) {
        return false;
    }

    if (!write_reg(BMP3_REG_OSR, osr)) {
        return false;
    }
    if (!write_reg(BMP3_REG_ODR, odr)) {
        return false;
    }
    if (!write_reg(BMP3_REG_CONFIG, config)) {
        return false;
    }
    if (!write_reg(BMP3_REG_PWR_CTRL, pwr_ctrl)) {
        return false;
    }

    return true;
}

static bool baro_update_bmp280(uint32_t now_ms)
{
    uint8_t raw[6];
    int32_t adc_p;
    int32_t adc_t;
    int32_t var1;
    int32_t var2;
    int64_t pvar1;
    int64_t pvar2;
    int64_t p;

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

    var1 = ((((adc_t >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2 = (((((adc_t >> 4) - ((int32_t)bmp280_cal.dig_T1)) *
              ((adc_t >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) *
            ((int32_t)bmp280_cal.dig_T3)) >> 14;
    bmp280_t_fine = var1 + var2;
    temperature_c = (float)((bmp280_t_fine * 5 + 128) >> 8) / 100.0f;

    pvar1 = ((int64_t)bmp280_t_fine) - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)bmp280_cal.dig_P6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)bmp280_cal.dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)bmp280_cal.dig_P3) >> 8) +
            ((pvar1 * (int64_t)bmp280_cal.dig_P2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1) * ((int64_t)bmp280_cal.dig_P1)) >> 33;

    if (pvar1 == 0) {
        baro_healthy = false;
        return false;
    }

    p = 1048576 - adc_p;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);

    return update_altitude_solution((float)p / 25600.0f, now_ms);
}

static bool baro_update_bmp3(uint32_t now_ms)
{
    uint8_t raw[6];
    uint32_t uncomp_press;
    uint32_t uncomp_temp;
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;
    double comp_press_pa;

    if (!read_reg(BMP3_REG_DATA, raw, sizeof(raw))) {
        baro_healthy = false;
        return false;
    }

    uncomp_press = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[0];
    uncomp_temp = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | (uint32_t)raw[3];
    if (uncomp_press == 0u || uncomp_temp == 0u) {
        baro_healthy = false;
        return false;
    }

    partial_data1 = ((double)uncomp_temp - bmp3_cal.par_t1);
    partial_data2 = partial_data1 * bmp3_cal.par_t2;
    bmp3_cal.t_lin = partial_data2 + (partial_data1 * partial_data1) * bmp3_cal.par_t3;
    temperature_c = (float)bmp3_cal.t_lin;
    if (temperature_c < -50.0f || temperature_c > 120.0f) {
        baro_healthy = false;
        return false;
    }

    partial_data1 = bmp3_cal.par_p6 * bmp3_cal.t_lin;
    partial_data2 = bmp3_cal.par_p7 * bmp3_cal.t_lin * bmp3_cal.t_lin;
    partial_data3 = bmp3_cal.par_p8 * bmp3_cal.t_lin * bmp3_cal.t_lin * bmp3_cal.t_lin;
    partial_out1 = bmp3_cal.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = bmp3_cal.par_p2 * bmp3_cal.t_lin;
    partial_data2 = bmp3_cal.par_p3 * bmp3_cal.t_lin * bmp3_cal.t_lin;
    partial_data3 = bmp3_cal.par_p4 * bmp3_cal.t_lin * bmp3_cal.t_lin * bmp3_cal.t_lin;
    partial_out2 = ((double)uncomp_press) * (bmp3_cal.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = ((double)uncomp_press) * ((double)uncomp_press);
    partial_data2 = bmp3_cal.par_p9 + bmp3_cal.par_p10 * bmp3_cal.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 +
                    ((double)uncomp_press) * ((double)uncomp_press) * ((double)uncomp_press) * bmp3_cal.par_p11;

    comp_press_pa = partial_out1 + partial_out2 + partial_data4;
    if (comp_press_pa < 30000.0 || comp_press_pa > 125000.0) {
        baro_healthy = false;
        return false;
    }

    return update_altitude_solution((float)(comp_press_pa * 0.01), now_ms);
}

void Baro_Init(I2C_HandleTypeDef *i2c, uint8_t i2c_addr_7bit)
{
    baro_i2c = i2c;
    baro_addr_8bit = 0u;
    baro_initialized = false;
    baro_healthy = false;
    baro_has_sample = false;
    baro_last_update_ms = 0u;
    baro_chip_id = 0u;
    baro_calibration_ok = false;
    baro_i2c_fail_count = 0u;
    baro_update_ok_count = 0u;
    baro_last_hal_status = HAL_OK;
    baro_is_bmp3 = false;
    bmp280_t_fine = 0;
    memset(&bmp280_cal, 0, sizeof(bmp280_cal));
    memset(&bmp3_cal, 0, sizeof(bmp3_cal));
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

    baro_is_bmp3 = (baro_chip_id == BMP3_CHIP_ID);
    if (baro_is_bmp3) {
        if (!init_bmp3_path()) {
            return;
        }
    } else {
        if (!init_bmp280_path()) {
            return;
        }
    }

    baro_initialized = true;
}

bool Baro_Update(uint32_t now_ms)
{
    if (!baro_initialized) {
        baro_healthy = false;
        return false;
    }

    if (baro_is_bmp3) {
        return baro_update_bmp3(now_ms);
    }

    return baro_update_bmp280(now_ms);
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
    return (baro_chip_id == BMP280_CHIP_ID) ||
           (baro_chip_id == BME280_CHIP_ID) ||
           (baro_chip_id == BMP3_CHIP_ID);
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
