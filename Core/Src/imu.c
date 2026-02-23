/*
 * imu.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#include "imu.h"
#include "flight_state.h"
#include "settings.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMU_I2C_TIMEOUT_MS 5u

static I2C_HandleTypeDef *hi2c;
static uint16_t imuAddr8bit = MPU6050_I2C_ADDR;
static float accelBiasX, accelBiasY, accelBiasZ;
static float gyroBiasX, gyroBiasY, gyroBiasZ;
static int16_t lastRawAx, lastRawAy, lastRawAz;
static int16_t lastRawGx, lastRawGy, lastRawGz;
static uint8_t imuWhoAmI;
static bool imuIdentityOk;
static uint32_t imuReadOkCount;
static uint32_t imuReadFailCount;
static uint32_t imuLastReadMs;
static HAL_StatusTypeDef imuLastHalStatus = HAL_OK;

static bool IMU_ProbeAddress(uint16_t addr8bit)
{
    uint8_t who = 0u;

    if (hi2c == NULL) {
        return false;
    }

    imuLastHalStatus = HAL_I2C_Mem_Read(hi2c, addr8bit, MPU6050_WHO_AM_I, 1,
                                        &who, 1, IMU_I2C_TIMEOUT_MS);
    if (imuLastHalStatus != HAL_OK) {
        return false;
    }
    if (who != 0x68u && who != 0x69u) {
        return false;
    }

    imuWhoAmI = who;
    imuAddr8bit = addr8bit;
    return true;
}

void IMU_Init(I2C_HandleTypeDef *i2c_handle) {
    uint8_t pwr;
    uint8_t cfg;

    hi2c = i2c_handle;
    imuWhoAmI = 0u;
    imuIdentityOk = false;
    imuReadOkCount = 0u;
    imuReadFailCount = 0u;
    imuLastReadMs = 0u;
    imuLastHalStatus = HAL_OK;

    if (hi2c == NULL) {
        return;
    }

    if (!IMU_ProbeAddress((uint16_t)(0x68u << 1)) &&
        !IMU_ProbeAddress((uint16_t)(0x69u << 1))) {
        return;
    }
    imuIdentityOk = true;

    // Wake up the MPU6050 (exit sleep)
    pwr = 0x00;
    HAL_I2C_Mem_Write(hi2c, imuAddr8bit, MPU6050_PWR_MGMT_1, 1, &pwr, 1, IMU_I2C_TIMEOUT_MS);
    // Configure accel ±4g, gyro ±500°/s (for example)
    cfg = (0x01 << 3); // accel FS_SEL=1 (±4g)
    HAL_I2C_Mem_Write(hi2c, imuAddr8bit, MPU6050_ACCEL_CONFIG, 1, &cfg, 1, IMU_I2C_TIMEOUT_MS);
    cfg = (0x01 << 3); // gyro FS_SEL=1 (±500 °/s)
    HAL_I2C_Mem_Write(hi2c, imuAddr8bit, MPU6050_GYRO_CONFIG, 1, &cfg, 1, IMU_I2C_TIMEOUT_MS);

    // Load any pre‐saved biases from settings.ini
    IMU_CalibrateOnBoot( (Settings_GetCalibrateOnBoot()? Settings_GetCalibSamples() : 0),
                         Settings_GetAccelTolG(),
                         &accelBiasX, &accelBiasY, &accelBiasZ );
}

bool IMU_CalibrateOnBoot(uint16_t samples, float tolG,
                         float *bax, float *bay, float *baz)
{
    if (bax == NULL || bay == NULL || baz == NULL || !imuIdentityOk) {
        return false;
    }

    // If user disabled recalibration, just load the saved values
    if (samples == 0) {
    	// No recalibration: load previous biases from settings
        Settings_GetAccelBiases(bax, bay, baz);
        // Copy into our local variables so Get*() will use them
        accelBiasX = *bax;
        accelBiasY = *bay;
        accelBiasZ = *baz;
        return true;
    }

    int64_t sumAx=0, sumAy=0, sumAz=0;
    for (uint16_t i=0; i<samples; i++) {
        int16_t ax,ay,az,gx,gy,gz;
        if (!IMU_ReadRaw(&ax,&ay,&az,&gx,&gy,&gz)) {
            return false;
        }
        sumAx += ax; sumAy += ay; sumAz += az;
        HAL_Delay(5);
    }
    // Convert raw to g: ±32768→±4g => 1g = 8192 LSB
    float avgAx = sumAx / (float)samples / 8192.0f;
    float avgAy = sumAy / (float)samples / 8192.0f;
    float avgAz = sumAz / (float)samples / 8192.0f;
    // az should ≈ +1g
    if (fabsf(avgAx) > tolG || fabsf(avgAy) > tolG || fabsf(avgAz - 1.0f) > tolG) {
        return false;
    }
    *bax = -avgAx; *bay = -avgAy; *baz = 1.0f - avgAz;
    accelBiasX = *bax; accelBiasY = *bay; accelBiasZ = *baz;
    // Save into settings
    Settings_SetAccelBiases(accelBiasX, accelBiasY, accelBiasZ);
    return true;
}

bool IMU_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                 int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    if (!imuIdentityOk || hi2c == NULL ||
        ax == NULL || ay == NULL || az == NULL ||
        gx == NULL || gy == NULL || gz == NULL) {
        return false;
    }
    imuLastHalStatus = HAL_I2C_Mem_Read(hi2c, imuAddr8bit, MPU6050_ACCEL_XOUT_H, 1, buf, 14, IMU_I2C_TIMEOUT_MS);
    if (imuLastHalStatus != HAL_OK) {
        imuReadFailCount++;
        return false;
    }
    *ax = (buf[0] << 8) | buf[1];
    *ay = (buf[2] << 8) | buf[3];
    *az = (buf[4] << 8) | buf[5];
    *gx = (buf[8] << 8) | buf[9];
    *gy = (buf[10]<< 8) | buf[11];
    *gz = (buf[12]<< 8) | buf[13];
    lastRawAx = *ax; lastRawAy = *ay; lastRawAz = *az;
    lastRawGx = *gx; lastRawGy = *gy; lastRawGz = *gz;
    imuReadOkCount++;
    imuLastReadMs = HAL_GetTick();
    return true;
}

void IMU_GetAccelMps2(float *x, float *y, float *z) {
    float ax = (lastRawAx / 8192.0f) + accelBiasX;
    float ay = (lastRawAy / 8192.0f) + accelBiasY;
    float az = (lastRawAz / 8192.0f) + accelBiasZ;
    // Convert g→m/s²
    *x = ax * 9.80665f;
    *y = ay * 9.80665f;
    *z = az * 9.80665f;
}

void IMU_GetGyroDPS(float *x, float *y, float *z) {
    *x = (lastRawGx / 65.5f) + gyroBiasX;  // 65.5 LSB/(°/s) at ±500°/s
    *y = (lastRawGy / 65.5f) + gyroBiasY;
    *z = (lastRawGz / 65.5f) + gyroBiasZ;
}

bool IMU_CheckPlausibility(void) {
    if (!imuIdentityOk) {
        return false;
    }
    // Reject if any raw value is at extreme ±32767 (clipping)
    if (abs(lastRawAx) == 32767 || abs(lastRawAy) == 32767 || abs(lastRawAz) == 32767) return false;
    if (abs(lastRawGx) == 32767 || abs(lastRawGy) == 32767 || abs(lastRawGz) == 32767) return false;
    // Check accel magnitude: between 0.2g and 4g
    float mag = sqrtf(powf(lastRawAx/8192.0f + accelBiasX,2)
                    + powf(lastRawAy/8192.0f + accelBiasY,2)
                    + powf(lastRawAz/8192.0f + accelBiasZ,2));
    return (mag > 0.2f && mag < 4.0f);
}

bool IMU_IsIdentityOK(void)
{
    return imuIdentityOk;
}

uint8_t IMU_GetWhoAmI(void)
{
    return imuWhoAmI;
}

uint32_t IMU_GetReadOkCount(void)
{
    return imuReadOkCount;
}

uint32_t IMU_GetReadFailCount(void)
{
    return imuReadFailCount;
}

uint32_t IMU_GetLastReadMs(void)
{
    return imuLastReadMs;
}

HAL_StatusTypeDef IMU_GetLastHalStatus(void)
{
    return imuLastHalStatus;
}
