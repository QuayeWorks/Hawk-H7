/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#include <stdbool.h>
#include "stm32h7xx_hal.h"

// ---------------- MPU6050 register definitions ----------------
#define MPU6050_I2C_ADDR        (0x68 << 1)
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_ZOUT_H     0x47

/*
 * MPU6050 6-axis IMU driver.
 * - Call IMU_Init(&hi2c1) once at startup.
 * - Use IMU_CalibrateOnBoot() in your pre-arm sequence to compute bias offsets.
 * - Call IMU_ReadRaw(&ax,&ay,&az,&gx,&gy,&gz) to fetch raw int16 data.
 * - Use IMU_GetAccelG(&x,&y,&z) and IMU_GetGyroDPS(&x,&y,&z) to get calibrated, scaled data.
 * - Call IMU_CheckPlausibility() in-flight to detect stuck/clipped sensors.
 */

void     IMU_Init(I2C_HandleTypeDef *i2c_handle);

// Level-board calibration: takes 'samples' readings, computes accel (g) biases and gyro (°/s) biases.
// Returns true if √(x²+y²+z²) ∈ [1−tolG,1+tolG] on each sample; otherwise false.
// Writes computed biases into the pointers.
bool     IMU_CalibrateOnBoot(uint16_t samples, float tolG,
                             float *accelBiasX, float *accelBiasY, float *accelBiasZ);

// Read one burst of raw accelerometer+gyro data (blocking I²C read).
// Returns true on success.
bool     IMU_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);

// Convert the last‐read raw accel into m/s² (accounts for accelBiasZ, etc.).
void     IMU_GetAccelMps2(float *x, float *y, float *z);

// Convert last‐read raw gyro into °/s (accounts for gyro biases).
void     IMU_GetGyroDPS(float *x, float *y, float *z);

// Quick plausibility check: no sensor reading is exactly zero or at full scale
// (±32767), and accelerometer magnitude is within [0.1g … 4g], for example.
// Returns true if OK, false if clipping/stuck.
bool     IMU_CheckPlausibility(void);

#endif // IMU_H
