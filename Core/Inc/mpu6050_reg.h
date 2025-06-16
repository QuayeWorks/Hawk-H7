/*
 * mpu6050_reg.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

// ================== mpu6050_reg.h ==================
#ifndef MPU6050_REG_H
#define MPU6050_REG_H

#include <stdint.h>

// I²C 7-bit address for MPU6050; left‐shifted by 1 for HAL functions
#define MPU6050_I2C_ADDR        (0x68 << 1)

// Power management register
#define MPU6050_PWR_MGMT_1      0x6B

// Configuration registers
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

// Data output registers (high byte addresses)
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_ZOUT_H     0x47

#endif // MPU6050_REG_H
