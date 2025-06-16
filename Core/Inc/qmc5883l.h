/*
 * qmc5883l.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

// ================== qmc5883l.h ==================
#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdint.h>
#include <stdbool.h>

// I²C address for QMC5883L (7‐bit), shifted for HAL routines
#define QMC5883L_I2C_ADDR             (0x0D << 1)

// Data output registers (X, Y, Z) start here
#define QMC5883L_REG_DATA             0x00

// Status register: DRDY and OVL/ERR flags
#define QMC5883L_REG_STATUS           0x06

// Temperature registers (if you ever need them)
#define QMC5883L_REG_TEMP_LSB         0x07
#define QMC5883L_REG_TEMP_MSB         0x08

// Configuration registers
#define QMC5883L_REG_CONF1            0x09  // OSR, RNG, ODR, MODE
#define QMC5883L_REG_CONF2            0x0A  // Interrupt and sampling config

// Set/Reset period register
#define QMC5883L_REG_SET_RESET_PERIOD 0x0B

// Bit masks for STATUS register
#define QMC5883L_STATUS_DRDY_MASK     0x01  // Data Ready
#define QMC5883L_STATUS_OVL_MASK      0x02  // Overflow
#define QMC5883L_STATUS_DOR_MASK      0x04  // Data Overrun

#endif // QMC5883L_H
