#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>

#include <stdbool.h>
#include "stm32h7xx_hal.h"

// ---------------- QMC5883L register definitions ----------------
#define QMC5883L_I2C_ADDR             (0x0D << 1)
#define QMC5883L_REG_DATA             0x00
#define QMC5883L_REG_STATUS           0x06
#define QMC5883L_REG_TEMP_LSB         0x07
#define QMC5883L_REG_TEMP_MSB         0x08
#define QMC5883L_REG_CONF1            0x09
#define QMC5883L_REG_CONF2            0x0A
#define QMC5883L_REG_SET_RESET_PERIOD 0x0B
#define QMC5883L_REG_CHIP_ID          0x0D
#define QMC5883L_STATUS_DRDY_MASK     0x01
#define QMC5883L_STATUS_OVL_MASK      0x02
#define QMC5883L_STATUS_DOR_MASK      0x04

/*
 * QMC5883L Magnetometer driver.
 * - Call Compass_Init(&hi2c1) once at startup.
 * - To configure offsets, call Compass_LoadCalibration(x,y,z soft, x,y,z hard).
 * - Call Compass_ReadRaw(&mx,&my,&mz) to fetch int16 raw counts.
 * - Compute heading with Compass_ComputeHeading(mx,my,mz).
 * - Validate with Compass_CheckMahalanobis(mx,my,mz,threshold).
 */

void  Compass_Init(I2C_HandleTypeDef *i2c_handle);
void  Compass_LoadCalibration(float softX, float softY, float softZ,
                              float hardX, float hardY, float hardZ);

bool  Compass_ReadRaw(int16_t *mx, int16_t *my, int16_t *mz);
float Compass_ComputeHeading(int16_t mx, int16_t my, int16_t mz);

// Returns true if the current reading is within Mahalanobis distance threshold
// (using the offsets from LoadCalibration and given scalar threshold).
bool  Compass_CheckMahalanobis(int16_t mx, int16_t my, int16_t mz, float threshold);

#endif // COMPASS_H
