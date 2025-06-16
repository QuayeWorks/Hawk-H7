#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

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
