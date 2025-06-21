#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "attitude.h"

/*
 * Memory-based magnetometer replacement.
 * Heading is derived from the IMU yaw angle assuming the craft
 * starts facing true north.
 */

void  Compass_Init(I2C_HandleTypeDef *i2c_handle);
void  Compass_LoadCalibration(float softX, float softY, float softZ,
                              float hardX, float hardY, float hardZ);

bool  Compass_ReadRaw(int16_t *mx, int16_t *my, int16_t *mz);
float Compass_ComputeHeading(int16_t mx, int16_t my, int16_t mz);

bool  Compass_CheckMahalanobis(int16_t mx, int16_t my, int16_t mz, float threshold);

#endif // COMPASS_H
