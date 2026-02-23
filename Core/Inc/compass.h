#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/*
 * QMC5883L magnetometer driver.
 * - Raw reads are returned in sensor LSB units.
 * - Heading is tilt-compensated using Attitude roll/pitch.
 * - Calibration is applied as:
 *     corrected = (raw - hardIron) * softIron
 *   where softIron values of 0 are treated as 1 (identity).
 */

void  Compass_Init(I2C_HandleTypeDef *i2c_handle);
void  Compass_LoadCalibration(float softX, float softY, float softZ,
                              float hardX, float hardY, float hardZ);

bool  Compass_ReadRaw(int16_t *mx, int16_t *my, int16_t *mz);
float Compass_ComputeHeading(int16_t mx, int16_t my, int16_t mz);

bool  Compass_CheckMahalanobis(int16_t mx, int16_t my, int16_t mz, float threshold);
bool  Compass_IsStub(void);

uint8_t Compass_GetChipId(void);
uint8_t Compass_GetAddress7bit(void);
uint32_t Compass_GetLastReadMs(void);
uint32_t Compass_GetReadOkCount(void);
uint32_t Compass_GetReadFailCount(void);
HAL_StatusTypeDef Compass_GetLastHalStatus(void);

#endif // COMPASS_H
