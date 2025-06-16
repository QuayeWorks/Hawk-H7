/*
 * baro.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#ifndef BARO_H
#define BARO_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

/*
 * BMP388 Barometer driver.
 * - Call Baro_Init(&hi2c1) once at startup.
 * - Call Baro_ReadPressure(&pressure_hPa) to get pressure in hPa (blocking ~5ms).
 * - Call Baro_GetAltitude(&alt_m) to compute altitude from the last pressure and
 *   the sea-level reference from settings.
 */

void    Baro_Init(I2C_HandleTypeDef *i2c_handle);

// Read a single pressure sample (blocking I2C + internal conversion).
// Returns true on success.
bool    Baro_ReadPressure(float *pressure_hPa);

// Compute altitude (m) from last‐read pressure, using
// sea-level reference from settings.
float   Baro_ComputeAltitude(float pressure_hPa);

/// @brief  Verifies baro, GPS, and sonar altitudes are mutually consistent.
/// @param  baroAlt   Baro‐derived altitude (m).
/// @param  gpsAlt    GPS‐derived altitude (m).
/// @param  sonarAlt  Sonar‐derived altitude above ground (m).
/// @return true if they agree within acceptable bounds (stub: always true).
bool Baro_Gps_Sonar_AltitudeAgreement(float baroAlt, float gpsAlt, float sonarAlt);

#endif // BARO_H
