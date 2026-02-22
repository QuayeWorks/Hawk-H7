#ifndef BARO_H
#define BARO_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define BARO_BMP280_ADDR_PRIMARY   0x76u
#define BARO_BMP280_ADDR_SECONDARY 0x77u

void Baro_Init(I2C_HandleTypeDef *i2c, uint8_t i2c_addr_7bit);
bool Baro_Update(uint32_t now_ms);

bool Baro_IsHealthy(void);
bool Baro_HasSample(void);
bool Baro_IsIdentityOK(void);

float Baro_GetPressureHpa(void);
float Baro_GetTemperatureC(void);
float Baro_GetAltitudeMeters(void);
float Baro_GetClimbRateMps(void);
uint8_t Baro_GetChipId(void);
uint32_t Baro_GetLastUpdateMs(void);
bool Baro_IsCalibrationLoaded(void);
uint32_t Baro_GetI2CFailCount(void);
uint32_t Baro_GetUpdateOkCount(void);
HAL_StatusTypeDef Baro_GetLastHalStatus(void);

#endif // BARO_H
