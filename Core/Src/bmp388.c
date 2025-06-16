/*
 * bmp388.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


// Core/Src/bmp388.c
#include "bmp388.h"
#include "stm32h7xx_hal.h"
#include <string.h>    // for memset()

// You need to point this at the same I2C handle you used in baro.c:
extern I2C_HandleTypeDef hi2c1;

// Read the calibration registers into cdata
bool BMP388_ReadCalibData(BMP388_CalibData *cdata)
{
    // TODO: replace these stub addresses with the actual register blocks
    // The real BMP388 calibration sequence is about 21 bytes at 0x31…
    uint8_t buf[21] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, BMP388_I2C_ADDR,
                         0x31, 1, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
    {
        return false;
    }
    // Unpack buf into cdata->par_t1 … par_p11 here per datasheet
    // For now, zero them:
    memset(cdata, 0, sizeof(*cdata));
    return true;
}

bool BMP388_TriggerOneShot(void)
{
    // Write the one‐shot‐mode command to CTRL_MEAS register:
    uint8_t cmd = 0x01;  // ONE_SHOT bit
    return HAL_I2C_Mem_Write(&hi2c1, BMP388_I2C_ADDR,
                             0x1B, 1, &cmd, 1, HAL_MAX_DELAY) == HAL_OK;
}

bool BMP388_WaitForData(uint32_t timeout_ms)
{
    // Poll the status bit DRDY in the STATUS register (0x1D)
    uint8_t status = 0;
    uint32_t t0 = HAL_GetTick();
    do {
        if (HAL_I2C_Mem_Read(&hi2c1, BMP388_I2C_ADDR,
                             0x1D, 1, &status, 1, HAL_MAX_DELAY) != HAL_OK)
        {
            return false;
        }
        if (status & 0x80) {  // DRDY = bit7
            return true;
        }
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return false;
}

bool BMP388_ReadOneShot(float *pressure, float *temperature)
{
    // Read 3 bytes pressure (0x04..0x06) and 3 bytes temperature (0x07..0x09)
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(&hi2c1, BMP388_I2C_ADDR,
                         0x04, 1, buf, 6, HAL_MAX_DELAY) != HAL_OK)
    {
        return false;
    }
    // Combine and apply calibration here. For now, do a naive conversion:
    uint32_t rawP = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    uint32_t rawT = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
    *pressure    = rawP / 100.0f;   // dummy scaling to hPa
    *temperature = rawT / 100.0f;   // dummy °C
    return true;
}
