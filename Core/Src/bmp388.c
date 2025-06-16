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
    /* Calibration registers occupy 21 bytes starting at 0x31.  The layout is
     * documented in the BMP388 datasheet. */

    uint8_t buf[21];
    if (HAL_I2C_Mem_Read(&hi2c1, BMP388_I2C_ADDR,
                         0x31, 1, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
    {
        return false;
    }

    cdata->par_t1  = (uint16_t)((buf[1]  << 8) | buf[0]);
    cdata->par_t2  = (int16_t) ((buf[3]  << 8) | buf[2]);
    cdata->par_t3  = (int8_t)  buf[4];
    cdata->par_p1  = (uint16_t)((buf[6]  << 8) | buf[5]);
    cdata->par_p2  = (int16_t) ((buf[8]  << 8) | buf[7]);
    cdata->par_p3  = (int8_t)  buf[9];
    cdata->par_p4  = (int8_t)  buf[10];
    cdata->par_p5  = (uint16_t)((buf[12] << 8) | buf[11]);
    cdata->par_p6  = (int16_t) ((buf[14] << 8) | buf[13]);
    cdata->par_p7  = (int8_t)  buf[15];
    cdata->par_p8  = (int8_t)  buf[16];
    cdata->par_p9  = (int16_t) ((buf[18] << 8) | buf[17]);
    cdata->par_p10 = buf[19];
    cdata->par_p11 = buf[20];

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
