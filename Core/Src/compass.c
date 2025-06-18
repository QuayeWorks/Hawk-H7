#include "compass.h"
#include "qmc5883l.h"   // low-level register definitions
#include <math.h>

static I2C_HandleTypeDef *hi2c_comp;
static float softX, softY, softZ;
static float hardX, hardY, hardZ;

void Compass_Init(I2C_HandleTypeDef *i2c_handle) {
    hi2c_comp = i2c_handle;
    // Reset
    uint8_t rst[2] = { QMC5883L_REG_CONF2, 0x80 };
    HAL_I2C_Master_Transmit(hi2c_comp, QMC5883L_I2C_ADDR, rst, 2, HAL_MAX_DELAY);
    HAL_Delay(10);
    // ID check (optional)
    uint8_t id;
    if (HAL_I2C_Mem_Read(hi2c_comp, QMC5883L_I2C_ADDR, QMC5883L_REG_CHIP_ID, 1, &id, 1, HAL_MAX_DELAY) == HAL_OK) {
        (void)id;
    }
    // Continuous mode 200Hz, ±8G
    uint8_t cfg1[2] = { QMC5883L_REG_CONF1, 0x1D };
    HAL_I2C_Master_Transmit(hi2c_comp, QMC5883L_I2C_ADDR, cfg1, 2, HAL_MAX_DELAY);
    uint8_t sr[2] = { QMC5883L_REG_SET_RESET_PERIOD, 0x01 };
    HAL_I2C_Master_Transmit(hi2c_comp, QMC5883L_I2C_ADDR, sr, 2, HAL_MAX_DELAY);
}

void Compass_LoadCalibration(float sX, float sY, float sZ,
                             float hX, float hY, float hZ)
{
    softX = sX; softY = sY; softZ = sZ;
    hardX = hX; hardY = hY; hardZ = hZ;
}

bool Compass_ReadRaw(int16_t *mx, int16_t *my, int16_t *mz) {
    // Check DRDY in status register
    uint8_t status;
    if (HAL_I2C_Mem_Read(hi2c_comp, QMC5883L_I2C_ADDR, QMC5883L_REG_STATUS, 1, &status, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    if (!(status & QMC5883L_STATUS_DRDY_MASK))
        return false;

    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(hi2c_comp, QMC5883L_I2C_ADDR, QMC5883L_REG_DATA, 1, raw, 6, HAL_MAX_DELAY) != HAL_OK)
        return false;

    int16_t X = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    int16_t Y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    int16_t Z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);

    // Apply hard‐iron
    float fx = ((float)X) - hardX;
    float fy = ((float)Y) - hardY;
    float fz = ((float)Z) - hardZ;
    // Apply soft‐iron scaling
    *mx = (int16_t)((fx - softX));
    *my = (int16_t)((fy - softY));
    *mz = (int16_t)((fz - softZ));
    return true;
}

float Compass_ComputeHeading(int16_t mx, int16_t my, int16_t mz) {
    // 2D heading: arctan2(Y, X)
    float heading = atan2f((float)my, (float)mx) * (180.0f / M_PI);
    return (heading < 0) ? (heading + 360.0f) : heading;
}

bool Compass_CheckMahalanobis(int16_t mx, int16_t my, int16_t mz, float threshold) {
    // Simple Euclidean distance in XYZ as a proxy for Mahalanobis
    float fx = (float)mx;
    float fy = (float)my;
    float fz = (float)mz;
    float dist = sqrtf(fx*fx + fy*fy + fz*fz);
    return (dist <= threshold);
}
