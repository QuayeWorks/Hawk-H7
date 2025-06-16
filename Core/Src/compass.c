#include "compass.h"
#include "qmc5883l.h"   // low-level register definitions
#include <math.h>

static I2C_HandleTypeDef *hi2c_comp;
static float softX, softY, softZ;
static float hardX, hardY, hardZ;

void Compass_Init(I2C_HandleTypeDef *i2c_handle) {
    hi2c_comp = i2c_handle;
    // Put into continuous measurement mode, 200Hz, ±8 Gauss
    uint8_t cfg1[3] = { QMC5883L_REG_CONF1, 0x1D, 0x00 }; // OSR=512, RNG=8G, ODR=200Hz, MODE=Continuous
    HAL_I2C_Master_Transmit(hi2c_comp, QMC5883L_I2C_ADDR, cfg1, 3, HAL_MAX_DELAY);
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
    HAL_I2C_Mem_Read(hi2c_comp, QMC5883L_I2C_ADDR, QMC5883L_REG_STATUS, 1, &status, 1, HAL_MAX_DELAY);
    if (!(status & 0x01)) return false;

    uint8_t raw[6];
    uint8_t reg = QMC5883L_REG_DATA;
    HAL_I2C_Master_Transmit(hi2c_comp, QMC5883L_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c_comp, QMC5883L_I2C_ADDR, raw, 6, HAL_MAX_DELAY);

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
