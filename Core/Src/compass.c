#include "compass.h"
#include "attitude.h"
#include <math.h>

void Compass_Init(I2C_HandleTypeDef *i2c_handle)
{
    (void)i2c_handle; // no hardware
}

void Compass_LoadCalibration(float sX, float sY, float sZ,
                             float hX, float hY, float hZ)
{
    (void)sX; (void)sY; (void)sZ;
    (void)hX; (void)hY; (void)hZ;
}

bool Compass_ReadRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
    if (mx) *mx = 0;
    if (my) *my = 0;
    if (mz) *mz = 0;
    return true; // always succeeds
}

float Compass_ComputeHeading(int16_t mx, int16_t my, int16_t mz)
{
    (void)mx; (void)my; (void)mz;
    AttitudeAngles a = Attitude_GetAngles();
    float heading = fmodf(a.yaw, 360.0f);
    if (heading < 0.0f)
        heading += 360.0f;
    return heading;
}

bool Compass_CheckMahalanobis(int16_t mx, int16_t my, int16_t mz, float threshold)
{
    (void)mx; (void)my; (void)mz; (void)threshold;
    return true;
}
