/*
 * attitude.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "attitude.h"
#include <math.h>    // for acosf, sqrtf, etc.

// Simple stubs: you’ll replace these with your real AHRS/MADGWICK/Mahony filter.

static float _tiltDeg = 0.0f;
static float _angRateDps = 0.0f;

static AttitudeAngles _angles = {0,0,0};

void Attitude_Update(float ax, float ay, float az,
                     float gx, float gy, float gz)
{
    /*
     * A full AHRS solution is beyond the scope of this example, however we can
     * provide a very small complementary filter so the rest of the code base
     * has sensible values to work with.  Roll and pitch are derived from the
     * accelerometer while yaw is integrated from the gyroscope.  The angular
     * rate magnitude is taken directly from the gyro inputs.
     */

    // Gyro magnitude
    _angRateDps = sqrtf(gx*gx + gy*gy + gz*gz);

    // Tilt angle = angle between gravity vector and Z-axis
    float magA = sqrtf(ax*ax + ay*ay + az*az);
    if (magA > 0.0f) {
        float cosTilt = az / magA;
        if (cosTilt >  1.0f) cosTilt =  1.0f;
        if (cosTilt < -1.0f) cosTilt = -1.0f;
        _tiltDeg = acosf(cosTilt) * (180.0f / 3.14159265f);

        _angles.roll  = atan2f(ay, az) * (180.0f / 3.14159265f);
        _angles.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159265f);
    } else {
        _tiltDeg     = 0.0f;
        _angles.roll = 0.0f;
        _angles.pitch = 0.0f;
    }

    static float yawAccum = 0.0f;
    yawAccum += gz * 0.005f;   // assume ~200 Hz update rate
    _angles.yaw = yawAccum;
}

float Attitude_GetTiltAngle(void)
{
    return _tiltDeg;
}

float Attitude_GetAngularRate(void)
{
    return _angRateDps;
}

AttitudeAngles Attitude_GetAngles(void)
{
    return _angles;
}
