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
    // TODO: run your filter here to update _tiltDeg and _angRateDps.
    // For now, we’ll just record the magnitude of the gyro as the “rate”,
    // and compute tilt as arccos(az / |a|) in degrees.

    // Gyro magnitude
    _angRateDps = sqrtf(gx*gx + gy*gy + gz*gz);

    // Tilt angle = angle between gravity vector and Z-axis
    float magA = sqrtf(ax*ax + ay*ay + az*az);
    if (magA > 0.0f) {
        float cosTilt = az / magA;
        if (cosTilt >  1.0f) cosTilt =  1.0f;
        if (cosTilt < -1.0f) cosTilt = -1.0f;
        _tiltDeg = acosf(cosTilt) * (180.0f / 3.14159265f);
    } else {
        _tiltDeg = 0.0f;
    }

    // Fill in _angles.roll/_angles.pitch/_angles.yaw however you compute them:
    _angles.roll  = /*...*/ 0.0f;
    _angles.pitch = /*...*/ 0.0f;
    _angles.yaw   = /*...*/ 0.0f;
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
