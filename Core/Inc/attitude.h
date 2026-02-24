/*
 * attitude.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

/**
 * @brief  Update the internal attitude estimate from the latest IMU sample.
 * @param  ax,ay,az  accelerometer (m/s²)
 * @param  gx,gy,gz  gyro (°/s)
 */
void Attitude_Update(float ax, float ay, float az,
                     float gx, float gy, float gz);

typedef struct {
    float roll;
    float pitch;
    float yaw;
} AttitudeAngles;

/**
 * @brief  Get the current tilt angle from level, in degrees.
 * @return 0° when level, up to 90° at vertical.
 */
float Attitude_GetTiltAngle(void);

/**
 * @brief  Get the current angular rate magnitude, in degrees per second.
 * @return Scalar norm of roll/pitch/yaw rates.
 */
float Attitude_GetAngularRate(void);

AttitudeAngles Attitude_GetAngles(void);

#endif // ATTITUDE_H
