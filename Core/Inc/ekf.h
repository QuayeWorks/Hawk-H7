/*
 * ekf.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#ifndef EKF_H
#define EKF_H

#include <stdbool.h>

/*
 * EKF/UKF Sensor Fusion Skeleton
 *
 * Usage:
 *   • Call EKF_Init() at startup (after settings loaded).
 *   • On each new sensor sample, call one of:
 *       EKF_PublishIMU(ax,ay,az,gx,gy,gz);
 *       EKF_PublishMag(heading);
 *       EKF_PublishGPS(lat,lon,alt);
 *   • Then call EKF_UpdateSensors() once (e.g. at 200 Hz).
 *   • Query:
 *       EKF_AllInnovationGatesOK() → true if no gate was tripped
 *       EKF_CovariancesConverged() → true if filter has run ≥ min_obs_time
 */

/// Initialize the EKF internals and timers.
void EKF_Init(void);

/// Feed raw IMU data (m/s² & °/s) into the filter.
void EKF_PublishIMU(float ax, float ay, float az,
                    float gx, float gy, float gz);

/// Feed compass heading (° [0–360)) into the filter.
void EKF_PublishMag(float heading_deg);


/// Feed GPS position & altitude (degrees, degrees, m) into the filter.
void EKF_PublishGPS(double lat, double lon, float alt);

/// Run one full EKF update step. Call this after publishing all sensors.
void EKF_UpdateSensors(void);

/// Returns true if ALL innovation gate thresholds are satisfied this step.
bool EKF_AllInnovationGatesOK(void);

/// Returns true once the filter has run for at least the minimum observation time.
bool EKF_CovariancesConverged(void);

#endif // EKF_H
