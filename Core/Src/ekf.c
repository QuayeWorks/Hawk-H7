/*
 * ekf.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#include "ekf.h"
#include "settings.h"
#include <math.h>
#include <string.h>

// Innovation values (Mahalanobis distances) for each sensor
static float innovGPS;
static float innovMag;
static float innovBaro;
// You may add innovFlow or others if you have an optical‐flow sensor

// Internal timer (ms) since EKF_Init
static uint32_t ekfTimeMs;

/// Initialize internal state
void EKF_Init(void) {
    ekfTimeMs = 0;
    innovGPS  = innovMag = innovBaro = 0.0f;
    // If you have state / covariance matrices, initialize them here
}

// Simple stubs: you’ll replace these with your actual EKF measurement updates

void EKF_PublishIMU(float ax, float ay, float az,
                    float gx, float gy, float gz)
{
    // Example: update attitude state using gyro/accel
    //   update_state_from_imu(ax, ay, az, gx, gy, gz);
    (void)ax; (void)ay; (void)az;
    (void)gx; (void)gy; (void)gz;
}

void EKF_PublishMag(float heading_deg)
{
    // Example: update EKF yaw using magnetometer
    //   update_state_from_mag(heading_deg);

    // For gating demonstration, compute a dummy Mahalanobis distance:
    // actual residual / threshold.
    float threshold = Settings_GetEKFInnovationMag();
    float residual = 0.0f;  // replace with (measuredHeading - predictedHeading)
    innovMag = (threshold > 0) ? (fabsf(residual)/threshold) : 0.0f;
}

void EKF_PublishBaro(float altitude_m)
{
    // Example: update EKF vertical position
    //   update_state_from_baro(altitude_m);

    float threshold = Settings_GetEKFInnovationBaro();
    float residual = 0.0f;  // replace with (measAlt - predAlt)
    innovBaro = (threshold > 0) ? (fabsf(residual)/threshold) : 0.0f;
}

void EKF_PublishGPS(double lat, double lon, float alt)
{
    // Example: update EKF position using GPS
    //   update_state_from_gps(lat, lon, alt);

    float threshold = Settings_GetEKFInnovationGPS();
    float residual = 0.0f;  // replace with computed position residual
    innovGPS = (threshold > 0) ? (fabsf(residual)/threshold) : 0.0f;
}

/// Run one EKF cycle: propagate and incorporate all queued measurements,
/// then advance the internal clock for covariance convergence checking.
void EKF_UpdateSensors(void)
{
    // 1) Propagate state by IMU (if using inertial nav)
    // 2) Incorporate magnetometer, baro, GPS measurements
    // 3) Compute actual Mahalanobis distances and store in innovGPS/innovMag/innovBaro

    // --- Stubs above already set innov* values in PublishXxx() ---

    // 4) Advance time
    ekfTimeMs += Settings_GetEKFMinObsTimeSec() * 1000; // or simply HAL_GetTick() in real code
}

/// Returns true if all innovation gates (GPS, Mag, Baro) PASS:
bool EKF_AllInnovationGatesOK(void)
{
    return (innovGPS  <= 1.0f)
        && (innovMag  <= 1.0f)
        && (innovBaro <= 1.0f);
}

/// Returns true after the filter has had time to converge:
bool EKF_CovariancesConverged(void)
{
    return ((float)ekfTimeMs / 1000.0f) >= Settings_GetEKFMinObsTimeSec();
}
