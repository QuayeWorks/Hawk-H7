/*
 * ekf.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

#include "ekf.h"
#include "settings.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <string.h>

// Innovation values (Mahalanobis distances) for each sensor
static float innovGPS;
static float innovMag;
static float innovBaro;

// Internal timer (ms) since EKF_Init
static uint32_t ekfTimeMs;
static uint32_t ekfLastUpdateMs;

static float ekfAltM;
static float ekfVelZMps;
static float pendingGpsAltM;
static float pendingBaroAltM;
static bool hasGpsUpdate;
static bool hasBaroUpdate;

/// Initialize internal state
void EKF_Init(void) {
    ekfTimeMs = 0;
    ekfLastUpdateMs = HAL_GetTick();
    innovGPS  = 0.0f;
    innovMag  = 0.0f;
    innovBaro = 0.0f;
    ekfAltM = 0.0f;
    ekfVelZMps = 0.0f;
    pendingGpsAltM = 0.0f;
    pendingBaroAltM = 0.0f;
    hasGpsUpdate = false;
    hasBaroUpdate = false;
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


void EKF_PublishGPS(double lat, double lon, float alt)
{
    // Example: update EKF position using GPS
    //   update_state_from_gps(lat, lon, alt);

    (void)lat;
    (void)lon;
    pendingGpsAltM = alt;
    hasGpsUpdate = true;
}

void EKF_PublishBaro(float alt_m)
{
    pendingBaroAltM = alt_m;
    hasBaroUpdate = true;
}

/// Run one EKF cycle: propagate and incorporate all queued measurements,
/// then advance the internal clock for covariance convergence checking.
void EKF_UpdateSensors(void)
{
    uint32_t now = HAL_GetTick();
    float dt_s = (float)(now - ekfLastUpdateMs) * 0.001f;
    float gpsThreshold;
    float baroThreshold;
    float residual;

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    } else if (dt_s > 0.1f) {
        dt_s = 0.1f;
    }

    ekfLastUpdateMs = now;
    ekfTimeMs += (uint32_t)(dt_s * 1000.0f);

    // 1) Propagate state by IMU (if using inertial nav)
    ekfAltM += ekfVelZMps * dt_s;

    // 2) Incorporate barometer measurement (primary vertical reference).
    if (hasBaroUpdate) {
        baroThreshold = Settings_GetEKFInnovationBaro();
        if (baroThreshold <= 0.0f) {
            baroThreshold = 1.0f;
        }

        residual = pendingBaroAltM - ekfAltM;
        innovBaro = fabsf(residual) / baroThreshold;
        if (innovBaro <= 1.0f) {
            ekfAltM += 0.35f * residual;
            ekfVelZMps += 0.08f * (residual / dt_s);
        }
        hasBaroUpdate = false;
    }

    // 3) Incorporate GPS altitude with lower gain.
    if (hasGpsUpdate) {
        gpsThreshold = Settings_GetEKFInnovationGPS();
        if (gpsThreshold <= 0.0f) {
            gpsThreshold = 1.0f;
        }

        residual = pendingGpsAltM - ekfAltM;
        innovGPS = fabsf(residual) / gpsThreshold;
        if (innovGPS <= 1.0f) {
            ekfAltM += 0.10f * residual;
            ekfVelZMps += 0.02f * (residual / dt_s);
        }
        hasGpsUpdate = false;
    }

    if (ekfVelZMps > 25.0f) {
        ekfVelZMps = 25.0f;
    } else if (ekfVelZMps < -25.0f) {
        ekfVelZMps = -25.0f;
    }
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
