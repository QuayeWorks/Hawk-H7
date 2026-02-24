#ifndef SENSOR_DIAG_H
#define SENSOR_DIAG_H

#include <stdint.h>

typedef struct {
    uint16_t imu_hz;
    uint16_t mag_hz;
    uint16_t baro_hz;
    uint16_t gps_hz;
    uint16_t sonar_hz;
    uint16_t battery_hz;
    uint16_t rc_hz;
} SensorRateReport;

void SensorDiag_GetRates(SensorRateReport *out);

#endif // SENSOR_DIAG_H
