#include "baro.h"
#include "bmp388.h"    // low-level register + calib definitions
#include "settings.h"
#include <math.h>

static I2C_HandleTypeDef *hi2c_baro;
static BMP388_CalibData    calib;

// Trigger + read blocking
bool Baro_ReadPressure(float *pressure_hPa) {
    // Trigger one-shot conversion
    if (!BMP388_TriggerOneShot()) return false;
    // Wait up to 10 ms
    if (!BMP388_WaitForData(10)) return false;
    // Read raw
    float p, t;
    if (!BMP388_ReadOneShot(&p, &t)) return false;
    *pressure_hPa = p / 100.0f;
    return true;
}

float Baro_ComputeAltitude(float pressure_hPa) {
    // Use standard barometric formula with sea-level from settings:
    float seaLevel = Settings_GetBaroPressureOffset();
    // altitude = 44330 * (1 - (p/ps)^0.1903)
    return 44330.0f * (1.0f - powf(pressure_hPa / seaLevel, 0.1903f));
}

void Baro_Init(I2C_HandleTypeDef *i2c_handle) {
    hi2c_baro = i2c_handle;
    // Read calibration data from BMP388 into 'calib'
    BMP388_ReadCalibData(&calib);
    // Optionally set sea-level reference in settings if unset
}

bool Baro_Gps_Sonar_AltitudeAgreement(float baroAlt,
                                      float gpsAlt,
                                      float sonarAlt)
{
    // TODO: pull thresholds from Settings (e.g. Settings_GetBaroAgreementThreshold())
    // and compare fabs(baroAlt - gpsAlt) and fabs(baroAlt - sonarAlt)
    (void)baroAlt;   // silence unused‐param warnings
    (void)gpsAlt;
    (void)sonarAlt;
    return true;
}
