#include "baro.h"
#include "bmp388.h"    // low-level register + calib definitions
#include "settings.h"
#include <math.h>

static I2C_HandleTypeDef *hi2c_baro;

// Trigger + read blocking
bool Baro_ReadPressure(float *pressure_hPa) {
    // Trigger one-shot conversion and read compensated value
    float p, t;
    if (!BMP388_ReadOneShot(&p, &t)) return false;
    *pressure_hPa = p; // already in hPa
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
    (void)hi2c_baro; // unused, kept for API symmetry
    BMP388_Init();
}

bool Baro_Gps_Sonar_AltitudeAgreement(float baroAlt,
                                      float gpsAlt,
                                      float sonarAlt)
{
    /* Simple sanity check comparing the barometric altitude with GPS and sonar
     * estimates.  A more sophisticated implementation could make the
     * thresholds configurable via the settings system.  For now we accept up to
     * five metres of disagreement between any pair of sensors. */

    const float maxDiff = 5.0f; // metres
    if (fabsf(baroAlt - gpsAlt)   > maxDiff) return false;
    if (fabsf(baroAlt - sonarAlt) > maxDiff) return false;
    return true;
}
