// ==================== settings.h ====================
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include "ff.h"    // FatFS API

//-----------------------------------------------------------------------------
// Public API:
//
//   bool Settings_Init(const char *ini_path);
//     • ini_path: e.g. "0:/settings.ini" (or "SD:/settings.ini").
//     • Returns true if the file was successfully created or parsed.
//     • If the file does not exist, it will create it with defaults.
//
//   bool Settings_Save(void);
//     • Rewrites the entire INI file on the SD card with current in-memory
//       values. Returns false only if f_open/f_write/f_close fail.
//
//   After calling Settings_Init(), you can call any Settings_GetXXX()
//   to retrieve the current parameter, or Settings_SetXXX() to update a
//   setting (each setter automatically calls Settings_Save()).
//
//   If you want to batch multiple updates without immediately rewriting,
//   you can write to the internal variables (by adding your own non‐saving
//   helper, or disabling saves temporarily) and call Settings_Save() once
//   at the end. But by default, every setter calls Settings_Save().
//
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Initialization & Save
//-----------------------------------------------------------------------------
bool Settings_Init(const char *ini_path);
bool Settings_Save(void);
void Settings_RequestSave(void);
void Settings_ProcessDeferredSave(uint32_t now_ms, bool allow_write);
const char* Settings_GetIniPath(void);
bool Settings_IsSavePending(void);
uint32_t Settings_GetLastSaveAttemptMs(void);
bool Settings_GetLastSaveAttemptFailed(void);
uint32_t Settings_GetLastSaveSuccessMs(void);

//-----------------------------------------------------------------------------
// [IMU] → Accelerometer & Gyro Calibration
//-----------------------------------------------------------------------------
void    Settings_GetAccelBiases(float *bx, float *by, float *bz);
void    Settings_SetAccelBiases(float bx, float by, float bz);

void 	Settings_GetGyroBiases(float *bx, float *by, float *bz);
void 	Settings_SetGyroBiases(float bx, float by, float bz);

bool    Settings_GetCalibrateOnBoot(void);
void    Settings_SetCalibrateOnBoot(bool on);

uint16_t Settings_GetCalibSamples(void);
void     Settings_SetCalibSamples(uint16_t samples);

float   Settings_GetAccelTolG(void);
void    Settings_SetAccelTolG(float tol);

//-----------------------------------------------------------------------------
// [COMPASS] → Magnetometer Calibration
//-----------------------------------------------------------------------------
bool    Settings_GetCompassEnabled(void);
void    Settings_SetCompassEnabled(bool on);

float   Settings_GetMagSoftIronX(void);
void    Settings_SetMagSoftIronX(float v);
float   Settings_GetMagSoftIronY(void);
void    Settings_SetMagSoftIronY(float v);
float   Settings_GetMagSoftIronZ(void);
void    Settings_SetMagSoftIronZ(float v);

float   Settings_GetMagHardIronX(void);
void    Settings_SetMagHardIronX(float v);
float   Settings_GetMagHardIronY(void);
void    Settings_SetMagHardIronY(float v);
float   Settings_GetMagHardIronZ(void);
void    Settings_SetMagHardIronZ(float v);

float   Settings_GetMagMahaThreshold(void);
void    Settings_SetMagMahaThreshold(float v);

bool    Settings_GetCompassAutoSave(void);
void    Settings_SetCompassAutoSave(bool on);

//-----------------------------------------------------------------------------
// [BARO] → Barometer Configuration
//-----------------------------------------------------------------------------
bool    Settings_GetBaroEnabled(void);
void    Settings_SetBaroEnabled(bool on);

float   Settings_GetBaroPressureOffsetHpa(void);
void    Settings_SetBaroPressureOffsetHpa(float v);

float   Settings_GetBaroToleranceHpa(void);
void    Settings_SetBaroToleranceHpa(float v);

float   Settings_GetBaroAltOffsetM(void);
void    Settings_SetBaroAltOffsetM(float v);

//-----------------------------------------------------------------------------
// [SONAR] → Rangefinder/Sonar Settings
//-----------------------------------------------------------------------------
bool    Settings_GetSonarEnabled(void);
void    Settings_SetSonarEnabled(bool on);

float   Settings_GetSonarMaxDistance(void);
void    Settings_SetSonarMaxDistance(float v);

float   Settings_GetSonarMinDistance(void);
void    Settings_SetSonarMinDistance(float v);

uint16_t Settings_GetSonarUpdateRateHz(void);
void     Settings_SetSonarUpdateRateHz(uint16_t v);

float   Settings_GetSonarGroundOffset(void);
void    Settings_SetSonarGroundOffset(float v);

//-----------------------------------------------------------------------------
// [GPS] → GPS Configuration & Health Check
//-----------------------------------------------------------------------------
bool    Settings_GetGPSEnabled(void);
void    Settings_SetGPSEnabled(bool on);

uint32_t Settings_GetGPSBaud(void);
void     Settings_SetGPSBaud(uint32_t baud);

uint8_t  Settings_GetGPSMinSatellites(void);
void     Settings_SetGPSMinSatellites(uint8_t v);

float    Settings_GetGPSMaxHDOP(void);
void     Settings_SetGPSMaxHDOP(float v);

uint16_t Settings_GetGPSRequiredTimeSec(void);
void     Settings_SetGPSRequiredTimeSec(uint16_t v);

uint16_t Settings_GetGPSHoldAfterLossSec(void);
void     Settings_SetGPSHoldAfterLossSec(uint16_t v);

double   Settings_GetHomeLatitude(void);
void     Settings_SetHomeLatitude(double v);

double   Settings_GetHomeLongitude(void);
void     Settings_SetHomeLongitude(double v);

float    Settings_GetHomeAltitude(void);
void     Settings_SetHomeAltitude(float v);

//-----------------------------------------------------------------------------
// [RC] → Remote Control Input & Arming
//-----------------------------------------------------------------------------
bool    Settings_GetRCInvertPWM(void);
void    Settings_SetRCInvertPWM(bool on);

uint16_t Settings_GetRCRSSIMin(void);
void     Settings_SetRCRSSIMin(uint16_t v);

uint8_t  Settings_GetRCThrottleChannel(void);
void     Settings_SetRCThrottleChannel(uint8_t v);

uint8_t  Settings_GetRCRollChannel(void);
void     Settings_SetRCRollChannel(uint8_t v);

uint8_t  Settings_GetRCPitchChannel(void);
void     Settings_SetRCPitchChannel(uint8_t v);

uint8_t  Settings_GetRCYawChannel(void);
void     Settings_SetRCYawChannel(uint8_t v);

uint16_t Settings_GetRCArmStickThreshold(void);
void     Settings_SetRCArmStickThreshold(uint16_t v);

uint16_t Settings_GetRCDisarmStickThreshold(void);
void     Settings_SetRCDisarmStickThreshold(uint16_t v);

uint8_t  Settings_GetRCCenterThreshold(void);
void     Settings_SetRCCenterThreshold(uint8_t v);

bool     Settings_GetPrearmSwitchEnabled(void);
void     Settings_SetPrearmSwitchEnabled(bool on);

uint8_t  Settings_GetPrearmSwitchChannel(void);
void     Settings_SetPrearmSwitchChannel(uint8_t v);

uint16_t Settings_GetPrearmSwitchHigh(void);
void     Settings_SetPrearmSwitchHigh(uint16_t v);

//-----------------------------------------------------------------------------
// [BATTERY] → Battery & Power Management
//-----------------------------------------------------------------------------
bool     Settings_GetBatteryMonitorEnabled(void);
void     Settings_SetBatteryMonitorEnabled(bool on);

float    Settings_GetBatteryVoltDivider(void);
void     Settings_SetBatteryVoltDivider(float v);

uint8_t  Settings_GetBatteryCellCount(void);
void     Settings_SetBatteryCellCount(uint8_t v);

float    Settings_GetBatteryWarningVolts(void);
void     Settings_SetBatteryWarningVolts(float v);

float    Settings_GetBatteryCriticalVolts(void);
void     Settings_SetBatteryCriticalVolts(float v);

bool     Settings_GetBatteryAutoRTLEnabled(void);
void     Settings_SetBatteryAutoRTLEnabled(bool on);

uint32_t Settings_GetBatteryLogmAh(void);
void     Settings_SetBatteryLogmAh(uint32_t v);

uint32_t Settings_GetBatteryLowmAh(void);
void     Settings_SetBatteryLowmAh(uint32_t v);

uint32_t Settings_GetBatteryCritmAh(void);
void     Settings_SetBatteryCritmAh(uint32_t v);

uint8_t  Settings_GetINA219I2CAddress(void);
void     Settings_SetINA219I2CAddress(uint8_t v);

float    Settings_GetINA219ShuntOhm(void);
void     Settings_SetINA219ShuntOhm(float v);

uint32_t Settings_GetINA219MaxExpectedAmps(void);
void     Settings_SetINA219MaxExpectedAmps(uint32_t v);

//-----------------------------------------------------------------------------
// [ESTIMATOR] → EKF/UKF & Sensor Fusion Settings
//-----------------------------------------------------------------------------
bool     Settings_GetEKFEnabled(void);
void     Settings_SetEKFEnabled(bool on);

float    Settings_GetEKFInnovationGPS(void);
void     Settings_SetEKFInnovationGPS(float v);

float    Settings_GetEKFInnovationMag(void);
void     Settings_SetEKFInnovationMag(float v);

float    Settings_GetEKFInnovationBaro(void);
void     Settings_SetEKFInnovationBaro(float v);

float    Settings_GetEKFGyroNoiseSigma(void);
void     Settings_SetEKFGyroNoiseSigma(float v);

float    Settings_GetEKFAccelNoiseSigma(void);
void     Settings_SetEKFAccelNoiseSigma(float v);

float    Settings_GetEKFMinObsTimeSec(void);
void     Settings_SetEKFMinObsTimeSec(float v);

float    Settings_GetEKFPosVarLimit(void);
void     Settings_SetEKFPosVarLimit(float v);

float    Settings_GetEKFVelVarLimit(void);
void     Settings_SetEKFVelVarLimit(float v);

//-----------------------------------------------------------------------------
// [ARMING] → Pre-Arm Check Toggles & Limits
//-----------------------------------------------------------------------------
uint32_t Settings_GetArmingChecksMask(void);
void     Settings_SetArmingChecksMask(uint32_t v);

float    Settings_GetArmTiltLimitDeg(void);
void     Settings_SetArmTiltLimitDeg(float v);

uint16_t Settings_GetArmWaitTimeMs(void);
void     Settings_SetArmWaitTimeMs(uint16_t v);

bool     Settings_GetArmBeeper(void);
void     Settings_SetArmBeeper(bool on);

//-----------------------------------------------------------------------------
// [MOTORS] → Motor Output & Orientation Settings
//-----------------------------------------------------------------------------
uint8_t  Settings_GetMotorCount(void);
void     Settings_SetMotorCount(uint8_t v);

uint8_t  Settings_GetMotorPin(uint8_t motorIndex);
void     Settings_SetMotorPin(uint8_t motorIndex, uint8_t pin);

uint16_t Settings_GetMotorPWMMinUs(void);
void     Settings_SetMotorPWMMinUs(uint16_t v);

uint16_t Settings_GetMotorPWMMaxUs(void);
void     Settings_SetMotorPWMMaxUs(uint16_t v);

uint8_t  Settings_GetMotorIdlePercent(void);
void     Settings_SetMotorIdlePercent(uint8_t v);

uint8_t  Settings_GetMotorDir(uint8_t motorIndex);
void     Settings_SetMotorDir(uint8_t motorIndex, uint8_t dir);

bool     Settings_GetMotorAutoDetect(void);
void     Settings_SetMotorAutoDetect(bool on);

bool     Settings_GetMotorOrientationAutoSave(void);
void     Settings_SetMotorOrientationAutoSave(bool on);

//-----------------------------------------------------------------------------
// [FAILSAFE] → Failsafe Actions & Timing
//-----------------------------------------------------------------------------
uint8_t  Settings_GetFSBatteryAction(void);
void     Settings_SetFSBatteryAction(uint8_t v);

uint8_t  Settings_GetFSGPSLossAction(void);
void     Settings_SetFSGPSLossAction(uint8_t v);

uint8_t  Settings_GetFSRCLossAction(void);
void     Settings_SetFSRCLossAction(uint8_t v);

uint8_t  Settings_GetFSEKFLossAction(void);
void     Settings_SetFSEKFLossAction(uint8_t v);

uint8_t  Settings_GetFSGeofenceAction(void);
void     Settings_SetFSGeofenceAction(uint8_t v);

uint16_t Settings_GetFSBattWarnDelaySec(void);
void     Settings_SetFSBattWarnDelaySec(uint16_t v);

float    Settings_GetFSRCLossDelaySec(void);
void     Settings_SetFSRCLossDelaySec(float v);

float    Settings_GetFSGPSLossDelaySec(void);
void     Settings_SetFSGPSLossDelaySec(float v);

//-----------------------------------------------------------------------------
// [GEOfence] → Geofence Settings
//-----------------------------------------------------------------------------
bool     Settings_GetGeofenceEnabled(void);
void     Settings_SetGeofenceEnabled(bool on);

double   Settings_GetGeofenceCenterLat(void);
void     Settings_SetGeofenceCenterLat(double v);

double   Settings_GetGeofenceCenterLon(void);
void     Settings_SetGeofenceCenterLon(double v);

float    Settings_GetGeofenceCenterAlt(void);
void     Settings_SetGeofenceCenterAlt(float v);

float    Settings_GetGeofenceRadiusM(void);
void     Settings_SetGeofenceRadiusM(float v);

float    Settings_GetGeofenceAltMaxM(void);
void     Settings_SetGeofenceAltMaxM(float v);

float    Settings_GetGeofenceMaxDistanceM(void);
void     Settings_SetGeofenceMaxDistanceM(float v);

//-----------------------------------------------------------------------------
// [ATTITUDE] → Attitude Limits & PID Gains
//-----------------------------------------------------------------------------
float    Settings_GetMaxTiltDeg(void);
void     Settings_SetMaxTiltDeg(float v);

float    Settings_GetMaxRollRateDPS(void);
void     Settings_SetMaxRollRateDPS(float v);

float    Settings_GetMaxPitchRateDPS(void);
void     Settings_SetMaxPitchRateDPS(float v);

float    Settings_GetMaxYawRateDPS(void);
void     Settings_SetMaxYawRateDPS(float v);

float    Settings_GetPIDRollP(void);
void     Settings_SetPIDRollP(float v);

float    Settings_GetPIDRollI(void);
void     Settings_SetPIDRollI(float v);

float    Settings_GetPIDRollD(void);
void     Settings_SetPIDRollD(float v);

float    Settings_GetPIDPitchP(void);
void     Settings_SetPIDPitchP(float v);

float    Settings_GetPIDPitchI(void);
void     Settings_SetPIDPitchI(float v);

float    Settings_GetPIDPitchD(void);
void     Settings_SetPIDPitchD(float v);

float    Settings_GetPIDYawP(void);
void     Settings_SetPIDYawP(float v);

float    Settings_GetPIDYawI(void);
void     Settings_SetPIDYawI(float v);

float    Settings_GetPIDYawD(void);
void     Settings_SetPIDYawD(float v);

float    Settings_GetYawFF(void);
void     Settings_SetYawFF(float v);

float    Settings_GetFilterGyroCutoffHz(void);
void     Settings_SetFilterGyroCutoffHz(float v);

//-----------------------------------------------------------------------------
// [TELEMETRY] → Telemetry Configuration
//-----------------------------------------------------------------------------
bool     Settings_GetTelemetryEnabled(void);
void     Settings_SetTelemetryEnabled(bool on);

uint8_t  Settings_GetTelemetryProtocol(void);
void     Settings_SetTelemetryProtocol(uint8_t v);

uint32_t Settings_GetTelemetryBaud(void);
void     Settings_SetTelemetryBaud(uint32_t v);

uint16_t Settings_GetTelemetryStreamRateHz(void);
void     Settings_SetTelemetryStreamRateHz(uint16_t v);

bool     Settings_GetTelemetrySendAttitude(void);
void     Settings_SetTelemetrySendAttitude(bool on);

bool     Settings_GetTelemetrySendStatus(void);
void     Settings_SetTelemetrySendStatus(bool on);

bool     Settings_GetTelemetrySendSensors(void);
void     Settings_SetTelemetrySendSensors(bool on);

uint8_t  Settings_GetTelemetrySysID(void);
void     Settings_SetTelemetrySysID(uint8_t v);

//-----------------------------------------------------------------------------
// [MISC] → Miscellaneous Flags & Debug
//-----------------------------------------------------------------------------
bool     Settings_GetLEDEnabled(void);
void     Settings_SetLEDEnabled(bool on);

uint8_t  Settings_GetLEDMode(void);
void     Settings_SetLEDMode(uint8_t v);

bool     Settings_GetBuzzerEnabled(void);
void     Settings_SetBuzzerEnabled(bool on);

bool     Settings_GetDebugLogEnabled(void);
void     Settings_SetDebugLogEnabled(bool on);

uint8_t  Settings_GetLogLevel(void);
void     Settings_SetLogLevel(uint8_t v);

bool     Settings_GetFactoryReset(void);
void     Settings_SetFactoryReset(bool on);

#endif // SETTINGS_H
