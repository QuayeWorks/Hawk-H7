// ==================== settings.c ====================
#include "settings.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>      // for snprintf()

//-----------------------------------------------------------------------------
// Internal: Path to the INI file (as passed to Settings_Init). Example:
//    "0:/settings.ini"  (drive 0, root)
// while parsing/writing, we always refer to this string:
//-----------------------------------------------------------------------------
static char ini_filepath[64] = {0};

//-----------------------------------------------------------------------------
// Internal storage of all parameters (defaults match the “full settings.ini”)
//-----------------------------------------------------------------------------
static float  imu_accelBiasX      = 0.0f;
static float  imu_accelBiasY      = 0.0f;
static float  imu_accelBiasZ      = 1.0f;
static float  imu_gyroBiasX      = 0.0f;
static float  imu_gyroBiasY      = 0.5f;
static float  imu_gyroBiasZ      = 1.0f;
static bool   imu_calibrateOnBoot = true;
static uint16_t imu_calibSamples  = 200;
static float  imu_accelTolG       = 0.5f;

static bool   compass_enabled      = true;
static float  compass_softIronX    = 0.0f;
static float  compass_softIronY    = 0.0f;
static float  compass_softIronZ    = 0.0f;
static float  compass_hardIronX    = 0.0f;
static float  compass_hardIronY    = 0.0f;
static float  compass_hardIronZ    = 0.0f;
static float  compass_mahaThresh   = 4.0f;
static bool   compass_autoSave     = true;

static bool   baro_enabled         = true;
static float  baro_pressureOffset  = 1013.25f;
static float  baro_tolHpa          = 10.0f;
static float  baro_altOffset       = 0.0f;

static bool   sonar_enabled        = false;
static float  sonar_maxDistance    = 6.0f;
static float  sonar_minDistance    = 0.2f;
static uint16_t sonar_updateRateHz = 10;
static float  sonar_groundOffset   = 0.0f;

static bool   gps_enabled          = true;
static uint32_t gps_baud           = 57600;
static uint8_t  gps_minSatellites  = 6;
static float   gps_maxHdop         = 2.0f;
static uint16_t gps_requiredTime   = 5;
static uint16_t gps_holdAfterLoss  = 3;
static double  gps_homeLat         = 0.0;
static double  gps_homeLon         = 0.0;
static float   gps_homeAlt         = 0.0f;

static bool     rc_invertPWM         = false;
static uint16_t rc_rssiMin         = 120;
static uint8_t  rc_thrChannel      = 2;
static uint8_t  rc_rollChannel     = 0;
static uint8_t  rc_pitchChannel    = 1;
static uint8_t  rc_yawChannel      = 3;
static uint16_t rc_armStickThresh  = 1800;
static uint16_t rc_disarmStickThresh= 200;
static uint8_t  rc_centerThresh    = 50;
static bool     rc_prearmSwitchEnabled = false;
static uint8_t  rc_prearmSwitchChannel = 4;
static uint16_t rc_prearmSwitchHigh   = 1800;

static bool     batt_monEnabled     = true;
static float    batt_voltDivider    = 11.0f;
static uint8_t  batt_cellCount      = 4;
static float    batt_warnVolts      = 3.5f;
static float    batt_critVolts      = 3.3f;
static bool     batt_autoRTLEnabled = true;
static uint32_t batt_logmAh        = 2500;
static uint32_t batt_lowmAh        = 200;
static uint32_t batt_critmAh       = 100;
static uint8_t  ina219_i2c_addr    = 0x40;
static float    ina219_shuntOhm    = 0.1f;
static uint32_t ina219_maxAmps     = 30;

static bool     ekf_enabled         = true;
static float    ekf_innovGPS        = 5.0f;
static float    ekf_innovMag        = 4.0f;
static float    ekf_innovBaro       = 3.0f;
static float    ekf_gyroNoiseSigma  = 0.000174f;
static float    ekf_accelNoiseSigma = 0.01f;
static float    ekf_minObsTime      = 2.0f;
static float    ekf_posVarLimit     = 2.0f;
static float    ekf_velVarLimit     = 1.5f;

static uint32_t arming_checksMask  = 0x00FF; // bits 0–7 enabled
static float    arm_tiltLimitDeg   = 25.0f;
static uint16_t arm_waitTimeMs     = 1000;
static bool     arm_beeper         = true;

static uint8_t  motor_count        = 4;
static uint8_t  motor_pin[6]       = { 1, 2, 3, 4, 0, 0 };
static uint16_t motor_pwmMinUs     = 1000;
static uint16_t motor_pwmMaxUs     = 2000;
static uint8_t  motor_idlePercent  = 5;
static uint8_t  motor_dir[6]       = { 0, 1, 0, 1, 0, 1 };
static bool     motor_autoDetect   = false;
static bool     motor_autoSave     = true;

static uint8_t  fs_battAction      = 2;  // 0=Disabled,1=Warn,2=RTL,3=Land
static uint8_t  fs_gpsLossAction   = 1;  // 0=Disabled,1=AltHold,2=RTL,3=Land
static uint8_t  fs_rcLossAction    = 2;
static uint8_t  fs_ekfLossAction   = 3;
static uint8_t  fs_geofenceAction  = 2;
static uint16_t fs_battWarnDelaySec= 2;
static float    fs_rcLossDelaySec  = 0.5f;
static float    fs_gpsLossDelaySec = 1.0f;

static bool     geofence_enabled   = true;
static double   geofence_centerLat = 0.0;
static double   geofence_centerLon = 0.0;
static float    geofence_centerAlt = 0.0f;
static float    geofence_radiusM   = 500.0f;
static float    geofence_altMaxM   = 120.0f;
static float    geofence_maxDistM  = 600.0f;

static float    att_maxTiltDeg     = 60.0f;
static float    att_maxRollRateDPS = 200.0f;
static float    att_maxPitchRateDPS= 200.0f;
static float    att_maxYawRateDPS  = 200.0f;
static float    pid_roll_P         = 4.50f;
static float    pid_roll_I         = 0.020f;
static float    pid_roll_D         = 25.0f;
static float    pid_pitch_P        = 4.50f;
static float    pid_pitch_I        = 0.020f;
static float    pid_pitch_D        = 25.0f;
static float    pid_yaw_P          = 4.00f;
static float    pid_yaw_I          = 0.020f;
static float    pid_yaw_D          = 0.0f;
static float    yaw_FF             = 0.0f;
static float    filter_gyroCutoffHz= 90.0f;

static bool     tel_enabled        = true;
static uint8_t  tel_protocol       = 2;    // 1=Custom,2=MAVLink,3=MSP
static uint32_t tel_baud           = 57600;
static uint16_t tel_streamRateHz   = 10;
static bool     tel_sendAttitude   = true;
static bool     tel_sendStatus     = true;
static bool     tel_sendSensors    = false;
static uint8_t  tel_sysID         = 1;

static bool     misc_ledEnabled    = true;
static uint8_t  misc_ledMode       = 2;    // 0=Off,1=Solid,2=Blink on status change
static bool     misc_buzzerEnabled = true;
static bool     misc_debugLogEnabled = false;
static uint8_t  misc_logLevel      = 2;    // 0=Error,1=Warn,2=Info,3=Debug
static bool     misc_factoryReset  = false;

//-----------------------------------------------------------------------------
// Forward‐declarations for parsing helpers
//-----------------------------------------------------------------------------
static void    iniLoadDefaults(void);
static bool    iniParseLine(char *line, char *currentSection);
static void    trimWhitespace(char *s);
static bool    startsWith(const char *s, const char *prefix);

//-----------------------------------------------------------------------------
// Public: Settings_Init()
//   • ini_path example: "0:/settings.ini" or "SD:/settings.ini"
//   • Mount must be done before calling this.
//   • If file does not exist, it will be created with defaults.
//   • If file exists, parse it into the static variables.
//-----------------------------------------------------------------------------
bool Settings_Init(const char *ini_path) {
    FIL file;
    FRESULT fr;
    char lineBuf[256];
    char currentSection[64];

    // Store the path for future saves
    strncpy(ini_filepath, ini_path, sizeof(ini_filepath)-1);
    ini_filepath[sizeof(ini_filepath)-1] = '\0';

    // First, attempt to open the existing file for reading
    fr = f_open(&file, ini_filepath, FA_READ);
    if (fr == FR_NO_FILE) {
        // File doesn't exist → create with defaults
        return Settings_Save();
    }
    else if (fr != FR_OK) {
        // Some other error opening file
        return false;
    }

    // File opened successfully → parse it line by line
    currentSection[0] = '\0';
    while (f_gets(lineBuf, sizeof(lineBuf), &file)) {
        // Strip newline/carriage return
        char *p = lineBuf + strlen(lineBuf) - 1;
        while (p >= lineBuf && (*p == '\r' || *p == '\n')) {
            *p-- = '\0';
        }
        iniParseLine(lineBuf, currentSection);
    }
    f_close(&file);

    // If factory_reset was set, wipe it and rewrite defaults
    if (misc_factoryReset) {
        misc_factoryReset = false;
        return Settings_Save();
    }

    return true;
}

//-----------------------------------------------------------------------------
// Public: Settings_Save()
//   • Rewrites the entire INI to 'ini_filepath' using FatFS f_open/f_puts/f_close.
//-----------------------------------------------------------------------------
bool Settings_Save(void) {
    FIL file;
    FRESULT fr;
    char buf[128];

    // Open (or create) with write access, truncate existing
    fr = f_open(&file, ini_filepath, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        return false;
    }

    // --- Write the comment block header ---
        f_puts(";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n", &file);
        f_puts("; QuayeWorks Hawk-H7 Flight Controller Settings (settings.ini)\n", &file);
        f_puts("; ----------------------------------------------------------------------------\n", &file);
        f_puts("; This INI file stores all user‐tunable parameters, calibration data, and\n", &file);
        f_puts("; configuration flags for the flight controller. After firmware startup,\n", &file);
        f_puts("; any changed value will be saved back here so that future boots use the same\n", &file);
        f_puts("; settings.\n", &file);
        f_puts(";\n", &file);
        f_puts("; Sections:\n", &file);
        f_puts(";   [IMU]           → Accelerometer & Gyro calibration offsets\n", &file);
        f_puts(";   [COMPASS]       → Magnetometer calibration & enabling\n", &file);
        f_puts(";   [BARO]          → Barometer (BMP388) configuration\n", &file);
        f_puts(";   [SONAR]         → Rangefinder/sonar configuration\n", &file);
        f_puts(";   [GPS]           → GPS-related settings & thresholds\n", &file);
        f_puts(";   [RC]            → RC input mapping, RSSI thresholds, stick‐arming\n", &file);
        f_puts(";   [BATTERY]       → Battery measurement & failsafe thresholds\n", &file);
        f_puts(";   [ESTIMATOR]     → EKF/UKF thresholds and toggles\n", &file);
        f_puts(";   [ARMING]        → Pre‐arm check enabling/disabling & arming angles\n", &file);
        f_puts(";   [MOTORS]        → Motor output mapping, idle, orientation flags\n", &file);
        f_puts(";   [FAILSAFE]      → Failsafe actions & timing\n", &file);
        f_puts(";   [GEOfENCE]      → Geofence parameters\n", &file);
        f_puts(";   [ATTITUDE]      → Attitude limits (tilt, rates) and PID gains\n", &file);
        f_puts(";   [TELEMETRY]     → Telemetry configuration (link type, baud, messages)\n", &file);
        f_puts(";   [MISC]          → Miscellaneous flags (LED, buzzer, debug)\n", &file);
        f_puts(";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\r\n\n", &file);
        // --- End of header ---

    // -- [IMU] --
    f_puts("[IMU]\n", &file);
    f_puts("; ─── Accelerometer & Gyro Biases (computed during Power‐On Calibration) ───\n", &file);
    snprintf(buf, sizeof(buf), "accel_bias_x       = %.6f\n", imu_accelBiasX);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "accel_bias_y       = %.6f\n", imu_accelBiasY);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "accel_bias_z       = %.6f\n", imu_accelBiasZ);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gyro_bias_x       = %.6f\n", imu_gyroBiasX);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gyro_bias_y       = %.6f\n", imu_gyroBiasY);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gyro_bias_z       = %.6f\r\n", imu_gyroBiasZ);
    f_puts(buf, &file);
    f_puts("; If “calibrate_on_boot = 1”, the FC will re‐compute these biases at each\n", &file);
    f_puts("; power‐on using N samples from MPU6050. Otherwise it will use stored values.\n", &file);
    snprintf(buf, sizeof(buf), "calibrate_on_boot  = %d\n", imu_calibrateOnBoot ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "calib_samples      = %u\n", imu_calibSamples);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "accel_tol_g        = %.6f\n\n", imu_accelTolG);
    f_puts(buf, &file);

    // -- [COMPASS] --
    f_puts("[COMPASS]\n", &file);
    f_puts("; ─── Magnetometer Calibration & Checks ───\n", &file);
    snprintf(buf, sizeof(buf), "compass_enabled        = %d\n", compass_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "mag_soft_iron_x        = %.6f\n", compass_softIronX);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "mag_soft_iron_y        = %.6f\n", compass_softIronY);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "mag_soft_iron_z        = %.6f\n", compass_softIronZ);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "mag_hard_iron_x        = %.6f\n", compass_hardIronX);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "mag_hard_iron_y        = %.6f\n", compass_hardIronY);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "mag_hard_iron_z        = %.6f\r\n", compass_hardIronZ);
    f_puts(buf, &file);
    f_puts("; Threshold (Mahalanobis distance) beyond which magnetometer is considered an outlier\n", &file);
    snprintf(buf, sizeof(buf), "mag_maha_threshold     = %.6f\r\n", compass_mahaThresh);
    f_puts(buf, &file);
    f_puts("; If “compass_auto_save = 1”, any updated offsets from calibration tool\n", &file);
    f_puts("; will be saved here automatically.\n", &file);
    snprintf(buf, sizeof(buf), "compass_auto_save      = %d\n\n", compass_autoSave ? 1 : 0);
    f_puts(buf, &file);

    // -- [BARO] --
    f_puts("[BARO]\n", &file);
    f_puts("; ─── BMP388 Barometer Settings ───\n", &file);
    snprintf(buf, sizeof(buf), "baro_enabled           = %d\n", baro_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "baro_pressure_offset   = %.6f\n", baro_pressureOffset);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "baro_tol_hpa           = %.6f\r\n", baro_tolHpa);
    f_puts(buf, &file);
    f_puts("; “baro_alt_offset” can store a ground‐level altitude reference if needed\n", &file);
    snprintf(buf, sizeof(buf), "baro_alt_offset        = %.6f\n\n", baro_altOffset);
    f_puts(buf, &file);

    // -- [SONAR] --
    f_puts("[SONAR]\n", &file);
    f_puts("; ─── Rangefinder/Sonar Settings ───\n", &file);
    snprintf(buf, sizeof(buf), "sonar_enabled          = %d\n", sonar_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "sonar_max_distance     = %.6f\n", sonar_maxDistance);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "sonar_min_distance     = %.6f\n", sonar_minDistance);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "sonar_update_rate_hz   = %u\r\n", sonar_updateRateHz);
    f_puts(buf, &file);
    f_puts("; If “sonar_ground_z_offset” is set after calibration (landing), \n", &file);
    f_puts("; can store average ground distance\n", &file);
    snprintf(buf, sizeof(buf), "sonar_ground_z_offset  = %.6f\n\n", sonar_groundOffset);
    f_puts(buf, &file);

    // -- [GPS] --
    f_puts("[GPS]\n", &file);
    f_puts("; ─── GPS Configuration & Health Check ───\n", &file);
    snprintf(buf, sizeof(buf), "gps_enabled            = %d\n", gps_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gps_baud               = %lu\n", gps_baud);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gps_min_satellites     = %u\n", gps_minSatellites);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gps_max_hdop           = %.6f\n", gps_maxHdop);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gps_required_time_sec  = %u\n", gps_requiredTime);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "gps_hold_after_loss_sec= %u\r\n", gps_holdAfterLoss);
    f_puts(buf, &file);
    f_puts("; Coordinates of “home” can be stored after first fix if desired\n", &file);
    snprintf(buf, sizeof(buf), "home_latitude          = %.9f\n", gps_homeLat);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "home_longitude         = %.9f\n", gps_homeLon);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "home_altitude          = %.6f\n\n", gps_homeAlt);
    f_puts(buf, &file);

    // -- [RC] --
    f_puts("[RC]\n", &file);
    f_puts("; ─── RC Input & Arming Settings ───\n", &file);
    snprintf(buf, sizeof(buf), "rc_invert_pwm          = %d\n", rc_invertPWM ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_rssi_min            = %u\n", rc_rssiMin);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_throttle_channel    = %u\n", rc_thrChannel);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_roll_channel        = %u\n", rc_rollChannel);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_pitch_channel       = %u\n", rc_pitchChannel);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_yaw_channel         = %u\n", rc_yawChannel);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_arm_stick_threshold = %u\n", rc_armStickThresh);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_disarm_stick_thresh = %u\n", rc_disarmStickThresh);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "rc_center_threshold    = %u\r\n", rc_centerThresh);
    f_puts(buf, &file);
    f_puts("; If “prearm_switch_channel” is set, that channel must be high before arm sticks apply.\n", &file);
    snprintf(buf, sizeof(buf), "prearm_switch_enabled  = %d\n", rc_prearmSwitchEnabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "prearm_switch_channel  = %u\n", rc_prearmSwitchChannel);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "prearm_switch_high     = %u\n\n", rc_prearmSwitchHigh);
    f_puts(buf, &file);

    // -- [BATTERY] --
    f_puts("[BATTERY]\n", &file);
    f_puts("; ─── Battery & Power Management ───\n", &file);
    snprintf(buf, sizeof(buf), "battery_monitor_enabled = %d\n", batt_monEnabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_volt_divider    = %.6f\n", batt_voltDivider);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_cell_count      = %u\n", batt_cellCount);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_warning_volts   = %.6f\n", batt_warnVolts);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_critical_volts  = %.6f\n", batt_critVolts);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_auto_rtl_enabled= %d\n", batt_autoRTLEnabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_log_mAh         = %lu\n", batt_logmAh);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_low_mAh         = %lu\n", batt_lowmAh);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "battery_crit_mAh        = %lu\r\n", batt_critmAh);
    f_puts(buf, &file);
    f_puts("; Current sensing (INA219)\n", &file);
    snprintf(buf, sizeof(buf), "ina219_i2c_address      = 0x%02X\n", ina219_i2c_addr);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ina219_shunt_ohm        = %.6f\n", ina219_shuntOhm);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ina219_max_expected_amps= %lu\n\n", ina219_maxAmps);
    f_puts(buf, &file);

    // -- [ESTIMATOR] --
    f_puts("[ESTIMATOR]\n", &file);
    f_puts("; ─── EKF/UKF & Sensor Fusion Settings ───\n", &file);
    snprintf(buf, sizeof(buf), "ekf_enabled                = %d\n", ekf_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_innovation_gps         = %.6f\n", ekf_innovGPS);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_innovation_mag         = %.6f\n", ekf_innovMag);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_innovation_baro        = %.6f\n", ekf_innovBaro);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_gyro_noise_sigma       = %.6f\n", ekf_gyroNoiseSigma);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_accel_noise_sigma      = %.6f\n", ekf_accelNoiseSigma);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_min_obs_time_sec       = %.6f\n", ekf_minObsTime);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_pos_var_limit          = %.6f\n", ekf_posVarLimit);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "ekf_vel_var_limit          = %.6f\n\n", ekf_velVarLimit);
    f_puts(buf, &file);

    // -- [ARMING] --
    f_puts("[ARMING]\n", &file);
    f_puts("; ─── Pre‐Arm Check Toggles & Limits ───\n", &file);
    f_puts(";   Each bit in “arming_checks_mask” corresponds to a specific check:\n", &file);
    f_puts(";   Bit 0 = IMU healthy\n", &file);
    f_puts(";   Bit 1 = Compass healthy\n", &file);
    f_puts(";   Bit 2 = Baro healthy\n", &file);
    f_puts(";   Bit 3 = Sonar healthy\n", &file);
    f_puts(";   Bit 4 = GPS healthy\n", &file);
    f_puts(";   Bit 5 = RC healthy\n", &file);
    f_puts(";   Bit 6 = Battery healthy\n", &file);
    f_puts(";   Bit 7 = EKF healthy\n", &file);
    f_puts(";   Bit 8 = CPU timing OK\n", &file);
    f_puts(";   others = reserved\n", &file);
    f_puts(";   To disable a specific check, clear its bit (0=disable, 1=enable).\n", &file);
    snprintf(buf, sizeof(buf), "arming_checks_mask      = 0x%08lX\r\n", arming_checksMask);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "arm_tilt_limit_deg      = %.6f\n", arm_tiltLimitDeg);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "arm_wait_time_ms        = %u\r\n", arm_waitTimeMs);
    f_puts(buf, &file);
    f_puts("; If “arm_beeper = 1”, play arming confirmation tone\n", &file);
    snprintf(buf, sizeof(buf), "arm_beeper              = %d\n\n", arm_beeper ? 1 : 0);
    f_puts(buf, &file);

    // -- [MOTORS] --
    f_puts("[MOTORS]\n", &file);
    f_puts("; ─── Motor Output & Orientation Settings ───\n", &file);
    snprintf(buf, sizeof(buf), "motor_count             = %u\n", motor_count);
    f_puts(buf, &file);
    for (uint8_t i = 0; i < 6; i++) {
        char key[24];
        snprintf(key, sizeof(key), "motor%u_pin", (unsigned)(i + 1));
        uint8_t val = (i < motor_count) ? motor_pin[i] : 0;
        snprintf(buf, sizeof(buf), "%-22s = %u\n", key, val);
        f_puts(buf, &file);
    }
    f_puts("\n", &file);
    f_puts("; Throttle endpoints (us for PWM; for DShot use integer values 48–2047)\n", &file);
    snprintf(buf, sizeof(buf), "motor_pwm_min_us        = %u\n", motor_pwmMinUs);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "motor_pwm_max_us        = %u\n", motor_pwmMaxUs);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "motor_idle_percent      = %u\r\n", motor_idlePercent);
    f_puts(buf, &file);
    f_puts("; Motor rotation direction flags (0 = CW, 1 = CCW). Enforce alternation.\n", &file);
    for (uint8_t i = 0; i < 6; i++) {
        char key[24];
        snprintf(key, sizeof(key), "motor%u_dir", (unsigned)(i + 1));
        snprintf(buf, sizeof(buf), "%-22s = %u\n", key, motor_dir[i]);
        f_puts(buf, &file);
    }
    f_puts("\n", &file);
    f_puts("; If “motor_auto_detect = 1”, try to detect orientation via IMU feedback:\n", &file);
    snprintf(buf, sizeof(buf), "motor_auto_detect       = %d\r\n", motor_autoDetect ? 1 : 0);
    f_puts(buf, &file);
    f_puts("; Save motor orientation back to settings.ini if changed (auto‐save)\n", &file);
    snprintf(buf, sizeof(buf), "motor_orientation_auto_save = %d\n\n", motor_autoSave ? 1 : 0);
    f_puts(buf, &file);

    // -- [FAILSAFE] --
    f_puts("[FAILSAFE]\n", &file);
    f_puts("; ─── Failsafe Actions & Timing ───\n", &file);
    snprintf(buf, sizeof(buf), "fs_battery_action         = %u\n", fs_battAction);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_gps_loss_action        = %u\n", fs_gpsLossAction);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_rc_loss_action         = %u\n", fs_rcLossAction);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_ekf_loss_action        = %u\n", fs_ekfLossAction);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_geofence_action        = %u\n", fs_geofenceAction);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_batt_warn_delay_sec    = %u\n", fs_battWarnDelaySec);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_rc_loss_delay_sec      = %.6f\n", fs_rcLossDelaySec);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "fs_gps_loss_delay_sec     = %.6f\n\n", fs_gpsLossDelaySec);
    f_puts(buf, &file);

    // -- [GEOfENCE] --
    f_puts("[GEOfENCE]\n", &file);
    f_puts("; ─── Geofence (Home Boundary) Settings ───\n", &file);
    snprintf(buf, sizeof(buf), "geofence_enabled          = %d\n", geofence_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "geofence_center_lat       = %.9f\n", geofence_centerLat);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "geofence_center_lon       = %.9f\n", geofence_centerLon);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "geofence_center_alt       = %.6f\n", geofence_centerAlt);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "geofence_radius_m         = %.6f\n", geofence_radiusM);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "geofence_alt_max_m        = %.6f\r\n", geofence_altMaxM);
    f_puts(buf, &file);
    f_puts("; Secondary boundary (beyond which we force Land immediately)\n", &file);
    snprintf(buf, sizeof(buf), "geofence_max_distance_m   = %.6f\n\n", geofence_maxDistM);
    f_puts(buf, &file);

    // -- [ATTITUDE] --
    f_puts("[ATTITUDE]\n", &file);
    f_puts("; ─── Attitude Limits & PID Gains ───\n", &file);
    snprintf(buf, sizeof(buf), "max_tilt_deg              = %.6f\n", att_maxTiltDeg);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "max_roll_rate_dps         = %.6f\n", att_maxRollRateDPS);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "max_pitch_rate_dps        = %.6f\n", att_maxPitchRateDPS);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "max_yaw_rate_dps          = %.6f\r\n", att_maxYawRateDPS);
    f_puts(buf, &file);
    f_puts("; PID Gains for Attitude Controller (example defaults)\n", &file);
    snprintf(buf, sizeof(buf), "pid_roll_p                = %.6f\n", pid_roll_P);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_roll_i                = %.6f\n", pid_roll_I);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_roll_d                = %.6f\r\n", pid_roll_D);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_pitch_p               = %.6f\n", pid_pitch_P);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_pitch_i               = %.6f\n", pid_pitch_I);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_pitch_d               = %.6f\r\n", pid_pitch_D);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_yaw_p                 = %.6f\n", pid_yaw_P);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_yaw_i                 = %.6f\n", pid_yaw_I);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "pid_yaw_d                 = %.6f\r\n", pid_yaw_D);
    f_puts(buf, &file);
    f_puts("; Feedforward and filtering parameters (if any)\n", &file);
    snprintf(buf, sizeof(buf), "yaw_ff                    = %.6f\n", yaw_FF);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "filter_gyro_cutoff_hz     = %.6f\n\n", filter_gyroCutoffHz);
    f_puts(buf, &file);

    // -- [TELEMETRY] --
    f_puts("[TELEMETRY]\n", &file);
    f_puts("; ─── Telemetry Link & Messages ───\n", &file);
    snprintf(buf, sizeof(buf), "telemetry_enabled         = %d\n", tel_enabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_protocol        = %u\n", tel_protocol);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_baud            = %lu\n", tel_baud);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_stream_rate_hz  = %u\n", tel_streamRateHz);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_send_attitude   = %d\n", tel_sendAttitude ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_send_status     = %d\n", tel_sendStatus ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_send_sensors    = %d\n", tel_sendSensors ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "telemetry_sysid           = %u\n\n", tel_sysID);
    f_puts(buf, &file);

    // -- [MISC] --
    f_puts("[MISC]\n", &file);
    f_puts("; ─── Miscellaneous Flags & Debug ───\n", &file);
    snprintf(buf, sizeof(buf), "led_enabled               = %d\n", misc_ledEnabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "led_mode                  = %u\n", misc_ledMode);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "buzzer_enabled            = %d\n", misc_buzzerEnabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "debug_log_enabled         = %d\n", misc_debugLogEnabled ? 1 : 0);
    f_puts(buf, &file);
    snprintf(buf, sizeof(buf), "log_level                 = %u\r\n", misc_logLevel);
    f_puts(buf, &file);
    f_puts("; If “factory_reset = 1”, the next reboot will wipe ini and restore defaults.\n", &file);
    snprintf(buf, sizeof(buf), "factory_reset             = %d\n", misc_factoryReset ? 1 : 0);
    f_puts(buf, &file);

    f_close(&file);
    return true;
}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// INTERNAL: Trim leading/trailing whitespace (in‐place)
//-----------------------------------------------------------------------------
static void trimWhitespace(char *s) {
    // Left‐trim
    while (*s && (*s == ' ' || *s == '\t')) {
        memmove(s, s+1, strlen(s));
    }
    // Right‐trim
    char *end = s + strlen(s) - 1;
    while (end >= s && (*end == ' ' || *end == '\t')) {
        *end-- = '\0';
    }
}

//-----------------------------------------------------------------------------
// INTERNAL: Case‐insensitive “does s start with prefix?” (prefix is ASCII)
//-----------------------------------------------------------------------------
static bool startsWith(const char *s, const char *prefix) {
    while (*prefix) {
        char a = *s++; if (a >= 'A' && a <= 'Z') a += ('a' - 'A');
        char b = *prefix++; if (b >= 'A' && b <= 'Z') b += ('a' - 'A');
        if (a != b) return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
// INTERNAL: Parse one “key = value” line, given a currentSection string.
//   Modifies the static variables based on what it finds. Ignores unknown keys.
//-----------------------------------------------------------------------------
static bool iniParseLine(char *line, char *currentSection) {
    trimWhitespace(line);
    if (line[0] == ';' || line[0] == '#' || line[0] == '\0') {
        // Comment or blank line
        return true;
    }

    // Section header?
    if (line[0] == '[') {
        char *endBracket = strchr(line, ']');
        if (!endBracket) return false;
        *endBracket = '\0';
        strncpy(currentSection, line + 1, 63);
        currentSection[63] = '\0';
        trimWhitespace(currentSection);
        return true;
    }

    // Attempt to split “key = value”
    char *eq = strchr(line, '=');
    if (!eq) return false;
    *eq = '\0';
    char *key = line;
    char *value = eq + 1;
    trimWhitespace(key);
    trimWhitespace(value);

    // Now check each SECTION
    if (strcmp(currentSection, "IMU") == 0) {
        if (startsWith(key, "accel_bias_x")) {
            imu_accelBiasX = strtof(value, NULL);
        }
        else if (startsWith(key, "accel_bias_y")) {
            imu_accelBiasY = strtof(value, NULL);
        }
        else if (startsWith(key, "accel_bias_z")) {
            imu_accelBiasZ = strtof(value, NULL);
        }
        else if (startsWith(key, "gyro_bias_x")) {
            imu_gyroBiasZ = strtof(value, NULL);
        }
        else if (startsWith(key, "gyro_bias_y")) {
            imu_gyroBiasY = strtof(value, NULL);
        }
        else if (startsWith(key, "gyro_bias_z")) {
            imu_gyroBiasZ = strtof(value, NULL);
        }
        else if (startsWith(key, "calibrate_on_boot")) {
            imu_calibrateOnBoot = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "calib_samples")) {
            imu_calibSamples = (uint16_t)strtol(value, NULL, 10);
        }
        else if (startsWith(key, "accel_tol_g")) {
            imu_accelTolG = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "COMPASS") == 0) {
        if (startsWith(key, "compass_enabled")) {
            compass_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "mag_soft_iron_x")) {
            compass_softIronX = strtof(value, NULL);
        }
        else if (startsWith(key, "mag_soft_iron_y")) {
            compass_softIronY = strtof(value, NULL);
        }
        else if (startsWith(key, "mag_soft_iron_z")) {
            compass_softIronZ = strtof(value, NULL);
        }
        else if (startsWith(key, "mag_hard_iron_x")) {
            compass_hardIronX = strtof(value, NULL);
        }
        else if (startsWith(key, "mag_hard_iron_y")) {
            compass_hardIronY = strtof(value, NULL);
        }
        else if (startsWith(key, "mag_hard_iron_z")) {
            compass_hardIronZ = strtof(value, NULL);
        }
        else if (startsWith(key, "mag_maha_threshold")) {
            compass_mahaThresh = strtof(value, NULL);
        }
        else if (startsWith(key, "compass_auto_save")) {
            compass_autoSave = (strtol(value, NULL, 10) != 0);
        }
        return true;
    }
    else if (strcmp(currentSection, "BARO") == 0) {
        if (startsWith(key, "baro_enabled")) {
            baro_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "baro_pressure_offset")) {
            baro_pressureOffset = strtof(value, NULL);
        }
        else if (startsWith(key, "baro_tol_hpa")) {
            baro_tolHpa = strtof(value, NULL);
        }
        else if (startsWith(key, "baro_alt_offset")) {
            baro_altOffset = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "SONAR") == 0) {
        if (startsWith(key, "sonar_enabled")) {
            sonar_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "sonar_max_distance")) {
            sonar_maxDistance = strtof(value, NULL);
        }
        else if (startsWith(key, "sonar_min_distance")) {
            sonar_minDistance = strtof(value, NULL);
        }
        else if (startsWith(key, "sonar_update_rate_hz")) {
            sonar_updateRateHz = (uint16_t)strtol(value, NULL, 10);
        }
        else if (startsWith(key, "sonar_ground_z_offset")) {
            sonar_groundOffset = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "GPS") == 0) {
        if (startsWith(key, "gps_enabled")) {
            gps_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "gps_baud")) {
            gps_baud = (uint32_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "gps_min_satellites")) {
            gps_minSatellites = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "gps_max_hdop")) {
            gps_maxHdop = strtof(value, NULL);
        }
        else if (startsWith(key, "gps_required_time_sec")) {
            gps_requiredTime = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "gps_hold_after_loss_sec")) {
            gps_holdAfterLoss = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "home_latitude")) {
            gps_homeLat = strtod(value, NULL);
        }
        else if (startsWith(key, "home_longitude")) {
            gps_homeLon = strtod(value, NULL);
        }
        else if (startsWith(key, "home_altitude")) {
            gps_homeAlt = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "RC") == 0) {
        if (startsWith(key, "rc_invert_pwm")) {
            rc_invertPWM = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "rc_rssi_min")) {
            rc_rssiMin = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_throttle_channel")) {
            rc_thrChannel = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_roll_channel")) {
            rc_rollChannel = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_pitch_channel")) {
            rc_pitchChannel = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_yaw_channel")) {
            rc_yawChannel = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_arm_stick_threshold")) {
            rc_armStickThresh = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_disarm_stick_thresh")) {
            rc_disarmStickThresh = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "rc_center_threshold")) {
            rc_centerThresh = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "prearm_switch_enabled")) {
            rc_prearmSwitchEnabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "prearm_switch_channel")) {
            rc_prearmSwitchChannel = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "prearm_switch_high")) {
            rc_prearmSwitchHigh = (uint16_t)strtoul(value, NULL, 10);
        }
        return true;
    }
    else if (strcmp(currentSection, "BATTERY") == 0) {
        if (startsWith(key, "battery_monitor_enabled")) {
            batt_monEnabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "battery_volt_divider")) {
            batt_voltDivider = strtof(value, NULL);
        }
        else if (startsWith(key, "battery_cell_count")) {
            batt_cellCount = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "battery_warning_volts")) {
            batt_warnVolts = strtof(value, NULL);
        }
        else if (startsWith(key, "battery_critical_volts")) {
            batt_critVolts = strtof(value, NULL);
        }
        else if (startsWith(key, "battery_auto_rtl_enabled")) {
            batt_autoRTLEnabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "battery_log_mAh")) {
            batt_logmAh = (uint32_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "battery_low_mAh")) {
            batt_lowmAh = (uint32_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "battery_crit_mAh")) {
            batt_critmAh = (uint32_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "ina219_i2c_address")) {
            if (startsWith(value, "0x") || startsWith(value, "0X")) {
                ina219_i2c_addr = (uint8_t)strtoul(value + 2, NULL, 16);
            } else {
                ina219_i2c_addr = (uint8_t)strtoul(value, NULL, 10);
            }
        }
        else if (startsWith(key, "ina219_shunt_ohm")) {
            ina219_shuntOhm = strtof(value, NULL);
        }
        else if (startsWith(key, "ina219_max_expected_amps")) {
            ina219_maxAmps = (uint32_t)strtoul(value, NULL, 10);
        }
        return true;
    }
    else if (strcmp(currentSection, "ESTIMATOR") == 0) {
        if (startsWith(key, "ekf_enabled")) {
            ekf_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "ekf_innovation_gps")) {
            ekf_innovGPS = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_innovation_mag")) {
            ekf_innovMag = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_innovation_baro")) {
            ekf_innovBaro = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_gyro_noise_sigma")) {
            ekf_gyroNoiseSigma = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_accel_noise_sigma")) {
            ekf_accelNoiseSigma = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_min_obs_time_sec")) {
            ekf_minObsTime = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_pos_var_limit")) {
            ekf_posVarLimit = strtof(value, NULL);
        }
        else if (startsWith(key, "ekf_vel_var_limit")) {
            ekf_velVarLimit = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "ARMING") == 0) {
        if (startsWith(key, "arming_checks_mask")) {
            if (startsWith(value, "0x") || startsWith(value, "0X")) {
                arming_checksMask = (uint32_t)strtoul(value + 2, NULL, 16);
            } else {
                arming_checksMask = (uint32_t)strtoul(value, NULL, 10);
            }
        }
        else if (startsWith(key, "arm_tilt_limit_deg")) {
            arm_tiltLimitDeg = strtof(value, NULL);
        }
        else if (startsWith(key, "arm_wait_time_ms")) {
            arm_waitTimeMs = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "arm_beeper")) {
            arm_beeper = (strtol(value, NULL, 10) != 0);
        }
        return true;
    }
    else if (strcmp(currentSection, "MOTORS") == 0) {
        if (startsWith(key, "motor_count")) {
            motor_count = (uint8_t)strtoul(value, NULL, 10);
            if (motor_count > 6) motor_count = 6;
        }
        else if (startsWith(key, "motor")) {
            int idx;
            if (sscanf(key, "motor%d_pin", &idx) == 1) {
                if (idx >= 1 && idx <= 6) {
                    motor_pin[idx - 1] = (uint8_t)strtoul(value, NULL, 10);
                }
            }
            else if (sscanf(key, "motor%d_dir", &idx) == 1) {
                if (idx >= 1 && idx <= 6) {
                    motor_dir[idx - 1] = (uint8_t)strtoul(value, NULL, 10);
                }
            }
        }
        else if (startsWith(key, "motor_pwm_min_us")) {
            motor_pwmMinUs = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "motor_pwm_max_us")) {
            motor_pwmMaxUs = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "motor_idle_percent")) {
            motor_idlePercent = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "motor_auto_detect")) {
            motor_autoDetect = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "motor_orientation_auto_save")) {
            motor_autoSave = (strtol(value, NULL, 10) != 0);
        }
        return true;
    }
    else if (strcmp(currentSection, "FAILSAFE") == 0) {
        if (startsWith(key, "fs_battery_action")) {
            fs_battAction = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "fs_gps_loss_action")) {
            fs_gpsLossAction = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "fs_rc_loss_action")) {
            fs_rcLossAction = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "fs_ekf_loss_action")) {
            fs_ekfLossAction = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "fs_geofence_action")) {
            fs_geofenceAction = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "fs_batt_warn_delay_sec")) {
            fs_battWarnDelaySec = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "fs_rc_loss_delay_sec")) {
            fs_rcLossDelaySec = strtof(value, NULL);
        }
        else if (startsWith(key, "fs_gps_loss_delay_sec")) {
            fs_gpsLossDelaySec = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "GEOfENCE") == 0) {
        if (startsWith(key, "geofence_enabled")) {
            geofence_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "geofence_center_lat")) {
            geofence_centerLat = strtod(value, NULL);
        }
        else if (startsWith(key, "geofence_center_lon")) {
            geofence_centerLon = strtod(value, NULL);
        }
        else if (startsWith(key, "geofence_center_alt")) {
            geofence_centerAlt = strtof(value, NULL);
        }
        else if (startsWith(key, "geofence_radius_m")) {
            geofence_radiusM = strtof(value, NULL);
        }
        else if (startsWith(key, "geofence_alt_max_m")) {
            geofence_altMaxM = strtof(value, NULL);
        }
        else if (startsWith(key, "geofence_max_distance_m")) {
            geofence_maxDistM = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "ATTITUDE") == 0) {
        if (startsWith(key, "max_tilt_deg")) {
            att_maxTiltDeg = strtof(value, NULL);
        }
        else if (startsWith(key, "max_roll_rate_dps")) {
            att_maxRollRateDPS = strtof(value, NULL);
        }
        else if (startsWith(key, "max_pitch_rate_dps")) {
            att_maxPitchRateDPS = strtof(value, NULL);
        }
        else if (startsWith(key, "max_yaw_rate_dps")) {
            att_maxYawRateDPS = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_roll_p")) {
            pid_roll_P = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_roll_i")) {
            pid_roll_I = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_roll_d")) {
            pid_roll_D = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_pitch_p")) {
            pid_pitch_P = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_pitch_i")) {
            pid_pitch_I = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_pitch_d")) {
            pid_pitch_D = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_yaw_p")) {
            pid_yaw_P = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_yaw_i")) {
            pid_yaw_I = strtof(value, NULL);
        }
        else if (startsWith(key, "pid_yaw_d")) {
            pid_yaw_D = strtof(value, NULL);
        }
        else if (startsWith(key, "yaw_ff")) {
            yaw_FF = strtof(value, NULL);
        }
        else if (startsWith(key, "filter_gyro_cutoff_hz")) {
            filter_gyroCutoffHz = strtof(value, NULL);
        }
        return true;
    }
    else if (strcmp(currentSection, "TELEMETRY") == 0) {
        if (startsWith(key, "telemetry_enabled")) {
            tel_enabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "telemetry_protocol")) {
            tel_protocol = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "telemetry_baud")) {
            tel_baud = (uint32_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "telemetry_stream_rate_hz")) {
            tel_streamRateHz = (uint16_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "telemetry_send_attitude")) {
            tel_sendAttitude = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "telemetry_send_status")) {
            tel_sendStatus = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "telemetry_send_sensors")) {
            tel_sendSensors = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "telemetry_sysid")) {
            tel_sysID = (uint8_t)strtoul(value, NULL, 10);
        }
        return true;
    }
    else if (strcmp(currentSection, "MISC") == 0) {
        if (startsWith(key, "led_enabled")) {
            misc_ledEnabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "led_mode")) {
            misc_ledMode = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "buzzer_enabled")) {
            misc_buzzerEnabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "debug_log_enabled")) {
            misc_debugLogEnabled = (strtol(value, NULL, 10) != 0);
        }
        else if (startsWith(key, "log_level")) {
            misc_logLevel = (uint8_t)strtoul(value, NULL, 10);
        }
        else if (startsWith(key, "factory_reset")) {
            misc_factoryReset = (strtol(value, NULL, 10) != 0);
        }
        return true;
    }

    // Unknown section/key → ignore
    return true;
}

//-----------------------------------------------------------------------------
// Below: all the getters & setters. Each setter calls Settings_Save() to
// automatically persist the new value immediately. If you want to batch
// multiple changes before saving, you can modify the static vars directly
// and call Settings_Save() once at the end.
//-----------------------------------------------------------------------------

// [IMU]
void Settings_GetAccelBiases(float *bx, float *by, float *bz) {
    *bx = imu_accelBiasX;  *by = imu_accelBiasY;  *bz = imu_accelBiasZ;
}
void Settings_SetAccelBiases(float bx, float by, float bz) {
    imu_accelBiasX = bx; imu_accelBiasY = by; imu_accelBiasZ = bz;
    Settings_Save();
}
void Settings_GetGyroBiases(float *bx, float *by, float *bz) {
    *bx = imu_gyroBiasX;  *by = imu_gyroBiasY;  *bz = imu_gyroBiasZ;
}
void Settings_SetGyroBiases(float bx, float by, float bz) {
    imu_gyroBiasX = bx; imu_gyroBiasY = by; imu_gyroBiasZ = bz;
    Settings_Save();
}
bool Settings_GetCalibrateOnBoot(void) {
    return imu_calibrateOnBoot;
}
void Settings_SetCalibrateOnBoot(bool on) {
    imu_calibrateOnBoot = on;
    Settings_Save();
}

uint16_t Settings_GetCalibSamples(void) {
    return imu_calibSamples;
}
void Settings_SetCalibSamples(uint16_t samples) {
    imu_calibSamples = samples;
    Settings_Save();
}

float Settings_GetAccelTolG(void) {
    return imu_accelTolG;
}
void Settings_SetAccelTolG(float tol) {
    imu_accelTolG = tol;
    Settings_Save();
}

// [COMPASS]
bool Settings_GetCompassEnabled(void) {
    return compass_enabled;
}
void Settings_SetCompassEnabled(bool on) {
    compass_enabled = on;
    Settings_Save();
}

float Settings_GetMagSoftIronX(void) { return compass_softIronX; }
void  Settings_SetMagSoftIronX(float v) { compass_softIronX = v; Settings_Save(); }
float Settings_GetMagSoftIronY(void) { return compass_softIronY; }
void  Settings_SetMagSoftIronY(float v) { compass_softIronY = v; Settings_Save(); }
float Settings_GetMagSoftIronZ(void) { return compass_softIronZ; }
void  Settings_SetMagSoftIronZ(float v) { compass_softIronZ = v; Settings_Save(); }

float Settings_GetMagHardIronX(void) { return compass_hardIronX; }
void  Settings_SetMagHardIronX(float v) { compass_hardIronX = v; Settings_Save(); }
float Settings_GetMagHardIronY(void) { return compass_hardIronY; }
void  Settings_SetMagHardIronY(float v) { compass_hardIronY = v; Settings_Save(); }
float Settings_GetMagHardIronZ(void) { return compass_hardIronZ; }
void  Settings_SetMagHardIronZ(float v) { compass_hardIronZ = v; Settings_Save(); }

float Settings_GetMagMahaThreshold(void) { return compass_mahaThresh; }
void  Settings_SetMagMahaThreshold(float v) { compass_mahaThresh = v; Settings_Save(); }

bool Settings_GetCompassAutoSave(void) {
    return compass_autoSave;
}
void Settings_SetCompassAutoSave(bool on) {
    compass_autoSave = on;
    Settings_Save();
}

// [BARO]
bool Settings_GetBaroEnabled(void) {
    return baro_enabled;
}
void Settings_SetBaroEnabled(bool on) {
    baro_enabled = on;
    Settings_Save();
}

float Settings_GetBaroPressureOffset(void) {
    return baro_pressureOffset;
}
void Settings_SetBaroPressureOffset(float v) {
    baro_pressureOffset = v;
    Settings_Save();
}

float Settings_GetBaroTolHpa(void) {
    return baro_tolHpa;
}
void Settings_SetBaroTolHpa(float v) {
    baro_tolHpa = v;
    Settings_Save();
}

float Settings_GetBaroAltOffset(void) {
    return baro_altOffset;
}
void Settings_SetBaroAltOffset(float v) {
    baro_altOffset = v;
    Settings_Save();
}

// [SONAR]
bool Settings_GetSonarEnabled(void) {
    return sonar_enabled;
}
void Settings_SetSonarEnabled(bool on) {
    sonar_enabled = on;
    Settings_Save();
}

float Settings_GetSonarMaxDistance(void) {
    return sonar_maxDistance;
}
void Settings_SetSonarMaxDistance(float v) {
    sonar_maxDistance = v;
    Settings_Save();
}

float Settings_GetSonarMinDistance(void) {
    return sonar_minDistance;
}
void Settings_SetSonarMinDistance(float v) {
    sonar_minDistance = v;
    Settings_Save();
}

uint16_t Settings_GetSonarUpdateRateHz(void) {
    return sonar_updateRateHz;
}
void Settings_SetSonarUpdateRateHz(uint16_t v) {
    sonar_updateRateHz = v;
    Settings_Save();
}

float Settings_GetSonarGroundOffset(void) {
    return sonar_groundOffset;
}
void Settings_SetSonarGroundOffset(float v) {
    sonar_groundOffset = v;
    Settings_Save();
}

// [GPS]
bool Settings_GetGPSEnabled(void) {
    return gps_enabled;
}
void Settings_SetGPSEnabled(bool on) {
    gps_enabled = on;
    Settings_Save();
}

uint32_t Settings_GetGPSBaud(void) {
    return gps_baud;
}
void Settings_SetGPSBaud(uint32_t v) {
    gps_baud = v;
    Settings_Save();
}

uint8_t Settings_GetGPSMinSatellites(void) {
    return gps_minSatellites;
}
void Settings_SetGPSMinSatellites(uint8_t v) {
    gps_minSatellites = v;
    Settings_Save();
}

float Settings_GetGPSMaxHDOP(void) {
    return gps_maxHdop;
}
void Settings_SetGPSMaxHDOP(float v) {
    gps_maxHdop = v;
    Settings_Save();
}

uint16_t Settings_GetGPSRequiredTimeSec(void) {
    return gps_requiredTime;
}
void Settings_SetGPSRequiredTimeSec(uint16_t v) {
    gps_requiredTime = v;
    Settings_Save();
}

uint16_t Settings_GetGPSHoldAfterLossSec(void) {
    return gps_holdAfterLoss;
}
void Settings_SetGPSHoldAfterLossSec(uint16_t v) {
    gps_holdAfterLoss = v;
    Settings_Save();
}

double Settings_GetHomeLatitude(void) {
    return gps_homeLat;
}
void Settings_SetHomeLatitude(double v) {
    gps_homeLat = v;
    Settings_Save();
}

double Settings_GetHomeLongitude(void) {
    return gps_homeLon;
}
void Settings_SetHomeLongitude(double v) {
    gps_homeLon = v;
    Settings_Save();
}

float Settings_GetHomeAltitude(void) {
    return gps_homeAlt;
}
void Settings_SetHomeAltitude(float v) {
    gps_homeAlt = v;
    Settings_Save();
}

// [RC]
bool Settings_GetRCInvertPWM(void) {
    return rc_invertPWM;
}
void Settings_SetRCInvertPWM(bool on) {
    rc_invertPWM = on;
    Settings_Save();
}

uint16_t Settings_GetRCRSSIMin(void) {
    return rc_rssiMin;
}
void Settings_SetRCRSSIMin(uint16_t v) {
    rc_rssiMin = v;
    Settings_Save();
}

uint8_t Settings_GetRCThrottleChannel(void) {
    return rc_thrChannel;
}
void Settings_SetRCThrottleChannel(uint8_t v) {
    rc_thrChannel = v;
    Settings_Save();
}

uint8_t Settings_GetRCRollChannel(void) {
    return rc_rollChannel;
}
void Settings_SetRCRollChannel(uint8_t v) {
    rc_rollChannel = v;
    Settings_Save();
}

uint8_t Settings_GetRCPitchChannel(void) {
    return rc_pitchChannel;
}
void Settings_SetRCPitchChannel(uint8_t v) {
    rc_pitchChannel = v;
    Settings_Save();
}

uint8_t Settings_GetRCYawChannel(void) {
    return rc_yawChannel;
}
void Settings_SetRCYawChannel(uint8_t v) {
    rc_yawChannel = v;
    Settings_Save();
}

uint16_t Settings_GetRCArmStickThreshold(void) {
    return rc_armStickThresh;
}
void Settings_SetRCArmStickThreshold(uint16_t v) {
    rc_armStickThresh = v;
    Settings_Save();
}

uint16_t Settings_GetRCDisarmStickThreshold(void) {
    return rc_disarmStickThresh;
}
void Settings_SetRCDisarmStickThreshold(uint16_t v) {
    rc_disarmStickThresh = v;
    Settings_Save();
}

uint8_t Settings_GetRCCenterThreshold(void) {
    return rc_centerThresh;
}
void Settings_SetRCCenterThreshold(uint8_t v) {
    rc_centerThresh = v;
    Settings_Save();
}

bool Settings_GetPrearmSwitchEnabled(void) {
    return rc_prearmSwitchEnabled;
}
void Settings_SetPrearmSwitchEnabled(bool on) {
    rc_prearmSwitchEnabled = on;
    Settings_Save();
}

uint8_t Settings_GetPrearmSwitchChannel(void) {
    return rc_prearmSwitchChannel;
}
void Settings_SetPrearmSwitchChannel(uint8_t v) {
    rc_prearmSwitchChannel = v;
    Settings_Save();
}

uint16_t Settings_GetPrearmSwitchHigh(void) {
    return rc_prearmSwitchHigh;
}
void Settings_SetPrearmSwitchHigh(uint16_t v) {
    rc_prearmSwitchHigh = v;
    Settings_Save();
}

// [BATTERY]
bool Settings_GetBatteryMonitorEnabled(void) {
    return batt_monEnabled;
}
void Settings_SetBatteryMonitorEnabled(bool on) {
    batt_monEnabled = on;
    Settings_Save();
}

float Settings_GetBatteryVoltDivider(void) {
    return batt_voltDivider;
}
void Settings_SetBatteryVoltDivider(float v) {
    batt_voltDivider = v;
    Settings_Save();
}

uint8_t Settings_GetBatteryCellCount(void) {
    return batt_cellCount;
}
void Settings_SetBatteryCellCount(uint8_t v) {
    batt_cellCount = v;
    Settings_Save();
}

float Settings_GetBatteryWarningVolts(void) {
    return batt_warnVolts;
}
void Settings_SetBatteryWarningVolts(float v) {
    batt_warnVolts = v;
    Settings_Save();
}

float Settings_GetBatteryCriticalVolts(void) {
    return batt_critVolts;
}
void Settings_SetBatteryCriticalVolts(float v) {
    batt_critVolts = v;
    Settings_Save();
}

bool Settings_GetBatteryAutoRTLEnabled(void) {
    return batt_autoRTLEnabled;
}
void Settings_SetBatteryAutoRTLEnabled(bool on) {
    batt_autoRTLEnabled = on;
    Settings_Save();
}

uint32_t Settings_GetBatteryLogmAh(void) {
    return batt_logmAh;
}
void Settings_SetBatteryLogmAh(uint32_t v) {
    batt_logmAh = v;
    Settings_Save();
}

uint32_t Settings_GetBatteryLowmAh(void) {
    return batt_lowmAh;
}
void Settings_SetBatteryLowmAh(uint32_t v) {
    batt_lowmAh = v;
    Settings_Save();
}

uint32_t Settings_GetBatteryCritmAh(void) {
    return batt_critmAh;
}
void Settings_SetBatteryCritmAh(uint32_t v) {
    batt_critmAh = v;
    Settings_Save();
}

uint8_t Settings_GetINA219I2CAddress(void) {
    return ina219_i2c_addr;
}
void Settings_SetINA219I2CAddress(uint8_t v) {
    ina219_i2c_addr = v;
    Settings_Save();
}

float Settings_GetINA219ShuntOhm(void) {
    return ina219_shuntOhm;
}
void Settings_SetINA219ShuntOhm(float v) {
    ina219_shuntOhm = v;
    Settings_Save();
}

uint32_t Settings_GetINA219MaxExpectedAmps(void) {
    return ina219_maxAmps;
}
void Settings_SetINA219MaxExpectedAmps(uint32_t v) {
    ina219_maxAmps = v;
    Settings_Save();
}

// [ESTIMATOR]
bool Settings_GetEKFEnabled(void) {
    return ekf_enabled;
}
void Settings_SetEKFEnabled(bool on) {
    ekf_enabled = on;
    Settings_Save();
}

float Settings_GetEKFInnovationGPS(void) {
    return ekf_innovGPS;
}
void Settings_SetEKFInnovationGPS(float v) {
    ekf_innovGPS = v;
    Settings_Save();
}

float Settings_GetEKFInnovationMag(void) {
    return ekf_innovMag;
}
void Settings_SetEKFInnovationMag(float v) {
    ekf_innovMag = v;
    Settings_Save();
}

float Settings_GetEKFInnovationBaro(void) {
    return ekf_innovBaro;
}
void Settings_SetEKFInnovationBaro(float v) {
    ekf_innovBaro = v;
    Settings_Save();
}

float Settings_GetEKFGyroNoiseSigma(void) {
    return ekf_gyroNoiseSigma;
}
void Settings_SetEKFGyroNoiseSigma(float v) {
    ekf_gyroNoiseSigma = v;
    Settings_Save();
}

float Settings_GetEKFAccelNoiseSigma(void) {
    return ekf_accelNoiseSigma;
}
void Settings_SetEKFAccelNoiseSigma(float v) {
    ekf_accelNoiseSigma = v;
    Settings_Save();
}

float Settings_GetEKFMinObsTimeSec(void) {
    return ekf_minObsTime;
}
void Settings_SetEKFMinObsTimeSec(float v) {
    ekf_minObsTime = v;
    Settings_Save();
}

float Settings_GetEKFPosVarLimit(void) {
    return ekf_posVarLimit;
}
void Settings_SetEKFPosVarLimit(float v) {
    ekf_posVarLimit = v;
    Settings_Save();
}

float Settings_GetEKFVelVarLimit(void) {
    return ekf_velVarLimit;
}
void Settings_SetEKFVelVarLimit(float v) {
    ekf_velVarLimit = v;
    Settings_Save();
}

// [ARMING]
uint32_t Settings_GetArmingChecksMask(void) {
    return arming_checksMask;
}
void Settings_SetArmingChecksMask(uint32_t v) {
    arming_checksMask = v;
    Settings_Save();
}

float Settings_GetArmTiltLimitDeg(void) {
    return arm_tiltLimitDeg;
}
void Settings_SetArmTiltLimitDeg(float v) {
    arm_tiltLimitDeg = v;
    Settings_Save();
}

uint16_t Settings_GetArmWaitTimeMs(void) {
    return arm_waitTimeMs;
}
void Settings_SetArmWaitTimeMs(uint16_t v) {
    arm_waitTimeMs = v;
    Settings_Save();
}

bool Settings_GetArmBeeper(void) {
    return arm_beeper;
}
void Settings_SetArmBeeper(bool on) {
    arm_beeper = on;
    Settings_Save();
}

// [MOTORS]
uint8_t Settings_GetMotorCount(void) {
    return motor_count;
}
void Settings_SetMotorCount(uint8_t v) {
    if (v > 6) v = 6;
    motor_count = v;
    Settings_Save();
}

uint8_t Settings_GetMotorPin(uint8_t index) {
    return (index < 6) ? motor_pin[index] : 0;
}
void Settings_SetMotorPin(uint8_t index, uint8_t pin) {
    if (index < 6) {
        motor_pin[index] = pin;
        Settings_Save();
    }
}

uint16_t Settings_GetMotorPWMMinUs(void) {
    return motor_pwmMinUs;
}
void Settings_SetMotorPWMMinUs(uint16_t v) {
    motor_pwmMinUs = v;
    Settings_Save();
}

uint16_t Settings_GetMotorPWMMaxUs(void) {
    return motor_pwmMaxUs;
}
void Settings_SetMotorPWMMaxUs(uint16_t v) {
    motor_pwmMaxUs = v;
    Settings_Save();
}

uint8_t Settings_GetMotorIdlePercent(void) {
    return motor_idlePercent;
}
void Settings_SetMotorIdlePercent(uint8_t v) {
    motor_idlePercent = v;
    Settings_Save();
}

uint8_t Settings_GetMotorDir(uint8_t index) {
    return (index < 6) ? motor_dir[index] : 0;
}
void Settings_SetMotorDir(uint8_t index, uint8_t dir) {
    if (index < 6) {
        motor_dir[index] = dir;
        Settings_Save();
    }
}

bool Settings_GetMotorAutoDetect(void) {
    return motor_autoDetect;
}
void Settings_SetMotorAutoDetect(bool on) {
    motor_autoDetect = on;
    Settings_Save();
}

bool Settings_GetMotorOrientationAutoSave(void) {
    return motor_autoSave;
}
void Settings_SetMotorOrientationAutoSave(bool on) {
    motor_autoSave = on;
    Settings_Save();
}

// [FAILSAFE]
uint8_t Settings_GetFSBatteryAction(void) {
    return fs_battAction;
}
void Settings_SetFSBatteryAction(uint8_t v) {
    fs_battAction = v;
    Settings_Save();
}

uint8_t Settings_GetFSGPSLossAction(void) {
    return fs_gpsLossAction;
}
void Settings_SetFSGPSLossAction(uint8_t v) {
    fs_gpsLossAction = v;
    Settings_Save();
}

uint8_t Settings_GetFSRCLossAction(void) {
    return fs_rcLossAction;
}
void Settings_SetFSRCLossAction(uint8_t v) {
    fs_rcLossAction = v;
    Settings_Save();
}

uint8_t Settings_GetFSEKFLossAction(void) {
    return fs_ekfLossAction;
}
void Settings_SetFSEKFLossAction(uint8_t v) {
    fs_ekfLossAction = v;
    Settings_Save();
}

uint8_t Settings_GetFSGeofenceAction(void) {
    return fs_geofenceAction;
}
void Settings_SetFSGeofenceAction(uint8_t v) {
    fs_geofenceAction = v;
    Settings_Save();
}

uint16_t Settings_GetFSBattWarnDelaySec(void) {
    return fs_battWarnDelaySec;
}
void Settings_SetFSBattWarnDelaySec(uint16_t v) {
    fs_battWarnDelaySec = v;
    Settings_Save();
}

float Settings_GetFSRCLossDelaySec(void) {
    return fs_rcLossDelaySec;
}
void Settings_SetFSRCLossDelaySec(float v) {
    fs_rcLossDelaySec = v;
    Settings_Save();
}

float Settings_GetFSGPSLossDelaySec(void) {
    return fs_gpsLossDelaySec;
}
void Settings_SetFSGPSLossDelaySec(float v) {
    fs_gpsLossDelaySec = v;
    Settings_Save();
}

// [GEOfENCE]
bool Settings_GetGeofenceEnabled(void) {
    return geofence_enabled;
}
void Settings_SetGeofenceEnabled(bool on) {
    geofence_enabled = on;
    Settings_Save();
}

double Settings_GetGeofenceCenterLat(void) {
    return geofence_centerLat;
}
void Settings_SetGeofenceCenterLat(double v) {
    geofence_centerLat = v;
    Settings_Save();
}

double Settings_GetGeofenceCenterLon(void) {
    return geofence_centerLon;
}
void Settings_SetGeofenceCenterLon(double v) {
    geofence_centerLon = v;
    Settings_Save();
}

float Settings_GetGeofenceCenterAlt(void) {
    return geofence_centerAlt;
}
void Settings_SetGeofenceCenterAlt(float v) {
    geofence_centerAlt = v;
    Settings_Save();
}

float Settings_GetGeofenceRadiusM(void) {
    return geofence_radiusM;
}
void Settings_SetGeofenceRadiusM(float v) {
    geofence_radiusM = v;
    Settings_Save();
}

float Settings_GetGeofenceAltMaxM(void) {
    return geofence_altMaxM;
}
void Settings_SetGeofenceAltMaxM(float v) {
    geofence_altMaxM = v;
    Settings_Save();
}

float Settings_GetGeofenceMaxDistanceM(void) {
    return geofence_maxDistM;
}
void Settings_SetGeofenceMaxDistanceM(float v) {
    geofence_maxDistM = v;
    Settings_Save();
}

// [ATTITUDE]
float Settings_GetMaxTiltDeg(void) {
    return att_maxTiltDeg;
}
void Settings_SetMaxTiltDeg(float v) {
    att_maxTiltDeg = v;
    Settings_Save();
}

float Settings_GetMaxRollRateDPS(void) {
    return att_maxRollRateDPS;
}
void Settings_SetMaxRollRateDPS(float v) {
    att_maxRollRateDPS = v;
    Settings_Save();
}

float Settings_GetMaxPitchRateDPS(void) {
    return att_maxPitchRateDPS;
}
void Settings_SetMaxPitchRateDPS(float v) {
    att_maxPitchRateDPS = v;
    Settings_Save();
}

float Settings_GetMaxYawRateDPS(void) {
    return att_maxYawRateDPS;
}
void Settings_SetMaxYawRateDPS(float v) {
    att_maxYawRateDPS = v;
    Settings_Save();
}

float Settings_GetPIDRollP(void) {
    return pid_roll_P;
}
void Settings_SetPIDRollP(float v) {
    pid_roll_P = v;
    Settings_Save();
}

float Settings_GetPIDRollI(void) {
    return pid_roll_I;
}
void Settings_SetPIDRollI(float v) {
    pid_roll_I = v;
    Settings_Save();
}

float Settings_GetPIDRollD(void) {
    return pid_roll_D;
}
void Settings_SetPIDRollD(float v) {
    pid_roll_D = v;
    Settings_Save();
}

float Settings_GetPIDPitchP(void) {
    return pid_pitch_P;
}
void Settings_SetPIDPitchP(float v) {
    pid_pitch_P = v;
    Settings_Save();
}

float Settings_GetPIDPitchI(void) {
    return pid_pitch_I;
}
void Settings_SetPIDPitchI(float v) {
    pid_pitch_I = v;
    Settings_Save();
}

float Settings_GetPIDPitchD(void) {
    return pid_pitch_D;
}
void Settings_SetPIDPitchD(float v) {
    pid_pitch_D = v;
    Settings_Save();
}

float Settings_GetPIDYawP(void) {
    return pid_yaw_P;
}
void Settings_SetPIDYawP(float v) {
    pid_yaw_P = v;
    Settings_Save();
}

float Settings_GetPIDYawI(void) {
    return pid_yaw_I;
}
void Settings_SetPIDYawI(float v) {
    pid_yaw_I = v;
    Settings_Save();
}

float Settings_GetPIDYawD(void) {
    return pid_yaw_D;
}
void Settings_SetPIDYawD(float v) {
    pid_yaw_D = v;
    Settings_Save();
}

float Settings_GetYawFF(void) {
    return yaw_FF;
}
void Settings_SetYawFF(float v) {
    yaw_FF = v;
    Settings_Save();
}

float Settings_GetFilterGyroCutoffHz(void) {
    return filter_gyroCutoffHz;
}
void Settings_SetFilterGyroCutoffHz(float v) {
    filter_gyroCutoffHz = v;
    Settings_Save();
}

// [TELEMETRY]
bool Settings_GetTelemetryEnabled(void) {
    return tel_enabled;
}
void Settings_SetTelemetryEnabled(bool on) {
    tel_enabled = on;
    Settings_Save();
}

uint8_t Settings_GetTelemetryProtocol(void) {
    return tel_protocol;
}
void Settings_SetTelemetryProtocol(uint8_t v) {
    tel_protocol = v;
    Settings_Save();
}

uint32_t Settings_GetTelemetryBaud(void) {
    return tel_baud;
}
void Settings_SetTelemetryBaud(uint32_t v) {
    tel_baud = v;
    Settings_Save();
}

uint16_t Settings_GetTelemetryStreamRateHz(void) {
    return tel_streamRateHz;
}
void Settings_SetTelemetryStreamRateHz(uint16_t v) {
    tel_streamRateHz = v;
    Settings_Save();
}

bool Settings_GetTelemetrySendAttitude(void) {
    return tel_sendAttitude;
}
void Settings_SetTelemetrySendAttitude(bool on) {
    tel_sendAttitude = on;
    Settings_Save();
}

bool Settings_GetTelemetrySendStatus(void) {
    return tel_sendStatus;
}
void Settings_SetTelemetrySendStatus(bool on) {
    tel_sendStatus = on;
    Settings_Save();
}

bool Settings_GetTelemetrySendSensors(void) {
    return tel_sendSensors;
}
void Settings_SetTelemetrySendSensors(bool on) {
    tel_sendSensors = on;
    Settings_Save();
}

uint8_t Settings_GetTelemetrySysID(void) {
    return tel_sysID;
}
void Settings_SetTelemetrySysID(uint8_t v) {
    tel_sysID = v;
    Settings_Save();
}

// [MISC]
bool Settings_GetLEDEnabled(void) {
    return misc_ledEnabled;
}
void Settings_SetLEDEnabled(bool on) {
    misc_ledEnabled = on;
    Settings_Save();
}

uint8_t Settings_GetLEDMode(void) {
    return misc_ledMode;
}
void Settings_SetLEDMode(uint8_t v) {
    misc_ledMode = v;
    Settings_Save();
}

bool Settings_GetBuzzerEnabled(void) {
    return misc_buzzerEnabled;
}
void Settings_SetBuzzerEnabled(bool on) {
    misc_buzzerEnabled = on;
    Settings_Save();
}

bool Settings_GetDebugLogEnabled(void) {
    return misc_debugLogEnabled;
}
void Settings_SetDebugLogEnabled(bool on) {
    misc_debugLogEnabled = on;
    Settings_Save();
}

uint8_t Settings_GetLogLevel(void) {
    return misc_logLevel;
}
void Settings_SetLogLevel(uint8_t v) {
    misc_logLevel = v;
    Settings_Save();
}

bool Settings_GetFactoryReset(void) {
    return misc_factoryReset;
}
void Settings_SetFactoryReset(bool on) {
    misc_factoryReset = on;
    Settings_Save();
}
