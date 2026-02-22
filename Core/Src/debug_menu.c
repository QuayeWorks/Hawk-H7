/*
 * debug_menu.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#include "debug_menu.h"
#include "imu.h"
#include "compass.h"
#include "battery.h"
#include "gps.h"
#include "sonar.h"
#include "baro.h"
#include "settings.h"
#include "rc_input.h"
#include "buzzer.h"
#include "servo.h"
#include "motor.h"
#include "flight_state.h"
#include "sensor_diag.h"
#include "cpu.h"
#include "fatfs.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>

#define DBG_UART_TX_TIMEOUT_MS                20u
#define BUS_SCAN_MAX_MS                       200u
#define BUS_SCAN_STEP_ADDRS                   4u
#define GPS_DROP_DELTA_FAIL_THRESHOLD         8u
#define PWM_TEST_STEP_MS                      50u
#define PWM_TEST_DURATION_MS                  4000u
#define PWM_TEST_STEP_US                      25u
#define ACTIVE_TEST_SONAR_EDGE_FRESH_MS       500u
#define ACTIVE_TEST_GPS_FRESH_MS              2000u
#define ACTIVE_TEST_RC_FRESH_MS               500u

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

static UART_HandleTypeDef *dbgUart;
static char cmdBuf[48];
static uint8_t cmdIdx;
static uint32_t lastBeat;
static volatile uint8_t rxByte;
static volatile uint8_t rxPending;
static volatile uint8_t actionMask;
static uint32_t gpsDropPrevForMenu;

typedef struct {
    bool active;
    uint8_t addr;
    uint32_t startMs;
    uint8_t found[32];
    uint8_t foundCount;
    HAL_StatusTypeDef lastStatus;
} BusScanState;

typedef struct {
    bool active;
    uint32_t startMs;
    uint32_t lastStepMs;
    uint16_t pwmUs;
    int16_t dir;
} PwmTestState;

static BusScanState busScan;
static PwmTestState pwmTest;

// Forward GPS byte handler so we can chain callbacks
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

static void process_char(uint8_t ch);
static void handle_cmd(void);
static void process_actions(void);
static void diagnostics_bus_scan_task(uint32_t now);
static void diagnostics_pwm_test_task(uint32_t now);
static bool diagnostics_lock_for_active_tests(void);

static const char* hal_status_str(HAL_StatusTypeDef st)
{
    switch (st) {
    case HAL_OK:
        return "OK";
    case HAL_ERROR:
        return "ERR";
    case HAL_BUSY:
        return "BUSY";
    case HAL_TIMEOUT:
        return "TIMEOUT";
    default:
        return "UNK";
    }
}

static const char* pass_fail(bool ok)
{
    return ok ? "PASS" : "FAIL";
}

static void send(const char *s)
{
    if (dbgUart == NULL || s == NULL) {
        return;
    }
    HAL_UART_Transmit(dbgUart, (uint8_t*)s, (uint16_t)strlen(s), DBG_UART_TX_TIMEOUT_MS);
}

static void sendf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int n;

    if (dbgUart == NULL || fmt == NULL) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        uint16_t len = (uint16_t)((n < (int)sizeof(buf)) ? n : (int)(sizeof(buf) - 1));
        HAL_UART_Transmit(dbgUart, (uint8_t*)buf, len, DBG_UART_TX_TIMEOUT_MS);
    }
}

static bool require_disarmed(const char *name)
{
    if (!FlightState_IsArmed()) {
        return true;
    }
    sendf("%s: FAIL (DISARMED ONLY)\r\n", name);
    return false;
}

static bool diagnostics_lock_for_active_tests(void)
{
    if (!FlightState_IsArmed()) {
        return false;
    }
    send("ARMED: DIAGNOSTICS LOCKED\r\n");
    return true;
}

static void show_menu(void)
{
    send("\r\n--- Debug Menu ---\r\n");
    send("h: help\r\n");
    send("d: diagnostics menu\r\n");
    send("1/s: quick health snapshot\r\n");
    send("2: active sensor test\r\n");
    send("3: i2c bus scan (disarmed only)\r\n");
    send("4: storage test (disarmed only)\r\n");
    send("5: pwm test banner\r\n");
    send("5 run: start pwm test (disarmed only)\r\n");
    send("5 stop: stop pwm test\r\n");
    send("g: gps status\r\n");
    send("o: sonar status\r\n");
    send("p: rc channels\r\n");
    send("b: buzzer tone\r\n");
    send("v: toggle servo debug motion\r\n");
    send("> ");
}

static void show_help(void)
{
    show_menu();
}

static void show_diag_menu(void)
{
    send("Diagnostics:\r\n");
    send("  1. Quick Health Snapshot (no IO)\r\n");
    send("  2. Active Sensor Test (single IO/read per sensor)\r\n");
    send("  3. Bus Scan (I2C 0x08..0x77, disarmed only, <=200ms)\r\n");
    send("  4. Storage Test (mount/create/read/delete, disarmed only)\r\n");
    send("  5. PWM Test (disarmed only, warning + auto-timeout)\r\n");
}

static void show_ppm(void)
{
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
                     "RC: ch=[%u %u %u %u %u %u] frame=%lu last=%lu stale=%s rssi=%u lost=%s\r\n",
                     RC_GetChannel(0), RC_GetChannel(1), RC_GetChannel(2),
                     RC_GetChannel(3), RC_GetChannel(4), RC_GetChannel(5),
                     (unsigned long)RC_GetFrameCount(),
                     (unsigned long)RC_GetLastFrameMs(),
                     RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
                     RC_GetRSSI(),
                     RC_LinkLostForSeconds(1u) ? "YES" : "NO");
    if (n > 0) {
        HAL_UART_Transmit(dbgUart, (uint8_t*)buf, (uint16_t)n, DBG_UART_TX_TIMEOUT_MS);
    }
}

static void show_sonar(void)
{
    sendf("SONAR: d=[%.2f %.2f %.2f]m trigger=%lu last_trig=%lu edge=[%lu %lu %lu] timeout=%lu\r\n",
          Sonar_ReadDistance(0), Sonar_ReadDistance(1), Sonar_ReadDistance(2),
          (unsigned long)Sonar_GetTriggerCount(),
          (unsigned long)Sonar_GetLastTriggerMs(),
          (unsigned long)Sonar_GetEdgeCount(0),
          (unsigned long)Sonar_GetEdgeCount(1),
          (unsigned long)Sonar_GetEdgeCount(2),
          (unsigned long)Sonar_GetEchoTimeoutCount());
}

static void show_gps(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t drop = GPS_GetDroppedByteCount();
    uint32_t dropDelta = drop - gpsDropPrevForMenu;
    bool fix = GPS_HasFix();

    gpsDropPrevForMenu = drop;
    sendf("GPS: %s lat=%.6f lon=%.6f alt=%.1f sats=%u hdop=%.1f up=%lums sps=%u drop=%lu dDrop=%lu ring=%u\r\n",
          fix ? "FIX" : "NOFIX",
          GPS_GetLatitude(), GPS_GetLongitude(), GPS_GetAltitude(),
          GPS_GetSatCount(), GPS_GetHDOP(),
          (unsigned long)((GPS_GetLastUpdateMs() > 0u) ? (now - GPS_GetLastUpdateMs()) : 0u),
          GPS_GetSentenceRateHz(),
          (unsigned long)drop,
          (unsigned long)dropDelta,
          GPS_GetRxRingLevel());
}

static void diagnostics_quick_snapshot(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t fs = FlightState_GetStateMask();
    SensorRateReport rates;
    bool gps_drop_ok;
    uint32_t gps_drop;
    uint32_t gps_drop_delta;
    bool imu_ok;
    bool baro_ok;
    bool gps_ok;
    bool sonar_ok;
    bool batt_ok;
    bool rc_ok;
    float accelBx;
    float accelBy;
    float accelBz;

    SensorDiag_GetRates(&rates);
    Settings_GetAccelBiases(&accelBx, &accelBy, &accelBz);

    gps_drop = GPS_GetDroppedByteCount();
    gps_drop_delta = gps_drop - gpsDropPrevForMenu;
    gps_drop_ok = (gps_drop_delta <= GPS_DROP_DELTA_FAIL_THRESHOLD);
    gpsDropPrevForMenu = gps_drop;

    imu_ok = IMU_IsIdentityOK() && (IMU_GetLastReadMs() > 0u) && ((now - IMU_GetLastReadMs()) < 500u);
    baro_ok = !Settings_GetBaroEnabled() ||
              (Baro_IsIdentityOK() && Baro_IsHealthy() && (Baro_GetLastUpdateMs() > 0u));
    gps_ok = !Settings_GetGPSEnabled() ||
             ((GPS_GetLastUpdateMs() > 0u) && ((now - GPS_GetLastUpdateMs()) <= ACTIVE_TEST_GPS_FRESH_MS) && gps_drop_ok);
    sonar_ok = !Settings_GetSonarEnabled() ||
               ((Sonar_GetLastEdgeMs(0) > 0u) || (Sonar_GetLastEdgeMs(1) > 0u) || (Sonar_GetLastEdgeMs(2) > 0u));
    batt_ok = (Battery_GetLastCurrentMs() > 0u) || (Battery_GetLastVoltageMs() > 0u);
    rc_ok = !RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS);

    send("SNAPSHOT:\r\n");
    sendf("  FLIGHT: state=0x%08lX READY=%s ARMED=%s ALL_HEALTH=%s\r\n",
          (unsigned long)fs,
          FlightState_IsReady() ? "YES" : "NO",
          FlightState_IsArmed() ? "YES" : "NO",
          FlightState_AllHealthOK() ? "YES" : "NO");

    sendf("  IMU: %s id=0x%02X id_ok=%s last=%lums rate=%uHz hal=%s(%d) ok=%lu fail=%lu bias[a]=%.3f/%.3f/%.3f\r\n",
          pass_fail(imu_ok),
          IMU_GetWhoAmI(),
          IMU_IsIdentityOK() ? "YES" : "NO",
          (unsigned long)IMU_GetLastReadMs(),
          rates.imu_hz,
          hal_status_str(IMU_GetLastHalStatus()),
          (int)IMU_GetLastHalStatus(),
          (unsigned long)IMU_GetReadOkCount(),
          (unsigned long)IMU_GetReadFailCount(),
          accelBx, accelBy, accelBz);

    sendf("  BARO: %s id=0x%02X id_ok=%s calib=%s healthy=%s last=%lums rate=%uHz hal=%s(%d) i2c_fail=%lu upd_ok=%lu\r\n",
          pass_fail(baro_ok),
          Baro_GetChipId(),
          Baro_IsIdentityOK() ? "YES" : "NO",
          Baro_IsCalibrationLoaded() ? "YES" : "NO",
          Baro_IsHealthy() ? "YES" : "NO",
          (unsigned long)Baro_GetLastUpdateMs(),
          rates.baro_hz,
          hal_status_str(Baro_GetLastHalStatus()),
          (int)Baro_GetLastHalStatus(),
          (unsigned long)Baro_GetI2CFailCount(),
          (unsigned long)Baro_GetUpdateOkCount());

    sendf("  GPS: %s fix=%s sats=%u hdop=%.1f last=%lums rate=%uHz sps=%u drop=%lu dDrop=%lu ring=%u\r\n",
          pass_fail(gps_ok),
          GPS_HasFix() ? "YES" : "NO",
          GPS_GetSatCount(),
          GPS_GetHDOP(),
          (unsigned long)GPS_GetLastUpdateMs(),
          rates.gps_hz,
          GPS_GetSentenceRateHz(),
          (unsigned long)gps_drop,
          (unsigned long)gps_drop_delta,
          GPS_GetRxRingLevel());

    sendf("  SONAR: %s lastTrig=%lums rate=%uHz d=[%.2f %.2f %.2f] edge=[%lu %lu %lu] tout=%lu\r\n",
          pass_fail(sonar_ok),
          (unsigned long)Sonar_GetLastTriggerMs(),
          rates.sonar_hz,
          Sonar_ReadDistance(0), Sonar_ReadDistance(1), Sonar_ReadDistance(2),
          (unsigned long)Sonar_GetEdgeCount(0),
          (unsigned long)Sonar_GetEdgeCount(1),
          (unsigned long)Sonar_GetEdgeCount(2),
          (unsigned long)Sonar_GetEchoTimeoutCount());

    sendf("  BAT: %s v_last=%lums i_last=%lums rate=%uHz adc_to=%lu i2c_err=%lu adc=%s(%d) i2c=%s(%d)\r\n",
          pass_fail(batt_ok),
          (unsigned long)Battery_GetLastVoltageMs(),
          (unsigned long)Battery_GetLastCurrentMs(),
          rates.battery_hz,
          (unsigned long)Battery_GetADCTimeoutCount(),
          (unsigned long)Battery_GetI2CErrorCount(),
          hal_status_str(Battery_GetLastADCStatus()),
          (int)Battery_GetLastADCStatus(),
          hal_status_str(Battery_GetLastI2CStatus()),
          (int)Battery_GetLastI2CStatus());

    sendf("  RC: %s last=%lums rate=%uHz frame=%lu stale=%s rssi=%u linklost=%s\r\n",
          pass_fail(rc_ok),
          (unsigned long)RC_GetLastFrameMs(),
          rates.rc_hz,
          (unsigned long)RC_GetFrameCount(),
          RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
          RC_GetRSSI(),
          RC_LinkLostForSeconds(1u) ? "YES" : "NO");

    sendf("  STORAGE: ini=%s pending=%s last_try=%lums try_fail=%s last_ok=%lums\r\n",
          Settings_GetIniPath(),
          Settings_IsSavePending() ? "YES" : "NO",
          (unsigned long)Settings_GetLastSaveAttemptMs(),
          Settings_GetLastSaveAttemptFailed() ? "YES" : "NO",
          (unsigned long)Settings_GetLastSaveSuccessMs());

    sendf("  CPU: loop=%uHz dt_last=%lums dt_max=%lums missed=%lu\r\n",
          CPU_GetLoopRateHz(),
          (unsigned long)CPU_GetLastLoopDtMs(),
          (unsigned long)CPU_GetMaxLoopDtMs(),
          (unsigned long)CPU_GetMissedDeadlineCount());
}

static void diagnostics_active_sensor_test(void)
{
    uint32_t now = HAL_GetTick();
    SensorRateReport rates;
    int16_t ax, ay, az, gx, gy, gz;
    bool imu_read_ok;
    bool imu_plausible;
    bool imu_pass;
    bool baro_upd_ok;
    bool baro_pass;
    uint32_t gps_drop_before;
    uint32_t gps_drop_after;
    uint32_t gps_drop_delta;
    bool gps_fresh;
    bool gps_pass;
    uint32_t sonar_edge_before[SONAR_COUNT];
    bool sonar_pass;
    uint32_t batt_i2c_before;
    uint32_t batt_adc_before;
    float pack_v;
    float curr_a;
    float cell_v;
    bool batt_pass;
    bool rc_pass;

    SensorDiag_GetRates(&rates);
    send("ACTIVE SENSOR TEST:\r\n");

    imu_read_ok = IMU_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
    imu_plausible = IMU_CheckPlausibility();
    imu_pass = IMU_IsIdentityOK() && imu_read_ok && imu_plausible;
    sendf("  IMU: %s id=0x%02X read=%s plaus=%s hal=%s(%d) last=%lums rate=%uHz fail=%lu\r\n",
          pass_fail(imu_pass),
          IMU_GetWhoAmI(),
          imu_read_ok ? "YES" : "NO",
          imu_plausible ? "YES" : "NO",
          hal_status_str(IMU_GetLastHalStatus()),
          (int)IMU_GetLastHalStatus(),
          (unsigned long)IMU_GetLastReadMs(),
          rates.imu_hz,
          (unsigned long)IMU_GetReadFailCount());

    baro_upd_ok = Baro_Update(now);
    baro_pass = (!Settings_GetBaroEnabled()) ||
                (Baro_IsIdentityOK() && Baro_IsCalibrationLoaded() && baro_upd_ok && Baro_IsHealthy());
    sendf("  BARO: %s id=0x%02X id_ok=%s calib=%s upd=%s P=%.2f T=%.2f Alt=%.2f Vz=%.2f hal=%s(%d) fail=%lu last=%lums rate=%uHz\r\n",
          pass_fail(baro_pass),
          Baro_GetChipId(),
          Baro_IsIdentityOK() ? "YES" : "NO",
          Baro_IsCalibrationLoaded() ? "YES" : "NO",
          baro_upd_ok ? "YES" : "NO",
          Baro_GetPressureHpa(),
          Baro_GetTemperatureC(),
          Baro_GetAltitudeMeters(),
          Baro_GetClimbRateMps(),
          hal_status_str(Baro_GetLastHalStatus()),
          (int)Baro_GetLastHalStatus(),
          (unsigned long)Baro_GetI2CFailCount(),
          (unsigned long)Baro_GetLastUpdateMs(),
          rates.baro_hz);

    gps_drop_before = GPS_GetDroppedByteCount();
    GPS_Update();
    gps_drop_after = GPS_GetDroppedByteCount();
    gps_drop_delta = gps_drop_after - gps_drop_before;
    gps_fresh = (GPS_GetLastUpdateMs() > 0u) && ((now - GPS_GetLastUpdateMs()) <= ACTIVE_TEST_GPS_FRESH_MS);
    gps_pass = (!Settings_GetGPSEnabled()) ||
               ((gps_drop_delta <= GPS_DROP_DELTA_FAIL_THRESHOLD) &&
                ((GPS_GetSentenceRateHz() > 0u) || gps_fresh || GPS_HasFix()));
    sendf("  GPS: %s fix=%s sats=%u hdop=%.1f fresh=%s last=%lums rate=%uHz sps=%u dropDelta=%lu ring=%u\r\n",
          pass_fail(gps_pass),
          GPS_HasFix() ? "YES" : "NO",
          GPS_GetSatCount(),
          GPS_GetHDOP(),
          gps_fresh ? "YES" : "NO",
          (unsigned long)GPS_GetLastUpdateMs(),
          rates.gps_hz,
          GPS_GetSentenceRateHz(),
          (unsigned long)gps_drop_delta,
          GPS_GetRxRingLevel());

    sonar_edge_before[0] = Sonar_GetEdgeCount(0);
    sonar_edge_before[1] = Sonar_GetEdgeCount(1);
    sonar_edge_before[2] = Sonar_GetEdgeCount(2);
    Sonar_TriggerAll();
    sonar_pass = (!Settings_GetSonarEnabled()) ||
                 ((Sonar_GetEdgeCount(0) != sonar_edge_before[0]) ||
                  (Sonar_GetEdgeCount(1) != sonar_edge_before[1]) ||
                  (Sonar_GetEdgeCount(2) != sonar_edge_before[2]) ||
                  ((now - Sonar_GetLastEdgeMs(0)) <= ACTIVE_TEST_SONAR_EDGE_FRESH_MS) ||
                  ((now - Sonar_GetLastEdgeMs(1)) <= ACTIVE_TEST_SONAR_EDGE_FRESH_MS) ||
                  ((now - Sonar_GetLastEdgeMs(2)) <= ACTIVE_TEST_SONAR_EDGE_FRESH_MS));
    sendf("  SONAR: %s trig=%lu edges=[%lu %lu %lu] tout=%lu d=[%.2f %.2f %.2f] rate=%uHz\r\n",
          pass_fail(sonar_pass),
          (unsigned long)Sonar_GetTriggerCount(),
          (unsigned long)Sonar_GetEdgeCount(0),
          (unsigned long)Sonar_GetEdgeCount(1),
          (unsigned long)Sonar_GetEdgeCount(2),
          (unsigned long)Sonar_GetEchoTimeoutCount(),
          Sonar_ReadDistance(0), Sonar_ReadDistance(1), Sonar_ReadDistance(2),
          rates.sonar_hz);

    batt_i2c_before = Battery_GetI2CErrorCount();
    batt_adc_before = Battery_GetADCTimeoutCount();
    pack_v = Battery_ReadPackVoltage();
    curr_a = Battery_ReadCurrent();
    cell_v = Battery_ReadPerCellVoltage();
    batt_pass = ((Battery_GetI2CErrorCount() == batt_i2c_before) &&
                 (Battery_GetADCTimeoutCount() == batt_adc_before) &&
                 (pack_v > 0.0f));
    sendf("  BATT: %s V=%.2f Cell=%.2f I=%.2f i2c_err=%lu adc_to=%lu halI2C=%s(%d) halADC=%s(%d) lastV=%lums lastI=%lums rate=%uHz\r\n",
          pass_fail(batt_pass),
          pack_v, cell_v, curr_a,
          (unsigned long)Battery_GetI2CErrorCount(),
          (unsigned long)Battery_GetADCTimeoutCount(),
          hal_status_str(Battery_GetLastI2CStatus()),
          (int)Battery_GetLastI2CStatus(),
          hal_status_str(Battery_GetLastADCStatus()),
          (int)Battery_GetLastADCStatus(),
          (unsigned long)Battery_GetLastVoltageMs(),
          (unsigned long)Battery_GetLastCurrentMs(),
          rates.battery_hz);

    rc_pass = !RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS);
    sendf("  RC: %s frame=%lu last=%lums stale=%s rssi=%u rate=%uHz\r\n",
          pass_fail(rc_pass),
          (unsigned long)RC_GetFrameCount(),
          (unsigned long)RC_GetLastFrameMs(),
          RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
          RC_GetRSSI(),
          rates.rc_hz);

    sendf("  MAG: %s (STUB=%s)\r\n",
          Compass_IsStub() ? "FAIL" : "PASS",
          Compass_IsStub() ? "YES" : "NO");
}

static void diagnostics_start_bus_scan(void)
{
    if (!require_disarmed("Bus Scan")) {
        return;
    }
    memset(&busScan, 0, sizeof(busScan));
    busScan.active = true;
    busScan.addr = 0x08u;
    busScan.startMs = HAL_GetTick();
    busScan.lastStatus = HAL_OK;
    send("BUS SCAN: started (0x08..0x77, max 200ms)\r\n");
}

static void diagnostics_finish_bus_scan(void)
{
    uint8_t i;

    sendf("BUS SCAN: done status=%s(%d) found=%u ",
          hal_status_str(busScan.lastStatus),
          (int)busScan.lastStatus,
          busScan.foundCount);
    if (busScan.foundCount == 0u) {
        send("[none]\r\n");
    } else {
        send("[");
        for (i = 0u; i < busScan.foundCount; i++) {
            sendf("0x%02X%s", busScan.found[i], (i + 1u < busScan.foundCount) ? " " : "");
        }
        send("]\r\n");
    }
    busScan.active = false;
}

static void diagnostics_bus_scan_task(uint32_t now)
{
    uint8_t step;

    if (!busScan.active) {
        return;
    }
    if (FlightState_IsArmed()) {
        send("ARMED: DIAGNOSTICS LOCKED\r\n");
        send("BUS SCAN: aborted\r\n");
        busScan.active = false;
        return;
    }
    if ((now - busScan.startMs) >= BUS_SCAN_MAX_MS || busScan.addr > 0x77u) {
        diagnostics_finish_bus_scan();
        return;
    }

    for (step = 0u; step < BUS_SCAN_STEP_ADDRS && busScan.addr <= 0x77u; step++) {
        busScan.lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(busScan.addr << 1), 1u, 1u);
        if (busScan.lastStatus == HAL_OK && busScan.foundCount < (uint8_t)sizeof(busScan.found)) {
            busScan.found[busScan.foundCount++] = busScan.addr;
        }
        busScan.addr++;
        if ((HAL_GetTick() - busScan.startMs) >= BUS_SCAN_MAX_MS) {
            break;
        }
    }

    if (busScan.addr > 0x77u || (HAL_GetTick() - busScan.startMs) >= BUS_SCAN_MAX_MS) {
        diagnostics_finish_bus_scan();
    }
}

static void diagnostics_storage_test(void)
{
    FIL file;
    FRESULT fr_mount;
    FRESULT fr = FR_OK;
    UINT bw = 0u;
    UINT br = 0u;
    char path[32] = {0};
    const char payload[] = "hawk_diag";
    char readback[sizeof(payload)] = {0};
    bool pass = true;

    if (!require_disarmed("Storage Test")) {
        return;
    }

    fr_mount = f_mount(&SDFatFS, SDPath, 1u);
    if (fr_mount != FR_OK) {
        pass = false;
    }

    if (pass) {
        snprintf(path, sizeof(path), "%s/diag.tmp", SDPath);
        fr = f_open(&file, path, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) {
            pass = false;
        }
    }

    if (pass) {
        fr = f_write(&file, payload, (UINT)(sizeof(payload) - 1u), &bw);
        if (fr != FR_OK || bw != (UINT)(sizeof(payload) - 1u)) {
            pass = false;
        }
        (void)f_close(&file);
    }

    if (pass) {
        fr = f_open(&file, path, FA_READ);
        if (fr != FR_OK) {
            pass = false;
        }
    }

    if (pass) {
        fr = f_read(&file, readback, (UINT)(sizeof(payload) - 1u), &br);
        if (fr != FR_OK || br != (UINT)(sizeof(payload) - 1u) || memcmp(payload, readback, sizeof(payload) - 1u) != 0) {
            pass = false;
        }
        (void)f_close(&file);
    }

    if (path[0] != '\0') {
        (void)f_unlink(path);
    }

    sendf("STORAGE TEST: %s mount_fr=%d io_fr=%d ini=%s pending=%s last_try=%lu last_ok=%lu\r\n",
          pass_fail(pass),
          (int)fr_mount,
          (int)fr,
          Settings_GetIniPath(),
          Settings_IsSavePending() ? "YES" : "NO",
          (unsigned long)Settings_GetLastSaveAttemptMs(),
          (unsigned long)Settings_GetLastSaveSuccessMs());
}

static void diagnostics_pwm_test_banner(void)
{
    send("PWM TEST: ARMED TEST DISABLED\r\n");
    send("PWM TEST: DISARMED ONLY. Type '5 run' to start, '5 stop' to abort.\r\n");
    sendf("PWM TEST: timer_state TIM1=%d TIM3=%d TIM5=%d min=%u max=%u\r\n",
          (int)htim1.State, (int)htim3.State, (int)htim5.State,
          Settings_GetMotorPWMMinUs(), Settings_GetMotorPWMMaxUs());
}

static void diagnostics_pwm_test_stop(const char *reason)
{
    if (!pwmTest.active) {
        return;
    }

    Motor_SetAllPWM(Settings_GetMotorPWMMinUs());
    Servo_SetPWM(2u, 1500u);
    Servo_SetPWM(3u, 1500u);
    pwmTest.active = false;
    sendf("PWM TEST: stopped (%s)\r\n", (reason != NULL) ? reason : "done");
}

static void diagnostics_pwm_test_start(void)
{
    if (!require_disarmed("PWM Test")) {
        return;
    }
    if (pwmTest.active) {
        send("PWM TEST: already active\r\n");
        return;
    }

    pwmTest.active = true;
    pwmTest.startMs = HAL_GetTick();
    pwmTest.lastStepMs = pwmTest.startMs;
    pwmTest.pwmUs = Settings_GetMotorPWMMinUs();
    pwmTest.dir = (int16_t)PWM_TEST_STEP_US;

    Motor_SetAllPWM(pwmTest.pwmUs);
    Servo_SetPWM(2u, 1500u);
    Servo_SetPWM(3u, 1500u);
    send("PWM TEST: started (auto-timeout 4s)\r\n");
}

static void diagnostics_pwm_test_task(uint32_t now)
{
    uint16_t minUs = Settings_GetMotorPWMMinUs();
    uint16_t maxUs = Settings_GetMotorPWMMaxUs();

    if (!pwmTest.active) {
        return;
    }
    if (FlightState_IsArmed()) {
        send("ARMED: DIAGNOSTICS LOCKED\r\n");
        diagnostics_pwm_test_stop("armed");
        return;
    }
    if ((now - pwmTest.startMs) >= PWM_TEST_DURATION_MS) {
        diagnostics_pwm_test_stop("timeout");
        return;
    }
    if ((now - pwmTest.lastStepMs) < PWM_TEST_STEP_MS) {
        return;
    }

    pwmTest.lastStepMs = now;

    if (pwmTest.dir > 0) {
        if ((uint32_t)pwmTest.pwmUs + (uint32_t)pwmTest.dir >= (uint32_t)maxUs) {
            pwmTest.pwmUs = maxUs;
            pwmTest.dir = -(int16_t)PWM_TEST_STEP_US;
        } else {
            pwmTest.pwmUs = (uint16_t)(pwmTest.pwmUs + (uint16_t)pwmTest.dir);
        }
    } else {
        if (pwmTest.pwmUs <= (uint16_t)(minUs + (uint16_t)(-pwmTest.dir))) {
            pwmTest.pwmUs = minUs;
            pwmTest.dir = (int16_t)PWM_TEST_STEP_US;
        } else {
            pwmTest.pwmUs = (uint16_t)(pwmTest.pwmUs - (uint16_t)(-pwmTest.dir));
        }
    }

    Motor_SetAllPWM(pwmTest.pwmUs);
    Servo_SetPWM(3u, pwmTest.pwmUs);
}

void DebugMenu_Init(UART_HandleTypeDef *huart)
{
    dbgUart = huart;
    cmdIdx = 0u;
    lastBeat = HAL_GetTick();
    rxPending = 0u;
    rxByte = 0u;
    actionMask = DEBUG_MENU_NONE;
    gpsDropPrevForMenu = GPS_GetDroppedByteCount();
    memset(&busScan, 0, sizeof(busScan));
    memset(&pwmTest, 0, sizeof(pwmTest));

    show_menu();
    HAL_UART_Receive_IT(dbgUart, (uint8_t *)&rxByte, 1);
}

static void process_actions(void)
{
    if (actionMask & DEBUG_MENU_SENSORS) {
        diagnostics_quick_snapshot();
    }
    if (actionMask & DEBUG_MENU_PPM) {
        show_ppm();
    }
    if (actionMask & DEBUG_MENU_SONAR) {
        show_sonar();
    }
    if (actionMask & DEBUG_MENU_GPS) {
        show_gps();
    }
    if (actionMask & DEBUG_MENU_BUZZER) {
        Buzzer_PlayTone(TONE_READY);
    }
    if (actionMask & DEBUG_MENU_HELP) {
        show_help();
    }
}

static void process_char(uint8_t ch)
{
    if (ch == '\r' || ch == '\n') {
        if (cmdIdx > 0u) {
            handle_cmd();
            cmdIdx = 0u;
        } else {
            send("> ");
        }
    } else if (cmdIdx < (uint8_t)(sizeof(cmdBuf) - 1u)) {
        cmdBuf[cmdIdx++] = (char)ch;
    }
}

static void handle_cmd(void)
{
    cmdBuf[cmdIdx] = '\0';

    if (strcmp(cmdBuf, "h") == 0 || strcmp(cmdBuf, "?") == 0) {
        show_help();
    } else if (strcmp(cmdBuf, "d") == 0) {
        show_diag_menu();
    } else if (strcmp(cmdBuf, "1") == 0 || strcmp(cmdBuf, "s") == 0) {
        diagnostics_quick_snapshot();
    } else if (strcmp(cmdBuf, "2") == 0) {
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_active_sensor_test();
        }
    } else if (strcmp(cmdBuf, "3") == 0) {
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_start_bus_scan();
        }
    } else if (strcmp(cmdBuf, "4") == 0) {
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_storage_test();
        }
    } else if (strcmp(cmdBuf, "5") == 0) {
        diagnostics_pwm_test_banner();
    } else if (strcmp(cmdBuf, "5 run") == 0) {
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_pwm_test_start();
        }
    } else if (strcmp(cmdBuf, "5 stop") == 0) {
        diagnostics_pwm_test_stop("manual");
    } else if (strcmp(cmdBuf, "g") == 0) {
        show_gps();
    } else if (strcmp(cmdBuf, "o") == 0) {
        show_sonar();
    } else if (strcmp(cmdBuf, "p") == 0) {
        show_ppm();
    } else if (strcmp(cmdBuf, "b") == 0) {
        Buzzer_PlayTone(TONE_READY);
    } else if (strcmp(cmdBuf, "v") == 0) {
        actionMask ^= DEBUG_MENU_SERVOS;
        sendf("SERVO DEBUG: %s\r\n", (actionMask & DEBUG_MENU_SERVOS) ? "ON" : "OFF");
    } else {
        send("Unknown command\r\n");
        show_help();
    }

    send("> ");
}

void DebugMenu_ForceInput(uint8_t ch)
{
    process_char(ch);
}

void DebugMenu_SetActionMask(uint8_t mask)
{
    actionMask = mask;
}

uint8_t DebugMenu_GetActionMask(void)
{
    return actionMask;
}

void DebugMenu_Task(void)
{
    uint32_t now = HAL_GetTick();

    diagnostics_bus_scan_task(now);
    diagnostics_pwm_test_task(now);

    if ((now - lastBeat) >= 1000u) {
        process_actions();
        lastBeat = now;
    }

    if (rxPending || rxByte != 0u) {
        uint8_t ch = rxByte;
        rxPending = 0u;
        rxByte = 0u;
        process_char(ch);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == dbgUart) {
        rxPending = 1u;
        HAL_UART_Receive_IT(dbgUart, (uint8_t *)&rxByte, 1);
    }

    // Chain other modules that use UART receive interrupts
    GPS_UART_RxCpltCallback(huart);
}
