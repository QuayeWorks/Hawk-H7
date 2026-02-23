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
#include "attitude.h"
#include "settings.h"
#include "rc_input.h"
#include "buzzer.h"
#include "servo.h"
#include "motor.h"
#include "flight_state.h"
#include "sensor_diag.h"
#include "cpu.h"
#include "power.h"
#include "main.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"
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
#define MENU_STREAM_PERIOD_MS                 750u
#define DEBUG_RX_RING_SIZE                    64u

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

static UART_HandleTypeDef *dbgUart;
static char cmdBuf[48];
static uint8_t cmdIdx;
static uint32_t lastBeat;
static volatile uint8_t rxByte;
static volatile uint8_t rxRing[DEBUG_RX_RING_SIZE];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static volatile uint8_t actionMask;
static uint32_t gpsDropPrevForMenu;
static bool imuStream;
static bool bmpStream;
static bool magStream;
static bool inaStream;

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
static bool busScanRepeat;
static uint32_t busScanNextStartMs;
static uint8_t escAnsiState;

// Forward GPS byte handler so we can chain callbacks
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

static void process_char(uint8_t ch);
static void handle_cmd(void);
static void process_actions(void);
static void diagnostics_bus_scan_task(uint32_t now);
static void diagnostics_pwm_test_task(uint32_t now);
static void diagnostics_pwm_test_stop(const char *reason);
static bool diagnostics_lock_for_active_tests(void);
static void diagnostics_start_bus_scan_internal(bool announce);
static void io_control_help(void);
static void io_control_status(void);
static void io_control_handle(const char *args);
static void io_control_handle_target_state(const char *target, const char *state);
static void stop_streaming_modes(bool announce);
static void debug_rx_push_isr(uint8_t ch);
static bool debug_rx_pop(uint8_t *ch);
static void show_mpu6050(void);
static void show_bmp_sensor(void);
static void show_magnetometer(void);
static void show_ina219(void);

static void debug_rx_push_isr(uint8_t ch)
{
    uint8_t next = (uint8_t)((rxHead + 1u) % DEBUG_RX_RING_SIZE);

    if (next == rxTail) {
        return;
    }

    rxRing[rxHead] = ch;
    rxHead = next;
}

static bool debug_rx_pop(uint8_t *ch)
{
    uint8_t tail;

    if (ch == NULL) {
        return false;
    }

    tail = rxTail;
    if (tail == rxHead) {
        return false;
    }

    *ch = rxRing[tail];
    rxTail = (uint8_t)((tail + 1u) % DEBUG_RX_RING_SIZE);
    return true;
}

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

static const char* on_off(GPIO_PinState st)
{
    return (st == GPIO_PIN_SET) ? "ON" : "OFF";
}

static const char* baro_chip_name(uint8_t chip_id)
{
    switch (chip_id) {
    case 0x58u:
        return "BMP280";
    case 0x60u:
        return "BME280";
    case 0x50u:
        return "BMP388";
    default:
        return "UNKNOWN";
    }
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
    send("2: toggle active sensor test stream (0.75s)\r\n");
    send("2 once: run active sensor test once\r\n");
    send("3: toggle i2c bus scan loop (0.75s, disarmed)\r\n");
    send("3 once: run one i2c bus scan (disarmed)\r\n");
    send("4: storage test (disarmed only)\r\n");
    send("5: pwm test banner\r\n");
    send("5 run: start pwm test (disarmed only)\r\n");
    send("5 stop: stop pwm test\r\n");
    send("mpu: toggle MPU6050 stream (0.75s), mpu once for single\r\n");
    send("bmp: toggle BMP sensor stream (0.75s), bmp once for single\r\n");
    send("mag: toggle magnetometer stream (0.75s), mag once for single\r\n");
    send("ina: toggle INA219 stream (0.75s), ina once for single\r\n");
    send("g: toggle gps stream (0.75s), g once for single\r\n");
    send("o: toggle sonar stream (0.75s), o once for single\r\n");
    send("p: toggle rc stream (0.75s), p once for single\r\n");
    send("b: toggle buzzer ON/OFF\r\n");
    send("v: toggle servo debug motion\r\n");
    send("io: io control help + status\r\n");
    send("ios: show module-enable pin states\r\n");
    send("ioi <on|off>: IMU EN pin\r\n");
    send("iobt <on|off>: BT EN pin\r\n");
    send("iobmpi2c <on|off>: BMP388 I2C EN (PA4)\r\n");
    send("iobmppwr <on|off>: BMP388 PWR EN (PD0)\r\n");
    send("ioqmcpwr <on|off>: QMC5883L PWR EN (PD1)\r\n");
    send("iob0|iob1|iob2|iob3 <on|off>: buck EN pins\r\n");
    send("ioba <on|off>: all buck EN pins\r\n");
    send("ESC: stop all active streams/loops\r\n");
    send("> ");
}

static void stop_streaming_modes(bool announce)
{
    bool hadStreams = (actionMask != DEBUG_MENU_NONE) || busScanRepeat || busScan.active || pwmTest.active ||
                      imuStream || bmpStream || magStream || inaStream;

    actionMask = DEBUG_MENU_NONE;
    imuStream = false;
    bmpStream = false;
    magStream = false;
    inaStream = false;
    busScanRepeat = false;
    busScan.active = false;
    busScanNextStartMs = 0u;
    diagnostics_pwm_test_stop("esc");
    Buzzer_Stop();

    if (announce && hadStreams) {
        send("STREAMS: OFF (ESC)\r\n");
    }
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

static void io_control_help(void)
{
    send("IO CONTROL:\r\n");
    send("  io      : help + status\r\n");
    send("  ios           : status\r\n");
    send("  ioi on|off    : imu enable pin (PA15)\r\n");
    send("  iobt on|off   : bt enable pin (PA12)\r\n");
    send("  iobmpi2c on|off : bmp388 i2c enable (PA4)\r\n");
    send("  iobmppwr on|off : bmp388 power enable (PD0)\r\n");
    send("  ioqmcpwr on|off : qmc5883l power enable (PD1)\r\n");
    send("  iob0 on|off  : buck0 enable\r\n");
    send("  iob1 on|off  : buck1 enable\r\n");
    send("  iob2 on|off  : buck2 enable\r\n");
    send("  iob3 on|off  : buck3 enable\r\n");
    send("  ioba on|off  : all bucks enable\r\n");
    send("  Note: IO control is DISARMED ONLY.\r\n");
}

static void io_control_status(void)
{
    sendf("IO STATUS: imu=%s bt=%s bmp_i2c=%s bmp_pwr=%s qmc_pwr=%s buck0=%s buck1=%s buck2=%s buck3=%s\r\n",
          on_off(HAL_GPIO_ReadPin(MPU6050_EN_GPIO_Port, MPU6050_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(HC_05_EN_GPIO_Port, HC_05_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(BMP388_I2C_EN_GPIO_Port, BMP388_I2C_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(BMP388_PWR_EN_GPIO_Port, BMP388_PWR_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(QMC5883_PWR_EN_GPIO_Port, QMC5883_PWR_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(YAW_Buck_EN_GPIO_Port, YAW_Buck_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(ARM_Buck_EN_GPIO_Port, ARM_Buck_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(PITCH_Buck_EN_GPIO_Port, PITCH_Buck_EN_Pin)),
          on_off(HAL_GPIO_ReadPin(ROL_Buck_EN_GPIO_Port, ROL_Buck_EN_Pin)));
}

static void io_control_handle(const char *args)
{
    char target[16];
    char state[8];
    GPIO_PinState pinState;
    bool setHigh;

    if (args == NULL) {
        io_control_help();
        io_control_status();
        return;
    }

    while (*args == ' ') {
        args++;
    }

    if (*args == '\0' || strcmp(args, "help") == 0) {
        io_control_help();
        io_control_status();
        return;
    }
    if (strcmp(args, "status") == 0) {
        io_control_status();
        return;
    }

    if (sscanf(args, "%15s %7s", target, state) != 2) {
        send("IO CONTROL: usage error\r\n");
        io_control_help();
        return;
    }

    if (strcmp(state, "on") == 0 || strcmp(state, "1") == 0) {
        setHigh = true;
        pinState = GPIO_PIN_SET;
    } else if (strcmp(state, "off") == 0 || strcmp(state, "0") == 0) {
        setHigh = false;
        pinState = GPIO_PIN_RESET;
    } else {
        send("IO CONTROL: state must be on/off\r\n");
        return;
    }

    if (!require_disarmed("IO Control")) {
        return;
    }

    if (strcmp(target, "imu") == 0) {
        HAL_GPIO_WritePin(MPU6050_EN_GPIO_Port, MPU6050_EN_Pin, pinState);
    } else if (strcmp(target, "bt") == 0) {
        HAL_GPIO_WritePin(HC_05_EN_GPIO_Port, HC_05_EN_Pin, pinState);
    } else if (strcmp(target, "bmpi2c") == 0) {
        HAL_GPIO_WritePin(BMP388_I2C_EN_GPIO_Port, BMP388_I2C_EN_Pin, pinState);
    } else if (strcmp(target, "bmppwr") == 0) {
        HAL_GPIO_WritePin(BMP388_PWR_EN_GPIO_Port, BMP388_PWR_EN_Pin, pinState);
    } else if (strcmp(target, "qmcpwr") == 0) {
        HAL_GPIO_WritePin(QMC5883_PWR_EN_GPIO_Port, QMC5883_PWR_EN_Pin, pinState);
    } else if (strcmp(target, "buck0") == 0) {
        Power_SetBuckEnable(0u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "buck1") == 0) {
        Power_SetBuckEnable(1u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "buck2") == 0) {
        Power_SetBuckEnable(2u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "buck3") == 0) {
        Power_SetBuckEnable(3u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "allbucks") == 0) {
        Power_SetBuckEnable(0u, setHigh ? 1u : 0u);
        Power_SetBuckEnable(1u, setHigh ? 1u : 0u);
        Power_SetBuckEnable(2u, setHigh ? 1u : 0u);
        Power_SetBuckEnable(3u, setHigh ? 1u : 0u);
    } else {
        sendf("IO CONTROL: unknown target '%s'\r\n", target);
        io_control_help();
        return;
    }

    HAL_Delay(5);
    sendf("IO CONTROL: %s -> %s\r\n", target, setHigh ? "ON" : "OFF");
    io_control_status();
}

static void io_control_handle_target_state(const char *target, const char *state)
{
    char args[32];

    if (target == NULL || state == NULL) {
        return;
    }
    (void)snprintf(args, sizeof(args), "%s %s", target, state);
    io_control_handle(args);
}

static void show_mpu6050(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    bool read_ok = IMU_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
    bool plaus_ok = IMU_CheckPlausibility();
    float ax_mps2 = 0.0f;
    float ay_mps2 = 0.0f;
    float az_mps2 = 0.0f;
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;
    AttitudeAngles angles = Attitude_GetAngles();
    float tilt = Attitude_GetTiltAngle();

    if (read_ok) {
        IMU_GetAccelMps2(&ax_mps2, &ay_mps2, &az_mps2);
        IMU_GetGyroDPS(&gx_dps, &gy_dps, &gz_dps);
    }

    sendf("MPU6050: id=0x%02X read=%s plaus=%s "
          "a[m/s2]=[%.2f %.2f %.2f] g[dps]=[%.2f %.2f %.2f] "
          "att[deg]=[r%.1f p%.1f y%.1f] tilt=%.1f last=%lums hal=%s(%d)\r\n",
          IMU_GetWhoAmI(),
          read_ok ? "YES" : "NO",
          plaus_ok ? "YES" : "NO",
          ax_mps2, ay_mps2, az_mps2,
          gx_dps, gy_dps, gz_dps,
          angles.roll, angles.pitch, angles.yaw,
          tilt,
          (unsigned long)IMU_GetLastReadMs(),
          hal_status_str(IMU_GetLastHalStatus()),
          (int)IMU_GetLastHalStatus());
}

static void show_bmp_sensor(void)
{
    uint32_t now = HAL_GetTick();
    bool upd_ok = Baro_Update(now);
    uint8_t chip_id = Baro_GetChipId();

    sendf("BMP: chip=%s(0x%02X) upd=%s id_ok=%s calib=%s healthy=%s "
          "P=%.2fhPa T=%.2fC Alt=%.2fm Vz=%.2fm/s "
          "last=%lums i2c_fail=%lu hal=%s(%d)\r\n",
          baro_chip_name(chip_id),
          chip_id,
          upd_ok ? "YES" : "NO",
          Baro_IsIdentityOK() ? "YES" : "NO",
          Baro_IsCalibrationLoaded() ? "YES" : "NO",
          Baro_IsHealthy() ? "YES" : "NO",
          Baro_GetPressureHpa(),
          Baro_GetTemperatureC(),
          Baro_GetAltitudeMeters(),
          Baro_GetClimbRateMps(),
          (unsigned long)Baro_GetLastUpdateMs(),
          (unsigned long)Baro_GetI2CFailCount(),
          hal_status_str(Baro_GetLastHalStatus()),
          (int)Baro_GetLastHalStatus());
}

static void show_magnetometer(void)
{
    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;
    float heading = 0.0f;
    float mahaThresh = Settings_GetMagMahaThreshold();
    bool read_ok = Compass_ReadRaw(&mx, &my, &mz);
    bool maha_ok = false;

    if (read_ok) {
        heading = Compass_ComputeHeading(mx, my, mz);
        maha_ok = Compass_CheckMahalanobis(mx, my, mz, mahaThresh);
    }

    sendf("MAG: addr=0x%02X chip=0x%02X read=%s maha=%s(thr=%.2f) "
          "raw=[%d %d %d] heading=%.1fdeg "
          "last=%lums ok=%lu fail=%lu hal=%s(%d)\r\n",
          Compass_GetAddress7bit(),
          Compass_GetChipId(),
          read_ok ? "YES" : "NO",
          maha_ok ? "PASS" : "FAIL",
          mahaThresh,
          (int)mx, (int)my, (int)mz,
          heading,
          (unsigned long)Compass_GetLastReadMs(),
          (unsigned long)Compass_GetReadOkCount(),
          (unsigned long)Compass_GetReadFailCount(),
          hal_status_str(Compass_GetLastHalStatus()),
          (int)Compass_GetLastHalStatus());
}

static void show_ina219(void)
{
    uint8_t count = Battery_GetINA219Count();
    uint8_t i;
    float pack_v = Battery_ReadPackVoltage();
    float cell_v = Battery_ReadPerCellVoltage();
    float total_a = 0.0f;

    if (count == 0u) {
        sendf("INA219: none detected (configured=0x%02X) hal=%s(%d)\r\n",
              Battery_GetI2CAddress7bit(),
              hal_status_str(Battery_GetLastI2CStatus()),
              (int)Battery_GetLastI2CStatus());
        return;
    }

    sendf("INA219: count=%u pack=%.2fV cell=%.2fV\r\n",
          (unsigned int)count,
          pack_v,
          cell_v);

    for (i = 0u; i < count; i++) {
        float ia = Battery_ReadCurrentAt(i);
        total_a += ia;
        sendf("  INA[%u]: addr=0x%02X I=%.3fA hal=%s(%d)\r\n",
              (unsigned int)i,
              Battery_GetINA219Address7bitAt(i),
              ia,
              hal_status_str(Battery_GetLastI2CStatusAt(i)),
              (int)Battery_GetLastI2CStatusAt(i));
    }

    sendf("  INA TOTAL: %.3fA\r\n", total_a);
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

    sendf("  BAT: %s ina_count=%u addr0=0x%02X v_last=%lums i_last=%lums rate=%uHz adc_to=%lu i2c_err=%lu adc=%s(%d) i2c=%s(%d)\r\n",
          pass_fail(batt_ok),
          (unsigned int)Battery_GetINA219Count(),
          Battery_GetI2CAddress7bit(),
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
    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;
    bool mag_read_ok;
    bool mag_maha_ok;
    float mag_heading = 0.0f;
    float mag_thresh = Settings_GetMagMahaThreshold();
    bool mag_pass;

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
    sendf("  BATT: %s ina_count=%u addr0=0x%02X V=%.2f Cell=%.2f I=%.2f i2c_err=%lu adc_to=%lu halI2C=%s(%d) halADC=%s(%d) lastV=%lums lastI=%lums rate=%uHz\r\n",
          pass_fail(batt_pass),
          (unsigned int)Battery_GetINA219Count(),
          Battery_GetI2CAddress7bit(),
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

    mag_read_ok = Compass_ReadRaw(&mx, &my, &mz);
    mag_maha_ok = mag_read_ok && Compass_CheckMahalanobis(mx, my, mz, mag_thresh);
    if (mag_read_ok) {
        mag_heading = Compass_ComputeHeading(mx, my, mz);
    }
    mag_pass = (!Settings_GetCompassEnabled()) || (mag_read_ok && mag_maha_ok);
    sendf("  MAG: %s addr=0x%02X chip=0x%02X read=%s maha=%s(thr=%.2f) raw=[%d %d %d] heading=%.1f last=%lums ok=%lu fail=%lu hal=%s(%d) rate=%uHz\r\n",
          pass_fail(mag_pass),
          Compass_GetAddress7bit(),
          Compass_GetChipId(),
          mag_read_ok ? "YES" : "NO",
          mag_maha_ok ? "PASS" : "FAIL",
          mag_thresh,
          (int)mx, (int)my, (int)mz,
          mag_heading,
          (unsigned long)Compass_GetLastReadMs(),
          (unsigned long)Compass_GetReadOkCount(),
          (unsigned long)Compass_GetReadFailCount(),
          hal_status_str(Compass_GetLastHalStatus()),
          (int)Compass_GetLastHalStatus(),
          rates.mag_hz);
}

static void diagnostics_start_bus_scan_internal(bool announce)
{
    if (!require_disarmed("Bus Scan")) {
        return;
    }
    memset(&busScan, 0, sizeof(busScan));
    busScan.active = true;
    busScan.addr = 0x08u;
    busScan.startMs = HAL_GetTick();
    busScan.lastStatus = HAL_OK;
    busScanNextStartMs = 0u;
    if (announce) {
        send("BUS SCAN: started (0x08..0x77, max 200ms)\r\n");
    }
}

static void diagnostics_start_bus_scan(void)
{
    diagnostics_start_bus_scan_internal(true);
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

    if (busScanRepeat && !FlightState_IsArmed()) {
        busScan.active = false;
        busScanNextStartMs = HAL_GetTick() + MENU_STREAM_PERIOD_MS;
        return;
    }
    busScan.active = false;
}

static void diagnostics_bus_scan_task(uint32_t now)
{
    uint8_t step;

    if (!busScan.active) {
        if (busScanRepeat &&
            !FlightState_IsArmed() &&
            (busScanNextStartMs == 0u || (int32_t)(now - busScanNextStartMs) >= 0)) {
            diagnostics_start_bus_scan_internal(false);
        }
        return;
    }
    if (FlightState_IsArmed()) {
        busScanRepeat = false;
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
    const char *mountPath = (SDPath[0] != '\0') ? SDPath : "0:/";
    FRESULT fr_mount;
    FRESULT fr = FR_OK;
    UINT bw = 0u;
    UINT br = 0u;
    char path[32] = {0};
    uint8_t sdDetected = BSP_SD_IsDetected();
    const char payload[] = "hawk_diag";
    char readback[sizeof(payload)] = {0};
    bool pass = true;

    if (!require_disarmed("Storage Test")) {
        return;
    }

    fr_mount = f_mount(&SDFatFS, mountPath, 1u);
    if (fr_mount == FR_NOT_READY) {
        HAL_Delay(20);
        fr_mount = f_mount(&SDFatFS, mountPath, 1u);
    }
    if (fr_mount != FR_OK) {
        pass = false;
    }

    if (pass) {
        size_t n = strlen(mountPath);
        if (n > 0u && mountPath[n - 1u] == '/') {
            snprintf(path, sizeof(path), "%sdiag.tmp", mountPath);
        } else {
            snprintf(path, sizeof(path), "%s/diag.tmp", mountPath);
        }
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

    sendf("STORAGE TEST: %s mount_fr=%d io_fr=%d sd_det=%u path=%s ini=%s pending=%s last_try=%lu last_ok=%lu\r\n",
          pass_fail(pass),
          (int)fr_mount,
          (int)fr,
          (unsigned int)sdDetected,
          mountPath,
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
    rxByte = 0u;
    rxHead = 0u;
    rxTail = 0u;
    actionMask = DEBUG_MENU_NONE;
    gpsDropPrevForMenu = GPS_GetDroppedByteCount();
    imuStream = false;
    bmpStream = false;
    magStream = false;
    inaStream = false;
    memset(&busScan, 0, sizeof(busScan));
    memset(&pwmTest, 0, sizeof(pwmTest));
    busScanRepeat = false;
    busScanNextStartMs = 0u;
    escAnsiState = 0u;

    show_menu();
    HAL_UART_Receive_IT(dbgUart, (uint8_t *)&rxByte, 1);
}

static void process_actions(void)
{
    if (actionMask & DEBUG_MENU_ACTIVE_TEST) {
        if (FlightState_IsArmed()) {
            send("ARMED: DIAGNOSTICS LOCKED\r\n");
            actionMask &= (uint8_t)(~DEBUG_MENU_ACTIVE_TEST);
        } else {
            diagnostics_active_sensor_test();
        }
    }
    if (actionMask & DEBUG_MENU_SENSORS) {
        diagnostics_quick_snapshot();
    }
    if (actionMask & DEBUG_MENU_PPM) {
        show_ppm();
    }
    if (imuStream) {
        show_mpu6050();
    }
    if (bmpStream) {
        show_bmp_sensor();
    }
    if (magStream) {
        show_magnetometer();
    }
    if (inaStream) {
        show_ina219();
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
    if (escAnsiState == 1u) {
        if (ch == '[' || ch == 'O') {
            escAnsiState = 2u;
            return;
        }
        escAnsiState = 0u;
    } else if (escAnsiState == 2u) {
        escAnsiState = 0u;
        return;
    }

    if (ch == 0x1Bu) {
        // ESC key: stop all repeating streams/loops immediately.
        stop_streaming_modes(true);
        cmdIdx = 0u;
        send("> ");
        // Swallow optional ANSI continuation sequence if present.
        escAnsiState = 1u;
        return;
    }
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
        if (actionMask & DEBUG_MENU_ACTIVE_TEST) {
            actionMask &= (uint8_t)(~DEBUG_MENU_ACTIVE_TEST);
            send("ACTIVE SENSOR STREAM: OFF\r\n");
        } else if (!diagnostics_lock_for_active_tests()) {
            actionMask |= DEBUG_MENU_ACTIVE_TEST;
            send("ACTIVE SENSOR STREAM: ON (0.75s)\r\n");
        }
    } else if (strcmp(cmdBuf, "2 once") == 0) {
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_active_sensor_test();
        }
    } else if (strcmp(cmdBuf, "3") == 0) {
        if (busScanRepeat) {
            busScanRepeat = false;
            send("BUS SCAN LOOP: OFF\r\n");
        } else if (!diagnostics_lock_for_active_tests()) {
            busScanRepeat = true;
            diagnostics_start_bus_scan();
            send("BUS SCAN LOOP: ON (0.75s)\r\n");
        }
    } else if (strcmp(cmdBuf, "3 once") == 0) {
        if (!diagnostics_lock_for_active_tests()) {
            busScanRepeat = false;
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
    } else if (strcmp(cmdBuf, "mpu") == 0 || strcmp(cmdBuf, "imu") == 0) {
        imuStream = !imuStream;
        sendf("MPU6050 STREAM: %s (0.75s)\r\n", imuStream ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "mpu once") == 0 || strcmp(cmdBuf, "imu once") == 0) {
        show_mpu6050();
    } else if (strcmp(cmdBuf, "bmp") == 0 || strcmp(cmdBuf, "baro") == 0) {
        bmpStream = !bmpStream;
        sendf("BMP STREAM: %s (0.75s)\r\n", bmpStream ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "bmp once") == 0 || strcmp(cmdBuf, "baro once") == 0) {
        show_bmp_sensor();
    } else if (strcmp(cmdBuf, "mag") == 0 || strcmp(cmdBuf, "compass") == 0) {
        magStream = !magStream;
        sendf("MAG STREAM: %s (0.75s)\r\n", magStream ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "mag once") == 0 || strcmp(cmdBuf, "compass once") == 0) {
        show_magnetometer();
    } else if (strcmp(cmdBuf, "ina") == 0 || strcmp(cmdBuf, "battery") == 0) {
        inaStream = !inaStream;
        sendf("INA219 STREAM: %s (0.75s)\r\n", inaStream ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "ina once") == 0 || strcmp(cmdBuf, "battery once") == 0) {
        show_ina219();
    } else if (strcmp(cmdBuf, "g") == 0) {
        actionMask ^= DEBUG_MENU_GPS;
        sendf("GPS STREAM: %s (0.75s)\r\n", (actionMask & DEBUG_MENU_GPS) ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "g once") == 0) {
        show_gps();
    } else if (strcmp(cmdBuf, "o") == 0) {
        actionMask ^= DEBUG_MENU_SONAR;
        sendf("SONAR STREAM: %s (0.75s)\r\n", (actionMask & DEBUG_MENU_SONAR) ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "o once") == 0) {
        show_sonar();
    } else if (strcmp(cmdBuf, "p") == 0) {
        actionMask ^= DEBUG_MENU_PPM;
        sendf("RC STREAM: %s (0.75s)\r\n", (actionMask & DEBUG_MENU_PPM) ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "p once") == 0) {
        show_ppm();
    } else if (strcmp(cmdBuf, "b") == 0) {
        actionMask ^= DEBUG_MENU_BUZZER;
        if (actionMask & DEBUG_MENU_BUZZER) {
            Buzzer_PlayTone(TONE_READY);
            send("BUZZER: ON\r\n");
        } else {
            Buzzer_Stop();
            send("BUZZER: OFF\r\n");
        }
    } else if (strcmp(cmdBuf, "v") == 0) {
        actionMask ^= DEBUG_MENU_SERVOS;
        sendf("SERVO DEBUG: %s\r\n", (actionMask & DEBUG_MENU_SERVOS) ? "ON" : "OFF");
    } else if (strcmp(cmdBuf, "io") == 0) {
        io_control_help();
        io_control_status();
    } else if (strcmp(cmdBuf, "ios") == 0) {
        io_control_status();
    } else if (strncmp(cmdBuf, "ioi ", 4) == 0) {
        io_control_handle_target_state("imu", cmdBuf + 4);
    } else if (strncmp(cmdBuf, "iobt ", 5) == 0) {
        io_control_handle_target_state("bt", cmdBuf + 5);
    } else if (strncmp(cmdBuf, "iobmpi2c ", 9) == 0) {
        io_control_handle_target_state("bmpi2c", cmdBuf + 9);
    } else if (strncmp(cmdBuf, "iobmppwr ", 9) == 0) {
        io_control_handle_target_state("bmppwr", cmdBuf + 9);
    } else if (strncmp(cmdBuf, "ioqmcpwr ", 9) == 0) {
        io_control_handle_target_state("qmcpwr", cmdBuf + 9);
    } else if (strncmp(cmdBuf, "iob0 ", 5) == 0) {
        io_control_handle_target_state("buck0", cmdBuf + 5);
    } else if (strncmp(cmdBuf, "iob1 ", 5) == 0) {
        io_control_handle_target_state("buck1", cmdBuf + 5);
    } else if (strncmp(cmdBuf, "iob2 ", 5) == 0) {
        io_control_handle_target_state("buck2", cmdBuf + 5);
    } else if (strncmp(cmdBuf, "iob3 ", 5) == 0) {
        io_control_handle_target_state("buck3", cmdBuf + 5);
    } else if (strncmp(cmdBuf, "ioba ", 5) == 0) {
        io_control_handle_target_state("allbucks", cmdBuf + 5);
    } else if (strcmp(cmdBuf, "x") == 0) {
        // Backward-compatible alias.
        io_control_help();
        io_control_status();
    } else if (strncmp(cmdBuf, "x ", 2) == 0) {
        // Backward-compatible alias.
        io_control_handle(cmdBuf + 2);
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
    uint8_t ch;

    diagnostics_bus_scan_task(now);
    diagnostics_pwm_test_task(now);

    if ((now - lastBeat) >= MENU_STREAM_PERIOD_MS) {
        process_actions();
        lastBeat = now;
    }

    while (debug_rx_pop(&ch)) {
        process_char(ch);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == dbgUart) {
        debug_rx_push_isr(rxByte);
        HAL_UART_Receive_IT(dbgUart, (uint8_t *)&rxByte, 1);
    }

    // Chain other modules that use UART receive interrupts
    GPS_UART_RxCpltCallback(huart);
}
