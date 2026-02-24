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
#include "telemetry.h"
#include "bt_link.h"
#include "main.h"
#include "at7456e.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>

#define DBG_UART_TX_TIMEOUT_MS               200u
#define BUS_SCAN_MAX_MS                       200u
#define BUS_SCAN_STEP_ADDRS                   4u
#define GPS_DROP_DELTA_FAIL_THRESHOLD         8u
#define PWM_TEST_STEP_MS                      50u
#define PWM_TEST_DURATION_MS                  4000u
#define PWM_TEST_STEP_US                      25u
#define ACTIVE_TEST_SONAR_EDGE_FRESH_MS       500u
#define ACTIVE_TEST_GPS_FRESH_MS              2000u
#define ACTIVE_TEST_RC_FRESH_MS               500u
#define STREAM_PERIOD_DEFAULT_MS              750u
#define STREAM_PERIOD_MIN_MS                  100u
#define STREAM_PERIOD_MAX_MS                 5000u
#define DEBUG_RX_RING_SIZE                    64u

#define DEPR_STREAM_RC_BIT               (1u << 0)
#define DEPR_STREAM_GPS_BIT              (1u << 1)
#define DEPR_STREAM_SONAR_BIT            (1u << 2)
#define DEPR_STREAM_IMU_BIT              (1u << 3)
#define DEPR_STREAM_BARO_BIT             (1u << 4)
#define DEPR_STREAM_MAG_BIT              (1u << 5)
#define DEPR_STREAM_INA_BIT              (1u << 6)
#define DEPR_STREAM_OSD_BIT              (1u << 7)
#define DEPR_BUZZER_BIT                  (1u << 8)
#define DEPR_SERVO_BIT                   (1u << 9)
#define DEPR_TEST_ACTIVE_BIT             (1u << 10)
#define DEPR_TEST_BUS_BIT                (1u << 11)
#define DEPR_TEST_STORAGE_BIT            (1u << 12)
#define DEPR_TEST_PWM_BIT                (1u << 13)
#define DEPR_IO_BIT                      (1u << 14)

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;

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
static bool osdStream;
static bool btStream;
static uint16_t streamPeriodMs;
static uint32_t deprecateMask;
static uint32_t btTestSeq;

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
static bool is_armed_readonly_mode(void);
static bool block_while_armed(bool condition);
static void debug_rx_push_isr(uint8_t ch);
static bool debug_rx_pop(uint8_t *ch);
static void show_mpu6050(bool verbose);
static void show_bmp_sensor(bool verbose);
static void show_magnetometer(bool verbose);
static void show_ina219(bool verbose);
static void show_osd(bool verbose);
static void show_ppm(bool verbose);
static void show_sonar(bool verbose);
static void show_gps(bool verbose);
static void show_bt(bool verbose);
static void bt_read_dump(uint16_t maxBytes);
static void show_sys_status(void);
static void show_health(void);
static void show_rates(void);
static void show_module(const char *module, bool verbose);
static bool set_stream_state(const char *module, bool on);
static bool handle_v2_command(const char *cmd);
static void maybe_print_deprecated(uint32_t bit, const char *use_hint);

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

static const char* rc_signal_str(RC_SignalType t)
{
    switch (t) {
    case RC_SIGNAL_PPM:
        return "PPM";
    case RC_SIGNAL_PWM1:
        return "PWM1";
    case RC_SIGNAL_NOISY:
        return "NOISY";
    case RC_SIGNAL_NONE:
    default:
        return "NONE";
    }
}

static void send(const char *s)
{
    size_t len;

    if (dbgUart == NULL || s == NULL) {
        return;
    }
    len = strlen(s);
    HAL_UART_Transmit(dbgUart, (uint8_t*)s, (uint16_t)len, DBG_UART_TX_TIMEOUT_MS);
    HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)len, DBG_UART_TX_TIMEOUT_MS);
#if BT_USE_OLD_BOARD_PE8_PE9
    if (BTLink_IsReady()) {
        (void)BTLink_Send((const uint8_t*)s, (uint16_t)len, DBG_UART_TX_TIMEOUT_MS);
    }
#endif
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
        HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, DBG_UART_TX_TIMEOUT_MS);
#if BT_USE_OLD_BOARD_PE8_PE9
        if (BTLink_IsReady()) {
            (void)BTLink_Send((const uint8_t*)buf, len, DBG_UART_TX_TIMEOUT_MS);
        }
#endif
    }
}

static bool require_disarmed(const char *name)
{
    (void)name;
    if (!FlightState_IsArmed()) {
        return true;
    }
    send("ERR code=ARMED_DIAGNOSTICS_LOCKED\r\n");
    return false;
}

static bool diagnostics_lock_for_active_tests(void)
{
    if (!FlightState_IsArmed()) {
        return false;
    }
    send("ERR code=ARMED_DIAGNOSTICS_LOCKED\r\n");
    return true;
}

static bool is_armed_readonly_mode(void)
{
    return FlightState_IsArmed();
}

static bool block_while_armed(bool condition)
{
    if (condition && is_armed_readonly_mode()) {
        send("ERR code=ARMED_DIAGNOSTICS_LOCKED\r\n");
        return true;
    }
    return false;
}

static void show_menu(void)
{
    send("\r\n=== Hawk Debug CLI v2 ===\r\n");
    send("SYSTEM:\r\n");
    send("  help | help <system|sensor|test|io|osd|bt|legacy>\r\n");
    send("  status | health | rates | period <ms> | stop\r\n");
    send("SENSORS:\r\n");
    send("  show <imu|baro|mag|ina|rc|gps|sonar|osd|bt|sys> [verbose]\r\n");
    send("  stream <module|all> on|off\r\n");
    send("TESTS:\r\n");
    send("  test active once|loop on|loop off\r\n");
    send("  test bus once|loop on|loop off\r\n");
    send("  test storage run\r\n");
    send("  test pwm start|stop\r\n");
    send("ACTUATORS:\r\n");
    send("  buzzer on|off|beep\r\n");
    send("  servo debug on|off (alias: servo test on|off)\r\n");
    send("IO:\r\n");
    send("  io status\r\n");
    send("  io set <imu|bt|bmp_i2c|bmp_pwr|qmc_pwr|buck0|buck1|buck2|buck3|buck_all> on|off\r\n");
    send("BT:\r\n");
    send("  bt status|show\r\n");
    send("  bt baud <rate>\r\n");
    send("  bt test (send marker line to phone)\r\n");
    send("  bt gps (send GPS line to phone)\r\n");
    send("  bt read [n] (RX disabled on old board; always empty)\r\n");
    send("  bt on|off (same as io set bt on|off)\r\n");
    send("OSD:\r\n");
    send("  osd show|init|on|off|cls|test\r\n");
    send("Legacy aliases are still accepted. Use: help legacy\r\n");
    send("> ");
}

static void stop_streaming_modes(bool announce)
{
    bool hadStreams = (actionMask != DEBUG_MENU_NONE) || busScanRepeat || busScan.active || pwmTest.active ||
                      imuStream || bmpStream || magStream || inaStream || osdStream || btStream;

    actionMask = DEBUG_MENU_NONE;
    imuStream = false;
    bmpStream = false;
    magStream = false;
    inaStream = false;
    osdStream = false;
    btStream = false;
    busScanRepeat = false;
    busScan.active = false;
    busScanNextStartMs = 0u;
    diagnostics_pwm_test_stop("stop");
    Buzzer_Stop();

    if (announce && hadStreams) {
        send("INFO code=STOPPED all=YES\r\n");
    }
}

static void show_help(void)
{
    show_menu();
}

static void show_diag_menu(void)
{
    send("TEST HELP:\r\n");
    send("  test active once|loop on|loop off\r\n");
    send("  test bus once|loop on|loop off\r\n");
    send("  test storage run\r\n");
    send("  test pwm start|stop\r\n");
}

static void io_control_help(void)
{
    send("IO HELP:\r\n");
    send("  io status\r\n");
    send("  io set imu|bt|bmp_i2c|bmp_pwr|qmc_pwr|buck0|buck1|buck2|buck3|buck_all on|off\r\n");
    send("  Note: io set is DISARMED only\r\n");
}

static void io_control_status(void)
{
    sendf("IO imu=%s bt=%s bmp_i2c=%s bmp_pwr=%s qmc_pwr=%s buck0=%s buck1=%s buck2=%s buck3=%s\r\n",
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
        send("ERR code=IO_USAGE\r\n");
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
        send("ERR code=IO_STATE\r\n");
        return;
    }

    if (!require_disarmed("IO Control")) {
        return;
    }

    if (strcmp(target, "imu") == 0) {
        HAL_GPIO_WritePin(MPU6050_EN_GPIO_Port, MPU6050_EN_Pin, pinState);
    } else if (strcmp(target, "bt") == 0) {
        HAL_GPIO_WritePin(HC_05_EN_GPIO_Port, HC_05_EN_Pin, pinState);
    } else if (strcmp(target, "bmpi2c") == 0 || strcmp(target, "bmp_i2c") == 0) {
        HAL_GPIO_WritePin(BMP388_I2C_EN_GPIO_Port, BMP388_I2C_EN_Pin, pinState);
    } else if (strcmp(target, "bmppwr") == 0 || strcmp(target, "bmp_pwr") == 0) {
        HAL_GPIO_WritePin(BMP388_PWR_EN_GPIO_Port, BMP388_PWR_EN_Pin, pinState);
    } else if (strcmp(target, "qmcpwr") == 0 || strcmp(target, "qmc_pwr") == 0) {
        HAL_GPIO_WritePin(QMC5883_PWR_EN_GPIO_Port, QMC5883_PWR_EN_Pin, pinState);
    } else if (strcmp(target, "buck0") == 0) {
        Power_SetBuckEnable(0u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "buck1") == 0) {
        Power_SetBuckEnable(1u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "buck2") == 0) {
        Power_SetBuckEnable(2u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "buck3") == 0) {
        Power_SetBuckEnable(3u, setHigh ? 1u : 0u);
    } else if (strcmp(target, "allbucks") == 0 || strcmp(target, "buck_all") == 0) {
        Power_SetBuckEnable(0u, setHigh ? 1u : 0u);
        Power_SetBuckEnable(1u, setHigh ? 1u : 0u);
        Power_SetBuckEnable(2u, setHigh ? 1u : 0u);
        Power_SetBuckEnable(3u, setHigh ? 1u : 0u);
    } else {
        sendf("ERR code=IO_TARGET target=%s\r\n", target);
        io_control_help();
        return;
    }

    sendf("INFO code=IO_SET target=%s value=%s\r\n", target, setHigh ? "ON" : "OFF");
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

static void show_mpu6050(bool verbose)
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

    if (!verbose) {
        sendf("IMU id=0x%02X id_ok=%s read_ok=%s plaus=%s roll=%.1f pitch=%.1f yaw=%.1f "
              "ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f last=%lums\r\n",
              IMU_GetWhoAmI(),
              IMU_IsIdentityOK() ? "YES" : "NO",
              read_ok ? "YES" : "NO",
              plaus_ok ? "YES" : "NO",
              angles.roll, angles.pitch, angles.yaw,
              ax_mps2, ay_mps2, az_mps2,
              gx_dps, gy_dps, gz_dps,
              (unsigned long)IMU_GetLastReadMs());
    } else {
        sendf("IMU id=0x%02X id_ok=%s read_ok=%s plaus=%s roll=%.1f pitch=%.1f yaw=%.1f tilt=%.1f "
              "ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f "
              "ok=%lu fail=%lu hal=%s hal_code=%d last=%lums\r\n",
              IMU_GetWhoAmI(),
              IMU_IsIdentityOK() ? "YES" : "NO",
              read_ok ? "YES" : "NO",
              plaus_ok ? "YES" : "NO",
              angles.roll, angles.pitch, angles.yaw, tilt,
              ax_mps2, ay_mps2, az_mps2,
              gx_dps, gy_dps, gz_dps,
              (unsigned long)IMU_GetReadOkCount(),
              (unsigned long)IMU_GetReadFailCount(),
              hal_status_str(IMU_GetLastHalStatus()),
              (int)IMU_GetLastHalStatus(),
              (unsigned long)IMU_GetLastReadMs());
    }
}

static void show_bmp_sensor(bool verbose)
{
    uint32_t now = HAL_GetTick();
    bool upd_ok = Baro_Update(now);
    uint8_t chip_id = Baro_GetChipId();

    if (!verbose) {
        sendf("BARO model=%s id=0x%02X id_ok=%s calib=%s healthy=%s upd=%s "
              "press=%.2f temp=%.2f alt=%.2f vz=%.2f last=%lums\r\n",
              baro_chip_name(chip_id),
              chip_id,
              Baro_IsIdentityOK() ? "YES" : "NO",
              Baro_IsCalibrationLoaded() ? "YES" : "NO",
              Baro_IsHealthy() ? "YES" : "NO",
              upd_ok ? "YES" : "NO",
              Baro_GetPressureHpa(),
              Baro_GetTemperatureC(),
              Baro_GetAltitudeMeters(),
              Baro_GetClimbRateMps(),
              (unsigned long)Baro_GetLastUpdateMs());
    } else {
        sendf("BARO model=%s id=0x%02X id_ok=%s calib=%s healthy=%s upd=%s "
              "press=%.2f temp=%.2f alt=%.2f vz=%.2f update_ok=%lu i2c_fail=%lu "
              "hal=%s hal_code=%d last=%lums\r\n",
              baro_chip_name(chip_id),
              chip_id,
              Baro_IsIdentityOK() ? "YES" : "NO",
              Baro_IsCalibrationLoaded() ? "YES" : "NO",
              Baro_IsHealthy() ? "YES" : "NO",
              upd_ok ? "YES" : "NO",
              Baro_GetPressureHpa(),
              Baro_GetTemperatureC(),
              Baro_GetAltitudeMeters(),
              Baro_GetClimbRateMps(),
              (unsigned long)Baro_GetUpdateOkCount(),
              (unsigned long)Baro_GetI2CFailCount(),
              hal_status_str(Baro_GetLastHalStatus()),
              (int)Baro_GetLastHalStatus(),
              (unsigned long)Baro_GetLastUpdateMs());
    }
}

static void show_magnetometer(bool verbose)
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

    if (!verbose) {
        sendf("MAG addr=0x%02X chip=0x%02X read=%s maha=%s heading=%.1f "
              "x=%d y=%d z=%d last=%lums\r\n",
              Compass_GetAddress7bit(),
              Compass_GetChipId(),
              read_ok ? "YES" : "NO",
              maha_ok ? "PASS" : "FAIL",
              heading,
              (int)mx, (int)my, (int)mz,
              (unsigned long)Compass_GetLastReadMs());
    } else {
        sendf("MAG addr=0x%02X chip=0x%02X read=%s maha=%s maha_thr=%.2f heading=%.1f "
              "x=%d y=%d z=%d ok=%lu fail=%lu hal=%s hal_code=%d last=%lums\r\n",
              Compass_GetAddress7bit(),
              Compass_GetChipId(),
              read_ok ? "YES" : "NO",
              maha_ok ? "PASS" : "FAIL",
              mahaThresh,
              heading,
              (int)mx, (int)my, (int)mz,
              (unsigned long)Compass_GetReadOkCount(),
              (unsigned long)Compass_GetReadFailCount(),
              hal_status_str(Compass_GetLastHalStatus()),
              (int)Compass_GetLastHalStatus(),
              (unsigned long)Compass_GetLastReadMs());
    }
}

static void show_ina219(bool verbose)
{
    uint8_t count = Battery_GetINA219Count();
    uint8_t i;
    float pack_v = Battery_ReadPackVoltage();
    float cell_v = Battery_ReadPerCellVoltage();
    float total_a = 0.0f;

    if (count == 0u) {
        sendf("INA count=0 addr0=0x%02X hal=%s hal_code=%d\r\n",
              Battery_GetI2CAddress7bit(),
              hal_status_str(Battery_GetLastI2CStatus()),
              (int)Battery_GetLastI2CStatus());
        return;
    }

    sendf("INA count=%u pack=%.2f cell=%.2f\r\n",
          (unsigned int)count,
          pack_v,
          cell_v);

    for (i = 0u; i < count; i++) {
        float ia = Battery_ReadCurrentAt(i);
        total_a += ia;
        if (verbose) {
            sendf("INA_NODE idx=%u addr=0x%02X curr=%.3f hal=%s hal_code=%d\r\n",
                  (unsigned int)i,
                  Battery_GetINA219Address7bitAt(i),
                  ia,
                  hal_status_str(Battery_GetLastI2CStatusAt(i)),
                  (int)Battery_GetLastI2CStatusAt(i));
        }
    }

    sendf("INA total=%.3f i2c_err=%lu adc_to=%lu last_v=%lums last_i=%lums\r\n",
          total_a,
          (unsigned long)Battery_GetI2CErrorCount(),
          (unsigned long)Battery_GetADCTimeoutCount(),
          (unsigned long)Battery_GetLastVoltageMs(),
          (unsigned long)Battery_GetLastCurrentMs());
}

static void show_osd(bool verbose)
{
    uint8_t stat = AT7456_ReadStatus();

    if (!verbose) {
        sendf("OSD init=%s present=%s enabled=%s spi8=%s stat=0x%02X\r\n",
              AT7456_IsInitialized() ? "YES" : "NO",
              AT7456_IsPresent() ? "YES" : "NO",
              AT7456_IsEnabled() ? "YES" : "NO",
              AT7456_IsSPI8Bit() ? "YES" : "NO",
              stat);
    } else {
        sendf("OSD init=%s present=%s enabled=%s spi8=%s stat=0x%02X tx=%lu fail=%lu hal=%s hal_code=%d\r\n",
              AT7456_IsInitialized() ? "YES" : "NO",
              AT7456_IsPresent() ? "YES" : "NO",
              AT7456_IsEnabled() ? "YES" : "NO",
              AT7456_IsSPI8Bit() ? "YES" : "NO",
              stat,
              (unsigned long)AT7456_GetTxCount(),
              (unsigned long)AT7456_GetFailCount(),
              hal_status_str(AT7456_GetLastHalStatus()),
              (int)AT7456_GetLastHalStatus());
    }
}

static void show_ppm(bool verbose)
{
    RC_Diagnostics diag;
    char buf[320];
    int n;

    RC_GetDiagnostics(&diag);
    if (!verbose) {
        n = snprintf(buf, sizeof(buf),
                     "RC mode=%s frame=%lu stale=%s lost=%s rssi=%u ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u last=%lums\r\n",
                     rc_signal_str(diag.signalType),
                     (unsigned long)RC_GetFrameCount(),
                     RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
                     RC_LinkLostForSeconds(1u) ? "YES" : "NO",
                     RC_GetRSSI(),
                     RC_GetChannel(0), RC_GetChannel(1), RC_GetChannel(2),
                     RC_GetChannel(3), RC_GetChannel(4), RC_GetChannel(5),
                     (unsigned long)RC_GetLastFrameMs());
    } else {
        n = snprintf(buf, sizeof(buf),
                     "RC mode=%s frame=%lu stale=%s lost=%s rssi=%u ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u "
                     "sync=%lu good=%lu reject=%lu edge=%lu last_int=%uus pwm_hi=%uus level=%c last=%lums\r\n",
                     rc_signal_str(diag.signalType),
                     (unsigned long)RC_GetFrameCount(),
                     RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
                     RC_LinkLostForSeconds(1u) ? "YES" : "NO",
                     RC_GetRSSI(),
                     RC_GetChannel(0), RC_GetChannel(1), RC_GetChannel(2),
                     RC_GetChannel(3), RC_GetChannel(4), RC_GetChannel(5),
                     (unsigned long)diag.syncCount,
                     (unsigned long)diag.goodPulseCount,
                     (unsigned long)diag.rejectCount,
                     (unsigned long)diag.edgeCount,
                     (unsigned int)diag.lastIntervalUs,
                     (unsigned int)diag.lastPwmHighUs,
                     diag.sampledHighEdge ? 'H' : 'L',
                     (unsigned long)RC_GetLastFrameMs());
    }
    if (n > 0) {
        HAL_UART_Transmit(dbgUart, (uint8_t*)buf, (uint16_t)n, DBG_UART_TX_TIMEOUT_MS);
    }
}

static void show_sonar(bool verbose)
{
    if (!verbose) {
        sendf("SONAR d0=%.2f d1=%.2f d2=%.2f trig=%lu last=%lums\r\n",
              Sonar_ReadDistance(0), Sonar_ReadDistance(1), Sonar_ReadDistance(2),
              (unsigned long)Sonar_GetTriggerCount(),
              (unsigned long)Sonar_GetLastTriggerMs());
    } else {
        sendf("SONAR d0=%.2f d1=%.2f d2=%.2f trig=%lu last=%lums edges0=%lu edges1=%lu edges2=%lu timeouts=%lu\r\n",
              Sonar_ReadDistance(0), Sonar_ReadDistance(1), Sonar_ReadDistance(2),
              (unsigned long)Sonar_GetTriggerCount(),
              (unsigned long)Sonar_GetLastTriggerMs(),
              (unsigned long)Sonar_GetEdgeCount(0),
              (unsigned long)Sonar_GetEdgeCount(1),
              (unsigned long)Sonar_GetEdgeCount(2),
              (unsigned long)Sonar_GetEchoTimeoutCount());
    }
}

static void show_gps(bool verbose)
{
    uint32_t now = HAL_GetTick();
    uint32_t drop = GPS_GetDroppedByteCount();
    uint32_t dropDelta = drop - gpsDropPrevForMenu;
    bool fix = GPS_HasFix();

    gpsDropPrevForMenu = drop;
    if (!verbose) {
        sendf("GPS fix=%s sats=%u hdop=%.1f lat=%.6f lon=%.6f alt=%.1f age=%lums\r\n",
              fix ? "YES" : "NO",
              GPS_GetSatCount(), GPS_GetHDOP(),
              GPS_GetLatitude(), GPS_GetLongitude(), GPS_GetAltitude(),
              (unsigned long)((GPS_GetLastUpdateMs() > 0u) ? (now - GPS_GetLastUpdateMs()) : 0u));
    } else {
        sendf("GPS fix=%s sats=%u hdop=%.1f lat=%.6f lon=%.6f alt=%.1f age=%lums sps=%u drop=%lu delta=%lu ring=%u\r\n",
              fix ? "YES" : "NO",
              GPS_GetSatCount(), GPS_GetHDOP(),
              GPS_GetLatitude(), GPS_GetLongitude(), GPS_GetAltitude(),
              (unsigned long)((GPS_GetLastUpdateMs() > 0u) ? (now - GPS_GetLastUpdateMs()) : 0u),
              GPS_GetSentenceRateHz(),
              (unsigned long)drop,
              (unsigned long)dropDelta,
              GPS_GetRxRingLevel());
    }
}

static void show_bt(bool verbose)
{
    uint32_t now = HAL_GetTick();
    uint32_t lastTx = BTLink_GetLastTxMs();
    uint32_t ageMs = (lastTx > 0u) ? (now - lastTx) : 0u;
    HAL_StatusTypeDef st = BTLink_GetLastTxStatus();

    if (!verbose) {
        sendf("BT en=%s uart7=%s tx_mode=UART7_PE8 rx_mode=DISABLED tel_en=%s baud=%lu rate=%uHz tx_ok=%lu tx_fail=%lu last_tx=%lums\r\n",
              on_off(HAL_GPIO_ReadPin(HC_05_EN_GPIO_Port, HC_05_EN_Pin)),
              BTLink_IsReady() ? "READY" : "DOWN",
              Settings_GetTelemetryEnabled() ? "YES" : "NO",
              (unsigned long)BTLink_GetBaud(),
              (unsigned int)Settings_GetTelemetryStreamRateHz(),
              (unsigned long)BTLink_GetTxOkCount(),
              (unsigned long)BTLink_GetTxFailCount(),
              (unsigned long)ageMs);
    } else {
        sendf("BT en=%s uart7=%s tx_mode=UART7_PE8 rx_mode=DISABLED tel_en=%s baud=%lu rate=%uHz tx_ok=%lu tx_fail=%lu last_tx=%lums last_len=%u "
              "hal=%s hal_code=%ld uart_err=0x%08lX\r\n",
              on_off(HAL_GPIO_ReadPin(HC_05_EN_GPIO_Port, HC_05_EN_Pin)),
              BTLink_IsReady() ? "READY" : "DOWN",
              Settings_GetTelemetryEnabled() ? "YES" : "NO",
              (unsigned long)BTLink_GetBaud(),
              (unsigned int)Settings_GetTelemetryStreamRateHz(),
              (unsigned long)BTLink_GetTxOkCount(),
              (unsigned long)BTLink_GetTxFailCount(),
              (unsigned long)ageMs,
              (unsigned int)BTLink_GetLastTxLen(),
              hal_status_str(st),
              (long)BTLink_GetLastTxStatus(),
              (unsigned long)BTLink_GetLastTxError());
    }
}

static void bt_read_dump(uint16_t maxBytes)
{
    (void)maxBytes;
    send("WARN code=BT_RX_DISABLED mode=TX_ONLY\r\n");
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
    uint16_t cpuLoopHz = CPU_GetLoopRateHz();

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

    sendf("SYS state=0x%08lX ready=%s armed=%s health_all=%s loop=%uHz dt_last=%lums dt_max=%lums missed=%lu\r\n",
          (unsigned long)fs,
          FlightState_IsReady() ? "YES" : "NO",
          FlightState_IsArmed() ? "YES" : "NO",
          FlightState_AllHealthOK() ? "YES" : "NO",
          cpuLoopHz,
          (unsigned long)CPU_GetLastLoopDtMs(),
          (unsigned long)CPU_GetMaxLoopDtMs(),
          (unsigned long)CPU_GetMissedDeadlineCount());

    sendf("IMU pass=%s id=0x%02X id_ok=%s rate=%uHz last=%lums ok=%lu fail=%lu bias_x=%.3f bias_y=%.3f bias_z=%.3f\r\n",
          imu_ok ? "YES" : "NO",
          IMU_GetWhoAmI(),
          IMU_IsIdentityOK() ? "YES" : "NO",
          rates.imu_hz,
          (unsigned long)IMU_GetLastReadMs(),
          (unsigned long)IMU_GetReadOkCount(),
          (unsigned long)IMU_GetReadFailCount(),
          accelBx, accelBy, accelBz);

    sendf("BARO pass=%s id=0x%02X id_ok=%s calib=%s healthy=%s rate=%uHz last=%lums update_ok=%lu i2c_fail=%lu\r\n",
          baro_ok ? "YES" : "NO",
          Baro_GetChipId(),
          Baro_IsIdentityOK() ? "YES" : "NO",
          Baro_IsCalibrationLoaded() ? "YES" : "NO",
          Baro_IsHealthy() ? "YES" : "NO",
          rates.baro_hz,
          (unsigned long)Baro_GetLastUpdateMs(),
          (unsigned long)Baro_GetUpdateOkCount(),
          (unsigned long)Baro_GetI2CFailCount());

    sendf("GPS pass=%s fix=%s sats=%u hdop=%.1f rate=%uHz sps=%u drop=%lu delta=%lu ring=%u last=%lums\r\n",
          gps_ok ? "YES" : "NO",
          GPS_HasFix() ? "YES" : "NO",
          GPS_GetSatCount(),
          GPS_GetHDOP(),
          rates.gps_hz,
          GPS_GetSentenceRateHz(),
          (unsigned long)gps_drop,
          (unsigned long)gps_drop_delta,
          GPS_GetRxRingLevel(),
          (unsigned long)GPS_GetLastUpdateMs());

    sendf("SONAR pass=%s rate=%uHz d0=%.2f d1=%.2f d2=%.2f edges0=%lu edges1=%lu edges2=%lu timeouts=%lu last=%lums\r\n",
          sonar_ok ? "YES" : "NO",
          rates.sonar_hz,
          Sonar_ReadDistance(0), Sonar_ReadDistance(1), Sonar_ReadDistance(2),
          (unsigned long)Sonar_GetEdgeCount(0),
          (unsigned long)Sonar_GetEdgeCount(1),
          (unsigned long)Sonar_GetEdgeCount(2),
          (unsigned long)Sonar_GetEchoTimeoutCount(),
          (unsigned long)Sonar_GetLastTriggerMs());

    sendf("INA pass=%s count=%u addr0=0x%02X rate=%uHz last_v=%lums last_i=%lums i2c_err=%lu adc_to=%lu\r\n",
          batt_ok ? "YES" : "NO",
          (unsigned int)Battery_GetINA219Count(),
          Battery_GetI2CAddress7bit(),
          rates.battery_hz,
          (unsigned long)Battery_GetLastVoltageMs(),
          (unsigned long)Battery_GetLastCurrentMs(),
          (unsigned long)Battery_GetI2CErrorCount(),
          (unsigned long)Battery_GetADCTimeoutCount());

    sendf("RC pass=%s frame=%lu stale=%s lost=%s rssi=%u rate=%uHz last=%lums\r\n",
          rc_ok ? "YES" : "NO",
          (unsigned long)RC_GetFrameCount(),
          RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
          RC_LinkLostForSeconds(1u) ? "YES" : "NO",
          RC_GetRSSI(),
          rates.rc_hz,
          (unsigned long)RC_GetLastFrameMs());

    sendf("STORAGE ini=%s save_pending=%s last_try=%lums last_try_fail=%s last_ok=%lums\r\n",
          Settings_GetIniPath(),
          Settings_IsSavePending() ? "YES" : "NO",
          (unsigned long)Settings_GetLastSaveAttemptMs(),
          Settings_GetLastSaveAttemptFailed() ? "YES" : "NO",
          (unsigned long)Settings_GetLastSaveSuccessMs());
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
    RC_Diagnostics rcDiag;

    SensorDiag_GetRates(&rates);
    send("TEST name=active start=YES\r\n");

    imu_read_ok = IMU_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
    imu_plausible = IMU_CheckPlausibility();
    imu_pass = IMU_IsIdentityOK() && imu_read_ok && imu_plausible;
    sendf("TEST mod=IMU pass=%s id=0x%02X read=%s plaus=%s hal=%s hal_code=%d last=%lums rate=%uHz fail=%lu\r\n",
          imu_pass ? "YES" : "NO",
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
    sendf("TEST mod=BARO pass=%s id=0x%02X id_ok=%s calib=%s upd=%s press=%.2f temp=%.2f alt=%.2f vz=%.2f "
          "hal=%s hal_code=%d fail=%lu last=%lums rate=%uHz\r\n",
          baro_pass ? "YES" : "NO",
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
    sendf("TEST mod=GPS pass=%s fix=%s sats=%u hdop=%.1f fresh=%s last=%lums rate=%uHz sps=%u drop_delta=%lu ring=%u\r\n",
          gps_pass ? "YES" : "NO",
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
    sendf("TEST mod=SONAR pass=%s trig=%lu edges0=%lu edges1=%lu edges2=%lu timeouts=%lu d0=%.2f d1=%.2f d2=%.2f rate=%uHz\r\n",
          sonar_pass ? "YES" : "NO",
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
    sendf("TEST mod=INA pass=%s count=%u addr0=0x%02X pack=%.2f cell=%.2f curr=%.2f i2c_err=%lu adc_to=%lu "
          "hal_i2c=%s hal_i2c_code=%d hal_adc=%s hal_adc_code=%d last_v=%lums last_i=%lums rate=%uHz\r\n",
          batt_pass ? "YES" : "NO",
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
    RC_GetDiagnostics(&rcDiag);
    sendf("TEST mod=RC pass=%s mode=%s frame=%lu stale=%s lost=%s rssi=%u rate=%uHz last=%lums sync=%lu reject=%lu\r\n",
          rc_pass ? "YES" : "NO",
          rc_signal_str(rcDiag.signalType),
          (unsigned long)RC_GetFrameCount(),
          RC_ChannelsAreStale(ACTIVE_TEST_RC_FRESH_MS) ? "YES" : "NO",
          RC_LinkLostForSeconds(1u) ? "YES" : "NO",
          RC_GetRSSI(),
          rates.rc_hz,
          (unsigned long)RC_GetLastFrameMs(),
          (unsigned long)rcDiag.syncCount,
          (unsigned long)rcDiag.rejectCount);

    mag_read_ok = Compass_ReadRaw(&mx, &my, &mz);
    mag_maha_ok = mag_read_ok && Compass_CheckMahalanobis(mx, my, mz, mag_thresh);
    if (mag_read_ok) {
        mag_heading = Compass_ComputeHeading(mx, my, mz);
    }
    mag_pass = (!Settings_GetCompassEnabled()) || (mag_read_ok && mag_maha_ok);
    sendf("TEST mod=MAG pass=%s addr=0x%02X chip=0x%02X read=%s maha=%s maha_thr=%.2f "
          "x=%d y=%d z=%d heading=%.1f last=%lums ok=%lu fail=%lu hal=%s hal_code=%d rate=%uHz\r\n",
          mag_pass ? "YES" : "NO",
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
    send("TEST name=active done=YES\r\n");
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
        send("BUS status=START range=0x08-0x77 budget_ms=200\r\n");
    }
}

static void diagnostics_start_bus_scan(void)
{
    diagnostics_start_bus_scan_internal(true);
}

static void diagnostics_finish_bus_scan(void)
{
    uint8_t i;

    sendf("BUS status=COMPLETE hal=%s hal_code=%d found_count=%u found=",
          hal_status_str(busScan.lastStatus), (int)busScan.lastStatus, busScan.foundCount);
    if (busScan.foundCount == 0u) {
        send("none\r\n");
    } else {
        for (i = 0u; i < busScan.foundCount; i++) {
            sendf("0x%02X%s", busScan.found[i], (i + 1u < busScan.foundCount) ? "," : "");
        }
        send("\r\n");
    }

    if (busScanRepeat && !FlightState_IsArmed()) {
        busScan.active = false;
        busScanNextStartMs = HAL_GetTick() + (uint32_t)streamPeriodMs;
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
        send("ERR code=ARMED_DIAGNOSTICS_LOCKED\r\n");
        send("BUS status=ABORT reason=ARMED\r\n");
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

    sendf("STORAGE pass=%s mount_fr=%d io_fr=%d sd_detected=%u path=%s ini=%s save_pending=%s last_try=%lums last_ok=%lums\r\n",
          pass ? "YES" : "NO",
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
    send("INFO code=PWM_TEST disarmed_only=YES\r\n");
    sendf("TEST mod=PWM state=BANNER tim1=%d tim3=%d tim5=%d min=%u max=%u\r\n",
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
    sendf("TEST mod=PWM state=STOP reason=%s\r\n", (reason != NULL) ? reason : "done");
}

static void diagnostics_pwm_test_start(void)
{
    if (!require_disarmed("PWM Test")) {
        return;
    }
    if (pwmTest.active) {
        send("INFO code=PWM_TEST_ALREADY_ACTIVE\r\n");
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
    sendf("TEST mod=PWM state=START timeout_ms=%u\r\n", (unsigned int)PWM_TEST_DURATION_MS);
}

static void diagnostics_pwm_test_task(uint32_t now)
{
    uint16_t minUs = Settings_GetMotorPWMMinUs();
    uint16_t maxUs = Settings_GetMotorPWMMaxUs();

    if (!pwmTest.active) {
        return;
    }
    if (FlightState_IsArmed()) {
        send("ERR code=ARMED_DIAGNOSTICS_LOCKED\r\n");
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
    sendf("TEST mod=PWM state=RUN pwm_us=%u elapsed_ms=%lu\r\n",
          pwmTest.pwmUs,
          (unsigned long)(now - pwmTest.startMs));
}

static void maybe_print_deprecated(uint32_t bit, const char *use_hint)
{
    if ((deprecateMask & bit) != 0u) {
        return;
    }
    deprecateMask |= bit;
    sendf("WARN code=DEPRECATED use=%s\r\n", (use_hint != NULL) ? use_hint : "help");
}

static void show_sys_status(void)
{
    sendf("SYS state=0x%08lX ready=%s armed=%s health_all=%s loop=%uHz dt_last=%lums dt_max=%lums missed=%lu period_ms=%u\r\n",
          (unsigned long)FlightState_GetStateMask(),
          FlightState_IsReady() ? "YES" : "NO",
          FlightState_IsArmed() ? "YES" : "NO",
          FlightState_AllHealthOK() ? "YES" : "NO",
          CPU_GetLoopRateHz(),
          (unsigned long)CPU_GetLastLoopDtMs(),
          (unsigned long)CPU_GetMaxLoopDtMs(),
          (unsigned long)CPU_GetMissedDeadlineCount(),
          (unsigned int)streamPeriodMs);
}

static void show_health(void)
{
    uint32_t fs = FlightState_GetStateMask();
    sendf("SYS_HEALTH mask=0x%08lX imu=%s compass=%s baro=%s sonar=%s gps=%s rc=%s batt=%s ekf=%s\r\n",
          (unsigned long)fs,
          (fs & FS_HEALTH_IMU_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_COMPASS_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_BARO_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_SONAR_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_GPS_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_RC_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_BATT_OK_BIT) ? "YES" : "NO",
          (fs & FS_HEALTH_EKF_OK_BIT) ? "YES" : "NO");
}

static void show_rates(void)
{
    SensorRateReport rates;
    SensorDiag_GetRates(&rates);
    sendf("SYS_RATES imu=%u baro=%u gps=%u sonar=%u batt=%u rc=%u mag=%u loop=%u\r\n",
          rates.imu_hz,
          rates.baro_hz,
          rates.gps_hz,
          rates.sonar_hz,
          rates.battery_hz,
          rates.rc_hz,
          rates.mag_hz,
          CPU_GetLoopRateHz());
}

static void show_module(const char *module, bool verbose)
{
    if (module == NULL) {
        send("ERR code=SHOW_USAGE\r\n");
        return;
    }
    if (strcmp(module, "imu") == 0 || strcmp(module, "mpu") == 0) {
        show_mpu6050(verbose);
    } else if (strcmp(module, "baro") == 0 || strcmp(module, "bmp") == 0) {
        show_bmp_sensor(verbose);
    } else if (strcmp(module, "mag") == 0 || strcmp(module, "compass") == 0) {
        show_magnetometer(verbose);
    } else if (strcmp(module, "ina") == 0 || strcmp(module, "battery") == 0) {
        show_ina219(verbose);
    } else if (strcmp(module, "rc") == 0 || strcmp(module, "ppm") == 0) {
        show_ppm(verbose);
    } else if (strcmp(module, "gps") == 0) {
        show_gps(verbose);
    } else if (strcmp(module, "sonar") == 0) {
        show_sonar(verbose);
    } else if (strcmp(module, "osd") == 0) {
        show_osd(verbose);
    } else if (strcmp(module, "bt") == 0 || strcmp(module, "bluetooth") == 0) {
        show_bt(verbose);
    } else if (strcmp(module, "sys") == 0) {
        show_sys_status();
    } else {
        sendf("ERR code=SHOW_MODULE module=%s\r\n", module);
    }
}

static bool set_stream_state(const char *module, bool on)
{
    if (module == NULL) {
        return false;
    }
    if (strcmp(module, "imu") == 0 || strcmp(module, "mpu") == 0) {
        imuStream = on;
    } else if (strcmp(module, "baro") == 0 || strcmp(module, "bmp") == 0) {
        bmpStream = on;
    } else if (strcmp(module, "mag") == 0 || strcmp(module, "compass") == 0) {
        magStream = on;
    } else if (strcmp(module, "ina") == 0 || strcmp(module, "battery") == 0) {
        inaStream = on;
    } else if (strcmp(module, "osd") == 0) {
        osdStream = on;
    } else if (strcmp(module, "bt") == 0 || strcmp(module, "bluetooth") == 0) {
        btStream = on;
    } else if (strcmp(module, "rc") == 0 || strcmp(module, "ppm") == 0) {
        if (on) {
            actionMask |= DEBUG_MENU_PPM;
        } else {
            actionMask &= (uint8_t)(~DEBUG_MENU_PPM);
        }
    } else if (strcmp(module, "gps") == 0) {
        if (on) {
            actionMask |= DEBUG_MENU_GPS;
        } else {
            actionMask &= (uint8_t)(~DEBUG_MENU_GPS);
        }
    } else if (strcmp(module, "sonar") == 0) {
        if (on) {
            actionMask |= DEBUG_MENU_SONAR;
        } else {
            actionMask &= (uint8_t)(~DEBUG_MENU_SONAR);
        }
    } else if (strcmp(module, "all") == 0) {
        imuStream = on;
        bmpStream = on;
        magStream = on;
        inaStream = on;
        osdStream = on;
        btStream = on;
        if (on) {
            actionMask |= (DEBUG_MENU_PPM | DEBUG_MENU_GPS | DEBUG_MENU_SONAR);
        } else {
            actionMask &= (uint8_t)(~(DEBUG_MENU_PPM | DEBUG_MENU_GPS | DEBUG_MENU_SONAR));
        }
    } else {
        return false;
    }

    sendf("INFO code=STREAM module=%s state=%s period_ms=%u\r\n",
          module,
          on ? "ON" : "OFF",
          (unsigned int)streamPeriodMs);
    return true;
}

static bool handle_v2_command(const char *cmd)
{
    char tmp[64];
    char *tok1;
    char *tok2;
    char *tok3;
    char *tok4;
    uint32_t v;

    if (cmd == NULL) {
        return false;
    }

    (void)snprintf(tmp, sizeof(tmp), "%s", cmd);
    tok1 = strtok(tmp, " ");
    if (tok1 == NULL) {
        return true;
    }

    if (strcmp(tok1, "help") == 0) {
        tok2 = strtok(NULL, " ");
        if (tok2 == NULL) {
            show_menu();
        } else if (strcmp(tok2, "system") == 0) {
            send("SYSTEM HELP: help status health rates period stop\r\n");
        } else if (strcmp(tok2, "sensor") == 0) {
            send("SENSOR HELP: show <imu|baro|mag|ina|rc|gps|sonar|osd|bt|sys> [verbose] | stream <module|all> on|off\r\n");
        } else if (strcmp(tok2, "test") == 0) {
            show_diag_menu();
        } else if (strcmp(tok2, "io") == 0) {
            io_control_help();
        } else if (strcmp(tok2, "osd") == 0) {
            send("OSD HELP: osd show|init|on|off|cls|test\r\n");
        } else if (strcmp(tok2, "bt") == 0) {
            send("BT HELP: bt status|show | bt baud <rate> | bt test | bt gps | bt read [n] | bt on|off | show bt [verbose] | stream bt on|off\r\n");
        } else if (strcmp(tok2, "legacy") == 0) {
            send("LEGACY HELP: h d 1 2 3 4 5 p g o b v mpu bmp mag ina osd io ios ioi iobt iob* x\r\n");
        } else {
            sendf("ERR code=HELP_GROUP group=%s\r\n", tok2);
        }
        return true;
    }

    if (strcmp(tok1, "status") == 0) {
        show_sys_status();
        return true;
    }
    if (strcmp(tok1, "health") == 0) {
        show_health();
        return true;
    }
    if (strcmp(tok1, "rates") == 0) {
        show_rates();
        return true;
    }
    if (strcmp(tok1, "period") == 0) {
        tok2 = strtok(NULL, " ");
        if (tok2 == NULL) {
            sendf("INFO code=PERIOD value_ms=%u\r\n", (unsigned int)streamPeriodMs);
            return true;
        }
        v = (uint32_t)strtoul(tok2, NULL, 10);
        if (v < STREAM_PERIOD_MIN_MS || v > STREAM_PERIOD_MAX_MS) {
            sendf("ERR code=PERIOD_RANGE min=%u max=%u\r\n",
                  (unsigned int)STREAM_PERIOD_MIN_MS,
                  (unsigned int)STREAM_PERIOD_MAX_MS);
            return true;
        }
        streamPeriodMs = (uint16_t)v;
        sendf("INFO code=PERIOD value_ms=%u\r\n", (unsigned int)streamPeriodMs);
        return true;
    }
    if (strcmp(tok1, "stop") == 0) {
        stop_streaming_modes(true);
        return true;
    }

    if (strcmp(tok1, "show") == 0) {
        tok2 = strtok(NULL, " ");
        tok3 = strtok(NULL, " ");
        show_module(tok2, (tok3 != NULL && strcmp(tok3, "verbose") == 0));
        return true;
    }

    if (strcmp(tok1, "stream") == 0) {
        tok2 = strtok(NULL, " ");
        tok3 = strtok(NULL, " ");
        if (tok2 == NULL || tok3 == NULL) {
            send("ERR code=STREAM_USAGE\r\n");
            return true;
        }
        if (strcmp(tok3, "on") == 0) {
            if (block_while_armed(true)) {
                return true;
            }
            (void)set_stream_state(tok2, true);
        } else if (strcmp(tok3, "off") == 0) {
            (void)set_stream_state(tok2, false);
        } else {
            send("ERR code=STREAM_STATE\r\n");
        }
        return true;
    }

    if (strcmp(tok1, "test") == 0) {
        tok2 = strtok(NULL, " ");
        tok3 = strtok(NULL, " ");
        tok4 = strtok(NULL, " ");
        if (tok2 == NULL) {
            send("ERR code=TEST_USAGE\r\n");
            return true;
        }
        if (strcmp(tok2, "active") == 0) {
            if (tok3 != NULL && strcmp(tok3, "once") == 0) {
                if (!block_while_armed(true)) {
                    diagnostics_active_sensor_test();
                }
            } else if (tok3 != NULL && strcmp(tok3, "loop") == 0 && tok4 != NULL) {
                if (strcmp(tok4, "on") == 0) {
                    if (!block_while_armed(true)) {
                        actionMask |= DEBUG_MENU_ACTIVE_TEST;
                        send("INFO code=TEST_ACTIVE_LOOP state=ON\r\n");
                    }
                } else if (strcmp(tok4, "off") == 0) {
                    actionMask &= (uint8_t)(~DEBUG_MENU_ACTIVE_TEST);
                    send("INFO code=TEST_ACTIVE_LOOP state=OFF\r\n");
                } else {
                    send("ERR code=TEST_ACTIVE_LOOP_STATE\r\n");
                }
            } else {
                send("ERR code=TEST_ACTIVE_USAGE\r\n");
            }
            return true;
        }
        if (strcmp(tok2, "bus") == 0) {
            if (tok3 != NULL && strcmp(tok3, "once") == 0) {
                if (!block_while_armed(true)) {
                    busScanRepeat = false;
                    diagnostics_start_bus_scan();
                }
            } else if (tok3 != NULL && strcmp(tok3, "loop") == 0 && tok4 != NULL) {
                if (strcmp(tok4, "on") == 0) {
                    if (!block_while_armed(true)) {
                        busScanRepeat = true;
                        diagnostics_start_bus_scan();
                        send("INFO code=TEST_BUS_LOOP state=ON\r\n");
                    }
                } else if (strcmp(tok4, "off") == 0) {
                    busScanRepeat = false;
                    send("INFO code=TEST_BUS_LOOP state=OFF\r\n");
                } else {
                    send("ERR code=TEST_BUS_LOOP_STATE\r\n");
                }
            } else {
                send("ERR code=TEST_BUS_USAGE\r\n");
            }
            return true;
        }
        if (strcmp(tok2, "storage") == 0 && tok3 != NULL && strcmp(tok3, "run") == 0) {
            if (!block_while_armed(true)) {
                diagnostics_storage_test();
            }
            return true;
        }
        if (strcmp(tok2, "pwm") == 0) {
            if (tok3 != NULL && strcmp(tok3, "start") == 0) {
                if (!block_while_armed(true)) {
                    diagnostics_pwm_test_start();
                }
            } else if (tok3 != NULL && strcmp(tok3, "stop") == 0) {
                diagnostics_pwm_test_stop("manual");
            } else {
                send("ERR code=TEST_PWM_USAGE\r\n");
            }
            return true;
        }
        sendf("ERR code=TEST_TARGET target=%s\r\n", tok2);
        return true;
    }

    if (strcmp(tok1, "buzzer") == 0) {
        tok2 = strtok(NULL, " ");
        if (tok2 == NULL) {
            send("ERR code=BUZZER_USAGE\r\n");
            return true;
        }
        if (strcmp(tok2, "on") == 0) {
            if (!block_while_armed(true)) {
                actionMask |= DEBUG_MENU_BUZZER;
                Buzzer_PlayTone(TONE_READY);
                send("INFO code=BUZZER state=ON\r\n");
            }
        } else if (strcmp(tok2, "off") == 0) {
            actionMask &= (uint8_t)(~DEBUG_MENU_BUZZER);
            Buzzer_Stop();
            send("INFO code=BUZZER state=OFF\r\n");
        } else if (strcmp(tok2, "beep") == 0) {
            if (!block_while_armed(true)) {
                Buzzer_PlayTone(TONE_READY);
                send("INFO code=BUZZER state=BEEP\r\n");
            }
        } else {
            send("ERR code=BUZZER_USAGE\r\n");
        }
        return true;
    }

    if (strcmp(tok1, "servo") == 0) {
        tok2 = strtok(NULL, " ");
        tok3 = strtok(NULL, " ");
        if (tok2 == NULL || tok3 == NULL ||
            (strcmp(tok2, "debug") != 0 && strcmp(tok2, "test") != 0)) {
            send("ERR code=SERVO_USAGE\r\n");
            return true;
        }
        if (strcmp(tok3, "on") == 0) {
            if (!block_while_armed(true)) {
                actionMask |= DEBUG_MENU_SERVOS;
                send("INFO code=SERVO_TEST state=ON seq=ALL4_SWEEP_4S_THEN_GIMBAL_RC roll=PA0_PA1 pitch=PA2 yaw=PA3\r\n");
            }
        } else if (strcmp(tok3, "off") == 0) {
            actionMask &= (uint8_t)(~DEBUG_MENU_SERVOS);
            send("INFO code=SERVO_TEST state=OFF\r\n");
        } else {
            send("ERR code=SERVO_USAGE\r\n");
        }
        return true;
    }

    if (strcmp(tok1, "bt") == 0 || strcmp(tok1, "bluetooth") == 0) {
        tok2 = strtok(NULL, " ");
        tok3 = strtok(NULL, " ");
        if (tok2 == NULL || strcmp(tok2, "status") == 0 || strcmp(tok2, "show") == 0) {
            show_bt(true);
            return true;
        }
        if (strcmp(tok2, "baud") == 0) {
            uint32_t baud;
            bool okBaud;
            if (tok3 == NULL) {
                send("ERR code=BT_BAUD_USAGE\r\n");
                return true;
            }
            baud = (uint32_t)strtoul(tok3, NULL, 10);
            if (baud < 1200u || baud > 921600u) {
                sendf("ERR code=BT_BAUD_RANGE value=%lu\r\n", (unsigned long)baud);
                return true;
            }
            okBaud = BTLink_SetBaud(baud);
            sendf("INFO code=BT_BAUD value=%lu ok=%s\r\n",
                  (unsigned long)baud,
                  okBaud ? "YES" : "NO");
            show_bt(true);
            return true;
        }
        if (strcmp(tok2, "test") == 0) {
            char line[96];
            bool ok;
            btTestSeq++;
            (void)snprintf(line, sizeof(line), "BTTEST seq=%lu ms=%lu",
                           (unsigned long)btTestSeq,
                           (unsigned long)HAL_GetTick());
            ok = Telemetry_SendRawLine(line);
            sendf("BT tx=%s type=TEST seq=%lu len=%u\r\n",
                  ok ? "YES" : "NO",
                  (unsigned long)btTestSeq,
                  (unsigned int)BTLink_GetLastTxLen());
            return true;
        }
        if (strcmp(tok2, "gps") == 0) {
            char line[160];
            bool ok;
            (void)snprintf(line, sizeof(line),
                           "GPS fix=%s sats=%u hdop=%.1f lat=%.6f lon=%.6f alt=%.1f",
                           GPS_HasFix() ? "YES" : "NO",
                           GPS_GetSatCount(),
                           GPS_GetHDOP(),
                           GPS_GetLatitude(),
                           GPS_GetLongitude(),
                           GPS_GetAltitude());
            ok = Telemetry_SendRawLine(line);
            sendf("BT tx=%s type=GPS len=%u\r\n",
                  ok ? "YES" : "NO",
                  (unsigned int)BTLink_GetLastTxLen());
            return true;
        }
        if (strcmp(tok2, "read") == 0) {
            uint16_t req = 16u;
            if (tok3 != NULL) {
                req = (uint16_t)strtoul(tok3, NULL, 10);
            }
            bt_read_dump(req);
            return true;
        }
        if (strcmp(tok2, "on") == 0 || strcmp(tok2, "off") == 0) {
            if (!block_while_armed(true)) {
                io_control_handle_target_state("bt", tok2);
            }
            return true;
        }
        sendf("ERR code=BT_ACTION action=%s\r\n", tok2);
        return true;
    }

    if (strcmp(tok1, "io") == 0) {
        tok2 = strtok(NULL, " ");
        tok3 = strtok(NULL, " ");
        tok4 = strtok(NULL, " ");
        if (tok2 == NULL || strcmp(tok2, "status") == 0) {
            io_control_status();
            return true;
        }
        if (strcmp(tok2, "set") == 0 && tok3 != NULL && tok4 != NULL) {
            if (!block_while_armed(true)) {
                io_control_handle_target_state(tok3, tok4);
            }
            return true;
        }
        io_control_help();
        return true;
    }

    if (strcmp(tok1, "osd") == 0) {
        tok2 = strtok(NULL, " ");
        if (tok2 == NULL || strcmp(tok2, "show") == 0) {
            show_osd(true);
            return true;
        }
        if (strcmp(tok2, "init") == 0) {
            bool ok;
            if (block_while_armed(true)) {
                return true;
            }
            ok = AT7456_Reinit();
            sendf("OSD action=init ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
            return true;
        }
        if (strcmp(tok2, "on") == 0) {
            bool ok;
            if (block_while_armed(true)) {
                return true;
            }
            ok = AT7456_EnableOSD(true);
            sendf("OSD action=on ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
            return true;
        }
        if (strcmp(tok2, "off") == 0) {
            bool ok;
            if (block_while_armed(true)) {
                return true;
            }
            ok = AT7456_EnableOSD(false);
            sendf("OSD action=off ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
            return true;
        }
        if (strcmp(tok2, "cls") == 0) {
            bool ok;
            if (block_while_armed(true)) {
                return true;
            }
            ok = AT7456_ClearScreen();
            sendf("OSD action=cls ok=%s\r\n", ok ? "YES" : "NO");
            return true;
        }
        if (strcmp(tok2, "test") == 0) {
            bool ok;
            if (block_while_armed(true)) {
                return true;
            }
            ok = AT7456_EnableOSD(true) &&
                 AT7456_ClearScreen() &&
                 AT7456_WriteString(1u, 2u, "QW HAWK AT7456E") &&
                 AT7456_WriteString(3u, 2u, "OSD TEST OK");
            sendf("OSD action=test ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
            return true;
        }
        sendf("ERR code=OSD_ACTION action=%s\r\n", tok2);
        return true;
    }

    return false;
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
    osdStream = false;
    btStream = false;
    streamPeriodMs = STREAM_PERIOD_DEFAULT_MS;
    deprecateMask = 0u;
    btTestSeq = 0u;
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
            send("ERR code=ARMED_DIAGNOSTICS_LOCKED\r\n");
            actionMask &= (uint8_t)(~DEBUG_MENU_ACTIVE_TEST);
        } else {
            diagnostics_active_sensor_test();
        }
    }
    if (actionMask & DEBUG_MENU_SENSORS) {
        diagnostics_quick_snapshot();
    }
    if (actionMask & DEBUG_MENU_PPM) {
        show_ppm(false);
    }
    if (imuStream) {
        show_mpu6050(false);
    }
    if (bmpStream) {
        show_bmp_sensor(false);
    }
    if (magStream) {
        show_magnetometer(false);
    }
    if (inaStream) {
        show_ina219(false);
    }
    if (osdStream) {
        show_osd(false);
    }
    if (btStream) {
        show_bt(false);
    }
    if (actionMask & DEBUG_MENU_SONAR) {
        show_sonar(false);
    }
    if (actionMask & DEBUG_MENU_GPS) {
        show_gps(false);
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
    bool current;
    bool targetOn;
    bool ok;

    cmdBuf[cmdIdx] = '\0';

    /*
     * Command Map (legacy -> v2)
     *  h/? -> help
     *  1/s -> status + snapshot block
     *  2/2 once -> test active loop on/off | test active once
     *  3/3 once -> test bus loop on/off | test bus once
     *  4 -> test storage run
     *  5/5 run/5 stop -> test pwm banner/start/stop
     *  p/g/o/imu/bmp/mag/ina/osd + once -> stream/show equivalents
     *  b -> buzzer on/off
     *  v -> servo test on/off
     *  io/ios/io* and x aliases -> io status/io set
     */
    if (handle_v2_command(cmdBuf)) {
        send("> ");
        return;
    }

    if (strcmp(cmdBuf, "h") == 0 || strcmp(cmdBuf, "?") == 0) {
        show_help();
    } else if (strcmp(cmdBuf, "d") == 0) {
        maybe_print_deprecated(DEPR_TEST_ACTIVE_BIT, "help_test");
        show_diag_menu();
    } else if (strcmp(cmdBuf, "1") == 0 || strcmp(cmdBuf, "s") == 0) {
        maybe_print_deprecated(DEPR_TEST_ACTIVE_BIT, "status");
        show_sys_status();
        diagnostics_quick_snapshot();
    } else if (strcmp(cmdBuf, "2") == 0) {
        maybe_print_deprecated(DEPR_TEST_ACTIVE_BIT, "test_active_loop_on");
        current = ((actionMask & DEBUG_MENU_ACTIVE_TEST) != 0u);
        if (current) {
            actionMask &= (uint8_t)(~DEBUG_MENU_ACTIVE_TEST);
            send("INFO code=TEST_ACTIVE_LOOP state=OFF\r\n");
        } else if (!diagnostics_lock_for_active_tests()) {
            actionMask |= DEBUG_MENU_ACTIVE_TEST;
            send("INFO code=TEST_ACTIVE_LOOP state=ON\r\n");
        }
    } else if (strcmp(cmdBuf, "2 once") == 0) {
        maybe_print_deprecated(DEPR_TEST_ACTIVE_BIT, "test_active_once");
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_active_sensor_test();
        }
    } else if (strcmp(cmdBuf, "3") == 0) {
        maybe_print_deprecated(DEPR_TEST_BUS_BIT, "test_bus_loop_on");
        if (busScanRepeat) {
            busScanRepeat = false;
            send("INFO code=TEST_BUS_LOOP state=OFF\r\n");
        } else if (!diagnostics_lock_for_active_tests()) {
            busScanRepeat = true;
            diagnostics_start_bus_scan();
            send("INFO code=TEST_BUS_LOOP state=ON\r\n");
        }
    } else if (strcmp(cmdBuf, "3 once") == 0) {
        maybe_print_deprecated(DEPR_TEST_BUS_BIT, "test_bus_once");
        if (!diagnostics_lock_for_active_tests()) {
            busScanRepeat = false;
            diagnostics_start_bus_scan();
        }
    } else if (strcmp(cmdBuf, "4") == 0) {
        maybe_print_deprecated(DEPR_TEST_STORAGE_BIT, "test_storage_run");
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_storage_test();
        }
    } else if (strcmp(cmdBuf, "5") == 0) {
        maybe_print_deprecated(DEPR_TEST_PWM_BIT, "test_pwm_start");
        diagnostics_pwm_test_banner();
    } else if (strcmp(cmdBuf, "5 run") == 0) {
        maybe_print_deprecated(DEPR_TEST_PWM_BIT, "test_pwm_start");
        if (!diagnostics_lock_for_active_tests()) {
            diagnostics_pwm_test_start();
        }
    } else if (strcmp(cmdBuf, "5 stop") == 0) {
        maybe_print_deprecated(DEPR_TEST_PWM_BIT, "test_pwm_stop");
        diagnostics_pwm_test_stop("manual");
    } else if (strcmp(cmdBuf, "mpu") == 0 || strcmp(cmdBuf, "imu") == 0) {
        maybe_print_deprecated(DEPR_STREAM_IMU_BIT, "stream_imu_on");
        targetOn = !imuStream;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("imu", targetOn);
        }
    } else if (strcmp(cmdBuf, "mpu once") == 0 || strcmp(cmdBuf, "imu once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_IMU_BIT, "show_imu");
        show_mpu6050(true);
    } else if (strcmp(cmdBuf, "bmp") == 0 || strcmp(cmdBuf, "baro") == 0) {
        maybe_print_deprecated(DEPR_STREAM_BARO_BIT, "stream_baro_on");
        targetOn = !bmpStream;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("baro", targetOn);
        }
    } else if (strcmp(cmdBuf, "bmp once") == 0 || strcmp(cmdBuf, "baro once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_BARO_BIT, "show_baro");
        show_bmp_sensor(true);
    } else if (strcmp(cmdBuf, "mag") == 0 || strcmp(cmdBuf, "compass") == 0) {
        maybe_print_deprecated(DEPR_STREAM_MAG_BIT, "stream_mag_on");
        targetOn = !magStream;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("mag", targetOn);
        }
    } else if (strcmp(cmdBuf, "mag once") == 0 || strcmp(cmdBuf, "compass once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_MAG_BIT, "show_mag");
        show_magnetometer(true);
    } else if (strcmp(cmdBuf, "ina") == 0 || strcmp(cmdBuf, "battery") == 0) {
        maybe_print_deprecated(DEPR_STREAM_INA_BIT, "stream_ina_on");
        targetOn = !inaStream;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("ina", targetOn);
        }
    } else if (strcmp(cmdBuf, "ina once") == 0 || strcmp(cmdBuf, "battery once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_INA_BIT, "show_ina");
        show_ina219(true);
    } else if (strcmp(cmdBuf, "osd") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "stream_osd_on");
        targetOn = !osdStream;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("osd", targetOn);
        }
    } else if (strcmp(cmdBuf, "osd once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "show_osd");
        show_osd(true);
    } else if (strcmp(cmdBuf, "osd init") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "osd_init");
        if (!block_while_armed(true)) {
            ok = AT7456_Reinit();
            sendf("OSD action=init ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
        }
    } else if (strcmp(cmdBuf, "osd on") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "osd_on");
        if (!block_while_armed(true)) {
            ok = AT7456_EnableOSD(true);
            sendf("OSD action=on ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
        }
    } else if (strcmp(cmdBuf, "osd off") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "osd_off");
        if (!block_while_armed(true)) {
            ok = AT7456_EnableOSD(false);
            sendf("OSD action=off ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
        }
    } else if (strcmp(cmdBuf, "osd cls") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "osd_cls");
        if (!block_while_armed(true)) {
            ok = AT7456_ClearScreen();
            sendf("OSD action=cls ok=%s\r\n", ok ? "YES" : "NO");
        }
    } else if (strcmp(cmdBuf, "osd test") == 0) {
        maybe_print_deprecated(DEPR_STREAM_OSD_BIT, "osd_test");
        if (!block_while_armed(true)) {
            ok = AT7456_EnableOSD(true) &&
                 AT7456_ClearScreen() &&
                 AT7456_WriteString(1u, 2u, "QW HAWK AT7456E") &&
                 AT7456_WriteString(3u, 2u, "OSD TEST OK");
            sendf("OSD action=test ok=%s\r\n", ok ? "YES" : "NO");
            show_osd(true);
        }
    } else if (strcmp(cmdBuf, "g") == 0) {
        maybe_print_deprecated(DEPR_STREAM_GPS_BIT, "stream_gps_on");
        current = ((actionMask & DEBUG_MENU_GPS) != 0u);
        targetOn = !current;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("gps", targetOn);
        }
    } else if (strcmp(cmdBuf, "g once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_GPS_BIT, "show_gps");
        show_gps(true);
    } else if (strcmp(cmdBuf, "o") == 0) {
        maybe_print_deprecated(DEPR_STREAM_SONAR_BIT, "stream_sonar_on");
        current = ((actionMask & DEBUG_MENU_SONAR) != 0u);
        targetOn = !current;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("sonar", targetOn);
        }
    } else if (strcmp(cmdBuf, "o once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_SONAR_BIT, "show_sonar");
        show_sonar(true);
    } else if (strcmp(cmdBuf, "p") == 0) {
        maybe_print_deprecated(DEPR_STREAM_RC_BIT, "stream_rc_on");
        current = ((actionMask & DEBUG_MENU_PPM) != 0u);
        targetOn = !current;
        if (!targetOn || !block_while_armed(true)) {
            (void)set_stream_state("rc", targetOn);
        }
    } else if (strcmp(cmdBuf, "p once") == 0) {
        maybe_print_deprecated(DEPR_STREAM_RC_BIT, "show_rc");
        show_ppm(true);
    } else if (strcmp(cmdBuf, "b") == 0) {
        maybe_print_deprecated(DEPR_BUZZER_BIT, "buzzer_on");
        current = ((actionMask & DEBUG_MENU_BUZZER) != 0u);
        if (current) {
            actionMask &= (uint8_t)(~DEBUG_MENU_BUZZER);
            Buzzer_Stop();
            send("INFO code=BUZZER state=OFF\r\n");
        } else if (!block_while_armed(true)) {
            actionMask |= DEBUG_MENU_BUZZER;
            Buzzer_PlayTone(TONE_READY);
            send("INFO code=BUZZER state=ON\r\n");
        }
    } else if (strcmp(cmdBuf, "v") == 0) {
        maybe_print_deprecated(DEPR_SERVO_BIT, "servo_debug_on");
        current = ((actionMask & DEBUG_MENU_SERVOS) != 0u);
        if (current) {
            actionMask &= (uint8_t)(~DEBUG_MENU_SERVOS);
            send("INFO code=SERVO_TEST state=OFF\r\n");
        } else if (!block_while_armed(true)) {
            actionMask |= DEBUG_MENU_SERVOS;
            send("INFO code=SERVO_TEST state=ON seq=ALL4_SWEEP_4S_THEN_GIMBAL_RC roll=PA0_PA1 pitch=PA2 yaw=PA3\r\n");
        }
    } else if (strcmp(cmdBuf, "io") == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_status");
        io_control_help();
        io_control_status();
    } else if (strcmp(cmdBuf, "ios") == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_status");
        io_control_status();
    } else if (strncmp(cmdBuf, "ioi ", 4) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_imu_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("imu", cmdBuf + 4);
        }
    } else if (strncmp(cmdBuf, "iobt ", 5) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_bt_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("bt", cmdBuf + 5);
        }
    } else if (strncmp(cmdBuf, "iobmpi2c ", 9) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_bmp_i2c_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("bmp_i2c", cmdBuf + 9);
        }
    } else if (strncmp(cmdBuf, "iobmppwr ", 9) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_bmp_pwr_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("bmp_pwr", cmdBuf + 9);
        }
    } else if (strncmp(cmdBuf, "ioqmcpwr ", 9) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_qmc_pwr_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("qmc_pwr", cmdBuf + 9);
        }
    } else if (strncmp(cmdBuf, "iob0 ", 5) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_buck0_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("buck0", cmdBuf + 5);
        }
    } else if (strncmp(cmdBuf, "iob1 ", 5) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_buck1_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("buck1", cmdBuf + 5);
        }
    } else if (strncmp(cmdBuf, "iob2 ", 5) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_buck2_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("buck2", cmdBuf + 5);
        }
    } else if (strncmp(cmdBuf, "iob3 ", 5) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_buck3_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("buck3", cmdBuf + 5);
        }
    } else if (strncmp(cmdBuf, "ioba ", 5) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set_buck_all_on");
        if (!block_while_armed(true)) {
            io_control_handle_target_state("buck_all", cmdBuf + 5);
        }
    } else if (strcmp(cmdBuf, "x") == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_status");
        io_control_help();
        io_control_status();
    } else if (strncmp(cmdBuf, "x ", 2) == 0) {
        maybe_print_deprecated(DEPR_IO_BIT, "io_set");
        io_control_handle(cmdBuf + 2);
    } else {
        sendf("ERR code=UNKNOWN_CMD cmd=%s\r\n", cmdBuf);
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

    if ((now - lastBeat) >= (uint32_t)streamPeriodMs) {
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
