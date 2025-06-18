/*
 * debug_menu.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#include "debug_menu.h"
#include "imu.h"
#include "compass.h"
#include "baro.h"
#include "battery.h"
#include "gps.h"
#include "sonar.h"
#include "rc_input.h"
#include "buzzer.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *dbgUart;
static char cmdBuf[32];
static uint8_t cmdIdx;
static uint32_t lastBeat;
static volatile uint8_t rxByte;
static volatile uint8_t rxPending;
static volatile uint8_t actionMask;

// Forward GPS byte handler so we can chain callbacks
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

static void process_char(uint8_t ch);
static void handle_cmd(void);
static void process_actions(void);

static void send(const char *s)
{
    HAL_UART_Transmit(dbgUart, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

static void show_menu(void)
{
    send("\r\n--- Debug Menu ---\r\n");
    send("h: help\r\ns: sensors\r\np: ppm\r\no: sonar\r\ng: gps\r\nb: buzzer test\r\n> ");
}

void DebugMenu_Init(UART_HandleTypeDef *huart)
{
    dbgUart = huart;
    cmdIdx = 0;
    show_menu();
    HAL_UART_Receive_IT(dbgUart, (uint8_t *)&rxByte, 1);
    rxPending = 0;
    actionMask = DEBUG_MENU_NONE;
}

static void show_help(void)
{
    send("Commands:\r\n");
    send("  s  show sensor data\r\n");
    send("  p  show PPM channels\r\n");
    send("  o  show sonar distances\r\n");
    send("  g  show GPS info\r\n");
    send("  b  play error tones\r\n");
    send("  h  this help\r\n");
}

static void show_sensors(void)
{
    /* Fetch fresh data from each sensor so the output reflects the
       current state rather than whatever values happened to be read
       during initialization.  If a read fails we still print the
       previous values but append an error notice so the user knows
       that something went wrong. */

    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    bool imu_ok = IMU_ReadRaw(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    IMU_GetAccelMps2(&ax, &ay, &az);
    IMU_GetGyroDPS(&gx, &gy, &gz);

    float pressure = 0.0f;
    bool baro_ok = Baro_ReadPressure(&pressure);
    float alt = Baro_ComputeAltitude(pressure);

    int16_t mx = 0, my = 0, mz = 0;
    bool mag_ok = Compass_ReadRaw(&mx, &my, &mz);

    float v = Battery_ReadPackVoltage();
    float i = Battery_ReadCurrent();

    char buf[160];
    int len = snprintf(buf, sizeof(buf),
                       "ACC:%.2f %.2f %.2f GYR:%.2f %.2f %.2f%s\r\n"
                       "MAG:%d %d %d%s BARO:%.2f hPa %.2f m%s\r\n"
                       "BAT:V=%.2f I=%.2f\r\n",
                       ax, ay, az, gx, gy, gz,
                       imu_ok ? "" : " [ERR]",
                       mx, my, mz,
                       mag_ok ? "" : " [ERR]",
                       pressure, alt,
                       baro_ok ? "" : " [ERR]",
                       v, i);
    HAL_UART_Transmit(dbgUart, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

static const char *ppm_ch_names[RC_MAX_CHANNELS] = {
    "ROLL",
    "PITCH",
    "THROT",
    "YAW",
    "VBA",
    "VBB"
};

static void show_ppm(void)
{
    char buf[16];

    // Header
    send("PPM:\r\n");

    // Channel names
    for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        int n = snprintf(buf, sizeof(buf), "%6s", ppm_ch_names[i]);
        if (n > 0) {
            HAL_UART_Transmit(dbgUart, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
        }
    }
    send("\r\n");

    // Channel values
    for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        int n = snprintf(buf, sizeof(buf), "%6u", RC_GetChannel(i));
        if (n > 0) {
            HAL_UART_Transmit(dbgUart, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
        }
    }
    send("\r\n");
}

static void show_sonar(void)
{
    char buf[32];
    send("SONAR:");
    for(uint8_t i = 0; i < SONAR_COUNT; i++) {
        float d = Sonar_ReadDistance(i);
        int n = snprintf(buf, sizeof(buf), " %.2f", d);
        if (n > 0) {
            HAL_UART_Transmit(dbgUart, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
        }
    }
    send(" m\r\n");
}

static void show_gps(void)
{
    char buf[128];
    if (!GPS_HasFix()) {
        send("GPS: no fix\r\n");
        return;
    }
    int len = snprintf(buf, sizeof(buf),
                       "GPS: lat=%.6f lon=%.6f alt=%.1f sats=%u hdop=%.1f\r\n",
                       GPS_GetLatitude(), GPS_GetLongitude(), GPS_GetAltitude(),
                       GPS_GetSatCount(), GPS_GetHDOP());
    HAL_UART_Transmit(dbgUart, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

static void test_buzzer(void)
{
    for(Buzzer_ToneID t=TONE_ERROR_IMU; t<=TONE_ERROR_MOTOR; t++) {
        Buzzer_PlayTone(t);
        HAL_Delay(600);
    }
    Buzzer_Stop();
}

static void process_actions(void)
{
    if (actionMask & DEBUG_MENU_SENSORS) {
        show_sensors();
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
        test_buzzer();
    }
    if (actionMask & DEBUG_MENU_HELP) {
        show_help();
    }
}


static void process_char(uint8_t ch)
{
    if (ch == '\r' || ch == '\n') {
        if (cmdIdx > 0) {
            handle_cmd();
            cmdIdx = 0;
        } else {
            send("> ");
        }
    } else if (cmdIdx < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdIdx++] = (char)ch;
    }
}

static void handle_cmd(void)
{
    cmdBuf[cmdIdx] = '\0';
    if(strcmp(cmdBuf,"s") == 0) {
        show_sensors();
    } else if(strcmp(cmdBuf,"p") == 0) {
        show_ppm();
    } else if(strcmp(cmdBuf,"o") == 0) {
        show_sonar();
    } else if(strcmp(cmdBuf,"g") == 0) {
        show_gps();
    } else if(strcmp(cmdBuf,"b") == 0) {
        test_buzzer();
    } else {
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
    if (now - lastBeat >= 1000) {
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "[%lu ms]\r\n", (unsigned long)now);
        HAL_UART_Transmit(dbgUart, (uint8_t*)buf, len, HAL_MAX_DELAY);
        show_menu();
        process_actions();
        lastBeat = now;
    }

    if (rxPending || rxByte != 0) {
        uint8_t ch = rxByte;
        rxPending = 0;
        rxByte = 0;
        process_char(ch);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == dbgUart) {
        rxPending = 1;
        HAL_UART_Receive_IT(dbgUart, (uint8_t *)&rxByte, 1);
    }

    // Chain other modules that use UART receive interrupts
    GPS_UART_RxCpltCallback(huart);
}
