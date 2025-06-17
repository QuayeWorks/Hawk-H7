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
    send("h: help\r\ns: sensors\r\np: ppm\r\nb: buzzer test\r\n> ");
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
    send("  b  play error tones\r\n");
    send("  h  this help\r\n");
}

static void show_sensors(void)
{
    float ax,ay,az,gx,gy,gz;
    IMU_GetAccelMps2(&ax,&ay,&az);
    IMU_GetGyroDPS(&gx,&gy,&gz);
    float pressure=0.0f;
    Baro_ReadPressure(&pressure);
    float alt = Baro_ComputeAltitude(pressure);
    int16_t mx,my,mz;
    Compass_ReadRaw(&mx,&my,&mz);
    float v = Battery_ReadPackVoltage();
    float i = Battery_ReadCurrent();
    char buf[128];
    int len = snprintf(buf,sizeof(buf),
                       "ACC:%.2f %.2f %.2f GYR:%.2f %.2f %.2f\r\nMAG:%d %d %d BARO:%.2f hPa %.2f m\r\nBAT:V=%.2f I=%.2f\r\n",
                       ax,ay,az,gx,gy,gz,mx,my,mz,pressure,alt,v,i);
    HAL_UART_Transmit(dbgUart,(uint8_t*)buf,len,HAL_MAX_DELAY);
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
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "PPM:\r\n");

    // Print channel names
    for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        len += snprintf(&buf[len], sizeof(buf)-len, "%6s", ppm_ch_names[i]);
    }
    len += snprintf(&buf[len], sizeof(buf)-len, "\r\n");

    // Print channel values
    for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        len += snprintf(&buf[len], sizeof(buf)-len, "%6u", RC_GetChannel(i));
    }
    len += snprintf(&buf[len], sizeof(buf)-len, "\r\n");

    HAL_UART_Transmit(dbgUart,(uint8_t*)buf,len,HAL_MAX_DELAY);
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
