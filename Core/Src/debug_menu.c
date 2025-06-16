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

static void send(const char *s)
{
    HAL_UART_Transmit(dbgUart, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

void DebugMenu_Init(UART_HandleTypeDef *huart)
{
    dbgUart = huart;
    cmdIdx = 0;
    send("\r\n--- Debug Menu ---\r\n");
    send("h: help\r\ns: sensors\r\np: ppm\r\nb: buzzer test\r\n> ");
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

static void show_ppm(void)
{
    char buf[128];
    int len = snprintf(buf,sizeof(buf),"PPM:");
    for(uint8_t i=0;i<RC_MAX_CHANNELS;i++) {
        len += snprintf(&buf[len], sizeof(buf)-len, " %u", RC_GetChannel(i));
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

void DebugMenu_Task(void)
{
    uint32_t now = HAL_GetTick();
    if (now - lastBeat >= 1000) {
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "[%lu ms]\r\n> ", (unsigned long)now);
        HAL_UART_Transmit(dbgUart, (uint8_t*)buf, len, HAL_MAX_DELAY);
        lastBeat = now;
    }
    uint8_t ch;
    while(HAL_UART_Receive(dbgUart,&ch,1,0) == HAL_OK) {
        if(ch=='\r' || ch=='\n') {
            if(cmdIdx>0) {
                handle_cmd();
                cmdIdx=0;
            } else {
                send("> ");
            }
        } else if(cmdIdx < sizeof(cmdBuf)-1) {
            cmdBuf[cmdIdx++] = (char)ch;
        }
    }
}

