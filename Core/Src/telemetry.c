/*
 * telemetry.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "telemetry.h"
#include "stm32h7xx_hal.h"
// UART handle for telemetry output (defined in main.c or elsewhere)
extern UART_HandleTypeDef huart3;

#include <stdio.h>

void Telemetry_SendHealth(uint32_t stateMask)
{
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "HS:%08lX\r\n", (unsigned long)stateMask);
    // TODO: migrate telemetry to DMA/IT for fully non-blocking TX.
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 50);
}

void Telemetry_SendAttitude(AttitudeAngles angles)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "ATT:%.2f,%.2f,%.2f\r\n",
                       angles.roll, angles.pitch, angles.yaw);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 50);
}
