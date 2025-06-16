/*
 * telemetry.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "telemetry.h"
// You may also need your UART handle here, e.g. extern UART_HandleTypeDef huartX;

void Telemetry_SendHealth(uint32_t stateMask)
{
    // TODO: format and send over your UART/MSC link
    (void)stateMask;
}

void Telemetry_SendAttitude(AttitudeAngles angles)
{
    // TODO: format (e.g. as CSV or JSON) and send roll/pitch/yaw
    (void)angles;
}
