/*
 * failsafe.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "failsafe.h"

bool Battery_OverCurrent(void)
{
    // TODO: read INA219 current register or external sensor
    return false;
}

bool MCU_OverTemp(void)
{
    // TODO: read on-die temperature sensor (e.g. ADC channel)
    return false;
}

bool ESC_OverTemp(void)
{
    // TODO: read ESC telemetry or dedicated temp sensor
    return false;
}

void CommenceReturnToHome(void)
{
    // TODO: implement real return-to-home behavior
}
