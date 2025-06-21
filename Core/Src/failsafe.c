/*
 * failsafe.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "failsafe.h"
#include "battery.h"
#include "settings.h"
#include "motor.h"
#include "flight_state.h"
#include "stm32h7xx_hal.h"

bool Battery_OverCurrent(void)
{
    float currentA = Battery_ReadCurrent();
    return currentA > (float)Settings_GetINA219MaxExpectedAmps();
}

bool MCU_OverTemp(void)
{
    /* This board does not provide an MCU temperature sensor yet. The
     * implementation is therefore a stub and simply reports no over-temp
     * condition.  Once a suitable ADC channel is wired up, replace this stub
     * with real temperature measurement logic.
     */
    return false;
}

bool ESC_OverTemp(void)
{
    /* In the absence of real ESC telemetry, this always returns false.  The
     * function is kept so the call sites remain valid and can be extended later
     * with board specific code. */
    return false;
}

void CommenceReturnToHome(void)
{
    /* Minimal RTL implementation: disarm and bring the motors to idle.  A real
     * system would navigate back to the recorded home coordinates. */
    Motor_SetAllPWM(Settings_GetMotorPWMMinUs());
    FlightState_Disarm();
}
