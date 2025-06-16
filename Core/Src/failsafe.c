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
    extern ADC_HandleTypeDef hadc1;  // ADC configured to read internal temp
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    const float vref   = 3.3f;
    float vsense       = ((float)raw * vref) / 4095.0f;
    float temperatureC = (vsense - 0.76f) / 0.0025f + 25.0f;
    return temperatureC > 80.0f;    // 80°C threshold
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
