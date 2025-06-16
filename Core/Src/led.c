/*
 * led.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "led.h"

// You may need to include HAL GPIO if you control actual pins:
// #include "stm32h7xx_hal.h"

void LED_UpdateStatus(uint32_t stateMask)
{
    // TODO: implement your LED logic:
    //   e.g. turn on a green LED if ready, red if any error bit set, blink for GPS/RC loss, etc.
    (void)stateMask;  // placeholder to avoid unused‐param warning
}
