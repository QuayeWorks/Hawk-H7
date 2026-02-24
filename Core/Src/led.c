/*
 * led.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "led.h"

#include <stdio.h>

// You may need to include HAL GPIO if you control actual pins:
// #include "stm32h7xx_hal.h"

void LED_UpdateStatus(uint32_t stateMask)
{
    /*
     * This project does not provide concrete GPIO definitions for the on board
     * LEDs.  To keep the function useful without tying it to a specific board
     * layout, we simply print the new LED state whenever it changes.  The
     * application can later replace this implementation with real GPIO toggling
     * without changing the public API.
     */

    static uint32_t lastMask = 0xffffffffu;
    if (stateMask == lastMask) {
        return;
    }
    lastMask = stateMask;

    /* Example mapping (replace with real GPIO writes in the future):
     *   - If all health bits are set, the board is "ready" and we would turn on
     *     a green LED.
     *   - If any health bit is missing, indicate an error (red LED).
     *   - If armed, blink the LED quickly.
     */

    printf("LED state updated: 0x%08lX\n", (unsigned long)stateMask);
}
