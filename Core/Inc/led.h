/*
 * led.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>

/**
 * @brief  Update on‐board LEDs to reflect the given health mask.
 * @param  stateMask  Bitmask of FlightState health/error bits.
 */
void LED_UpdateStatus(uint32_t stateMask);

#endif // LED_H
