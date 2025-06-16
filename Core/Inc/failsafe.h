/*
 * failsafe.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdbool.h>

/**
 * @brief  Returns true if the battery over‐current protection has tripped.
 * @note   Stub: always returns false until you wire up a real sensor.
 */
bool Battery_OverCurrent(void);

/**
 * @brief  Returns true if the MCU temperature is above safe limits.
 * @note   Stub: always returns false until you add a temperature sensor.
 */
bool MCU_OverTemp(void);

/**
 * @brief  Returns true if any ESC has overheated.
 * @note   Stub: always returns false until you add ESC temperature monitoring.
 */
bool ESC_OverTemp(void);

/**
 * @brief  Trigger an automatic Return-To-Home (RTL) sequence.
 * @note   Stub implementation; replace with real RTL logic when ready.
 */
void CommenceReturnToHome(void);

#endif // FAILSAFE_H
