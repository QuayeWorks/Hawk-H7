/*
 * debug_menu.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef DEBUG_MENU_H
#define DEBUG_MENU_H

#include "stm32h7xx_hal.h"

void DebugMenu_Init(UART_HandleTypeDef *huart);
void DebugMenu_Task(void);
void DebugMenu_ForceInput(uint8_t ch);

typedef enum {
    DEBUG_MENU_NONE    = 0x00,
    DEBUG_MENU_SENSORS = 0x01,
    DEBUG_MENU_PPM     = 0x02,
    DEBUG_MENU_BUZZER  = 0x04,
    DEBUG_MENU_HELP    = 0x08,
    DEBUG_MENU_SONAR   = 0x10,
    DEBUG_MENU_GPS     = 0x20,
    DEBUG_MENU_SERVOS  = 0x40,
    DEBUG_MENU_ACTIVE_TEST = 0x80
} DebugMenu_Mask;

void DebugMenu_SetActionMask(uint8_t mask);
uint8_t DebugMenu_GetActionMask(void);

#endif // DEBUG_MENU_H
