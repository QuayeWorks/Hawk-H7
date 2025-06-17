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

#endif // DEBUG_MENU_H
