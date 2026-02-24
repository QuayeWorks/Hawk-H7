#ifndef AT7456E_H
#define AT7456E_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

void AT7456_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
bool AT7456_Reinit(void);

bool AT7456_IsInitialized(void);
bool AT7456_IsPresent(void);
bool AT7456_IsEnabled(void);
bool AT7456_IsSPI8Bit(void);

uint8_t AT7456_ReadStatus(void);
HAL_StatusTypeDef AT7456_GetLastHalStatus(void);
uint32_t AT7456_GetTxCount(void);
uint32_t AT7456_GetFailCount(void);

bool AT7456_EnableOSD(bool enable);
bool AT7456_ClearScreen(void);
bool AT7456_WriteString(uint8_t row, uint8_t col, const char *text);

#endif // AT7456E_H
