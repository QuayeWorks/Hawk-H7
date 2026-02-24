#ifndef BT_LINK_H
#define BT_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

bool BTLink_Init(TIM_HandleTypeDef *sampleTim, uint32_t baud);
bool BTLink_IsReady(void);
bool BTLink_SetBaud(uint32_t baud);

bool BTLink_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

uint32_t BTLink_GetTxOkCount(void);
uint32_t BTLink_GetTxFailCount(void);
uint32_t BTLink_GetLastTxMs(void);
uint16_t BTLink_GetLastTxLen(void);
HAL_StatusTypeDef BTLink_GetLastTxStatus(void);
uint32_t BTLink_GetLastTxError(void);
uint32_t BTLink_GetBaud(void);

void BTLink_TimerTickISR(void);
bool BTLink_ReadByte(uint8_t *out);
uint8_t BTLink_GetRxRingLevel(void);
uint32_t BTLink_GetRxOkCount(void);
uint32_t BTLink_GetRxFrameErrCount(void);
uint32_t BTLink_GetRxDropCount(void);
uint32_t BTLink_GetLastRxMs(void);
uint32_t BTLink_GetTickCount(void);
uint32_t BTLink_GetRxSampleEdgeCount(void);
uint32_t BTLink_GetRxLowSampleCount(void);
uint8_t  BTLink_GetRxPinLevel(void);

#endif /* BT_LINK_H */
