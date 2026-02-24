/*
 * telemetry.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "telemetry.h"
#include "bt_link.h"
#include "main.h"
#include "stm32h7xx_hal.h"
// UART handle for telemetry output (defined in main.c or elsewhere)
extern UART_HandleTypeDef huart3;

#include <stdio.h>
#include <string.h>

#define TELEMETRY_UART_TX_TIMEOUT_MS 200u

static uint32_t txOkCount;
static uint32_t txFailCount;
static uint32_t lastTxMs;
static uint16_t lastTxLen;
static HAL_StatusTypeDef lastHalStatus = HAL_OK;
static uint32_t lastUartError;

static bool telemetry_uart_write(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st_u3 = HAL_ERROR;
    HAL_StatusTypeDef st_bt = HAL_ERROR;
    HAL_StatusTypeDef st;
    bool any_ok = false;

    if (data == NULL || len == 0u) {
        return false;
    }

#if BT_USE_OLD_BOARD_PE8_PE9
    if (BTLink_IsReady()) {
        st_bt = BTLink_Send(data, len, TELEMETRY_UART_TX_TIMEOUT_MS) ? HAL_OK : HAL_ERROR;
        if (st_bt == HAL_OK) {
            any_ok = true;
        }
    }
#endif

    st_u3 = HAL_UART_Transmit(&huart3, (uint8_t *)data, len, TELEMETRY_UART_TX_TIMEOUT_MS);
    if (st_u3 == HAL_OK) {
        any_ok = true;
    }

    st = any_ok ? HAL_OK : HAL_ERROR;
    lastHalStatus = st;
#if BT_USE_OLD_BOARD_PE8_PE9
    if (st_bt != HAL_OK && st_u3 != HAL_OK && BTLink_IsReady()) {
        lastUartError = BTLink_GetLastTxError();
    } else {
        lastUartError = huart3.ErrorCode;
    }
#else
    lastUartError = huart3.ErrorCode;
#endif
    lastTxLen = len;
    lastTxMs = HAL_GetTick();

    if (st == HAL_OK) {
        txOkCount++;
        return true;
    }

    txFailCount++;
    return false;
}

void Telemetry_SendHealth(uint32_t stateMask)
{
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "HS:%08lX\r\n", (unsigned long)stateMask);
    // TODO: migrate telemetry to DMA/IT for fully non-blocking TX.
    if (len > 0) {
        (void)telemetry_uart_write((const uint8_t *)buf, (uint16_t)len);
    }
}

void Telemetry_SendAttitude(AttitudeAngles angles)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "ATT:%.2f,%.2f,%.2f\r\n",
                       angles.roll, angles.pitch, angles.yaw);
    if (len > 0) {
        (void)telemetry_uart_write((const uint8_t *)buf, (uint16_t)len);
    }
}

bool Telemetry_SendRawLine(const char *line)
{
    size_t n;
    char buf[160];

    if (line == NULL || line[0] == '\0') {
        return false;
    }

    n = strlen(line);
    if (n >= sizeof(buf)) {
        n = sizeof(buf) - 1u;
    }
    memcpy(buf, line, n);
    if (n < 2u || !(buf[n - 2u] == '\r' && buf[n - 1u] == '\n')) {
        if (n + 2u >= sizeof(buf)) {
            return false;
        }
        buf[n++] = '\r';
        buf[n++] = '\n';
    }
    return telemetry_uart_write((const uint8_t *)buf, (uint16_t)n);
}

uint32_t Telemetry_GetTxOkCount(void)
{
    return txOkCount;
}

uint32_t Telemetry_GetTxFailCount(void)
{
    return txFailCount;
}

uint32_t Telemetry_GetLastTxMs(void)
{
    return lastTxMs;
}

uint16_t Telemetry_GetLastTxLen(void)
{
    return lastTxLen;
}

int32_t Telemetry_GetLastHalStatusCode(void)
{
    return (int32_t)lastHalStatus;
}

uint32_t Telemetry_GetLastUartError(void)
{
    return lastUartError;
}
