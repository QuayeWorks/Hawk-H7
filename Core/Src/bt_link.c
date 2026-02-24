#include "bt_link.h"
#include "main.h"
#include <string.h>

#define BT_DEFAULT_BAUD                  9600u
#define BT_TX_GPIO_PORT                  GPIOE
#define BT_TX_GPIO_PIN                   GPIO_PIN_8

static UART_HandleTypeDef s_uart7;
static bool s_ready;
static uint32_t s_baud = BT_DEFAULT_BAUD;

static uint32_t s_txOkCount;
static uint32_t s_txFailCount;
static uint32_t s_lastTxMs;
static uint16_t s_lastTxLen;
static HAL_StatusTypeDef s_lastTxStatus = HAL_OK;
static uint32_t s_lastTxError;

static bool bt_uart7_tx_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clk = {0};

    clk.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    clk.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) {
        return false;
    }

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();

    gpio.Pin = BT_TX_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(BT_TX_GPIO_PORT, &gpio);

    memset((void *)&s_uart7, 0, sizeof(s_uart7));
    s_uart7.Instance = UART7;
    s_uart7.Init.BaudRate = baud;
    s_uart7.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart7.Init.StopBits = UART_STOPBITS_1;
    s_uart7.Init.Parity = UART_PARITY_NONE;
    s_uart7.Init.Mode = UART_MODE_TX;
    s_uart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_uart7.Init.OverSampling = UART_OVERSAMPLING_16;
    s_uart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_uart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    s_uart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&s_uart7) != HAL_OK) {
        return false;
    }
    (void)HAL_UARTEx_SetTxFifoThreshold(&s_uart7, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_DisableFifoMode(&s_uart7);
    return true;
}

bool BTLink_Init(TIM_HandleTypeDef *sampleTim, uint32_t baud)
{
    (void)sampleTim;
    s_baud = (baud > 0u) ? baud : BT_DEFAULT_BAUD;
    s_ready = false;

    s_txOkCount = 0u;
    s_txFailCount = 0u;
    s_lastTxMs = 0u;
    s_lastTxLen = 0u;
    s_lastTxStatus = HAL_OK;
    s_lastTxError = 0u;

    s_ready = bt_uart7_tx_init(s_baud);
    return s_ready;
}

bool BTLink_IsReady(void)
{
    return s_ready;
}

bool BTLink_SetBaud(uint32_t baud)
{
    if (baud == 0u) {
        return false;
    }
    s_baud = baud;
    s_ready = bt_uart7_tx_init(s_baud);
    return s_ready;
}

bool BTLink_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef st;

    if (!s_ready || data == NULL || len == 0u) {
        return false;
    }

    st = HAL_UART_Transmit(&s_uart7, (uint8_t *)data, len, timeout_ms);
    s_lastTxStatus = st;
    s_lastTxError = s_uart7.ErrorCode;
    s_lastTxLen = len;
    s_lastTxMs = HAL_GetTick();

    if (st == HAL_OK) {
        s_txOkCount++;
        return true;
    }

    s_txFailCount++;
    return false;
}

uint32_t BTLink_GetTxOkCount(void)
{
    return s_txOkCount;
}

uint32_t BTLink_GetTxFailCount(void)
{
    return s_txFailCount;
}

uint32_t BTLink_GetLastTxMs(void)
{
    return s_lastTxMs;
}

uint16_t BTLink_GetLastTxLen(void)
{
    return s_lastTxLen;
}

HAL_StatusTypeDef BTLink_GetLastTxStatus(void)
{
    return s_lastTxStatus;
}

uint32_t BTLink_GetLastTxError(void)
{
    return s_lastTxError;
}

uint32_t BTLink_GetBaud(void)
{
    return s_baud;
}

void BTLink_TimerTickISR(void)
{
    /* RX path intentionally disabled on old PE8/PE9 hardware. */
}

bool BTLink_ReadByte(uint8_t *out)
{
    (void)out;
    return false;
}

uint8_t BTLink_GetRxRingLevel(void)
{
    return 0u;
}

uint32_t BTLink_GetRxOkCount(void)
{
    return 0u;
}

uint32_t BTLink_GetRxFrameErrCount(void)
{
    return 0u;
}

uint32_t BTLink_GetRxDropCount(void)
{
    return 0u;
}

uint32_t BTLink_GetLastRxMs(void)
{
    return 0u;
}

uint32_t BTLink_GetTickCount(void)
{
    return 0u;
}

uint32_t BTLink_GetRxSampleEdgeCount(void)
{
    return 0u;
}

uint32_t BTLink_GetRxLowSampleCount(void)
{
    return 0u;
}

uint8_t BTLink_GetRxPinLevel(void)
{
    return 0u;
}
