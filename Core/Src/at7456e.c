#include "at7456e.h"
#include <string.h>

#define AT7456_SPI_TIMEOUT_MS      5u
#define AT7456_RESET_DELAY_MS      50u
#define AT7456_CLR_TIMEOUT_MS      40u
#define AT7456_COLS                30u
#define AT7456_ROWS                16u

// Register map (MAX7456/AT7456E-compatible).
#define AT7456_REG_VM0             0x00u
#define AT7456_REG_DMM             0x04u
#define AT7456_REG_DMAH            0x05u
#define AT7456_REG_DMAL            0x06u
#define AT7456_REG_DMDI            0x07u
#define AT7456_REG_STAT_READ       0xA0u

// VM0 bits.
#define AT7456_VM0_SOFT_RESET      0x02u
#define AT7456_VM0_OSD_ENABLE      0x08u

// DMM bits.
#define AT7456_DMM_CLR_DISPLAY     0x04u

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    bool configured;
    bool present;
    bool enabled;
    uint8_t last_status;
    HAL_StatusTypeDef last_hal;
    uint32_t tx_count;
    uint32_t fail_count;
} At7456Ctx;

static At7456Ctx osd;

static void cs_high(void)
{
    if (osd.cs_port != NULL) {
        HAL_GPIO_WritePin(osd.cs_port, osd.cs_pin, GPIO_PIN_SET);
    }
}

static void cs_low(void)
{
    if (osd.cs_port != NULL) {
        HAL_GPIO_WritePin(osd.cs_port, osd.cs_pin, GPIO_PIN_RESET);
    }
}

static bool spi_txrx(uint8_t tx, uint8_t *rx)
{
    uint8_t r = 0u;

    if (osd.hspi == NULL) {
        osd.last_hal = HAL_ERROR;
        osd.fail_count++;
        return false;
    }

    osd.last_hal = HAL_SPI_TransmitReceive(osd.hspi, &tx, &r, 1u, AT7456_SPI_TIMEOUT_MS);
    if (osd.last_hal != HAL_OK) {
        osd.fail_count++;
        return false;
    }

    osd.tx_count++;
    if (rx != NULL) {
        *rx = r;
    }
    return true;
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    bool ok;
    uint8_t addr = (uint8_t)(reg & 0x7Fu);

    cs_low();
    ok = spi_txrx(addr, NULL) && spi_txrx(value, NULL);
    cs_high();
    return ok;
}

static bool read_reg(uint8_t reg, uint8_t *value)
{
    bool ok;
    uint8_t dummy = 0u;
    uint8_t addr = (uint8_t)(reg | 0x80u);

    if (value == NULL) {
        return false;
    }

    cs_low();
    ok = spi_txrx(addr, NULL) && spi_txrx(0x00u, &dummy);
    cs_high();
    if (!ok) {
        return false;
    }

    *value = dummy;
    return true;
}

static bool prepare_spi_8bit(void)
{
    if (osd.hspi == NULL) {
        osd.last_hal = HAL_ERROR;
        return false;
    }

    if (osd.hspi->Init.DataSize != SPI_DATASIZE_8BIT ||
        osd.hspi->Init.BaudRatePrescaler < SPI_BAUDRATEPRESCALER_16) {
        osd.hspi->Init.DataSize = SPI_DATASIZE_8BIT;
        osd.hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        osd.last_hal = HAL_SPI_Init(osd.hspi);
        if (osd.last_hal != HAL_OK) {
            osd.fail_count++;
            return false;
        }
    }

    return true;
}

static bool detect_device(void)
{
    uint8_t old_dmm = 0u;
    uint8_t test_dmm = 0u;
    uint8_t readback = 0u;

    if (!read_reg(AT7456_REG_DMM, &old_dmm)) {
        return false;
    }

    test_dmm = (uint8_t)(old_dmm ^ 0x01u);
    if (!write_reg(AT7456_REG_DMM, test_dmm)) {
        return false;
    }
    if (!read_reg(AT7456_REG_DMM, &readback)) {
        return false;
    }
    (void)write_reg(AT7456_REG_DMM, old_dmm);

    return (readback == test_dmm);
}

bool AT7456_Reinit(void)
{
    uint8_t stat = 0u;
    uint32_t t0;
    uint8_t dmm = 0u;

    if (!osd.configured) {
        osd.last_hal = HAL_ERROR;
        osd.present = false;
        return false;
    }

    cs_high();
    if (!prepare_spi_8bit()) {
        osd.present = false;
        return false;
    }

    if (!detect_device()) {
        osd.present = false;
        return false;
    }

    if (!write_reg(AT7456_REG_VM0, AT7456_VM0_SOFT_RESET)) {
        osd.present = false;
        return false;
    }
    HAL_Delay(AT7456_RESET_DELAY_MS);

    if (!write_reg(AT7456_REG_VM0, AT7456_VM0_OSD_ENABLE)) {
        osd.present = false;
        return false;
    }

    if (!write_reg(AT7456_REG_DMM, AT7456_DMM_CLR_DISPLAY)) {
        osd.present = false;
        return false;
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < AT7456_CLR_TIMEOUT_MS) {
        if (!read_reg(AT7456_REG_DMM, &dmm)) {
            osd.present = false;
            return false;
        }
        if ((dmm & AT7456_DMM_CLR_DISPLAY) == 0u) {
            break;
        }
        HAL_Delay(1);
    }

    if (!read_reg(AT7456_REG_STAT_READ, &stat)) {
        osd.present = false;
        return false;
    }

    osd.last_status = stat;
    osd.present = true;
    osd.enabled = true;
    return true;
}

void AT7456_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    memset(&osd, 0, sizeof(osd));
    osd.hspi = hspi;
    osd.cs_port = cs_port;
    osd.cs_pin = cs_pin;
    osd.configured = (hspi != NULL && cs_port != NULL);
    osd.last_hal = HAL_OK;
    cs_high();

    if (osd.configured) {
        (void)AT7456_Reinit();
    }
}

bool AT7456_IsInitialized(void)
{
    return osd.configured;
}

bool AT7456_IsPresent(void)
{
    return osd.present;
}

bool AT7456_IsEnabled(void)
{
    return osd.enabled;
}

bool AT7456_IsSPI8Bit(void)
{
    if (osd.hspi == NULL) {
        return false;
    }
    return (osd.hspi->Init.DataSize == SPI_DATASIZE_8BIT);
}

uint8_t AT7456_ReadStatus(void)
{
    uint8_t stat = 0u;

    if (read_reg(AT7456_REG_STAT_READ, &stat)) {
        osd.last_status = stat;
    } else {
        osd.present = false;
    }
    return osd.last_status;
}

HAL_StatusTypeDef AT7456_GetLastHalStatus(void)
{
    return osd.last_hal;
}

uint32_t AT7456_GetTxCount(void)
{
    return osd.tx_count;
}

uint32_t AT7456_GetFailCount(void)
{
    return osd.fail_count;
}

bool AT7456_EnableOSD(bool enable)
{
    uint8_t vm0 = 0u;

    if (!read_reg(AT7456_REG_VM0, &vm0)) {
        osd.present = false;
        return false;
    }

    if (enable) {
        vm0 |= AT7456_VM0_OSD_ENABLE;
    } else {
        vm0 &= (uint8_t)(~AT7456_VM0_OSD_ENABLE);
    }

    if (!write_reg(AT7456_REG_VM0, vm0)) {
        osd.present = false;
        return false;
    }

    osd.enabled = enable;
    return true;
}

bool AT7456_ClearScreen(void)
{
    uint32_t t0;
    uint8_t dmm = 0u;

    if (!write_reg(AT7456_REG_DMM, AT7456_DMM_CLR_DISPLAY)) {
        osd.present = false;
        return false;
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < AT7456_CLR_TIMEOUT_MS) {
        if (!read_reg(AT7456_REG_DMM, &dmm)) {
            osd.present = false;
            return false;
        }
        if ((dmm & AT7456_DMM_CLR_DISPLAY) == 0u) {
            return true;
        }
        HAL_Delay(1);
    }

    return false;
}

bool AT7456_WriteString(uint8_t row, uint8_t col, const char *text)
{
    uint16_t addr;
    const uint8_t *p = (const uint8_t *)text;

    if (text == NULL || row >= AT7456_ROWS || col >= AT7456_COLS) {
        return false;
    }

    addr = (uint16_t)((uint16_t)row * AT7456_COLS + (uint16_t)col);
    while (*p != '\0' && col < AT7456_COLS) {
        if (!write_reg(AT7456_REG_DMAH, (uint8_t)((addr >> 8) & 0x01u))) {
            osd.present = false;
            return false;
        }
        if (!write_reg(AT7456_REG_DMAL, (uint8_t)(addr & 0xFFu))) {
            osd.present = false;
            return false;
        }
        if (!write_reg(AT7456_REG_DMDI, *p)) {
            osd.present = false;
            return false;
        }
        addr++;
        col++;
        p++;
    }

    return true;
}
