/*
 * bmp388.c - BMP388 barometer driver with compensation
 */

#include "bmp388.h"
#include "stm32h7xx_hal.h"
#include <string.h>

// I2C handle provided by application
extern I2C_HandleTypeDef hi2c1;

static uint8_t        bmp_addr = BMP388_I2C_ADDR1;
static BMP388_CalibData calib;

// Low-level helpers --------------------------------------------------------
static bool bmp388_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, bmp_addr, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    if (HAL_I2C_Master_Receive(&hi2c1, bmp_addr, buf, len, HAL_MAX_DELAY) != HAL_OK)
        return false;
    return true;
}

static bool bmp388_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return HAL_I2C_Master_Transmit(&hi2c1, bmp_addr, b, 2, HAL_MAX_DELAY) == HAL_OK;
}

// -------------------------------------------------------------------------

bool BMP388_ReadCalibData(BMP388_CalibData *cdata)
{
    uint8_t raw[21];
    if (!bmp388_read_regs(0x31, raw, 21))
        return false;

    cdata->par_t1  = (uint16_t)((raw[1] << 8) | raw[0]);
    cdata->par_t2  = (int16_t)((raw[3] << 8) | raw[2]);
    cdata->par_t3  = (int8_t)raw[4];
    cdata->par_p1  = (uint16_t)((raw[6] << 8) | raw[5]);
    cdata->par_p2  = (int16_t)((raw[8] << 8) | raw[7]);
    cdata->par_p3  = (int8_t)raw[9];
    cdata->par_p4  = (int8_t)raw[10];
    cdata->par_p5  = (uint16_t)((raw[12] << 8) | raw[11]);
    cdata->par_p6  = (int16_t)((raw[14] << 8) | raw[13]);
    cdata->par_p7  = (int8_t)raw[15];
    cdata->par_p8  = (int8_t)raw[16];
    cdata->par_p9  = (int16_t)((raw[18] << 8) | raw[17]);
    cdata->par_p10 = raw[19];
    cdata->par_p11 = raw[20];

    calib = *cdata;
    return true;
}

// Internal calibration state used for pressure compensation
static int64_t t_lin = 0;

int64_t BMP388_CompensateTemperature(uint32_t uncomp_t)
{
    int64_t pd1 = (int64_t)uncomp_t - ((int64_t)calib.par_t1 << 8);
    int64_t pd2 = (pd1 * (int64_t)calib.par_t2) >> 19;
    int64_t pd3 = (pd1 * pd1) >> 8;
    int64_t pd4 = (pd3 * (int64_t)calib.par_t3) >> 33;
    t_lin = pd2 + pd4;
    return (t_lin * 5 + 128) >> 8; // 0.01 degC
}

uint64_t BMP388_CompensatePressure(uint32_t uncomp_p)
{
    int64_t pd1 = (int64_t)t_lin * t_lin;            // t_lin^2
    int64_t pd2 = pd1 >> 8;                           // t_lin^2 / 256
    int64_t pd3 = (pd2 * t_lin) >> 19;                // t_lin^3 / 2^27
    int64_t po1 = ((int64_t)calib.par_p8 * pd3) >> 5;
    int64_t po2 = ((int64_t)calib.par_p7 * pd1) << 4;
    int64_t po3 = ((int64_t)calib.par_p6 * t_lin) << 22;
    int64_t offset = ((int64_t)calib.par_p5 << 47) + po1 + po2 + po3;

    int64_t ps1 = ((int64_t)calib.par_p4 * pd3) >> 5;
    int64_t ps2 = ((int64_t)calib.par_p3 * pd1) << 1;
    int64_t ps3 = (((int64_t)calib.par_p2 - 16384) * t_lin) << 15;
    int64_t sensitivity = (((int64_t)calib.par_p1 - 16384) << 31) + ps1 + ps2 + ps3;

    int64_t s1 = (sensitivity >> 14) * (int64_t)uncomp_p;
    int64_t s2 = ((int64_t)calib.par_p10 * t_lin) >> 10;
    int64_t s3 = (s2 + ((int64_t)calib.par_p9 << 16)) * (int64_t)uncomp_p >> 13;
    int64_t s4 = ((int64_t)calib.par_p11 * (int64_t)uncomp_p * (int64_t)uncomp_p) >> 16;

    int64_t pressure = ((offset >> 14) + s1 + s3 + s4) >> 11;
    return (uint64_t)pressure;
}

bool BMP388_ReadRaw(uint32_t *press, uint32_t *temp)
{
    uint8_t buf[BMP388_DATA_LEN];
    if (!bmp388_read_regs(BMP388_PRESS_MSB_REG, buf, BMP388_DATA_LEN))
        return false;

    *press = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
    *temp  = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);
    return true;
}

bool BMP388_ReadPressureTempInt(int32_t *pressure, int32_t *temperature)
{
    uint32_t up, ut;
    if (!BMP388_ReadRaw(&up, &ut))
        return false;
    int64_t ct = BMP388_CompensateTemperature(ut);       // 0.01 degC
    uint64_t cp = BMP388_CompensatePressure(up);         // Pa

    if (pressure)
        *pressure = (int32_t)cp;
    if (temperature)
        *temperature = (int32_t)(ct);
    return true;
}

bool BMP388_TriggerOneShot(void)
{
    return bmp388_write_reg(BMP388_PWR_CTRL_REG, (1 << 0) | (1 << 1) | (BMP388_MODE_FORCED << 4));
}

bool BMP388_WaitForData(uint32_t timeout_ms)
{
    uint8_t st;
    uint32_t start = HAL_GetTick();
    do {
        if (!bmp388_read_regs(BMP388_STATUS_REG, &st, 1))
            return false;
        if (st & (1 << 3))
            return true;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return false;
}

bool BMP388_ReadOneShot(float *press, float *temp)
{
    if (!BMP388_TriggerOneShot())
        return false;
    if (!BMP388_WaitForData(10))
        return false;
    int32_t ip, it;
    if (!BMP388_ReadPressureTempInt(&ip, &it))
        return false;
    if (press)
        *press = (float)ip / 100.0f;      // Pa to hPa
    if (temp)
        *temp = (float)it / 100.0f;       // 0.01°C -> °C
    return true;
}

static bool BMP388_Configure(void)
{
    uint8_t osr = ((0x03 & BMP388_OSR_P_MASK) << BMP388_OSR_P_BIT) |
                  ((0x00 << BMP388_OSR_T_BIT) & BMP388_OSR_T_MASK);
    if (!bmp388_write_reg(BMP388_OSR_REG, osr))
        return false;
    if (!bmp388_write_reg(BMP388_ODR_REG, 0x02))        // 50Hz
        return false;
    return bmp388_write_reg(BMP388_PWR_CTRL_REG, (1 << 0) | (1 << 1) | (BMP388_MODE_NORMAL << 4));
}

bool BMP388_Init(void)
{
    uint8_t id;
    uint8_t addrs[2] = { BMP388_I2C_ADDR1, BMP388_I2C_ADDR2 };
    for (int i = 0; i < 2; i++) {
        bmp_addr = addrs[i];
        if (bmp388_read_regs(BMP388_CHIP_ID_REG, &id, 1) && (id == 0x50 || id == 0x60))
            break;
        if (i == 1)
            return false;
    }

    if (!bmp388_write_reg(BMP388_CMD_REG, BMP388_CMD_SOFTRESET))
        return false;
    HAL_Delay(10);

    if (!BMP388_ReadCalibData(&calib))
        return false;

    return BMP388_Configure();
}

