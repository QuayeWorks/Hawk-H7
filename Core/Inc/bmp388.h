/*
 * bmp388.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

// ================== bmp388.h ==================
#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include <stdbool.h>

// I²C 7-bit addresses for BMP388; shift left by 1 for HAL routines
#define BMP388_I2C_ADDR1   (0x76 << 1)
#define BMP388_I2C_ADDR2   (0x77 << 1)

// Register map (subset)
#define BMP388_CHIP_ID_REG     0x00
#define BMP388_STATUS_REG      0x03
#define BMP388_PRESS_MSB_REG   0x04
#define BMP388_DATA_LEN        6
#define BMP388_PWR_CTRL_REG    0x1B
#define BMP388_OSR_REG         0x1C
#define BMP388_ODR_REG         0x1D
#define BMP388_CMD_REG         0x7E

#define BMP388_CMD_SOFTRESET   0xB6

// Bit fields for OSR register
#define BMP388_OSR_P_BIT   0
#define BMP388_OSR_P_MASK  0x07
#define BMP388_OSR_T_BIT   3
#define BMP388_OSR_T_MASK  0x38

// Power modes
#define BMP388_MODE_SLEEP   0x00
#define BMP388_MODE_FORCED  0x01
#define BMP388_MODE_NORMAL  0x03

// Calibration parameters, as laid out in the BMP388 datasheet
typedef struct {
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    int16_t  par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;
    uint8_t  par_p11;
} BMP388_CalibData;

/**
 * @brief Reads calibration registers from the BMP388 into calib struct.
 * @param cdata Pointer to a BMP388_CalibData struct to fill.
 * @return true on success, false on I2C error.
 */
bool BMP388_ReadCalibData(BMP388_CalibData *cdata);

/** Initialize the sensor: detect address, reset, load calibration and configure. */
bool BMP388_Init(void);

/**
 * @brief Triggers a single one‐shot pressure+temperature conversion.
 * @return true if the command was successfully issued.
 */
bool BMP388_TriggerOneShot(void);

/**
 * @brief Waits for the one‐shot conversion to complete.
 * @param timeout_ms How many milliseconds to wait before giving up.
 * @return true if data is ready before timeout, false otherwise.
 */
bool BMP388_WaitForData(uint32_t timeout_ms);

/**
 * @brief Reads a completed one‐shot conversion and applies calibration.
 * @param pressure Pointer to receive pressure in Pa.
 * @param temperature Pointer to receive temperature in °C.
 * @return true on success, false on I2C error.
 */
bool BMP388_ReadOneShot(float *pressure, float *temperature);

// Raw and compensated access helpers
bool    BMP388_ReadRaw(uint32_t *p, uint32_t *t);
int64_t BMP388_CompensateTemperature(uint32_t uncomp_temperature);
uint64_t BMP388_CompensatePressure(uint32_t uncomp_pressure);
bool    BMP388_ReadPressureTempInt(int32_t *pressure, int32_t *temperature);

#endif // BMP388_H
