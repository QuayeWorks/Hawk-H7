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

// I²C 7-bit address for BMP388; shift left by 1 for HAL
#define BMP388_I2C_ADDR   (0x76 << 1)

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

#endif // BMP388_H
