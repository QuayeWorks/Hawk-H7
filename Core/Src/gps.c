/*
 * Cube Programmer
 * Copyright (c) 2025 QuayeWorks
 *
 * Licensed under the MIT License.
 * See the LICENSE file for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 */

#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "settings.h"
#include <math.h>  // for sin, cos, sqrt

#define GPS_RX_BUFFER_SIZE 128
#define GPS_RX_RING_SIZE   512

static double homeLat, homeLon;
static uint32_t firstFixMs = 0;
static uint32_t gpsUpdateCount = 0;
static uint32_t gpsLastUpdateMs = 0;
static uint32_t gpsSentenceCount = 0;
static uint16_t gpsSentenceRateHz = 0;
static uint16_t gpsSentenceWindowCount = 0;
static uint32_t gpsSentenceWindowStartMs = 0;

// Simple structure to hold the latest GPS data
static struct {
    bool    hasFix;
    uint8_t satCount;
    float   hdop;
    double  latitude;
    double  longitude;
    float   altitude;
} gpsData;

// UART handle & RX byte
static UART_HandleTypeDef *gpsUart;
static uint8_t rxByte;
static volatile uint8_t rxRing[GPS_RX_RING_SIZE];
static volatile uint16_t rxHead;
static volatile uint16_t rxTail;
static volatile uint32_t rxDroppedBytes;

// Line buffer for NMEA sentences
static char lineBuf[GPS_RX_BUFFER_SIZE];
static uint16_t lineIdx;

// Helpers to convert NMEA lat/lon format (ddmm.mmmm) to decimal degrees
static double NmeaToDeg(const char *nmea, char dir) {
    // nmea: ddmm.mmmm or dddmm.mmmm
    double val = atof(nmea);
    int degrees = (int)(val / 100);
    double minutes = val - (degrees * 100);
    double deg = degrees + (minutes / 60.0);
    if (dir == 'S' || dir == 'W') deg = -deg;
    return deg;
}

// Called by HAL_UART_RxCpltCallback when a byte arrives
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint16_t nextHead;

    if (huart != gpsUart) {
        return;
    }

    // IRQ path: only enqueue the received byte and re-arm reception.
    nextHead = (uint16_t)((rxHead + 1u) % GPS_RX_RING_SIZE);
    if (nextHead != rxTail) {
        rxRing[rxHead] = rxByte;
        rxHead = nextHead;
    } else {
        rxDroppedBytes++;
    }

    // Re-arm UART receive interrupt
    HAL_UART_Receive_IT(gpsUart, &rxByte, 1);
}

void GPS_Init(UART_HandleTypeDef *huart) {
    gpsUart   = huart;

    // Reconfigure UART baud rate from settings before starting reception.
    gpsUart->Init.BaudRate = Settings_GetGPSBaud();
    HAL_UART_DeInit(gpsUart);
    if (HAL_UART_Init(gpsUart) != HAL_OK) {
        // If re-init fails, GPS comms won't work but we'll continue running.
    }

    lineIdx   = 0;
    gpsData.hasFix   = false;
    gpsData.satCount = 0;
    gpsData.hdop     = 999.0f;
    gpsData.latitude = 0.0;
    gpsData.longitude= 0.0;
    gpsData.altitude = 0.0f;
    firstFixMs = 0;
    gpsUpdateCount = 0;
    gpsLastUpdateMs = 0;
    gpsSentenceCount = 0;
    gpsSentenceRateHz = 0;
    gpsSentenceWindowCount = 0;
    gpsSentenceWindowStartMs = HAL_GetTick();
    rxHead = 0u;
    rxTail = 0u;
    rxDroppedBytes = 0u;
    memset(lineBuf, 0, sizeof(lineBuf));
    // Start the first byte reception interrupt
    HAL_UART_Receive_IT(gpsUart, &rxByte, 1);
}

// Called periodically (if you want to implement drift checks or timeouts).
// For now, we do nothing here.
void GPS_Update(void) {
    uint16_t headSnapshot = rxHead;
    uint32_t nowMs;

    while (rxTail != headSnapshot) {
        char c = (char)rxRing[rxTail];
        rxTail = (uint16_t)((rxTail + 1u) % GPS_RX_RING_SIZE);

        if (c == '\n' || lineIdx >= (GPS_RX_BUFFER_SIZE - 1u)) {
            lineBuf[lineIdx] = '\0';
            GPS_ProcessSentence(lineBuf);
            gpsSentenceCount++;
            gpsSentenceWindowCount++;
            lineIdx = 0u;
        } else if (c != '\r') {
            lineBuf[lineIdx++] = c;
        }
    }

    nowMs = HAL_GetTick();
    if (gpsSentenceWindowStartMs == 0u) {
        gpsSentenceWindowStartMs = nowMs;
    }
    if ((nowMs - gpsSentenceWindowStartMs) >= 1000u) {
        gpsSentenceRateHz = gpsSentenceWindowCount;
        gpsSentenceWindowCount = 0u;
        gpsSentenceWindowStartMs = nowMs;
    }

    if (gpsData.hasFix && gpsLastUpdateMs != 0U) {
        if ((nowMs - gpsLastUpdateMs) > 2000U) {
            gpsData.hasFix = false;
        }
    }
}

// Parse a single NMEA sentence (zero-terminated, no LF/CR). We only handle GGA here.
void GPS_ProcessSentence(const char *s) {
    // Quick check for GGA sentences
    if ((strncmp(s, "$GNGGA,",6) != 0) && (strncmp(s, "$GPGGA,",6) != 0)) {
        return;
    }
    // Tokenize by commas
    // $--GGA,hhmmss.ss,lat,NS,lon,EW,quality,sat,hdop,alt,M,...
    char copy[GPS_RX_BUFFER_SIZE];
    char *tok;
    int idx = 0;
    char *fields[15];
    strncpy(copy, s, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    tok = strtok(copy, ",");
    while (tok && idx < 15) {
        fields[idx++] = tok;
        tok = strtok(NULL, ",");
    }
    if (idx >= 10) {
        int quality = atoi(fields[6]);
        uint8_t sats = (uint8_t)atoi(fields[7]);
        float hdopf  = atof(fields[8]);
        float altf   = atof(fields[9]);

        // Only accept if quality >= 1 (GPS fix)
        gpsLastUpdateMs = HAL_GetTick();
        gpsUpdateCount++;

        if (quality >= 1 &&
            sats >= Settings_GetGPSMinSatellites() &&
            hdopf <= Settings_GetGPSMaxHDOP() &&
            fields[2] != NULL && fields[3] != NULL &&
            fields[4] != NULL && fields[5] != NULL &&
            fields[3][0] != '\0' && fields[5][0] != '\0') {

            double lat = NmeaToDeg(fields[2], fields[3][0]);
            double lon = NmeaToDeg(fields[4], fields[5][0]);

            gpsData.hasFix    = true;
            if (!firstFixMs) {
                firstFixMs = HAL_GetTick();
                homeLat = lat;
                homeLon = lon;
            }
            gpsData.satCount  = sats;
            gpsData.hdop      = hdopf;
            gpsData.latitude  = lat;
            gpsData.longitude = lon;
            gpsData.altitude  = altf;

            // Optionally store home position on first fix:
            static bool homeSet = false;
            if (!homeSet) {
                homeSet = true;
                Settings_SetHomeLatitude(lat);
                Settings_SetHomeLongitude(lon);
                Settings_SetHomeAltitude(altf);
            }
        } else {
            gpsData.hasFix = false;
        }
    }
}

// Haversine formula to compute distance in meters between two lat/lon points
static double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Earth radius in meters
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2)*sin(dLat/2) +
               cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0) *
               sin(dLon/2)*sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

bool GPS_CheckDriftOK(uint16_t holdSeconds) {
    if (!gpsData.hasFix) return false;
    uint32_t elapsed = HAL_GetTick() - firstFixMs;
    if (elapsed < (uint32_t)holdSeconds * 1000U) {
        // Still within hold period, consider OK
        return true;
    }
    // Check distance from home
    double dist = haversine(homeLat, homeLon,
                            gpsData.latitude, gpsData.longitude);
    // Allow small drift, e.g. 2 meters
    return (dist <= 2.0);
}

bool GPS_HasFix(void)        { return gpsData.hasFix; }
uint8_t GPS_GetSatCount(void){ return gpsData.satCount; }
float   GPS_GetHDOP(void)    { return gpsData.hdop; }
double  GPS_GetLatitude(void){ return gpsData.latitude; }
double  GPS_GetLongitude(void){return gpsData.longitude; }
float   GPS_GetAltitude(void){ return gpsData.altitude; }
uint32_t GPS_GetUpdateCount(void) { return gpsUpdateCount; }
uint32_t GPS_GetLastUpdateMs(void) { return gpsLastUpdateMs; }
uint16_t GPS_GetSentenceRateHz(void) { return gpsSentenceRateHz; }
uint32_t GPS_GetDroppedByteCount(void) { return (uint32_t)rxDroppedBytes; }
uint16_t GPS_GetRxRingLevel(void)
{
    uint16_t head = rxHead;
    uint16_t tail = rxTail;
    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(GPS_RX_RING_SIZE - (tail - head));
}
