/*
 * geofence.h
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */

#ifndef GEOFENCE_H
#define GEOFENCE_H

#include <stdbool.h>

/**
 * @brief  Checks whether the given position is outside the configured geofence.
 * @param  lat         Current latitude (degrees).
 * @param  lon         Current longitude (degrees).
 * @param  alt         Current altitude (meters).
 * @param  centerLat   Geofence center latitude (degrees).
 * @param  centerLon   Geofence center longitude (degrees).
 * @param  centerAlt   Geofence center altitude (meters).
 * @param  radiusM     Geofence radius (meters).
 * @param  maxAltM     Geofence maximum altitude (meters).
 * @return true if any limit is breached.
 */
bool Geofence_Breached(double lat, double lon, float alt,
                       double centerLat, double centerLon, float centerAlt,
                       float radiusM, float maxAltM);

#endif // GEOFENCE_H
