/*
 * geofence.c
 *
 *  Created on: Jun 15, 2025
 *      Author: Quayen01
 */


#include "geofence.h"
#include <math.h>

// Simple Haversine formula for horizontal distance
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

bool Geofence_Breached(double lat, double lon, float alt,
                       double centerLat, double centerLon, float centerAlt,
                       float radiusM, float maxAltM)
{
    // Check horizontal radius
    double dist = haversine(lat, lon, centerLat, centerLon);
    if (dist > radiusM) {
        return true;
    }
    // Check altitude limit
    if (alt > maxAltM) {
        return true;
    }
    return false;
}
