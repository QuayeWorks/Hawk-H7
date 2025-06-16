// ================== flight_state.c ==================
#include <string.h>
#include "flight_state.h"
#include "buzzer.h"
#include "settings.h"  // for any thresholds, if you want to re‐load them dynamically

// The single 32‐bit container for all health+state flags
uint32_t fsState = 0;

// ——— Internal forward declarations ———
//static void    PlayErrorBuzzer(uint32_t healthBit);

// ——— Public API implementations ———

// Set a single health bit
void FlightState_SetHealth(uint32_t bit) {
    fsState |= bit;
    // Whenever a health bit changes (either set or clear), we should call
    // FlightState_Update() to re‐evaluate FS_READY_BIT.
    FlightState_Update();
}

// Clear a single health bit
void FlightState_ClearHealth(uint32_t bit) {
    fsState &= ~bit;
    FlightState_Update();
}

// Set the “Ready” bit (only if all health bits are OK)
void FlightState_SetReady(void) {
    fsState |= FS_READY_BIT;
}

// Clear the “Ready” bit
void FlightState_ClearReady(void) {
    fsState &= ~FS_READY_BIT;
}

// Set the “Armed” bit (perform any buzzer tones here)
void FlightState_Arm(void) {
    if (fsState & FS_READY_BIT) {
        fsState |= FS_ARMED_BIT;
        Buzzer_PlayTone(TONE_ARMED);
    }
}

// Clear the “Armed” bit (and play disarmed tone)
void FlightState_Disarm(void) {
    fsState &= ~FS_ARMED_BIT;
    Buzzer_PlayTone(TONE_DISARMED);
}

// Set the “Battery OK under load” bit
void FlightState_SetBatteryOK(void) {
    fsState |= FS_BATTERY_OK_BIT;
}

// Clear the “Battery OK under load” bit
void FlightState_ClearBatteryOK(void) {
    fsState &= ~FS_BATTERY_OK_BIT;
}

// Query functions
bool FlightState_IsReady(void) {
    return (fsState & FS_READY_BIT) != 0;
}

bool FlightState_IsArmed(void) {
    return (fsState & FS_ARMED_BIT) != 0;
}

bool FlightState_IsBatteryOK(void) {
    return (fsState & FS_BATTERY_OK_BIT) != 0;
}

bool FlightState_AllHealthOK(void) {
    return ((fsState & FS_ALL_HEALTH_BITS) == FS_ALL_HEALTH_BITS);
}

uint32_t FlightState_GetStateMask(void) {
    return fsState;
}

// Called after any health bit change. This automatically toggles FS_READY_BIT based
// on whether all health bits are now set, and forces a disarm if any health has failed.
void FlightState_Update(void) {
    if (FlightState_AllHealthOK()) {
        FlightState_SetReady();
    } else {
        // If we ever lose a health bit, we must clear Ready & disarm immediately:
        FlightState_ClearReady();
        FlightState_Disarm();
    }
}

// You can add a helper if you want to immediately sound an error tone whenever a health bit
// trips. For example, if IMU calibration fails on power‐up, you might do something like:
//
//   FlightState_ClearHealth(FS_HEALTH_IMU_OK_BIT);
//   PlayErrorBuzzer(FS_HEALTH_IMU_OK_BIT);
//


void CommenceAutoLand(void)
{
    // TODO: fly back to home or descend vertically.
    // Right now this is a no-op so your code links.
}

void CommenceAltHold(void)
{
    // TODO: engage altitude‐hold controller
}
