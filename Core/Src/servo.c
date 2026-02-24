#include "servo.h"
#include "stm32h7xx_hal.h"
#include "rc_input.h"
#include "attitude.h"
#include "debug_menu.h"
#include "settings.h"
#include <stdbool.h>

// TIM handle is provided by CubeMX in main.c
extern TIM_HandleTypeDef htim5;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t pulse;
} ServoChannel;

static ServoChannel servos[4];
static ServoGimbalMode gimbalMode = SERVO_GIMBAL_MANUAL;
static float           yawHold   = 0.0f;
static float           pitchHold = 0.0f;
static bool            servoDebugPrev = false;
static bool            servoDebugStabilize = false;
static uint32_t        servoDebugStartMs = 0u;

#define SERVO_US_MIN                1000u
#define SERVO_US_NEUTRAL            1500u
#define SERVO_US_MAX                2000u
#define SERVO_TEST_PHASE_MS         2000u
#define SERVO_PITCH_GAIN_US_PER_DEG 10.0f
#define SERVO_YAW_GAIN_US_PER_DEG   10.0f
#define SERVO_RC_TRIM_GAIN          0.5f
#define SERVO_RC_STALE_MS           120u

static inline void set_pulse(ServoChannel *s, uint16_t us)
{
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, us);
}

static uint16_t clamp_pulse_us(int32_t us)
{
    if (us < (int32_t)SERVO_US_MIN) {
        return SERVO_US_MIN;
    }
    if (us > (int32_t)SERVO_US_MAX) {
        return SERVO_US_MAX;
    }
    return (uint16_t)us;
}

static void gimbal_capture_hold(void)
{
    AttitudeAngles a = Attitude_GetAngles();
    yawHold = a.yaw;
    pitchHold = a.pitch;
}

static uint16_t rc_channel_or_neutral(uint8_t channel)
{
    uint16_t v;

    if (RC_ChannelsAreStale(SERVO_RC_STALE_MS)) {
        return SERVO_US_NEUTRAL;
    }

    v = RC_GetChannel(channel);
    if (v < SERVO_US_MIN || v > SERVO_US_MAX) {
        return SERVO_US_NEUTRAL;
    }
    return v;
}

static void gimbal_apply_stabilization_with_trim(int16_t pitchTrimUs, int16_t yawTrimUs)
{
    AttitudeAngles a = Attitude_GetAngles();
    float yawErr = yawHold - a.yaw;
    float pitchErr = pitchHold - a.pitch;
    int32_t pitchOutUs = (int32_t)(SERVO_US_NEUTRAL + (pitchErr * SERVO_PITCH_GAIN_US_PER_DEG) + pitchTrimUs);
    int32_t yawOutUs = (int32_t)(SERVO_US_NEUTRAL + (yawErr * SERVO_YAW_GAIN_US_PER_DEG) + yawTrimUs);

    /* PA2 (index 2) = Pitch servo, PA3 (index 3) = Yaw servo. */
    Servo_SetPWM(2u, clamp_pulse_us(pitchOutUs));
    Servo_SetPWM(3u, clamp_pulse_us(yawOutUs));
}

static void gimbal_apply_stabilization(void)
{
    gimbal_apply_stabilization_with_trim(0, 0);
}

void Servo_Init(void)
{
    servos[0] = (ServoChannel){ &htim5, TIM_CHANNEL_1, SERVO_US_NEUTRAL };
    servos[1] = (ServoChannel){ &htim5, TIM_CHANNEL_2, SERVO_US_NEUTRAL };
    servos[2] = (ServoChannel){ &htim5, TIM_CHANNEL_3, SERVO_US_NEUTRAL };
    servos[3] = (ServoChannel){ &htim5, TIM_CHANNEL_4, SERVO_US_NEUTRAL };

    for(int i=0;i<4;i++) {
        HAL_TIM_PWM_Start(servos[i].htim, servos[i].channel);
        set_pulse(&servos[i], servos[i].pulse);
    }
    gimbalMode = SERVO_GIMBAL_MANUAL;
    yawHold = 0.0f;
    pitchHold = 0.0f;
    servoDebugPrev = false;
    servoDebugStabilize = false;
    servoDebugStartMs = 0u;
}

void Servo_SetPWM(uint8_t index, uint16_t pulse_us)
{
    if(index < 4) {
        set_pulse(&servos[index], pulse_us);
        servos[index].pulse = pulse_us;
    }
}

uint16_t Servo_GetPWM(uint8_t index)
{
    if(index < 4) {
        return servos[index].pulse;
    }
    return 0;
}

void Servo_SetGimbalMode(ServoGimbalMode mode)
{
    gimbalMode = mode;
    if(mode == SERVO_GIMBAL_STABILIZE) {
        gimbal_capture_hold();
    }
}

ServoGimbalMode Servo_GetGimbalMode(void)
{
    return gimbalMode;
}

void Servo_Task(uint8_t debugMask)
{
    bool servoDebug = ((debugMask & DEBUG_MENU_SERVOS) != 0u);
    uint32_t now = HAL_GetTick();

    if (servoDebug && !servoDebugPrev) {
        servoDebugStartMs = now;
        servoDebugStabilize = false;
    } else if (!servoDebug && servoDebugPrev) {
        servoDebugStabilize = false;
        gimbalMode = SERVO_GIMBAL_MANUAL;
        Servo_SetPWM(2u, SERVO_US_NEUTRAL);
        Servo_SetPWM(3u, SERVO_US_NEUTRAL);
    }
    servoDebugPrev = servoDebug;

    if (servoDebug) {
        uint32_t elapsed = now - servoDebugStartMs;

        if (elapsed < SERVO_TEST_PHASE_MS) {
            /* Phase 1 (2s): move all servos to opposite extremes. */
            Servo_SetPWM(0u, SERVO_US_MAX);
            Servo_SetPWM(1u, SERVO_US_MAX);
            Servo_SetPWM(2u, SERVO_US_MIN);
            Servo_SetPWM(3u, SERVO_US_MIN);
            return;
        }

        if (elapsed < (2u * SERVO_TEST_PHASE_MS)) {
            /* Phase 2 (2s): swap extremes so all four servos reverse. */
            Servo_SetPWM(0u, SERVO_US_MIN);
            Servo_SetPWM(1u, SERVO_US_MIN);
            Servo_SetPWM(2u, SERVO_US_MAX);
            Servo_SetPWM(3u, SERVO_US_MAX);
            return;
        }

        /* Phase 3: yaw/pitch gimbal stabilization test. */
        if (!servoDebugStabilize) {
            gimbal_capture_hold();
            servoDebugStabilize = true;
            gimbalMode = SERVO_GIMBAL_STABILIZE;
        }
        {
            uint8_t rollChan = Settings_GetRCRollChannel();
            uint8_t pitchChan = Settings_GetRCPitchChannel();
            uint8_t yawChan = Settings_GetRCYawChannel();
            uint16_t rollUs = rc_channel_or_neutral(rollChan);
            uint16_t pitchUs = rc_channel_or_neutral(pitchChan);
            uint16_t yawUs = rc_channel_or_neutral(yawChan);
            int16_t pitchTrimUs = (int16_t)(((int32_t)pitchUs - (int32_t)SERVO_US_NEUTRAL) * SERVO_RC_TRIM_GAIN);
            int16_t yawTrimUs = (int16_t)(((int32_t)yawUs - (int32_t)SERVO_US_NEUTRAL) * SERVO_RC_TRIM_GAIN);

            /* After stabilization starts: roll controls PA0/PA1, pitch/yaw trim PA2/PA3. */
            Servo_SetPWM(0u, rollUs);
            Servo_SetPWM(1u, rollUs);
            gimbal_apply_stabilization_with_trim(pitchTrimUs, yawTrimUs);
        }
        return;
    }

    // Channels 1 and 2 controlled directly by RC channels 1 and 2
    Servo_SetPWM(0, RC_GetChannel(0));
    Servo_SetPWM(1, RC_GetChannel(1));

    if(gimbalMode == SERVO_GIMBAL_MANUAL) {
        // Channel 3 from RC channel 6
        Servo_SetPWM(2, RC_GetChannel(5));
        Servo_SetPWM(3, SERVO_US_NEUTRAL);
    } else {
        gimbal_apply_stabilization();
    }
}
