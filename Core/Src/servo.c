#include "servo.h"
#include "stm32h7xx_hal.h"
#include "rc_input.h"
#include "attitude.h"
#include "debug_menu.h"

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

static inline void set_pulse(ServoChannel *s, uint16_t us)
{
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, us);
}

void Servo_Init(void)
{
    servos[0] = (ServoChannel){ &htim5, TIM_CHANNEL_1, 1500 };
    servos[1] = (ServoChannel){ &htim5, TIM_CHANNEL_2, 1500 };
    servos[2] = (ServoChannel){ &htim5, TIM_CHANNEL_3, 1500 };
    servos[3] = (ServoChannel){ &htim5, TIM_CHANNEL_4, 1500 };

    for(int i=0;i<4;i++) {
        HAL_TIM_PWM_Start(servos[i].htim, servos[i].channel);
        set_pulse(&servos[i], servos[i].pulse);
    }
    gimbalMode = SERVO_GIMBAL_MANUAL;
    yawHold = 0.0f;
    pitchHold = 0.0f;
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
        AttitudeAngles a = Attitude_GetAngles();
        yawHold   = a.yaw;
        pitchHold = a.pitch;
    }
}

ServoGimbalMode Servo_GetGimbalMode(void)
{
    return gimbalMode;
}

void Servo_Task(uint8_t debugMask)
{
    // Channels 1 and 2 controlled directly by RC channels 1 and 2
    Servo_SetPWM(0, RC_GetChannel(0));
    Servo_SetPWM(1, RC_GetChannel(1));

    if(gimbalMode == SERVO_GIMBAL_MANUAL) {
        // Channel 3 from RC channel 6
        Servo_SetPWM(2, RC_GetChannel(5));

        if (debugMask & DEBUG_MENU_SERVOS) {
            static uint16_t pos = 1000;
            static int8_t dir = 1;
            pos += dir * 10;
            if(pos >= 2000){
                pos = 2000;
                dir = -1;
            } else if(pos <= 1000){
                pos = 1000;
                dir = 1;
            }
            Servo_SetPWM(3, pos);
        } else {
            Servo_SetPWM(3, 1500);
        }
    } else {
        AttitudeAngles a = Attitude_GetAngles();
        float yawErr   = yawHold - a.yaw;
        float pitchErr = pitchHold - a.pitch;
        int16_t out3 = 1500 + (int16_t)(yawErr * 10.0f);
        int16_t out4 = 1500 + (int16_t)(pitchErr * 10.0f);
        if(out3 < 1000) out3 = 1000;
        if(out3 > 2000) out3 = 2000;
        if(out4 < 1000) out4 = 1000;
        if(out4 > 2000) out4 = 2000;
        Servo_SetPWM(2, (uint16_t)out3);
        Servo_SetPWM(3, (uint16_t)out4);
    }
}
