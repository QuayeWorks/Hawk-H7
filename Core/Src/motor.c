/*
 * motor.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Quayen01
 */

// ================== motor_control.c ==================
#include "motor.h"
#include "settings.h"
#include "stm32h7xx_hal.h"
#include "battery.h"

// Structure to hold timer handle + channel
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} MotorChannel;

// Array for up to 8 motors
static MotorChannel motors[8];

// Helper: start PWM on one channel
static inline void start_pwm(MotorChannel *m, uint16_t pulse_us) {
    // Convert µs to timer ticks (assuming timer is configured for 1 MHz count)
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, pulse_us);
}

// Public: initialize motors
void Motor_Init(void) {
    // Map motors 1–4 to TIM1_CH1..CH4
    extern TIM_HandleTypeDef htim1, htim3;  // from CubeMX
    motors[0] = (MotorChannel){ .htim=&htim1, .channel=TIM_CHANNEL_1 };
    motors[1] = (MotorChannel){ .htim=&htim1, .channel=TIM_CHANNEL_2 };
    motors[2] = (MotorChannel){ .htim=&htim1, .channel=TIM_CHANNEL_3 };
    motors[3] = (MotorChannel){ .htim=&htim1, .channel=TIM_CHANNEL_4 };
    // Map motors 5–8 to TIM3_CH1..CH4
    motors[4] = (MotorChannel){ .htim=&htim3, .channel=TIM_CHANNEL_1 };
    motors[5] = (MotorChannel){ .htim=&htim3, .channel=TIM_CHANNEL_2 };
    motors[6] = (MotorChannel){ .htim=&htim3, .channel=TIM_CHANNEL_3 };
    motors[7] = (MotorChannel){ .htim=&htim3, .channel=TIM_CHANNEL_4 };

    // Start PWM output on each channel
    uint8_t count = Settings_GetMotorCount();
    uint16_t idle = Settings_GetMotorPWMMinUs();
    for (uint8_t i = 0; i < count && i < 8; i++) {
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
        start_pwm(&motors[i], idle);
    }
}

// Public: set one motor's PWM (µs)
void Motor_SetPWM(uint8_t index, uint16_t pulse_us) {
    uint8_t count = Settings_GetMotorCount();
    if (index < count && index < 8) {
        start_pwm(&motors[index], pulse_us);
    }
}

// Public: set all motors (0..count-1) to the same PWM
void Motor_SetAllPWM(uint16_t pulse_us) {
    uint8_t count = Settings_GetMotorCount();
    for (uint8_t i = 0; i < count && i < 8; i++) {
        start_pwm(&motors[i], pulse_us);
    }
}

// Map throttle in [1000,2000] µs to ESC endpoints from settings
uint16_t Motor_ThrottleToPWM(uint16_t throttle_ppm) {
    uint16_t min_us = Settings_GetMotorPWMMinUs();
    uint16_t max_us = Settings_GetMotorPWMMaxUs();
    if (throttle_ppm < 1000) throttle_ppm = 1000;
    if (throttle_ppm > 2000) throttle_ppm = 2000;
    // Linear map: 1000→min_us, 2000→max_us
    uint32_t scaled = (uint32_t)(throttle_ppm - 1000) * (max_us - min_us);
    scaled = scaled / 1000 + min_us;
    return (uint16_t)scaled;
}

bool Motor_EscResponds(uint8_t index)
{
    /* Send a short spin-up pulse and check the resulting current draw.  If the
     * current increases slightly we assume the ESC responded.  This is a very
     * rough heuristic but provides a useful placeholder until a real RPM
     * sensor or telemetry feedback is implemented. */

    uint16_t test = Settings_GetMotorPWMMinUs() + 100;
    Motor_SetPWM(index, test);
    HAL_Delay(5);
    float current = Battery_ReadCurrent();
    Motor_SetPWM(index, Settings_GetMotorPWMMinUs());
    return current > 0.1f; // 100 mA threshold
}


void Motor_AutoDetectOrientation(void)
{
    /* Without sensor feedback we default all motors to normal orientation.  The
     * settings system allows saving this so that users can override it later. */
    uint8_t count = Settings_GetMotorCount();
    for (uint8_t i = 0; i < count; i++) {
        Settings_SetMotorDir(i, 0);  // 0 = normal rotation
    }
}
