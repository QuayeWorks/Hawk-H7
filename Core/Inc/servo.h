#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

typedef enum {
    SERVO_GIMBAL_STABILIZE = 0,
    SERVO_GIMBAL_MANUAL    = 1
} ServoGimbalMode;

void Servo_Init(void);
void Servo_SetPWM(uint8_t index, uint16_t pulse_us);
uint16_t Servo_GetPWM(uint8_t index);

void Servo_SetGimbalMode(ServoGimbalMode mode);
ServoGimbalMode Servo_GetGimbalMode(void);

void Servo_Task(uint8_t debugMask);

#endif // SERVO_H
