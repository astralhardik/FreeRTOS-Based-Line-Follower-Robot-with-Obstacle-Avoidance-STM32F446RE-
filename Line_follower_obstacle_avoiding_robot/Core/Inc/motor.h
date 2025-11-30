/*
 * motor.h
 *
 *  Created on: Nov 11, 2025
 *      Author: Hardik Agrawal
 *      Roll No= 22UEC045
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"      // for TIM handles
#include "FreeRTOS.h"

// Motor PWM channels
#define MOTOR_A_IN1_CH  TIM_CHANNEL_3   // PA10
#define MOTOR_A_IN2_CH  TIM_CHANNEL_4   // PA11
#define MOTOR_B_IN3_CH  TIM_CHANNEL_1   // PA8
#define MOTOR_B_IN4_CH  TIM_CHANNEL_2   // PA9

#define MAX_PWM_VALUE   1999
#define BASE_SPEED      30
#define TURN_SPEED      30

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_SHARP_LEFT,
    MOTOR_SHARP_RIGHT
} MotorCommand_t;

void MotorControl(MotorCommand_t cmd, uint8_t speed);
void Motor_SetSpeedsPID(int16_t leftPercent, int16_t rightPercent);

#endif /* INC_MOTOR_H_ */
