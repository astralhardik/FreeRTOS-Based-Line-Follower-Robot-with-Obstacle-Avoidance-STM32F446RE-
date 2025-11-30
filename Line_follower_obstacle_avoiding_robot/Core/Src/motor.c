/*
 * motor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: Hardik Agrawal
 *      Roll No= 22UEC045
 */

#include "motor.h"
#include <stdlib.h>   // for abs()
#include <stdio.h>

extern TIM_HandleTypeDef htim1;

void Motor_SetSpeedsPID(int16_t leftPercent, int16_t rightPercent)
{
    if (leftPercent > 100)  leftPercent = 100;
    if (leftPercent < -100) leftPercent = -100;
    if (rightPercent > 100)  rightPercent = 100;
    if (rightPercent < -100) rightPercent = -100;

    uint32_t leftPwm  = (abs(leftPercent)  * MAX_PWM_VALUE) / 100;
    uint32_t rightPwm = (abs(rightPercent) * MAX_PWM_VALUE) / 100;

    if (leftPercent >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, leftPwm);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, leftPwm);
    }

    if (rightPercent >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, rightPwm);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, rightPwm);
    }
}

void MotorControl(MotorCommand_t cmd, uint8_t speed)
{
    uint32_t speedPwm = (speed * MAX_PWM_VALUE) / 100;
    uint32_t turnPwm = (TURN_SPEED * MAX_PWM_VALUE) / 100;

    switch(cmd)
    {
        case MOTOR_FORWARD:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, speedPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, speedPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, 0);
            break;

        case MOTOR_BACKWARD:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, speedPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, speedPwm);
            break;

        case MOTOR_LEFT:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH,0 );
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH,speedPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH,speedPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH,0);
            break;

        case MOTOR_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, speedPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, speedPwm);
            break;

        case MOTOR_SHARP_LEFT:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, turnPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, turnPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, 0);
            break;

        case MOTOR_SHARP_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, turnPwm);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, turnPwm);
            break;

        case MOTOR_STOP:
        default:
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_A_IN2_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN3_CH, 0);
            __HAL_TIM_SET_COMPARE(&htim1, MOTOR_B_IN4_CH, 0);
            break;
    }
}
