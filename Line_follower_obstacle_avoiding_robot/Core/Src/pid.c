/*
 * pid.c
 *
 *  Created on: Nov 11, 2025
 *      Author: Hardik Agrawal
 *      Roll No= 22UEC045
 */


#include "pid.h"

float PID_Compute(PID_t *pid, float error, float dt)
{
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}
