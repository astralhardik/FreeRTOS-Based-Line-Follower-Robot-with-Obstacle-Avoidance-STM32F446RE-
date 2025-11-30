/*
 * pid.h
 *
 *  Created on: Nov 27, 2025
 *      Author: hardi
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prevError;
    float integral;
} PID_t;

float PID_Compute(PID_t *pid, float error, float dt);

#endif /* INC_PID_H_ */
