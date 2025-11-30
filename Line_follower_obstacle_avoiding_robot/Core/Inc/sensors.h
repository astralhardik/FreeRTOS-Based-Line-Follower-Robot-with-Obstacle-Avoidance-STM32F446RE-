/*
 * sensor.h
 *
 *  Created on: Nov 27, 2025
 *      Author: hardi
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"
#include "FreeRTOS.h"

#define IR_LEFT_MOST    0
#define IR_LEFT         1
#define IR_CENTER       2
#define IR_RIGHT        3
#define IR_RIGHT_MOST   4

extern uint8_t  irSensorValues[5];
extern volatile uint32_t distance;
extern volatile uint8_t  obstacleDetected;

void ReadIRSensors(void);
uint32_t ReadUltrasonic(void);

// HAL callback for ultrasonic – called from ISR
void Ultrasonic_IC_Callback(TIM_HandleTypeDef *htim);

#endif /* INC_SENSORS_H_ */
