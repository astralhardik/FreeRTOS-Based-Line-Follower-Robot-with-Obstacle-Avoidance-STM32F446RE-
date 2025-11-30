/*
 * robot_tasks.h
 *
 *  Created on: Nov 27, 2025
 *      Author: hardi
 */

#ifndef INC_ROBOT_TASKS_H_
#define INC_ROBOT_TASKS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "motor.h"
#include "sensors.h"
#include "pid.h"

typedef struct {
    int16_t leftPercent;
    int16_t rightPercent;
} MotorPIDCmd_t;

/* Shared state */
extern volatile uint8_t obstacleActive;


extern TaskHandle_t xLineFollowerTaskHandle;
extern TaskHandle_t xObstacleTaskHandle;
extern TaskHandle_t xMotorTaskHandle;
extern TaskHandle_t xMonitorTaskHandle;

extern QueueHandle_t xMotorQueue;
extern SemaphoreHandle_t xUltrasonicSemaphore;
extern SemaphoreHandle_t xUARTMutex;
extern TimerHandle_t xSensorTimer;


// Tasks & timer callback
void vLineFollowerTask(void *pvParameters);
void vObstacleDetectionTask(void *pvParameters);
void vMotorControlTask(void *pvParameters);
void vSystemMonitorTask(void *pvParameters);
void vSensorTimerCallback(TimerHandle_t xTimer);
extern void Ultrasonic_Trigger(void);

#endif /* INC_ROBOT_TASKS_H_ */
