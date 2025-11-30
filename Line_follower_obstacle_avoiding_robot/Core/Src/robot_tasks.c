/*
 * robot_tasks.c
 *
 *  Created on: Nov 11, 2025
 *      Author: Hardik Agrawal
 *      Roll No= 22UEC045
 */


#include "robot_tasks.h"
#include <stdio.h>

// ==== PID CONFIG ==========================================================
PID_t linePID = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.5f,
    .prevError = 0.0f,
    .integral  = 0.0f
};

#define OBSTACLE_THRESHOLD   10   // cm
#define LINE_DT_SEC          0.01f
#define LINE_PERIOD_TICKS    pdMS_TO_TICKS(10)
#define OBST_PERIOD_TICKS    pdMS_TO_TICKS(80)
#define MONITOR_PERIOD_TICKS pdMS_TO_TICKS(2000)


extern QueueHandle_t      xMotorQueue;
extern SemaphoreHandle_t  xUltrasonicSemaphore;
extern SemaphoreHandle_t  xUARTMutex;
extern  uint8_t   irSensorValues[5];
extern volatile uint8_t   obstacleDetected;
extern volatile uint32_t  distance;


// ==== LINE FOLLOWER TASK ==================================================
void vLineFollowerTask(void *pvParameters)
{
    printf("[TASK] Line Follower Started!\r\n");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float kOut = 10.0f;
    uint32_t debugCounter = 0;

    for (;;)
    {
        //Periodic wake-up
        vTaskDelayUntil(&xLastWakeTime, LINE_PERIOD_TICKS);

        debugCounter++;

        //Skip control while obstacle handler is doing avoidance
        if (obstacleDetected)
            continue;

        ReadIRSensors();

        int8_t weights[5] = { -2, -1, 0, 1, 2 };
        int sum = 0, ones = 0;

        for (int i = 0; i < 5; i++) {
            if (irSensorValues[i]) {
                sum += weights[i];
                ones++;
            }
        }

        //debug (every 100 ms)
        if ((debugCounter % 10) == 0) {
            printf("[LINE] IR:[%d%d%d%d%d] ones=%d\r\n",
                   irSensorValues[0], irSensorValues[1],
                   irSensorValues[2], irSensorValues[3],
                   irSensorValues[4], ones);
        }

        if (ones == 0) {
            // Line lost -> stop
            Motor_SetSpeedsPID(0, 0);
            if ((debugCounter % 10) == 0)
                printf("[LINE] Line lost (ones=0)\r\n");
            continue;
        }

        if (ones >= 4) {
            // Full black / intersection -> go straight
            Motor_SetSpeedsPID(BASE_SPEED, BASE_SPEED);
            if ((debugCounter % 10) == 0)
                printf("[LINE] Full black -> straight\r\n");
            continue;
        }

        // Normal PID control
        float pos = (float)sum / (float)ones;
        float err = pos;
        float corr = PID_Compute(&linePID, err, LINE_DT_SEC);
        int16_t corrP = (int16_t)(corr * kOut);

        int16_t left  = BASE_SPEED - corrP;
        int16_t right = BASE_SPEED + corrP;

        // Clamp to [0..100]
        if (left  < 0)   left  = 0;
        if (left  > 100) left  = 100;
        if (right < 0)   right = 0;
        if (right > 100) right = 100;

        Motor_SetSpeedsPID(left, right);

        if ((debugCounter % 10) == 0) {
            printf("[LINE] pos=%.2f err=%.2f corr=%d L=%d R=%d\r\n",
                   pos, err, corrP, left, right);
        }
    }
}

// ==== OBSTACLE DETECTION TASK ============================================
void vObstacleDetectionTask(void *pvParameters)
{
    printf("[TASK] Obstacle Detect task Started!\r\n");

    static uint32_t dbgCnt = 0;
    MotorCommand_t stopCmd    = MOTOR_STOP;
    MotorCommand_t turnRight  = MOTOR_SHARP_RIGHT;
    MotorCommand_t turnLeft   = MOTOR_SHARP_LEFT;
    MotorCommand_t forwardCmd = MOTOR_FORWARD;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // Run every OBST_PERIOD_TICKS
        vTaskDelayUntil(&xLastWakeTime, OBST_PERIOD_TICKS);

        // Take semaphore briefly to read the sensor
        if (xSemaphoreTake(xUltrasonicSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            distance = ReadUltrasonic();
               dbgCnt++;

               // Print distance every 10th measurement (~2 s if period 200 ms)
               if (dbgCnt % 10 == 0)
               {
                   printf("\n[OBSTACLE] Distance = %lu cm\r\n", distance);
               }
            if (distance > 0 && distance < OBSTACLE_THRESHOLD && !obstacleDetected)
            {
                obstacleDetected = 1;
                xQueueSend(xMotorQueue, &stopCmd, 0);
                printf("[OBSTACLE] DETECTED at %lu cm - STOP\r\n", distance);

                // Simple avoidance – all via MotorControl task
                xQueueSend(xMotorQueue, &turnRight, 0);
                vTaskDelay(pdMS_TO_TICKS(400));

                xQueueSend(xMotorQueue, &forwardCmd, 0);
                vTaskDelay(pdMS_TO_TICKS(600));

                xQueueSend(xMotorQueue, &turnLeft, 0);
                vTaskDelay(pdMS_TO_TICKS(400));

                xQueueSend(xMotorQueue, &forwardCmd, 0);
                vTaskDelay(pdMS_TO_TICKS(400));

                obstacleDetected = 0;
                printf("[OBSTACLE] Avoidance complete\r\n");
            }

            xSemaphoreGive(xUltrasonicSemaphore);
        }

//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==== MOTOR CONTROL TASK ==================================================
void vMotorControlTask(void *pvParameters)
{
    printf("[TASK] Motor Control Started!\r\n");

    MotorCommand_t command;

    for (;;)
    {
        // BLOCKED until some task sends a command
        if (xQueueReceive(xMotorQueue, &command, portMAX_DELAY) == pdPASS) {
            MotorControl(command, BASE_SPEED);
        }
    }
}

// ==== SYSTEM MONITOR TASK ================================================
void vSystemMonitorTask(void *pvParameters)
{
	printf("[TASK] System Monitor Started!\r\n");

	    for (;;)
	    {
	        uint32_t freeHeap       = xPortGetFreeHeapSize();
	        UBaseType_t stackFree   = uxTaskGetStackHighWaterMark(NULL);
//	        uint32_t qItems         = (uint32_t)uxQueueMessagesWaiting(xMotorQueue);
	        printf("\r\n");
	        printf("╔═══════════════════════════════════════╗\r\n");
	        printf("║        SYSTEM MONITOR REPORT          ║\r\n");
	        printf("╠═══════════════════════════════════════╣\r\n");
	        printf("║ Free Heap      : %-6lu bytes          ║\r\n", freeHeap);
	        printf("║ Stack Free     : %-6lu words          ║\r\n", stackFree);
	        printf("║ Obstacle Status: %-6s                 ║\r\n",
	               obstacleDetected ? "DETECT" : "CLEAR");
	        printf("║ Distance       : %-6lu cm             ║\r\n", distance);
	        printf("╠═══════════════════════════════════════╣\r\n");
	        printf("║ IR Sensors: [%d][%d][%d][%d][%d]      ║\r\n",
	               irSensorValues[0], irSensorValues[1], irSensorValues[2],
	               irSensorValues[3], irSensorValues[4]);
	        printf("╚═══════════════════════════════════════╝\r\n\r\n");

	        vTaskDelay(pdMS_TO_TICKS(5000));   // run every 5 s
	    }
}
void vSensorTimerCallback(TimerHandle_t xTimer)
{
	 Ultrasonic_Trigger();                   // send ping every period
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // debug LED
}
