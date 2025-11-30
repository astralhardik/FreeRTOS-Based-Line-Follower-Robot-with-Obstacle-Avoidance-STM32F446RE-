/*
 * debug_uart.c
 *
 *  Created on: Nov 27, 2025
 *      Author: hardik
 */


#include "main.h"
#include "robot_tasks.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len)
{
    if (xUARTMutex != NULL && xPortIsInsideInterrupt() == pdFALSE) {
        xSemaphoreTake(xUARTMutex, portMAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
        xSemaphoreGive(xUARTMutex);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    }
    return len;
}
