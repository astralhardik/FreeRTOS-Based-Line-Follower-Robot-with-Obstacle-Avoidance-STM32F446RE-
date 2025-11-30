/*
 * sensor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: Hardik Agrawal
 *      Roll No= 22UEC045
 */

#include <stdio.h>
#include "sensors.h"
#include "FreeRTOS.h"
#include "task.h"

#define ULTRASONIC_TRIG_PIN   GPIO_PIN_6
#define ULTRASONIC_TRIG_PORT  GPIOB

extern TIM_HandleTypeDef htim4;

uint8_t  irSensorValues[5] = {0};
volatile uint32_t icValue1 = 0;
volatile uint32_t icValue2 = 0;
volatile uint8_t  captureFlag = 0;
volatile uint32_t distance = 0;
volatile uint8_t  obstacleDetected = 0;


static void ul_delay_us(uint32_t us)
{
    volatile uint32_t count = us * 60U;  // ~10 µs at 180 MHz (rough)
    while (count--)
    {
        __NOP();
    }
}


void ReadIRSensors(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    irSensorValues[IR_LEFT_MOST]  = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    irSensorValues[IR_LEFT]       = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
    irSensorValues[IR_CENTER]     = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
    irSensorValues[IR_RIGHT]      = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
    irSensorValues[IR_RIGHT_MOST] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
}

/* ----------------------------------------------------------------------
 * Ultrasonic_Trigger
 *   - Generates a short pulse on TRIG pin
 *   - Arms TIM4 CH2 capture interrupt and resets state
 *   - Can safely be called from the FreeRTOS timer callback.
 * --------------------------------------------------------------------*/
void Ultrasonic_Trigger(void)
{
    // Prepare capture state
    captureFlag = 0;
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);

    // Generate ~10 µs trigger pulse on PB6
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
    ul_delay_us(2);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_SET);
    ul_delay_us(10);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------------
 * ReadUltrasonic
 *   - Assumes Ultrasonic_Trigger() was called shortly before.
 *   - Waits (max 50 ms) for captureFlag == 2 and then computes distance.
 *   - Returns 999 on timeout (no echo / no obstacle).
 * --------------------------------------------------------------------*/
uint32_t ReadUltrasonic(void)
{
    uint32_t localDistance = 0;
    TickType_t startTick = xTaskGetTickCount();

    // Wait for echo with timeout
    while (captureFlag != 2 &&
           (xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(50))
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (captureFlag == 2)
    {
        uint32_t timeDiff;
        if (icValue2 > icValue1)
            timeDiff = icValue2 - icValue1;
        else
            timeDiff = (0xFFFFu - icValue1) + icValue2;

        // timeDiff in timer ticks, prescaler 89 => 1 MHz => 1 µs per tick
        // distance (cm) = (time_us * speed_of_sound_cm_per_us) / 2
        // speed_of_sound ≈ 0.0343 cm/us => multiply by 343 and divide by 10000
        localDistance = (timeDiff * 343u) / (2u * 10000u);
    }
    else
    {
        // Timeout => no echo
        localDistance = 999;
    }

    // Ready for next measurement
    captureFlag = 0;
    return localDistance;
}

/* ----------------------------------------------------------------------
 * Ultrasonic input capture ISR callback
 *   - Called from HAL_TIM_IRQHandler (TIM4)
 *   - Captures rising and falling edges and signals completion via
 *     captureFlag == 2.
 * --------------------------------------------------------------------*/
void Ultrasonic_IC_Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (captureFlag == 0)
        {
            icValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            captureFlag = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (captureFlag == 1)
        {
            icValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_COUNTER(htim, 0);
            captureFlag = 2;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
        }
    }
}
