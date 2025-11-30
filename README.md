FreeRTOS-Based Line Follower Robot with Obstacle Avoidance (STM32F446RE)

Project Overview

This project is a fully functional autonomous line-following robot developed using the STM32F446RE Nucleo board. It integrates FreeRTOS for real-time multitasking, PID control for precise steering, and ultrasonic sensors for automatic obstacle avoidance.

The system demonstrates advanced embedded concepts including hardware abstraction, sensor fusion, inter-task communication (Queues/Semaphores), and real-time system analysis.

Components

Microcontroller: STM32F446RE Nucleo Board

Sensors: 5-Channel IR Sensor Array, HC-SR04 Ultrasonic Sensor

Actuators: 2x Geared DC Motors with L298N Driver

Power: Li-ion Battery Pack

Chassis: 2-Wheel Drive BO Platform

Project Structure

Core/ - STM32 application code (Src/Inc).

Drivers/ & Middlewares/ - HAL drivers and FreeRTOS kernel source.

robot_tasks.c / .h - FreeRTOS task handlers.

sensors.c / .h - IR and Ultrasonic sensor modules.

motor.c / .h - DC Motor driver (PWM control).

Trace_file.bin - Tracealyzer log file.

AESD-2025_22UEC045_Project_Report.pdf - Detailed project report.

Visuals

The Robot

Robot Top View

Working Model





Debugging & Analysis

TeraTerm Debug Output

Percepio Tracealyzer





Pin Configuration

Hardware Pin Configuration

IR Sensor Array

Left Most: PC0

Left: PC1

Center: PC2

Right: PC3

Right Most: PC4

Ultrasonic Sensor (HC-SR04)

Trigger: PB6

Echo: PB7 (TIM4_CH2 Input Capture)

Motor Driver (L298N)

Motor A (IN1/IN2): PA10 / PA11 (TIM1 PWM)

Motor B (IN3/IN4): PA8 / PA9 (TIM1 PWM)

UART (Debugging)

TX / RX: PA2 / PA3 (115200 Baud)

Firmware Architecture (FreeRTOS)

The system uses a preemptive scheduling policy with 4 main tasks:

Line Follower Task: Reads IR sensor array, computes weighted position, runs PID correction, and sends PWM commands. Suspends during obstacle avoidance.

Obstacle Detection Task: Periodically reads ultrasonic sensor using hardware timer input capture. If distance < 15 cm, triggers stop/avoid routine.

Motor Control Task: Receives PWM values via queue and updates TIM1 channels for smooth, deterministic control.

System Monitor Task: Background task that prints heap, stack, queue usage, and sensor states.

How the System Works

Line Detection:

Sensor pattern [0 1 1 0 0] → Position = -0.5 (Slightly Left)

Sensor pattern [0 0 1 0 0] → Position = 0 (Centered)

PID Steering:

correction = Kp * error + Kd * (error - prev);
LeftMotor  = base_speed - correction;
RightMotor = base_speed + correction;


Obstacle Handling:

Distance measured via Input Capture.

If < 15 cm, stop motors → wait until clear → resume line following.

How to Run (Build & Flash)

Open STM32CubeIDE: Import existing STM32 project.

Build: Press Ctrl + B to compile.

Connect: Plug in NUCLEO board via USB.

Flash: Run > Debug to upload firmware.

Monitor: Open TeraTerm at 115200 baud to view real-time logs (IR readings, PID values, Obstacle distance).

Future Enhancements

Integration of TFT LCD for status.

Encoder-based closed-loop motor control.

Battery monitoring.

BLE/WiFi remote control.

Machine Learning-based vision line tracking.
