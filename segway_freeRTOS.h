#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "MPU9250.h"
#include "Kalman.h"

#define AIN2 4
#define AIN1 5
#define STBY 6
#define BIN1 7
#define BIN2 8
#define PWM_MOTOR1 11
#define PWM_MOTOR2 3

#define KP 5.0
#define KI 0.0
#define KD 0.0

SemaphoreHandle_t angleSemaphore;

Kalman kalman;
MPU9250 mpu1;
MPU9250 mpu2;

float integral = 0.0;
float last_error = 0.0;

float last_pid_time = 0.0;
float last_angle_time = 0.0;

float last_angle_x = 0.0;
float last_angle_y = 0.0;

float angle_robot = 0.0;
