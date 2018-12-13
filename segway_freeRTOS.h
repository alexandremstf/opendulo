#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "MPU9250.h"
#include "Kalman.h"


MPU9250 mpu1;
MPU9250 mpu2;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;
float gyroYrate, gyroYrateComp;
float rad_to_reg = 180 / 3.141592654;

Kalman kalmanX;
Kalman kalmanY;

double gyroXangle, gyroYangle;
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;
double corrected_x, corrected_y;

const byte AIN2 = 4;
const byte AIN1 = 5;
const byte STBY = 6;
const byte BIN1 = 7;
const byte BIN2 = 8;
const byte PWM_MOTOR1 = 9;
const byte PWM_MOTOR2 = 10;

// Timer
float now_time;
float pas_time;
float dif_time;

float now_time1;
float pas_time1;
float dif_time1;

// PID
float kp = 22;// 10.1 18
float ki = 0.4;// 0.3
float kd = 20;// 9.3 16
float kp_error = 0.0;
float ki_error = 0.0;
float kd_error = 0.0;
float kp_pass_error = 0.0;
float kp_result = 0;
float ki_result = 0;
float kd_result = 0;
float final_result = 0;

// Special angle
float overshoot_angle = 30;
float PID_angle = 8;
float reference_angle = 0.0;
