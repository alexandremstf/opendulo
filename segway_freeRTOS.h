#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "MPU9250.h"
#include "Kalman.h"


MPU9250 mpu1;
MPU9250 mpu2;

float accelX, accelY, accelZ;
float gyroY;
float pitch;
float gyroYrate, gyroYrateComp;
float rad_to_reg = 180 / 3.141592654;

Kalman kalmanY;

double gyroYangle;
double compAngleY;
double kalAngleY;

const byte AIN2 = 4;
const byte AIN1 = 5;
const byte STBY = 6;
const byte BIN1 = 7;
const byte BIN2 = 8;
const byte PWM_MOTOR1 = 11;
const byte PWM_MOTOR2 = 3;

// Timer
float now_time;
float pas_time;
float dif_time;

// PID
float kp = 6;// 10.1 18
float ki = 0;// 0.3
float kd = 0.5;// 9.3 16
float kp_error = 0.0;
float ki_error = 0.0;
float kd_error = 0.0;
float kp_pass_error = 0.0;
float kp_result = 0;
float ki_result = 0;
float kd_result = 0;
float final_result = 0;
float reference_angle = 71.0;
