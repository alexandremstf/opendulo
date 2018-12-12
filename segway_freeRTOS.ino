#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include "SparkFunMPU9250-DMP.h"
#include "I2Cdev.h"
#include "Kalman.h"

// MPU9250
MPU9250_DMP imu_9250;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;
float gyroXrate, gyroYrate;
float rad_to_reg = 180 / 3.141592654;

// Kalman Filter
Kalman kalmanX;
Kalman kalmanY;
double gyroXangle, gyroYangle; // Gyroscope angle
double compAngleX, compAngleY; // Complementary filter angle
double kalAngleX, kalAngleY; // Angle after Kalman filter
double corrected_x, corrected_y; // Corrected with offset
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

void UpdateIMUData(void)
{
  accelX = imu_9250.calcAccel(imu_9250.ax);
  accelY = imu_9250.calcAccel(imu_9250.ay);
  accelZ = imu_9250.calcAccel(imu_9250.az);
  gyroX = imu_9250.calcGyro(imu_9250.gx);
  gyroY = imu_9250.calcGyro(imu_9250.gy);
  gyroZ = imu_9250.calcGyro(imu_9250.gz);

  // Convert to deg/s
  roll = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * rad_to_reg;
  pitch = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
  gyroXrate = gyroX / 131.0;
  gyroYrate = gyroY / 131.0;
}

float pid_control() { // ONLY PD RIGHT NOW
  kp_error = kalAngleY - reference_angle;

  // If the car is about to fall down, adjust it quickly
  if (kp_error >= overshoot_angle && kp_error <= -overshoot_angle) {kp = 40;}
  else {kp = 22;}

  ki_error += kp_error * dif_time;
  kd_error = (kp_error - kp_pass_error) / dif_time;
  kp_result = kp_error * kp;
  ki_result = ki_error * ki;
  kd_result = kd_error * kd;
  kp_pass_error = kp_error;
  final_result = kp_result + kd_result;

  // PID only invoided when angle is small
  if (kp_error <= PID_angle && kp_error >= -PID_angle) {
    final_result = kp_result + kd_result + ki_result;
  }

  return final_result;
}

void kalman() {
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dif_time); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dif_time);
  gyroXangle += gyroXrate * dif_time; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dif_time;
  
  Serial.print("X Angle: ");
  Serial.print(kalAngleX);
  Serial.print(" Y Angle: ");
  Serial.println(kalAngleY);  
  
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

int pwm1 = 0;
int pwm2 = 0;

// Declare a mutex Semaphore Handle which we will use to manage the sensor data.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t angleSemaphore;

// define two Tasks for Control & Sensor Read
void control( void *pvParameters );
void readSensor( void *pvParameters );

void setup() {  
  Serial.begin(9600);

  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWM_MOTOR2, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);

  if ( angleSemaphore == NULL )  // Check to confirm that the Angle Semaphore has not already been created.
  {
    angleSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Angle
    if ( ( angleSemaphore ) != NULL )
      xSemaphoreGive( ( angleSemaphore ) );  // Make the angle available for use, by "Giving" the Semaphore.
  }

  xTaskCreate(control, (const portCHAR *)"Control", 128, NULL, 2, NULL );
  xTaskCreate(readSensor, (const portCHAR *)"Control", 128, NULL, 1, NULL );

  // MPU-9250
  if (imu_9250.begin() != INV_SUCCESS)
  {
    imu_9250.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);// Enable all sensors:
    imu_9250.setGyroFSR(2000); // Set gyro to 2000 dps
    imu_9250.setAccelFSR(2); // Set accel to +/-2g
    imu_9250.setLPF(5); // Set LPF corner frequency to 5Hz
    imu_9250.setSampleRate(10); // Set sample rate to 10Hz
  }

  // Timer
  pas_time = millis();
}

void loop() {
  // calculate time
  now_time = millis();
  dif_time = (now_time - pas_time) / 1000; // in seconds. We work in ms so we haveto divide the value by 1000
  pas_time = now_time;

  // Update IMU data
  if ( imu_9250.dataReady() )
  {
    imu_9250.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    UpdateIMUData();
    kalman();
  }
}

void control(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    forward();
  
    OCR1A = pwm2;
    OCR1B = pwm1;
    
    vTaskDelay( 1000 / portTICK_PERIOD_MS );

    // See if we can obtain or "Take" the Angle Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // getAngle
      xSemaphoreGive( angleSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
}

void readSensor(void *pvParameters){
  (void) pvParameters;

  for (;;) {
    vTaskDelay(1);

    // See if we can obtain or "Take" the Angle Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // setAngle
      xSemaphoreGive( angleSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
}

void forward() {
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    digitalWrite(STBY, HIGH);
}

void back() {
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    digitalWrite(STBY, HIGH);
}

void stopped() {
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    digitalWrite(STBY, HIGH);
}
