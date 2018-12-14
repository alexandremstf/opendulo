#include "segway_freeRTOS.h"

void setup() {  
  Serial.begin(115200);

  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWM_MOTOR2, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);

  mpu1.init(0x68, 0x0C);
  mpu2.init(0x69, 0x0C);

  if ( angleSemaphore == NULL ) {
    angleSemaphore = xSemaphoreCreateBinary();  // Create a binary semaphore we will use to manage the Angle
    if ( ( angleSemaphore ) != NULL )
      xSemaphoreGive( ( angleSemaphore ) );  // Make the angle available for use, by "Giving" the Semaphore.
  }

  xTaskCreate(readSensor, (const portCHAR *)"readSensor", 128, NULL, 1, NULL );
  xTaskCreate(control, (const portCHAR *)"Control", 128, NULL, 1, NULL );
}

void loop() {
//  mpu1.read();
//  mpu2.read();
//  
//  angle_robot = calculateAngle();
//
//  float controlSignal = pid(angle_robot, 0);
//      
//  if (controlSignal > 0) forward();
//  if (controlSignal < 0) back();
//
//  analogWrite(PWM_MOTOR1, abs(controlSignal));
//  analogWrite(PWM_MOTOR2, abs(controlSignal));
}

void control(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  
  xLastWakeTime = xTaskGetTickCount();

  int controlSignal = 0;
  
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 2 ) == pdTRUE ){
   
      controlSignal = pid(angle_robot, -2.65 );
        
      if (controlSignal > 0) forward();
      if (controlSignal < 0) back();

      analogWrite(PWM_MOTOR1, abs(controlSignal));
      analogWrite(PWM_MOTOR2, abs(controlSignal));
      
      xSemaphoreGive( angleSemaphore ); // signal
    }
  }
}

void readSensor(void *pvParameters){
  (void) pvParameters;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
      
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 2 ) == pdTRUE ){
      
      mpu1.read();
      mpu2.read();
      
      angle_robot = calculateAngle();
      
      xSemaphoreGive( angleSemaphore ); // signal
    }
  }
}

float calculateAngle() {
  // cálculo do dt para achar o ângulo
  float t = millis();
  float dt = (t - last_angle_time) / 1000;
  last_angle_time = t;

  // média entre leituras dos sensores
  float accel_x = ((mpu1.getAccelerationX() + mpu2.getAccelerationX())/2);
  float accel_y = ((mpu1.getAccelerationY() + mpu2.getAccelerationY())/2);
  float accel_z = ((mpu1.getAccelerationZ() + mpu2.getAccelerationZ())/2);
  float gyro_y = ((mpu1.getGyroscopeY() + mpu2.getGyroscopeY())/2);
  
  float pitch = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * (180.0 / 3.141592);

  //float angle_y = kalman.getAngle(pitch, gyro_y/131.0, dt);

  float angle_y = 0.95 * (last_angle_y + gyro_y * dt) + 0.05 * pitch; 
  last_angle_y = angle_y;

  //Serial.println(angle_y);
  
  return angle_y;
}

float pid(float setpoint, float input) {

  // cálculo do dt para controle
  float t = millis();
  float dt = (t - last_pid_time) / 1000;
  last_pid_time = t;

  float error = (setpoint - input);
  float derivative = (error - last_error) / dt;
  
  integral += error * dt;  
  last_error = error;

  if (integral > 5) integral = 5;
  if (integral < -5) integral = -5;
 
  float output = KP * (error) + KI * (integral) + KD * (derivative);
  
  float max = 100;
  if (output > max) output = max;
  if (output < -max) output = -max;

  return output;
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
