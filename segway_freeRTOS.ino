#include "segway_freeRTOS.h"

SemaphoreHandle_t angleSemaphore;

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
  
  // Timer
  pas_time = millis();
}

void loop() {
}

void control(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  
  xLastWakeTime = xTaskGetTickCount();

  int controlSignal = 0;
  
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 1 ) == pdTRUE ){
      
      controlSignal = pid_control();
      
      if(compAngleY > 70){
        forward();
      }else{
        back();
      }

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
      
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 1 ) == pdTRUE ){
      now_time = millis();
      dif_time = (now_time - pas_time) / 1000;
      pas_time = now_time;
      
      mpu1.read();
      mpu2.read();
      
      updateValues();
      kalman();

      //Serial.println(compAngleY);
      xSemaphoreGive( angleSemaphore ); // signal
    }
  }
}

void updateValues(void) {
  accelX = ((mpu1.getAccelerationX() + mpu2.getAccelerationX())/2);
  accelY = ((mpu1.getAccelerationY() + mpu2.getAccelerationY())/2);
  accelZ = ((mpu1.getAccelerationZ() + mpu2.getAccelerationZ())/2);
  gyroY = ((mpu1.getGyroscopeY() + mpu2.getGyroscopeY())/2);

  pitch = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
  gyroYrate = gyroY / 131.0;
  gyroYrateComp = gyroYrate;
}

void kalman() {
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dif_time);
  gyroYangle += gyroYrate * dif_time;
  compAngleY = 0.5 * (compAngleY + gyroYrateComp * dif_time) + 0.5 * pitch;  
}

float pid_control() { // ONLY PD RIGHT NOW
  kp_error = compAngleY - reference_angle;

  ki_error += kp_error * dif_time;
  kd_error = (kp_error - kp_pass_error) / dif_time;
  kp_result = kp_error * kp;
  ki_result = ki_error * ki;
  kd_result = kd_error * kd;
  kp_pass_error = kp_error;
  final_result = kp_result + kd_result;

  float max = 100;
  if(final_result > max){
    final_result = max;
  }
  if(final_result < -max){
    final_result = -max;
  }
  
  return final_result;
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
