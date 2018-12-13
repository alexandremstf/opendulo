#include "segway_freeRTOS.h"

void UpdateIMUData(void) {
  accelX = ((mpu1.getAccelerationX() + mpu2.getAccelerationX())/2);
  accelY = ((mpu1.getAccelerationY() + mpu2.getAccelerationY())/2);
  accelZ = ((mpu1.getAccelerationZ() + mpu2.getAccelerationZ())/2);
  gyroY = ((mpu1.getGyroscopeY() + mpu2.getGyroscopeY())/2);

  // Convert to deg/s
  pitch = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
  gyroYrate = gyroY / 131.0;
  gyroYrateComp = gyroYrate;
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
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dif_time);
  gyroYangle += gyroYrate * dif_time;
  compAngleY = 0.5 * (compAngleY + gyroYrateComp * dif_time) + 0.5 * pitch;
  
  Serial.print("Kalman: ");
  Serial.print(kalAngleY);
  Serial.print("- Comp: ");
  Serial.println(compAngleY);   
}

int pwm1 = 0;
int pwm2 = 0;

// Declare a mutex Semaphore Handle which we will use to manage the sensor data.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t angleSemaphore;

//
//Define two Tasks for Control & Sensor Read
void control( void *pvParameters );
void readSensor( void *pvParameters );

void setup() {  
  Serial.begin(115000);

  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWM_MOTOR2, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);

  mpu1.init(0x68, 0x0C);
  mpu2.init(0x69, 0x0C);

  if ( angleSemaphore == NULL )  // Check to confirm that the Angle Semaphore has not already been created.
  {
    angleSemaphore = xSemaphoreCreateBinary();  // Create a binary semaphore we will use to manage the Angle
    if ( ( angleSemaphore ) != NULL )
      xSemaphoreGive( ( angleSemaphore ) );  // Make the angle available for use, by "Giving" the Semaphore.
  }

  xTaskCreate(readSensor, (const portCHAR *)"readSensor", 128, NULL, 1, NULL ); // Higher frequency, lower priority
  xTaskCreate(control, (const portCHAR *)"Control", 128, NULL, 3, NULL ); // Higher priority, lower frequency
  

  // Timer
  pas_time = millis();
  pas_time1 = millis();
}

void loop() {
}

void control(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2;
  //pdMS_TO_TICKS(100) Converte tempo em ms para ticks
  // 1 TICK = 16ms

  //Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    
    //forward();

    //analogWrite(PWM_MOTOR1, pwm1);
    //analogWrite(PWM_MOTOR2, pwm2);
  

    // See if we can obtain or "Take" the Angle Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 1 ) == pdTRUE )
    {
      Serial.println(" Controle: ");
      now_time1 = millis();
      dif_time1 = (now_time1 - pas_time1) / 1000;
      Serial.println(dif_time1);
      pas_time1 = now_time1;
      //Serial.print(pid_control());
      xSemaphoreGive( angleSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
}

void readSensor(void *pvParameters){
  (void) pvParameters;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  //pdMS_TO_TICKS(100) Converte tempo em ms para ticks
  // 1 TICK = 16ms

  //Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
      
    // See if we can obtain or "Take" the Angle Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    //Serial.println("Fora");
        
    if ( xSemaphoreTake( angleSemaphore, ( TickType_t ) 1 ) == pdTRUE )
    {
      Serial.println(" Leitura: ");
      now_time = millis();
      dif_time = (now_time - pas_time) / 1000;
      Serial.println(dif_time);
      pas_time = now_time;
////    
////      mpu1.read();
////      mpu2.read();
////      UpdateIMUData();
////      kalman();
//
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
