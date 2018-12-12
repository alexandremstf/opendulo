#include <Arduino_FreeRTOS.h>

const byte AIN2 = 4;
const byte AIN1 = 5;
const byte STBY = 6;
const byte BIN1 = 7;
const byte BIN2 = 8;
const byte PWM_MOTOR1 = 9;
const byte PWM_MOTOR2 = 10;

int pwm1 = 0;
int pwm2 = 0;

void setup() {  
  Serial.begin(9600);

  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWM_MOTOR2, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);

  xTaskCreate(control, (const portCHAR *)"Control", 128, NULL, 2, NULL );
  xTaskCreate(readSensor, (const portCHAR *)"Control", 128, NULL, 1, NULL );
}

void loop() {
  // Empty. Things are done in Tasks.
}

void control(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    forward();
  
    OCR1A = pwm2;
    OCR1B = pwm1;
    
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}

void readSensor(void *pvParameters){
  (void) pvParameters;

  for (;;) {
    vTaskDelay(1);
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
