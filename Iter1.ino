
#include <Arduino.h>
#include <QTRSensors.h>

// =============================
// PINI
// =============================
const uint8_t LEFT_PWM  = 9;
const uint8_t LEFT_IN1  = 2;
const uint8_t LEFT_IN2  = 4;

const uint8_t RIGHT_PWM = 10;
const uint8_t RIGHT_IN1 = 7;
const uint8_t RIGHT_IN2 = 8;

// =============================
// SENZORI
// =============================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const int16_t SETPOINT = 3500;

// =============================
// PD SIMPLU
// =============================
float Kp = 0.10;
float Kd = 0.55;

int baseSpeed = 140;
int maxSpeed  = 255;

// =============================
// VARIABILE CONTROL
// =============================
int16_t lastError = 0;
int8_t lastDirection = 1;   // 1 = dreapta, -1 = stanga

// =============================
// MOTOR
// =============================
void setLeftMotor(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue >= 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_PWM, speedValue);
  } else {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, -speedValue);
  }
}

void setRightMotor(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue >= 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, speedValue);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_PWM, -speedValue);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

// =============================
// UTILE
// =============================
bool lineLost() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    sum += sensorValues[i];
  }
  return sum < 200;
}

void searchLine() {
  if (lastDirection > 0) {
    drive(110, -70);
  } else {
    drive(-70, 110);
  }
}

// =============================
// SETUP
// =============================
void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  ADCSRA = (ADCSRA & ~0x07) | 0x06;

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
}

// =============================
// LOOP
// =============================
void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int16_t error = (int16_t)position - SETPOINT;

  if (error > 50) lastDirection = 1;
  else if (error < -50) lastDirection = -1;

  if (lineLost()) {
    searchLine();
    return;
  }

  // HARD TURN pentru viraje foarte stranse / varf de W
  if (error > 2000) {
    drive(185, -100);
    lastError = error;
    return;
  }

  if (error < -2200) {
    drive(-90, 170);
    lastError = error;
    return;
  }

  // PD
  float P = error;
  float D = error - lastError;

  float correction = Kp * P + Kd * D;

  // viteza de baza dinamica
  int dynamicBase = baseSpeed - abs(error) / 35;
  if (dynamicBase < 85) dynamicBase = 85;

  int leftSpeed  = dynamicBase + (int)correction;
  int rightSpeed = dynamicBase - (int)correction;

  // fara reverse in urmarirea normala
  if (leftSpeed < 0) leftSpeed = 0;
  if (rightSpeed < 0) rightSpeed = 0;

  if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
  if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;

  drive(leftSpeed, rightSpeed);

  lastError = error;
}