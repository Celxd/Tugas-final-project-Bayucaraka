#include <Arduino.h>
#include <ESP32Servo.h>

#define PIN_SERVO_LIFT 18
#define PIN_SERVO_GRIPPER 19
#define PIN_STEP_X 33
#define PIN_DIR_X 25
#define PIN_STEP_Y 27
#define PIN_DIR_Y 26
#define PIN_ENABLE 14

Servo servoGripper;
Servo servoLift;

const float stepsPerRevolution = 1600;
const float maxX = 41.4;
const float maxY = 45.4;
const float minReachX = 8;
const float minStartY = maxY - 8;
const float maxReachY = maxY - 8;
const float maxRevX = 7;
const float maxRevY = 7.275;
const float stepsPerCmX = (stepsPerRevolution * maxRevX) / maxX;
const float stepsPerCmY = (stepsPerRevolution * maxRevY) / maxY;
float currentX = 0;
float currentY = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_STEP_X, OUTPUT);
  pinMode(PIN_DIR_X, OUTPUT);
  pinMode(PIN_STEP_Y, OUTPUT);
  pinMode(PIN_DIR_Y, OUTPUT);
  pinMode(PIN_SERVO_GRIPPER, OUTPUT);
  pinMode(PIN_SERVO_LIFT, OUTPUT);

  digitalWrite(PIN_ENABLE, LOW);

  servoGripper.attach(PIN_SERVO_GRIPPER);
  servoLift.attach(PIN_SERVO_LIFT);

  servoGripper.write(80);
  servoLift.write(0);

  moveToTarget(8, 8);
}

void loop() {
  naikturun(true);
  capit(true);
  delay(1000);

  naikturun(false);
  capit(false);
  delay(1000);
}

void moveToTarget(float x, float y) {
  float deltaX = x - currentX;
  int stepsX = abs(deltaX * stepsPerCmX);

  digitalWrite(PIN_DIR_X, deltaX > 0 ? HIGH : LOW);

  for (int i = 0; i < stepsX; i++) {
    digitalWrite(PIN_STEP_X, HIGH);
    delayMicroseconds(750);
    digitalWrite(PIN_STEP_X, LOW);
    delayMicroseconds(750);
  }

  currentX = x;

  float deltaY = y - currentY;
  int stepsY = abs(deltaY * stepsPerCmY);

  digitalWrite(PIN_DIR_Y, deltaY > 0 ? LOW : HIGH);

  for (int i = 0; i < stepsY; i++) {
    digitalWrite(PIN_STEP_Y, HIGH);
    delayMicroseconds(750);
    digitalWrite(PIN_STEP_Y, LOW);
    delayMicroseconds(750);
  }

  currentY = y;
}

void arahX(bool direction) {
  digitalWrite(PIN_DIR_X, direction ? HIGH : LOW);

  for (long i = 0; i < stepsPerRevolution * maxRevX; i++) {
    digitalWrite(PIN_STEP_X, HIGH);
    delayMicroseconds(750);
    digitalWrite(PIN_STEP_X, LOW);
    delayMicroseconds(750);
  }
}

void arahY(bool direction) {
  digitalWrite(PIN_DIR_Y, direction ? LOW : HIGH);

  for (long i = 0; i < stepsPerRevolution * maxRevY; i++) {
    digitalWrite(PIN_STEP_Y, HIGH);
    delayMicroseconds(750);
    digitalWrite(PIN_STEP_Y, LOW);
    delayMicroseconds(750);
  }
}

void capit(bool closeGrip) {
  int openAngle = 80;
  int closeAngle = 140;

  if (closeGrip) {
    for (int i = openAngle; i <= closeAngle; i++) {
      servoGripper.write(i);
      delay(5);
    }
  } else {
    for (int i = closeAngle; i >= openAngle; i--) {
      servoGripper.write(i);
      delay(5);
    }
  }

  delay(10);
}

void naikturun(bool liftUp) {
  int upAngle = 180;
  int downAngle = 0;

  if (liftUp) {
    for (int i = downAngle; i <= upAngle; i++) {
      servoLift.write(i);
      delay(5);
    }
  } else {
    for (int i = upAngle; i >= downAngle; i--) {
      servoLift.write(i);
      delay(5);
    }
  }

  delay(10);
}