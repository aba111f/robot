#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "scanner.h"

static Servo scanServo;
static int servoAngle = SERVO_CENTER;
static unsigned long settleMs = 0;
static unsigned long moveStart = 0;

void scannerInit() {
  ESP32PWM::allocateTimer(3);
  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(SERVO_CENTER);
  servoAngle = SERVO_CENTER;
}

void scannerMove(int angle) {
  angle = constrain(angle, SERVO_RIGHT, SERVO_LEFT);
  int delta = abs(angle - servoAngle);
  settleMs = 120 + delta * 6;
  servoAngle = angle;
  scanServo.write(angle);          // команда уходит каждый раз
  moveStart = millis();
}

bool scannerReady() {
  return millis() - moveStart >= settleMs;
}

void scannerCenterNow() {
  scanServo.write(SERVO_CENTER);
  servoAngle = SERVO_CENTER;
  moveStart = millis();
  settleMs = 120;
}

int scannerGetAngle() {
    return servoAngle;
}