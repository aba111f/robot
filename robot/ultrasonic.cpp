#include <Arduino.h>
#include "config.h"
#include "ultrasonic.h"

void ultrasonicInit() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

float ultrasonicRead() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long t = pulseIn(ECHO_PIN, HIGH, 20000);
  if (t == 0) return 0;
  return t * 0.0343f / 2.0f;
}

float ultrasonicScanRead() {
  float sum = 0;
  int n = 0;
  for (int i = 0; i < 3; i++) {
    float d = ultrasonicRead();
    if (d > 2.0f) { sum += d; n++; }
    delay(25);
    yield();
  }
  if (n == 0) return NO_ECHO_CM;
  return sum / n;
}