#include <Arduino.h>
#include "config.h"
#include "motors.h"

static int speedPercent = 80;   // 0..100, задаётся ползунком
static int motorSpeed   = 0;    // ШИМ 0..255
static Move currentMove = M_STOP;

static int toDuty(int percent) {          // 0..100 -> 0 или MIN_PWM..255
  if (percent <= 0) return 0;
  return map(percent, 1, 100, MIN_PWM, 255);
}

static void pwmInit(int pin)            { ledcAttach(pin, PWM_FREQ, PWM_RES); }
static void pwmWrite(int pin, int duty) { ledcWrite(pin, duty); }

// l, r: -255..255 (знак = направление)
static void drive(int l, int r) {
  pwmWrite(PIN_L_1, l > 0 ? l : 0);
  pwmWrite(PIN_L_2, l < 0 ? -l : 0);
  pwmWrite(PIN_R_1, r > 0 ? r : 0);
  pwmWrite(PIN_R_2, r < 0 ? -r : 0);
}

void motorsInit() {
  pwmInit(PIN_L_1); pwmInit(PIN_L_2);
  pwmInit(PIN_R_1); pwmInit(PIN_R_2);
  motorSpeed = toDuty(speedPercent);
  drive(0, 0);
}

void motorsForward()  { drive( motorSpeed,  motorSpeed); }
void motorsBackward() { drive(-motorSpeed, -motorSpeed); }
void motorsRight()    { drive( TURN_PWM,   -TURN_PWM);   }
void motorsLeft()     { drive(-TURN_PWM,    TURN_PWM);   }
void motorsStop()     { drive(0, 0); }

void motorsMove(Move m, bool brake) {
  if (brake) { motorsStop(); delay(BRAKE_MS); yield(); }
  currentMove = m;
  switch (m) {
    case M_FORWARD:  motorsForward();  break;
    case M_BACKWARD: motorsBackward(); break;
    case M_LEFT:     motorsLeft();     break;
    case M_RIGHT:    motorsRight();    break;
    default:         motorsStop();     break;
  }
}

void motorsReapply() { motorsMove(currentMove, false); }

int motorsSpeedPercent() { return speedPercent; }

void motorsSetSpeedPercent(int percent) {
  speedPercent = constrain(percent, 0, 100);
  motorSpeed = toDuty(speedPercent);
  Serial.printf(">>> Скорость %d%% = ШИМ %d\n", speedPercent, motorSpeed);
}