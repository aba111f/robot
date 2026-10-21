#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "motors.h"
#include "scanner.h"
#include "ultrasonic.h"
#include "autopilot.h"

enum State { S_FORWARD, S_SCAN, S_BACK, S_TURN };

static const int scanAngles[] = {
  SERVO_LEFT,
  SERVO_CENTER + 35,
  SERVO_CENTER + 18,
  SERVO_CENTER,
  SERVO_CENTER - 18,
  SERVO_CENTER - 35,
  SERVO_RIGHT
};
static const int NUM_ANGLES = sizeof(scanAngles) / sizeof(scanAngles[0]);

static bool  enabled = false;
static State state = S_FORWARD;

static unsigned long stateStart = 0;
static unsigned long lastMeasure = 0;
static unsigned long lastCenterCmd = 0;
static unsigned long turnDuration = TURN_90_MS;
static bool  turnLeftDir = true;
static int   hits = 0;
static int   scanIdx = 0;
static float scanDist[NUM_ANGLES];

// <-- ДОБАВЛЕНЫ ПЕРЕМЕННЫЕ ДЛЯ СТАТУСА
static float lastDistanceCm = 0;
static char lastDecisionStr[64] = "Старт";

static bool angleIsLeft(int angle) {
  return (angle - SERVO_CENTER) * (SERVO_LEFT - SERVO_CENTER) > 0;
}

static float corridorWidthCm(int i) {
  int a = max(0, i - 1);
  int b = min(NUM_ANGLES - 1, i + 1);
  float d = scanDist[a];
  for (int k = a; k <= b; k++) d = min(d, scanDist[k]);
  float spanDeg = abs(scanAngles[b] - scanAngles[a]);
  float spanRad = radians(spanDeg);
  return 2.0f * d * tanf(spanRad / 2.0f);
}

static void setState(State s) {
  state = s;
  stateStart = millis();
  switch (s) {
    case S_FORWARD:
      scannerMove(SERVO_CENTER);
      motorsForward();
      hits = 0;
      lastCenterCmd = millis();
      break;
    case S_SCAN:
      motorsStop();
      scanIdx = 0;
      scannerMove(scanAngles[0]);
      break;
    case S_BACK:
      scannerMove(SERVO_CENTER);
      motorsBackward();
      break;
    case S_TURN:
      scannerMove(SERVO_CENTER);
      if (turnLeftDir) motorsLeft(); else motorsRight();
      break;
  }
}

static void decideDirection() {
  int bestIdx = -1;        
  float bestScore = -1000;
  int farIdx = 0;          

  Serial.println("--- Результат сканирования ---");
  for (int i = 0; i < NUM_ANGLES; i++) {
    float w = corridorWidthCm(i);
    bool fits = (w >= NEED_WIDTH_CM) && (scanDist[i] > STOP_DIST_CM + 10);
    float score = scanDist[i] - 0.25f * abs(scanAngles[i] - SERVO_CENTER);

    Serial.printf("угол %3d (%s): %.1f см, ширина %.1f см  %s\n",
                  scanAngles[i], angleIsLeft(scanAngles[i]) ? "Л" : "П",
                  scanDist[i], w, fits ? "OK" : "узко");

    if (fits && score > bestScore) { bestScore = score; bestIdx = i; }
    if (scanDist[i] > scanDist[farIdx]) farIdx = i;
  }

  if (bestIdx < 0) {
    snprintf(lastDecisionStr, sizeof(lastDecisionStr), "Тупик! Разворот на 180°");
    Serial.println("Нигде не пролезть: назад и разворот");
    turnLeftDir = angleIsLeft(scanAngles[farIdx]);
    turnDuration = TURN_90_MS * 2;
    setState(S_BACK);
    return;
  }

  // <-- bestAngle объявляется ЗДЕСЬ, до использования
  int bestAngle = scanAngles[bestIdx];
  turnLeftDir = angleIsLeft(bestAngle);
  
  snprintf(lastDecisionStr, sizeof(lastDecisionStr), "Поворот на %d° (%.1f см)", bestAngle, scanDist[bestIdx]);
  Serial.printf("Лучшее: угол %d, %.1f см\n", bestAngle, scanDist[bestIdx]);

  turnDuration = (unsigned long)(abs(bestAngle - SERVO_CENTER) / 90.0f * TURN_90_MS);
  if (turnDuration < 100) {
      snprintf(lastDecisionStr, sizeof(lastDecisionStr), "Путь свободен, едем прямо");
      setState(S_FORWARD);
  } else {
      setState(S_TURN);
  }
}

void autoStart() {
  enabled = true;
  snprintf(lastDecisionStr, sizeof(lastDecisionStr), "Автопилот запущен");
  setState(S_FORWARD);
  Serial.println(">>> РЕЖИМ: АВТОНОМНЫЙ");
}

void autoStop() {
  enabled = false;
  snprintf(lastDecisionStr, sizeof(lastDecisionStr), "Ручной режим");
  scannerMove(SERVO_CENTER);
  motorsMove(M_STOP, false);
}

bool autoIsEnabled() { return enabled; }

void autoReapplyMotion() {
  if (!enabled) return;
  switch (state) {
    case S_FORWARD:  motorsForward();  break;
    case S_BACK:     motorsBackward(); break;
    case S_TURN:
      if (turnLeftDir) motorsLeft(); else motorsRight();
      break;
    default: break;
  }
}

void autoUpdate() {
  if (!enabled) return;
  unsigned long now = millis();

  switch (state) {
    case S_FORWARD:
      if (now - lastCenterCmd >= CENTER_REFRESH_MS) {
        lastCenterCmd = now;
        scannerCenterNow();
      }
      if (!scannerReady()) break;                 
      if (now - lastMeasure >= 60) {
        lastMeasure = now;
        float d = ultrasonicRead();
        lastDistanceCm = d; // <-- Обновляем глобальную переменную
        
        if (d > 0 && d < STOP_DIST_CM) hits++;
        else hits = 0;
        
        if (hits >= 2) {
          Serial.printf("Препятствие на %.1f см, сканирую...\n", d);
          snprintf(lastDecisionStr, sizeof(lastDecisionStr), "Обнаружено препятствие, сканирую...");
          setState(S_SCAN);
        }
      }
      break;

    case S_SCAN:
      if (scannerReady()) {
        scanDist[scanIdx] = ultrasonicScanRead();
        lastDistanceCm = scanDist[scanIdx]; // <-- Обновляем для статуса
        scanIdx++;
        if (scanIdx < NUM_ANGLES) scannerMove(scanAngles[scanIdx]);
        else                      decideDirection();
      }
      break;

    case S_BACK:
      if (now - stateStart >= BACK_TIME_MS) setState(S_TURN);
      break;

    case S_TURN:
      if (now - stateStart >= turnDuration) {
        scannerCenterNow();                       
        setState(S_FORWARD);
      }
      break;
  }
}

// <-- ФУНКЦИЯ ПОЛУЧЕНИЯ СТАТУСА
RobotStatus autoGetStatus() {
    RobotStatus status;
    switch (state) {
        case S_FORWARD: strcpy(status.stateName, "ДВИЖЕНИЕ ВПЕРЁД"); break;
        case S_SCAN:    strcpy(status.stateName, "СКАНИРОВАНИЕ"); break;
        case S_BACK:    strcpy(status.stateName, "ДВИЖЕНИЕ НАЗАД"); break;
        case S_TURN:    strcpy(status.stateName, "ПОВОРОТ"); break;
        default:        strcpy(status.stateName, "НЕИЗВЕСТНО");
    }
    
    status.distanceCm = lastDistanceCm;
    status.servoAngle = scannerGetAngle();  
    status.servoReady = scannerReady();
    status.hits = hits;
    status.stateTimeMs = millis() - stateStart;
    strcpy(status.lastDecision, lastDecisionStr);
    status.autoEnabled = enabled;
    
    return status;
}