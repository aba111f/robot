#pragma once

void autoStart();
void autoStop();              // выключает автопилот, ставит серво в центр, останавливает моторы
bool autoIsEnabled();
void autoUpdate();            // вызывать в loop()
void autoReapplyMotion();     // применить новую скорость к текущему движению

// Получение состояния для веб-интерфейса
struct RobotStatus {
    char stateName[20];
    float distanceCm;
    int servoAngle;
    bool servoReady;
    int hits;
    unsigned long stateTimeMs;
    char lastDecision[64];
    bool autoEnabled;
};

RobotStatus autoGetStatus();