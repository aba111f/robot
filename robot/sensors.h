#pragma once

struct SensorData {
  float temperatureC;
  float humidity;
  bool  dhtOk;
  int   gasMv;       // напряжение на выходе MQ5, мВ (до делителя)
  int   gasPercent;  // относительный уровень 0..100
  bool  gasWarm;     // прогрев завершён?
  int   soundMv;     // пик за окно, мВ
  int   soundPercent;
};

void sensorsInit();
void sensorsUpdate();     // вызывать в loop()
SensorData sensorsGet();