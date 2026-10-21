#include <Arduino.h>
#include <DHT.h>
#include "config.h"
#include "sensors.h"

static DHT dht(DHT_PIN, DHT22);
static SensorData data = {NAN, NAN, false, 0, 0, false, 0, 0};

static unsigned long lastDht = 0, lastGas = 0, winStart = 0;
static int winMax = 0;

static int readMv(int pin) {
  return (int)(analogReadMilliVolts(pin) * ADC_DIVIDER);
}

void sensorsInit() {
  dht.begin();
  pinMode(MQ5_PIN, INPUT);
  pinMode(SOUND_PIN, INPUT);
  winStart = millis();
}

void sensorsUpdate() {
  unsigned long now = millis();

  // --- DHT22: раз в 2.5 с ---
  if (now - lastDht >= DHT_INTERVAL_MS) {
    lastDht = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      data.humidity = h;
      data.temperatureC = t;
      data.dhtOk = true;
    } else {
      data.dhtOk = false;
    }
  }

  // --- MQ5: раз в 500 мс, среднее из 8 замеров ---
  if (now - lastGas >= 500) {
    lastGas = now;
    long sum = 0;
    for (int i = 0; i < 8; i++) sum += readMv(MQ5_PIN);
    data.gasMv = sum / 8;
    data.gasPercent = constrain(map(data.gasMv, 0, 5000, 0, 100), 0, 100);
    data.gasWarm = now >= MQ5_WARMUP_MS;
  }

  // --- Звук: пик за окно 100 мс (без блокировки loop) ---
  int v = readMv(SOUND_PIN);
  if (v > winMax) winMax = v;
  if (now - winStart >= SOUND_WINDOW_MS) {
    data.soundMv = winMax;
    data.soundPercent = constrain(map(winMax, 0, 5000, 0, 100), 0, 100);
    winMax = 0;
    winStart = now;
  }
}

SensorData sensorsGet() { return data; }