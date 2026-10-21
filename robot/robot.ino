// Главный файл: только setup() и loop().
// Папка и этот файл должны называться одинаково: robot_modular/robot_modular.ino
#include "config.h"
#include "scanner.h"
#include "motors.h"
#include "ultrasonic.h"
#include "autopilot.h"
#include "web.h"
#include "sensors.h"
#include "mapping.h"
#include <esp_system.h>

static const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external reset";
    case ESP_RST_SW: return "software reset";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "SDIO reset";
    default: return "unknown";
  }
}

void setup() {
  Serial.begin(115200);
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("ESP reset reason: %s (%d)\n", resetReasonName(reason), (int)reason);

  scannerInit();      // 1. серво первым (занимает свой таймер ШИМ)
  motorsInit();       // 2. потом ШИМ моторов
  ultrasonicInit();
  sensorsInit();

  Serial.printf("Размеры: ширина %d см, длина %d см. Стоп-дистанция %d см, нужная ширина прохода %d см\n",
                ROBOT_WIDTH_CM, ROBOT_LENGTH_CM, STOP_DIST_CM, NEED_WIDTH_CM);

  webInit();
}

void loop() {
  webUpdate();
  if (mappingIsActive()) mappingUpdate();
  else                   autoUpdate();
  sensorsUpdate();
  yield();
}
