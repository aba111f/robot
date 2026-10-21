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

void setup() {
  Serial.begin(115200);

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
