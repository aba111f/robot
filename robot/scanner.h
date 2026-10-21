#pragma once

void scannerInit();
void scannerMove(int angle);     // повернуть серво, запомнить время, нужное на дорогу
bool scannerReady();             // серво уже дошло до позиции?
void scannerCenterNow();         // принудительно отправить команду "центр"
int scannerGetAngle();  // вернуть текущий угол серво