#pragma once

enum Move { M_STOP, M_FORWARD, M_BACKWARD, M_LEFT, M_RIGHT };

void motorsInit();
void motorsForward();
void motorsBackward();
void motorsLeft();
void motorsRight();
void motorsStop();

// Ручное движение: brake=true делает короткий стоп перед сменой направления
void motorsMove(Move m, bool brake);
void motorsReapply();                 // повторить текущее ручное движение (после смены скорости)

int  motorsSpeedPercent();            // 0..100
void motorsSetSpeedPercent(int percent);