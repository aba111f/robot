#pragma once

void ultrasonicInit();
float ultrasonicRead();       // см, 0 = нет эхо
float ultrasonicScanRead();   // 3 замера, нули игнорируются; нет эха = NO_ECHO_CM