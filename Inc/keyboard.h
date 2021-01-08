#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include "stm32f4xx_hal.h"

unsigned char KeyPressed();
unsigned char DetectCol();
unsigned char KeyCode();
void  ScanRow(int row);

#endif
