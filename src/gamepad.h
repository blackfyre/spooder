#ifndef GAMEPAD_H
#define GAMEPAD_H

#include "main.h"
#include <Bluepad32.h>
#include "display.h"

extern Bluepad32 BP32;

void setupGamepad();

void loopGamepad();

#endif