#ifndef GAMEPAD_H
#define GAMEPAD_H

#include "../main.h"
#include <Bluepad32.h>
#include "../display/display.h"

extern Bluepad32 BP32;

void setupGamepad();

void loopGamepad();

void onConnectedController(ControllerPtr ctl);

void onDisconnectedController(ControllerPtr ctl);

void processGamepad(ControllerPtr gamepad);

#endif