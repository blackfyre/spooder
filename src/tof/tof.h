#ifndef TOF_H
#define TOF_H

#include "Adafruit_VL6180X.h"
#include "../display/display.h"


// TOF
extern Adafruit_VL6180X vl;

extern bool tofSensorAvailable;
extern uint8_t lastRangeFromGround;

void setupTOF();
void loopTOF();

#endif
