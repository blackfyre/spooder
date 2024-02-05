#ifndef MLC_H
#define MLC_H

#include "../main.h"
#include <math.h>
#include "LSM6DSOXSensor.h"
#include "activity_recognition.h"

void setupMLC();

void INT1Event_cb();

void printMLCStatus(uint8_t status);

void loopMLC();

#endif