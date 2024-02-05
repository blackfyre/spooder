// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>

extern void setupDisplay();

extern void addToLogBuffer(const char* message);

extern void loopDisplay();

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BUFFER_SIZE 10

#define MAX_IDLE_LOOPS 1000

// Display
extern Adafruit_SH1107 display;

#endif // DISPLAY_H