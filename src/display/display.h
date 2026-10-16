// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

extern void setupDisplay();

extern void addToLogBuffer(const char* message);

extern void loopDisplay();

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BUFFER_SIZE 10

// Show the logo after this long without a new log line.
#define IDLE_LOGO_MS 50000
// Minimum time between full-frame redraws (each is ~25 ms of I2C at 400 kHz).
#define MIN_REDRAW_INTERVAL_MS 200

// Display
extern Adafruit_SH1107 display;

#endif // DISPLAY_H