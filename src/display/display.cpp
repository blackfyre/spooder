#include <Arduino.h>
#include <mbed.h>
#include "logo.h"
#include "display.h"
#include "../globals/wiring.h"

// The driver switches the bus to preclk for a frame and to postclk after it;
// keep both at the shared bus speed (400 kHz) instead of the 100 kHz default.
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, globalWire, -1, 400000, 400000);

#define LOG_LINE_LENGTH 80
#define VISIBLE_LINES (SCREEN_HEIGHT / 8)

// The log is written from both loops (gamepad callbacks run in loop2), so the
// buffer, index, change flag and timestamp are only touched under logMutex.
static rtos::Mutex logMutex;
static char logBuffer[BUFFER_SIZE][LOG_LINE_LENGTH];
static int logIndex = 0;
static bool logChanged = true;
static uint32_t lastLogMs = 0;

uint32_t lastRedrawMs = 0;
bool showingLogo = false;

/**
 * @brief Adds a message to the log buffer.
 * 
 * @param message The message to be added.
 */
void addToLogBuffer(const char* message) {
  logMutex.lock();
  strncpy(logBuffer[logIndex], message, sizeof(logBuffer[logIndex]) - 1);
  logBuffer[logIndex][sizeof(logBuffer[logIndex]) - 1] = '\0';  // Ensure null-termination
  logIndex = (logIndex + 1) % BUFFER_SIZE;
  lastLogMs = millis();
  logChanged = true;
  logMutex.unlock();
}

/**
 * @brief Initializes the display.
 * 
 * This function sets up the display for further use.
 * It should be called once at the beginning of the program.
 */
void setupDisplay() {
  delay(250);                 // wait for the OLED to power up
  display.begin(0x3C, true);  // Address 0x3C default

  Serial.println("OLED begun");
  display.setRotation(1);

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Boot");
  display.display();  // actually display all of the above
}

/**
 * Draws a snapshot of the most recent log lines, newest first.
 */
void displayLogMessages(const char lines[VISIBLE_LINES][LOG_LINE_LENGTH]) {
  display.clearDisplay();

  for (int i = 0; i < VISIBLE_LINES; ++i) {
    display.setCursor(0, i * 8);  // 8 pixels per line
    display.print(lines[i]);
  }

  display.display();
}

/**
 * Displays the logo on the screen.
 */
void displayLogo() {
  display.clearDisplay();  // Make sure the display is cleared
  // Draw the bitmap:
  // drawBitmap(x position, y position, bitmap data, bitmap width, bitmap height, color)
  display.drawBitmap(0, 0, logo, SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);

  // Update the display
  display.display();
}


/**
 * @brief Redraws the display when the log has changed or the idle logo is due,
 * at most every MIN_REDRAW_INTERVAL_MS.
 *
 * It is called repeatedly in the main loop.
 */
void loopDisplay() {
  uint32_t now = millis();
  if (now - lastRedrawMs < MIN_REDRAW_INTERVAL_MS)
    return;

  // Take what we need under the lock, then draw without holding it: a frame
  // is ~25 ms of I2C and the gamepad thread must not wait on it.
  char lines[VISIBLE_LINES][LOG_LINE_LENGTH];
  logMutex.lock();
  // Sample under the lock: a message logged after the earlier read would
  // otherwise be newer than `now` and wrap the idle check.
  now = millis();
  bool idle = now - lastLogMs >= IDLE_LOGO_MS;
  bool redrawLog = !idle && (logChanged || showingLogo);
  if (redrawLog) {
    logChanged = false;
    for (int i = 0; i < VISIBLE_LINES; ++i) {
      int bufferIndex = (logIndex - 1 - i + BUFFER_SIZE * 2) % BUFFER_SIZE;
      memcpy(lines[i], logBuffer[bufferIndex], LOG_LINE_LENGTH);
    }
  }
  logMutex.unlock();

  if (idle) {
    if (!showingLogo) {
      displayLogo();
      showingLogo = true;
      lastRedrawMs = now;
    }
    return;
  }

  if (redrawLog) {
    showingLogo = false;
    displayLogMessages(lines);
    lastRedrawMs = now;
  }
}
