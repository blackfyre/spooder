#include <Arduino.h>
#include "logo.h"
#include "display.h"
#include "../globals/wiring.h"

// The driver switches the bus to preclk for a frame and to postclk after it;
// keep both at the shared bus speed (400 kHz) instead of the 100 kHz default.
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, globalWire, -1, 400000, 400000);

char logBuffer[BUFFER_SIZE][80];
int logIndex = 0;
// Set by addToLogBuffer, which is also called from the gamepad loop.
volatile bool logChanged = true;
volatile uint32_t lastLogMs = 0;
uint32_t lastRedrawMs = 0;
bool showingLogo = false;

/**
 * @brief Adds a message to the log buffer.
 * 
 * @param message The message to be added.
 */
void addToLogBuffer(const char* message) {
  strncpy(logBuffer[logIndex], message, sizeof(logBuffer[logIndex]) - 1);
  logBuffer[logIndex][sizeof(logBuffer[logIndex]) - 1] = '\0';  // Ensure null-termination
  logIndex = (logIndex + 1) % BUFFER_SIZE;
  // Publish after the buffer is updated, so a redraw that clears the flag
  // cannot miss this message.
  lastLogMs = millis();
  logChanged = true;
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
 * Displays log messages on the screen.
 */
void displayLogMessages() {
  display.clearDisplay();

  int currentLine = 0;
  int offset = 0;  // Adjust the offset to control how many lines to skip for the display

  for (int i = 0; i < SCREEN_HEIGHT / 8; ++i) {
    int bufferIndex = (logIndex - 1 - offset + BUFFER_SIZE) % BUFFER_SIZE;

    int yOffset = currentLine * 8;  // Assuming 8 pixels per line
    display.setCursor(0, yOffset);
    display.print(logBuffer[bufferIndex]);

    currentLine = (currentLine + 1) % (SCREEN_HEIGHT / 8);
    offset = (offset + 1) % BUFFER_SIZE;  // Increase the offset for the next line
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

  bool idle = now - lastLogMs >= IDLE_LOGO_MS;
  if (idle) {
    if (!showingLogo) {
      displayLogo();
      showingLogo = true;
      lastRedrawMs = now;
    }
    return;
  }

  if (logChanged || showingLogo) {
    logChanged = false;
    showingLogo = false;
    displayLogMessages();
    lastRedrawMs = now;
  }
}
