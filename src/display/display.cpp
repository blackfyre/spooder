#include <Arduino.h>
#include "logo.h"
#include "display.h"

Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, &Wire);

char logBuffer[BUFFER_SIZE][80];
int idleLoopCount = 0;
int logIndex = 0;

/**
 * @brief Adds a message to the log buffer.
 * 
 * @param message The message to be added.
 */
void addToLogBuffer(const char* message) {
  idleLoopCount = 0;
  strncpy(logBuffer[logIndex], message, sizeof(logBuffer[logIndex]) - 1);
  logBuffer[logIndex][sizeof(logBuffer[logIndex]) - 1] = '\0';  // Ensure null-termination
  logIndex = (logIndex + 1) % BUFFER_SIZE;
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
 * @brief This function is responsible for the continuous display operation.
 * 
 * It is called repeatedly in the main loop to update the display.
 */
void loopDisplay() {
  display.clearDisplay();
  idleLoopCount++;

  // Check if it's time to display the logo
  if (idleLoopCount >= MAX_IDLE_LOOPS) {
    displayLogo();                       // Implement a function to display your logo
    idleLoopCount = MAX_IDLE_LOOPS + 1;  // prevent overflow
  } else {
    displayLogMessages();
    display.display();
  }
}
