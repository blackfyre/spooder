#include "main.h"
#include "tof/tof.h"
#include "display/display.h"
#include "control/gamepad.h"
#include "mlc/mlc.h"


// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //define RGB LED as outputs
  BP32.pinMode(LED_R, OUTPUT);
  BP32.pinMode(LED_G, OUTPUT);
  BP32.pinMode(LED_B, OUTPUT);
  BP32.digitalWrite(LED_R, LED_OFF);
  BP32.digitalWrite(LED_G, LED_OFF);
  BP32.digitalWrite(LED_B, LED_OFF);

  
  setupDisplay();

  // Initialize serial
  Serial.begin(115200);
  
  display.print(".");
  display.display();

  //setupGamepad();
  display.print(".");
  display.display();

  setupMLC();
  display.print(".");
  display.display();
  
  delay(300);

  setupTOF();
  display.print(".");
  display.display();

  display.print("done!");
  display.display();

  delay(500);

  display.clearDisplay();

  //Scheduler.startLoop(loop2);

  addToLogBuffer("Ready!");
  addToLogBuffer("Waiting on gamepad...");
  // amber led represents setup complete
  digitalWrite(LED_BUILTIN, HIGH);
}

// loop for processing sensory data
void loop() {
  loopGamepad();
  loopMLC();
  loopTOF();
  loopDisplay();
  
  delay(50);
}

// loop for acting on recieved data
// do not overwhelm i2c
//void loop2() {
  

//  delay(50);
//}

