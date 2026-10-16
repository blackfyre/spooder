#include "main.h"
#include "tof/tof.h"
#include "display/display.h"
#include "control/gamepad.h"
#include "mlc/mlc.h"
#include "movement/movement.h"
#include "globals/wiring.h"
#include "control/serial_commands.h"
#include <Scheduler.h>

TwoWire *globalWire = &Wire;

// Main loop period; movement advances by elapsed time, so this is a target,
// not a hard deadline.
constexpr uint32_t LOOP_PERIOD_MS = 20;
// Every device on the bus (PCA9685, SH1107, VL6180X, LSM6DSOX) supports 400 kHz.
constexpr uint32_t I2C_CLOCK_HZ = 400000;

static uint32_t nextLoopMs = 0;

#ifndef SPOODER_SERIAL_COMMANDS
// Until a controller supplies commands, hold Stand with a fresh timestamp so
// the stale-command rule never fires.
static MovementCommand defaultCommand(uint32_t nowMs)
{
  MovementCommand cmd;
  cmd.mode = RequestedMode::Stand;
  cmd.issuedAtMs = nowMs;
  return cmd;
}
#endif

// loop for acting on recieved data
// do not overwhelm i2c
void loop2()
{
  loopGamepad();

  delay(50);
}

// Arduino setup function. Runs in CPU 1
void setup()
{
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  pinMode(LED_BUILTIN, OUTPUT);
  // define RGB LED as outputs
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

  setupGamepad();
  display.print(".");
  display.display();

  setupMLC();
  display.print(".");
  display.display();

  delay(300);

  setupTOF();
  display.print(".");
  display.display();

  setupMovement();
  display.print(".");
  display.display();

  display.print("done!");
  display.display();

  delay(500);

  display.clearDisplay();

  Scheduler.startLoop(loop2);
  nextLoopMs = millis();

  addToLogBuffer("Ready!");
  addToLogBuffer("Waiting on gamepad...");
  // amber led represents setup complete
  digitalWrite(LED_BUILTIN, HIGH);
}

// loop for processing sensory data
void loop()
{
  uint32_t now = millis();
#ifdef SPOODER_SERIAL_COMMANDS
  movementTick(now, serialCommand(now));
#else
  movementTick(now, defaultCommand(now));
#endif

  loopMLC();
  loopTOF();
  loopDisplay();

  nextLoopMs += LOOP_PERIOD_MS;
  int32_t remaining = (int32_t)(nextLoopMs - millis());
  if (remaining > 0)
    delay(remaining);
  else
    nextLoopMs = millis(); // overran: start the next period now
}


