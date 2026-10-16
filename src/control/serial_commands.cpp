#ifdef SPOODER_SERIAL_COMMANDS

#include "serial_commands.h"

#include <Arduino.h>
#include "../movement/gait.h"

static MovementCommand current;

static void report(const char *what)
{
  Serial.print("serial command: ");
  Serial.println(what);
}

MovementCommand serialCommand(uint32_t nowMs)
{
  while (Serial.available() > 0)
  {
    switch (Serial.read())
    {
    case 's':
      current.mode = RequestedMode::Stand;
      current.translate = Vector2();
      current.rotate = 0;
      report("stand");
      break;
    case 'w':
      current.mode = RequestedMode::Walk;
      current.translate = Vector2(0, 60);
      current.rotate = 0;
      report("walk forward");
      break;
    case 'r':
      current.mode = RequestedMode::Walk;
      current.translate = Vector2();
      current.rotate = 60;
      report("walk rotating");
      break;
    case 'x':
      current.mode = RequestedMode::Walk;
      current.translate = Vector2();
      current.rotate = 0;
      report("walk, no input");
      break;
    case 'g':
      current.gait = (current.gait + 1) % gait::GAIT_COUNT;
      Serial.print("serial command: gait ");
      Serial.println(current.gait);
      break;
    case 'f':
      current.stride = current.stride == StrideMode::Fixed ? StrideMode::Dynamic : StrideMode::Fixed;
      report(current.stride == StrideMode::Fixed ? "fixed stride" : "dynamic stride");
      break;
    case 'z':
      current.mode = RequestedMode::Sleep;
      report("sleep");
      break;
    case 'c':
      current.mode = RequestedMode::Calibrate;
      report("calibrate");
      break;
    case '+':
      current.height = current.height + 10 > 100 ? 100 : current.height + 10;
      report("height up");
      break;
    case '-':
      current.height = current.height - 10 < 0 ? 0 : current.height - 10;
      report("height down");
      break;
    default:
      break;
    }
  }
  current.issuedAtMs = nowMs;
  return current;
}

#endif // SPOODER_SERIAL_COMMANDS
