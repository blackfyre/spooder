#include "serial_commands.h"

#include <cstdio>
#include "pico/stdlib.h"
#include "movement/gait.h"

namespace serial_commands
{

static MovementCommand current;
static uint32_t requestSeq = 0;

static void report(const char *what)
{
  printf("serial command: %s\n", what);
}

Input poll(uint32_t nowMs)
{
  Input in;
  in.clearFaults = false;
  in.printReadings = false;

  for (int n = 0; n < MAX_CHARS_PER_POLL; n++)
  {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT)
      break;
    switch (c)
    {
    case 's':
    case 'w':
    case 'r':
    case 'x':
    case 'z':
    case 'c':
      requestSeq++; // an explicit mode request
      break;
    default:
      break;
    }
    switch (c)
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
      printf("serial command: gait %d\n", current.gait);
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
    case '!':
      in.clearFaults = true;
      report("clear faults");
      break;
    case 'p':
      in.printReadings = true;
      break;
    default:
      break;
    }
  }
  current.issuedAtMs = nowMs;
  in.command = current;
  in.requestSeq = requestSeq;
  return in;
}

} // namespace serial_commands
