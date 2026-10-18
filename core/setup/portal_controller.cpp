#include "portal_controller.h"

namespace portal_controller
{

void poll(Controller &c, uint32_t nowMs)
{
  c.polled = true;
  c.lastPollMs = nowMs;
}

bool requestMode(Controller &c, RequestedMode mode, uint32_t nowMs)
{
  if (mode != RequestedMode::Stand && mode != RequestedMode::Calibrate && mode != RequestedMode::Sleep)
    return false;
  poll(c, nowMs);
  c.mode = mode;
  c.modeRequestCounter++;
  return true;
}

void requestClear(Controller &c, uint32_t nowMs)
{
  poll(c, nowMs);
  c.clearCounter++;
}

bool connected(const Controller &c, uint32_t nowMs)
{
  return c.polled && nowMs - c.lastPollMs <= CONNECTED_WINDOW_MS;
}

gamepad_mapping::Desired desired(Controller &c, uint32_t nowMs)
{
  bool on = connected(c, nowMs);
  if (!on)
    c.mode = RequestedMode::Sleep;
  gamepad_mapping::Desired d = {};
  d.mode = c.mode;
  d.speed = SPEED;
  d.stride = StrideMode::Dynamic;
  d.controllerConnected = on;
  d.clearCounter = c.clearCounter;
  d.modeRequestCounter = c.modeRequestCounter;
  return d;
}

} // namespace portal_controller
