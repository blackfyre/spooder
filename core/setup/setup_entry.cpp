#include "setup_entry.h"

namespace setup_entry
{

bool update(Hold &h, bool pressed, uint32_t nowMs)
{
  if (!pressed)
  {
    h.down = false;
    h.fired = false;
    return false;
  }
  if (!h.down)
  {
    h.down = true;
    h.sinceMs = nowMs;
  }
  if (!h.fired && nowMs - h.sinceMs >= HOLD_MS)
  {
    h.fired = true;
    return true;
  }
  return false;
}

bool held(const Hold &h)
{
  return h.down;
}

bool startInSetupMode(uint32_t rtcMagic, bool softwareRestart)
{
  return rtcMagic == MAGIC && softwareRestart;
}

} // namespace setup_entry
