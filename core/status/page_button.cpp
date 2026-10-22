#include "page_button.h"

namespace page_button
{

bool update(State &s, bool pressed, uint32_t nowMs)
{
  if (!s.armed)
  {
    s.armed = !pressed;
    return false;
  }
  if (pressed)
  {
    if (!s.down)
    {
      s.down = true;
      s.sinceMs = nowMs;
    }
    return false;
  }
  if (!s.down)
    return false;
  s.down = false;
  uint32_t held = nowMs - s.sinceMs;
  if (held < MIN_PRESS_MS || held >= MAX_PRESS_MS)
    return false;
  s.page = (Page)(((int)s.page + 1) % PAGE_COUNT);
  return true;
}

} // namespace page_button
