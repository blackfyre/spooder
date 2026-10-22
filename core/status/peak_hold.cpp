#include "peak_hold.h"

namespace peak_hold
{

void update(Hold &h, Extreme extreme, float ratePerS, float warnMargin, uint32_t nowMs, bool fresh, float reading)
{
  if (!fresh)
  {
    h.present = false;
    h.warning = false;
    return;
  }
  if (h.warning && (int32_t)(nowMs - h.warnUntilMs) >= 0)
    h.warning = false;
  bool beyond = extreme == Extreme::Max ? reading >= h.value : reading <= h.value;
  if (h.present && beyond)
  {
    float by = extreme == Extreme::Max ? reading - h.value : h.value - reading;
    // A new extreme past the margin starts the warning; while warning, any
    // new extreme keeps it.
    if (by > warnMargin || (h.warning && by > 0))
    {
      h.warning = true;
      h.warnUntilMs = nowMs + WARN_MS;
    }
  }
  if (!h.present || beyond)
  {
    h.present = true;
    h.value = reading;
    h.reachedMs = nowMs;
    h.lastMs = nowMs;
    return;
  }
  // Return towards the reading only for the time past the hold.
  uint32_t holdEnd = h.reachedMs + HOLD_MS;
  uint32_t from = (int32_t)(h.lastMs - holdEnd) > 0 ? h.lastMs : holdEnd;
  h.lastMs = nowMs;
  if ((int32_t)(nowMs - from) <= 0)
    return;
  float step = ratePerS * (float)(nowMs - from) / 1000.0f;
  if (extreme == Extreme::Max)
    h.value = h.value - step > reading ? h.value - step : reading;
  else
    h.value = h.value + step < reading ? h.value + step : reading;
}

} // namespace peak_hold
