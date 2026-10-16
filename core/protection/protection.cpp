#include "protection.h"

namespace protection
{

void init(State &s, const Limits &limits)
{
  s = State();
  s.limits = limits;
  int buckets = (int)(limits.sustainedWindowMs / BUCKET_MS);
  if (buckets < 1)
    buckets = 1;
  if (buckets > MAX_BUCKETS)
    buckets = MAX_BUCKETS;
  s.bucketCount = buckets;
}

static uint32_t windowMs(const State &s)
{
  return (uint32_t)s.bucketCount * BUCKET_MS;
}

// Adds current·dt to the window and recomputes the average once the window has
// filled. Long gaps are split across buckets.
static void accumulate(State &s, float currentA, uint32_t dtMs)
{
  while (dtMs > 0)
  {
    uint32_t room = BUCKET_MS - s.bucketFillMs;
    uint32_t h = dtMs < room ? dtMs : room;
    s.bucketCharge[s.bucketIndex] += currentA * (float)h;
    s.bucketFillMs += h;
    dtMs -= h;
    if (s.bucketFillMs >= BUCKET_MS)
    {
      s.bucketIndex = (s.bucketIndex + 1) % s.bucketCount;
      s.bucketCharge[s.bucketIndex] = 0; // oldest bucket leaves the window
      s.bucketFillMs = 0;
    }
    if (s.windowFillMs < windowMs(s))
      s.windowFillMs += h;
  }

  if (s.windowFillMs < windowMs(s))
  {
    s.averageCurrentA = 0;
    return;
  }
  // The current (partial) bucket plus the other full buckets span exactly one
  // window minus the unfilled part of the current bucket's slot, which was
  // zeroed when it became current; divide by the time actually covered.
  float charge = 0;
  for (int i = 0; i < s.bucketCount; i++)
    charge += s.bucketCharge[i];
  uint32_t covered = windowMs(s) - BUCKET_MS + s.bucketFillMs;
  s.averageCurrentA = covered > 0 ? charge / (float)covered : 0;
}

// Whether the condition behind each fault is present in this reading.
static uint16_t presentConditions(const State &s, const Reading &r)
{
  const Limits &l = s.limits;
  uint16_t present = 0;
  if (r.currentA > l.tripCurrentA)
    present |= OVERCURRENT;
  if (s.averageCurrentA > l.sustainedCurrentA)
    present |= SUSTAINED_CURRENT;
  if (r.voltageV < l.minV)
    present |= UNDERVOLTAGE;
  if (r.voltageV < l.brownoutV)
    present |= BROWNOUT;
  if (r.voltageV > l.maxV)
    present |= OVERVOLTAGE;
  return present;
}

Result step(State &s, const Reading &r, uint32_t nowMs, bool clearRequested)
{
  const Limits &l = s.limits;
  uint32_t dtMs = s.started ? nowMs - s.lastMs : 0;
  s.started = true;
  s.lastMs = nowMs;

  accumulate(s, r.currentA, dtMs);

  bool supplyValid = r.voltageV >= l.minV && r.voltageV <= l.maxV;
  uint16_t present = presentConditions(s, r);

  // Clearing first, so a clear and a still-present condition re-latch below.
  if (clearRequested)
    s.faults &= present;

  // Trips. Voltage-low faults only apply once the supply has been accepted:
  // before that, a low or absent supply just keeps output off.
  uint16_t latch = present & (OVERCURRENT | OVERVOLTAGE | SUSTAINED_CURRENT);
  if (s.gateOpen)
  {
    if (present & BROWNOUT)
      latch |= BROWNOUT;
    if (present & UNDERVOLTAGE)
    {
      if (!s.belowMin)
      {
        s.belowMin = true;
        s.belowMinSinceMs = nowMs;
      }
      if (nowMs - s.belowMinSinceMs >= l.undervoltDebounceMs)
        latch |= UNDERVOLTAGE;
    }
    else
    {
      s.belowMin = false;
    }
  }
  else
  {
    s.belowMin = false;
  }

  uint16_t newly = latch & ~s.faults;
  s.faults |= latch;

  // An inhibiting fault closes the gate: output then needs a fresh settle.
  if (s.faults & INHIBITING_FAULTS)
  {
    s.gateOpen = false;
    s.validRun = false;
  }
  else if (!s.gateOpen)
  {
    if (supplyValid)
    {
      if (!s.validRun)
      {
        s.validRun = true;
        s.validSinceMs = nowMs;
      }
      if (nowMs - s.validSinceMs >= SUPPLY_SETTLE_MS)
        s.gateOpen = true;
    }
    else
    {
      s.validRun = false;
    }
  }

  Result res;
  res.allowOutput = s.gateOpen && !(s.faults & INHIBITING_FAULTS);
  res.forceSleep = s.faults != 0;
  res.supplyValid = supplyValid;
  res.faults = s.faults;
  res.newlyLatched = newly;
  res.averageCurrentA = s.averageCurrentA;
  return res;
}

} // namespace protection
