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

static int slotCount(const State &s)
{
  return s.bucketCount + 1;
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
    if (s.windowFillMs < windowMs(s))
      s.windowFillMs += h;
    if (s.bucketFillMs >= BUCKET_MS)
    {
      s.bucketIndex = (s.bucketIndex + 1) % slotCount(s);
      s.bucketCharge[s.bucketIndex] = 0; // reuses the slot that just left the window
      s.bucketFillMs = 0;
    }
  }

  if (s.windowFillMs < windowMs(s))
  {
    s.averageCurrentA = 0;
    return;
  }
  // The window is exactly windowMs: the current partial bucket, the
  // bucketCount - 1 full buckets before it, and the part of the oldest full
  // bucket still inside the window (current assumed uniform within a bucket).
  int slots = slotCount(s);
  float charge = 0;
  for (int k = 0; k < s.bucketCount; k++)
    charge += s.bucketCharge[(s.bucketIndex - k + slots) % slots];
  int oldest = (s.bucketIndex - s.bucketCount + slots) % slots;
  float oldestShare = (float)(BUCKET_MS - s.bucketFillMs) / (float)BUCKET_MS;
  charge += s.bucketCharge[oldest] * oldestShare;
  s.averageCurrentA = charge / (float)windowMs(s);
}

// Whether the condition behind each fault is present in this reading.
static uint16_t presentConditions(const State &s, const Reading &r, bool emergencyStop)
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
  if (emergencyStop)
    present |= EMERGENCY_STOP;
  return present;
}

Result step(State &s, const Reading &r, uint32_t nowMs, bool clearRequested, bool emergencyStop)
{
  const Limits &l = s.limits;
  uint32_t dtMs = s.started ? nowMs - s.lastMs : 0;
  s.started = true;
  s.lastMs = nowMs;

  accumulate(s, r.currentA, dtMs);

  bool supplyValid = r.voltageV >= l.minV && r.voltageV <= l.maxV;
  uint16_t present = presentConditions(s, r, emergencyStop);

  // Clearing first, so a clear and a still-present condition re-latch below.
  if (clearRequested)
    s.faults &= present;

  // Trips. Voltage-low faults only apply once the supply has been accepted:
  // before that, a low or absent supply just keeps output off.
  uint16_t latch = present & (OVERCURRENT | OVERVOLTAGE | SUSTAINED_CURRENT | EMERGENCY_STOP);
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
