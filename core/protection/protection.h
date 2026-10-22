#ifndef PROTECTION_H
#define PROTECTION_H

// Hardware-free power protection: turns supply readings into "may servo output
// be on" and "must the body sleep". Compiled for the board and for host tests.
#include <stdint.h>

namespace protection
{

enum Fault : uint16_t
{
  NONE = 0,
  OVERCURRENT = 1u << 0,       // above the trip current for its persistence time, or above the ceiling: output inhibited
  SUSTAINED_CURRENT = 1u << 1, // window average above the sustained current: sleep
  UNDERVOLTAGE = 1u << 2,      // below the minimum voltage for the debounce time: sleep
  BROWNOUT = 1u << 3,          // below the brown-out voltage: output inhibited
  OVERVOLTAGE = 1u << 4,       // above the maximum voltage: output inhibited
  EMERGENCY_STOP = 1u << 5,    // emergency stop signalled: output inhibited
};

// Faults that switch servo output off immediately; the others put the body to
// sleep, which folds the legs before switching output off.
constexpr uint16_t INHIBITING_FAULTS = OVERCURRENT | BROWNOUT | OVERVOLTAGE | EMERGENCY_STOP;

// Supply must be continuously valid this long before output is allowed.
constexpr uint32_t SUPPLY_SETTLE_MS = 500;
// Resolution of the sustained-current window.
constexpr uint32_t BUCKET_MS = 50;
constexpr int MAX_BUCKETS = 200; // windows up to 10 s

struct Limits
{
  float tripCurrentA;
  uint32_t tripPersistMs; // the current must stay above tripCurrentA this long to trip
  float ceilingCurrentA;  // a single reading above this trips at once
  float sustainedCurrentA;
  uint32_t sustainedWindowMs; // multiple of BUCKET_MS, at most MAX_BUCKETS buckets
  float minV;
  uint32_t undervoltDebounceMs;
  float brownoutV;
  float maxV;
};

struct Reading
{
  float currentA;
  float voltageV;
};

struct Result
{
  bool allowOutput;      // servo output may be on
  bool forceSleep;       // every movement command must be treated as Sleep
  bool supplyValid;      // voltage currently within [minV, maxV]
  uint16_t faults;       // latched faults
  uint16_t newlyLatched; // faults that latched on this step
  float averageCurrentA; // sustained-window average (0 until the window has filled)
};

struct State
{
  Limits limits;
  bool started;
  uint32_t lastMs;

  // Power-on / post-inhibit gate.
  bool gateOpen;
  bool validRun;
  uint32_t validSinceMs;

  // Overcurrent persistence: an unbroken run of readings above the trip current.
  bool aboveTrip;
  uint32_t aboveTripSinceMs;

  // Undervoltage debounce.
  bool belowMin;
  uint32_t belowMinSinceMs;

  // Sustained-current window: per-bucket integral of current over time (A·ms).
  // One slot more than the window's buckets: the oldest full bucket is still
  // partly inside the window while the current one fills.
  float bucketCharge[MAX_BUCKETS + 1];
  int bucketCount; // buckets per window
  int bucketIndex;
  uint32_t bucketFillMs;  // time accumulated in the current bucket
  uint32_t windowFillMs;  // total time covered, capped at the window length
  float averageCurrentA;

  uint16_t faults;
};

/**
 * @brief Resets the state with the given limits. Output starts disallowed.
 */
void init(State &s, const Limits &limits);

/**
 * @brief Evaluates one reading taken at @p nowMs.
 *
 * @param clearRequested clears each latched fault whose condition is absent
 *        in this reading; faults whose condition persists stay latched.
 * @param emergencyStop the operator's emergency stop is signalled; it latches
 *        EMERGENCY_STOP, which clears only once the stop is released.
 */
Result step(State &s, const Reading &reading, uint32_t nowMs, bool clearRequested, bool emergencyStop);

} // namespace protection

#endif // PROTECTION_H
