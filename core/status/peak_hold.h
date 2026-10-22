#ifndef PEAK_HOLD_H
#define PEAK_HOLD_H

// A held extreme of a reading: the highest current or the lowest voltage
// seen recently. It moves at once to a reading at or beyond it, holds HOLD_MS
// from when it was last reached, then moves towards the reading at a set rate until it
// meets it. The desktop calibrator's charts follow the same rule.
// On the bridge, a reading that beats the held value by more than a margin
// (a new high current, a deeper sag) also raises a warning, kept WARN_MS
// after the last new extreme.
// Hardware-free: the bridge feeds it from telemetry; host tests drive it.
#include <stdint.h>

namespace peak_hold
{

constexpr uint32_t HOLD_MS = 3000;
constexpr float CURRENT_FALL_MA_PER_S = 2000;
constexpr float VOLTAGE_RISE_MV_PER_S = 500;
constexpr uint32_t WARN_MS = 1000;
constexpr float CURRENT_WARN_MARGIN_MA = 100;
constexpr float VOLTAGE_WARN_MARGIN_MV = 50;

enum class Extreme : uint8_t
{
  Max,
  Min,
};

struct Hold
{
  bool present = false;
  float value = 0;
  uint32_t reachedMs = 0; // when the held value was reached
  uint32_t lastMs = 0;    // the previous update
  bool warning = false;   // a new extreme beyond the margin, recently
  uint32_t warnUntilMs = 0;
};

/**
 * @brief Advances the hold to @p nowMs with @p reading; @p ratePerS is how
 * fast it returns after the hold. A reading beyond the held value by more
 * than @p warnMargin starts a warning; each further new extreme keeps it
 * WARN_MS longer. While not @p fresh (link lost) nothing is held or warned,
 * and holding restarts from the next fresh reading.
 */
void update(Hold &h, Extreme extreme, float ratePerS, float warnMargin, uint32_t nowMs, bool fresh, float reading);

} // namespace peak_hold

#endif // PEAK_HOLD_H
