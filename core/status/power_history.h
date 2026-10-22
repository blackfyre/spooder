#ifndef POWER_HISTORY_H
#define POWER_HISTORY_H

// The robot's supply voltage and current over the last minute, as the bridge
// saw them in telemetry: one sample every SAMPLE_MS while the link is up, a
// gap while it is lost. The bridge's power page draws it.
// Hardware-free: the bridge feeds it from loop(); host tests drive it.
#include <stdint.h>

namespace power_history
{

constexpr uint32_t SAMPLE_MS = 100;
constexpr int SAMPLE_COUNT = 600; // 60 s
constexpr uint32_t WINDOW_MS = SAMPLE_MS * SAMPLE_COUNT;
// A late call fills up to this many missed samples with its reading (a
// redraw can delay loop() that long); longer delays are gaps.
constexpr int BACKFILL_SAMPLES = 5;

struct Sample
{
  bool present; // false: a gap (the link was lost)
  uint16_t voltageMv;
  uint16_t currentMa;
};

struct History
{
  Sample samples[SAMPLE_COUNT] = {};
  int next = 0;  // where the next sample goes
  int count = 0; // samples recorded, at most SAMPLE_COUNT
  bool started = false;
  uint32_t dueMs = 0; // when the next sample is due
};

// One column of a trace: the range of the samples it covers.
struct Column
{
  bool present; // false when every sample it covers is a gap or unrecorded
  uint16_t minVoltageMv, maxVoltageMv;
  uint16_t minCurrentMa, maxCurrentMa;
};

/**
 * @brief Records the samples due by @p nowMs: the given reading while
 * @p fresh, a gap otherwise. A late call gives the reading to the newest
 * BACKFILL_SAMPLES due, and gaps to any older ones; a pause longer than the
 * window fills it once.
 */
void record(History &h, uint32_t nowMs, bool fresh, uint16_t voltageMv, uint16_t currentMa);

/**
 * @brief The newest sample; false when there is none or it is a gap.
 */
bool newest(const History &h, Sample &out);

/**
 * @brief The window's lowest and highest voltage and highest current; false
 * when it holds no sample.
 */
bool extremes(const History &h, uint16_t &minVoltageMv, uint16_t &maxVoltageMv, uint16_t &maxCurrentMa);

/**
 * @brief The window as @p width columns, oldest on the left, the newest
 * sample in the last column. Columns before the first recorded sample are
 * absent.
 */
void columns(const History &h, int width, Column out[]);

} // namespace power_history

#endif // POWER_HISTORY_H
