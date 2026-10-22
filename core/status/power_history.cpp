#include "power_history.h"

namespace power_history
{

static void push(History &h, const Sample &s)
{
  h.samples[h.next] = s;
  h.next = (h.next + 1) % SAMPLE_COUNT;
  if (h.count < SAMPLE_COUNT)
    h.count++;
}

void record(History &h, uint32_t nowMs, bool fresh, uint16_t voltageMv, uint16_t currentMa)
{
  if (!h.started)
  {
    h.started = true;
    h.dueMs = nowMs;
  }
  Sample s = {fresh, fresh ? voltageMv : (uint16_t)0, fresh ? currentMa : (uint16_t)0};
  const Sample gap = {false, 0, 0};
  int pushed = 0;
  while ((int32_t)(nowMs - h.dueMs) >= 0)
  {
    if (pushed == SAMPLE_COUNT)
    {
      // Paused longer than the window: it is full; resume from now.
      h.dueMs = nowMs + SAMPLE_MS;
      break;
    }
    // Samples due longer ago than the backfill allows are gaps.
    bool recent = nowMs - h.dueMs < (uint32_t)BACKFILL_SAMPLES * SAMPLE_MS;
    push(h, recent ? s : gap);
    pushed++;
    h.dueMs += SAMPLE_MS;
  }
}

// The k-th recorded sample, oldest first.
static const Sample &at(const History &h, int k)
{
  return h.samples[(h.next - h.count + k + SAMPLE_COUNT) % SAMPLE_COUNT];
}

bool newest(const History &h, Sample &out)
{
  if (h.count == 0)
    return false;
  out = at(h, h.count - 1);
  return out.present;
}

bool extremes(const History &h, uint16_t &minVoltageMv, uint16_t &maxVoltageMv, uint16_t &maxCurrentMa)
{
  bool any = false;
  for (int k = 0; k < h.count; k++)
  {
    const Sample &s = at(h, k);
    if (!s.present)
      continue;
    if (!any || s.voltageMv < minVoltageMv)
      minVoltageMv = s.voltageMv;
    if (!any || s.voltageMv > maxVoltageMv)
      maxVoltageMv = s.voltageMv;
    if (!any || s.currentMa > maxCurrentMa)
      maxCurrentMa = s.currentMa;
    any = true;
  }
  return any;
}

void columns(const History &h, int width, Column out[])
{
  int unrecorded = SAMPLE_COUNT - h.count;
  for (int c = 0; c < width; c++)
  {
    Column &col = out[c];
    col = Column();
    // Slots of the whole window this column covers; slot SAMPLE_COUNT - 1 is
    // the newest.
    int first = c * SAMPLE_COUNT / width;
    int last = (c + 1) * SAMPLE_COUNT / width;
    if (last == first)
      last = first + 1;
    for (int slot = first; slot < last; slot++)
    {
      if (slot < unrecorded)
        continue;
      const Sample &s = at(h, slot - unrecorded);
      if (!s.present)
        continue;
      if (!col.present)
      {
        col.present = true;
        col.minVoltageMv = col.maxVoltageMv = s.voltageMv;
        col.minCurrentMa = col.maxCurrentMa = s.currentMa;
        continue;
      }
      if (s.voltageMv < col.minVoltageMv)
        col.minVoltageMv = s.voltageMv;
      if (s.voltageMv > col.maxVoltageMv)
        col.maxVoltageMv = s.voltageMv;
      if (s.currentMa < col.minCurrentMa)
        col.minCurrentMa = s.currentMa;
      if (s.currentMa > col.maxCurrentMa)
        col.maxCurrentMa = s.currentMa;
    }
  }
}

} // namespace power_history
