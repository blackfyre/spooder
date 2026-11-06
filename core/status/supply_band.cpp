#include "status/supply_band.h"

namespace supply_band
{

Limits fromDiagnostics(const protocol::Diagnostics &d)
{
  return {d.brownoutMv, d.minMv, d.maxMv, d.sustainedCurrentMa, d.tripCurrentMa};
}

bool valid(const Limits &l)
{
  return l.brownoutMv < l.minMv && l.minMv < l.maxMv && l.sustainedCurrentMa > 0 &&
         l.sustainedCurrentMa < l.tripCurrentMa;
}

Band voltage(const Limits &l, uint16_t mv)
{
  if (!valid(l))
    return Band::Unknown;
  if (mv <= l.minMv || mv >= l.maxMv)
    return Band::Fault;
  // The warning margin: the gap protection leaves between undervoltage and
  // brown-out, on both sides (design D3). In int: the sums can pass 65535.
  int margin = l.minMv - l.brownoutMv;
  if ((int)mv < l.minMv + margin || (int)mv > l.maxMv - margin)
    return Band::Warning;
  return Band::Ok;
}

Band current(const Limits &l, uint16_t ma)
{
  if (!valid(l))
    return Band::Unknown;
  if (ma >= l.tripCurrentMa)
    return Band::Fault;
  if (ma >= l.sustainedCurrentMa)
    return Band::Warning;
  return Band::Ok;
}

} // namespace supply_band
