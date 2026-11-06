#ifndef SUPPLY_BAND_H
#define SUPPLY_BAND_H

// The band a supply reading is in against the protection limits the robot
// reports in its diagnostics frames (openspec change handheld-observations,
// design D3): ok, warning, or at or past the level at which protection
// faults. The bridge's screen colours its voltage and current by it; the
// handheld has a Go copy (desktop/handheld/internal/supplyband), and both
// are run over test/golden/supply_bands.txt.
// Hardware-free: host tests drive it.
#include <stdint.h>
#include "protocol/protocol.h"

namespace supply_band
{

enum class Band : uint8_t
{
  Unknown, // no limits reported, or limits out of order
  Ok,
  Warning,
  Fault,
};

struct Limits
{
  uint16_t brownoutMv;
  uint16_t minMv; // undervoltage
  uint16_t maxMv; // overvoltage
  uint16_t sustainedCurrentMa;
  uint16_t tripCurrentMa;
};

/**
 * @brief The limits a diagnostics frame reports.
 */
Limits fromDiagnostics(const protocol::Diagnostics &d);

/**
 * @brief Whether @p l is ordered as protection requires: brown-out below
 * the minimum below the maximum voltage, and a sustained current above zero
 * and below the trip current.
 */
bool valid(const Limits &l);

/**
 * @brief The voltage's band: Fault at or below the minimum or at or above
 * the maximum; Warning within the brown-out-to-minimum gap of either; else
 * Ok. Unknown for limits that are not valid.
 */
Band voltage(const Limits &l, uint16_t mv);

/**
 * @brief The current's band: Fault at or above the trip current; Warning at
 * or above the sustained current; else Ok. Unknown for limits that are not
 * valid.
 */
Band current(const Limits &l, uint16_t ma);

} // namespace supply_band

#endif // SUPPLY_BAND_H
