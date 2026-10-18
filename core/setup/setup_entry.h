#ifndef SETUP_ENTRY_H
#define SETUP_ENTRY_H

// How the bridge enters setup mode: the left button (GPIO0) held for
// HOLD_MS while the boot logo is shown, then a software restart with MAGIC in
// RTC memory that survives the restart. Only that combination starts setup
// mode: a power-up or any other reset starts normal operation, whatever the
// RTC memory holds.
// Hardware-free: the bridge samples the button and reads the reset reason;
// host tests drive it directly.
#include <stdint.h>

namespace setup_entry
{

constexpr uint32_t HOLD_MS = 1000;
// Written to RTC memory just before the restart into setup mode.
constexpr uint32_t MAGIC = 0x53505355u; // "SPSU"

struct Hold
{
  bool down = false;
  bool fired = false;
  uint32_t sinceMs = 0;
};

/**
 * @brief Advances the hold detector by one button sample at @p nowMs.
 * @return true once per hold, when the button has been down for HOLD_MS
 */
bool update(Hold &h, bool pressed, uint32_t nowMs);

/**
 * @brief True while the button is down (the logo stays up meanwhile).
 */
bool held(const Hold &h);

/**
 * @brief The boot decision: setup mode only when @p rtcMagic is MAGIC and
 * the reset was a software restart. The caller clears the magic either way.
 */
bool startInSetupMode(uint32_t rtcMagic, bool softwareRestart);

} // namespace setup_entry

#endif // SETUP_ENTRY_H
