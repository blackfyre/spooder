#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

// The bridge's drive mode: decided once at boot from the setup request and
// the drive mode switch (a latching switch between GPIO32 and ground, with
// the internal pull-up), then kept until the bridge restarts. While it runs,
// the switch is sampled for the restart notice only: a 100 ms debounce of
// the switch against the running drive mode.
// Hardware-free: the bridge reads the pin and the setup request; host tests
// drive it directly.
#include <stdint.h>

namespace drive_mode
{

// Bluetooth first, so a zero-initialised value is the mode without a switch.
enum class Mode : uint8_t
{
  Bluetooth, // switch open (or not fitted, or a broken wire)
  Wifi,      // switch closed to ground
  Setup,     // the setup request (setup_entry), whatever the switch
};

constexpr uint32_t NOTICE_MS = 100;

/**
 * @brief The boot decision: setup mode when @p setupRequested, otherwise
 * WiFi drive when @p switchClosed, otherwise Bluetooth drive.
 */
Mode select(bool setupRequested, bool switchClosed);

struct Notice
{
  bool shown = false;
  bool pending = false; // the samples since pendingSinceMs disagree with shown
  uint32_t pendingSinceMs = 0;
};

/**
 * @brief Advances the switch notice by one sample of the switch at @p nowMs.
 * The notice is shown once the switch has selected another drive mode than
 * @p running for NOTICE_MS, and cleared once it has selected @p running
 * again for NOTICE_MS; a shorter change is bounce and changes nothing. In
 * setup mode the switch is ignored and no notice is shown.
 * @return true while the notice is shown
 */
bool update(Notice &n, Mode running, bool switchClosed, uint32_t nowMs);

} // namespace drive_mode

#endif // DRIVE_MODE_H
