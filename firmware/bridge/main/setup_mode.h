#ifndef BRIDGE_SETUP_MODE_H
#define BRIDGE_SETUP_MODE_H

// The bridge's setup mode: a WiFi access point with a captive portal through
// which a phone calibrates the robot. It is a boot path of its own, entered
// by a software restart with a magic value in RTC memory
// (core/setup/setup_entry), and it never starts Bluetooth: WiFi and
// Bluetooth never share the radio. Leaving it is a restart into normal
// operation.
//
// Tasks in setup mode:
// - robot_link, priority 5, core 1: as in normal operation, but commands
//   come from the portal controller and calibration edits go out as sets.
// - setup, priority 1, core 1: the HTTP server and the screen.
// - WiFi, lwIP and the DNS server (asynchronous UDP): core 0.
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Called first in app_main: true if this boot is to be setup mode.
 * Clears the RTC magic either way, so any later reset starts normally.
 */
bool setup_mode_requested(void);

/**
 * @brief Starts Arduino, the robot link, the access point, the DNS and HTTP
 * servers and the setup task. Returns once they run; never starts Bluetooth.
 */
void setup_mode_start(void);

#ifdef __cplusplus
}

namespace setup_mode
{

/**
 * @brief Sets the RTC magic and restarts the bridge into setup mode.
 */
[[noreturn]] void restartIntoSetup();

} // namespace setup_mode
#endif

#endif // BRIDGE_SETUP_MODE_H
