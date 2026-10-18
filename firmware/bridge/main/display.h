#ifndef BRIDGE_DISPLAY_H
#define BRIDGE_DISPLAY_H

// Status screen on the T-Display's ST7789 (135×240, used landscape).
// At power-up: the Spooder logo, then a start-up log, then the status screen
// (an active emergency stop skips straight to it). update() redraws the
// status at most every REFRESH_MS and only the lines that changed; the whole
// screen is cleared only when the link state changes.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "link.h"

namespace display
{

constexpr uint32_t REFRESH_MS = 250;
constexpr uint32_t LOGO_MS = 1500;
constexpr uint32_t BOOT_LOG_MS = 1500; // after the logo

/**
 * @brief Starts the panel and shows the logo.
 */
void setup();

/**
 * @brief Adds a line to the start-up log (shown after the logo).
 */
void bootMessage(const char *text, bool ok = true);

void update(uint32_t nowMs, bool gamepadConnected, const gamepad_mapping::Desired &desired,
            const robot_link::Status &linkStatus);

} // namespace display

#endif // BRIDGE_DISPLAY_H
