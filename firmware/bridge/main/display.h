#ifndef BRIDGE_DISPLAY_H
#define BRIDGE_DISPLAY_H

// Status screen on the T-Display's ST7789 (135×240, used landscape).
// update() redraws at most every REFRESH_MS and only the lines that changed;
// the whole screen is cleared only when the link state changes.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "link.h"

namespace display
{

constexpr uint32_t REFRESH_MS = 250;

void setup();
void update(uint32_t nowMs, bool gamepadConnected, const gamepad_mapping::Desired &desired,
            const robot_link::Status &linkStatus);

} // namespace display

#endif // BRIDGE_DISPLAY_H
