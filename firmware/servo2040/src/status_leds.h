#ifndef STATUS_LEDS_H
#define STATUS_LEDS_H

// Robot state on the Servo 2040's six RGB LEDs. The pattern itself is computed
// by core/status/led_pattern (host-tested); this adapter only drives the LEDs.
#include "status/led_pattern.h"

namespace status_leds
{

void setup();

/**
 * @brief Shows the given state. Call every tick; patterns are time-based.
 */
void show(const led_pattern::State &state);

} // namespace status_leds

#endif // STATUS_LEDS_H
