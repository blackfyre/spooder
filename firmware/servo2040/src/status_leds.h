#ifndef STATUS_LEDS_H
#define STATUS_LEDS_H

// Robot state on the Servo 2040's six RGB LEDs.
#include <stdint.h>
#include "movement/body.h"

namespace status_leds
{

void setup();

/**
 * @brief Shows the current state. Call every tick; the pattern is time-based.
 *
 * @param supplyAccepted false while protection is still waiting for a valid supply
 * @param faults latched protection faults (protection::Fault bits)
 */
void show(uint32_t nowMs, bool supplyAccepted, uint16_t faults, body::Mode mode);

} // namespace status_leds

#endif // STATUS_LEDS_H
