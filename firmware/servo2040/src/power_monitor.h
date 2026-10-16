#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

// Servo supply current and voltage via the Servo 2040's analog multiplexer.
#include "protection/protection.h"

namespace power_monitor
{

void setup();

/**
 * @brief Averages several samples of the supply current (A) and voltage (V).
 * Takes roughly 0.1 ms.
 */
protection::Reading read();

} // namespace power_monitor

#endif // POWER_MONITOR_H
