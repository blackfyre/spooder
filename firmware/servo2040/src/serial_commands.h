#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

// Bring-up input over the USB serial console: single characters select
// movement commands until the gamepad bridge exists.
#include <stdint.h>
#include "movement/movement_command.h"

namespace serial_commands
{

constexpr int MAX_CHARS_PER_POLL = 4;

struct Input
{
  MovementCommand command; // stamped with the time it was read
  uint32_t requestSeq;     // incremented on every mode key (s w r x z c)
  bool clearFaults;        // '!' pressed since the last poll
  bool printReadings;      // 'p' pressed since the last poll
};

/**
 * @brief Reads at most MAX_CHARS_PER_POLL pending characters without blocking,
 * so a flood of input cannot hold up the control loop. With @p readKeys false
 * it reads and echoes nothing (the keys wait for the next poll) and returns
 * the current command: the loop does so on a tick that logs the build banner,
 * which must come before any echo.
 *
 *   s  stand            w  walk forward      r  walk rotating
 *   x  walk, no input   g  next gait         f  toggle fixed stride
 *   z  sleep            c  calibrate         +/-  body height
 *   !  clear faults     p  print power readings
 */
Input poll(uint32_t nowMs, bool readKeys = true);

} // namespace serial_commands

#endif // SERIAL_COMMANDS_H
