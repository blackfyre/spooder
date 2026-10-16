#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

// Bring-up input over the USB serial console: single characters select
// movement commands until the gamepad bridge exists.
#include <stdint.h>
#include "movement/movement_command.h"

namespace serial_commands
{

struct Input
{
  MovementCommand command; // stamped with the time it was read
  bool clearFaults;        // '!' pressed since the last poll
  bool printReadings;      // 'p' pressed since the last poll
};

/**
 * @brief Reads pending characters without blocking.
 *
 *   s  stand            w  walk forward      r  walk rotating
 *   x  walk, no input   g  next gait         f  toggle fixed stride
 *   z  sleep            c  calibrate         +/-  body height
 *   !  clear faults     p  print power readings
 */
Input poll(uint32_t nowMs);

} // namespace serial_commands

#endif // SERIAL_COMMANDS_H
