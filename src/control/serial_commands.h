#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

// Temporary hardware-check input: single characters on the USB serial port
// select movement commands until the gamepad controller exists.
// Built only with -DSPOODER_SERIAL_COMMANDS.
#ifdef SPOODER_SERIAL_COMMANDS

#include <stdint.h>
#include "../movement/movement_command.h"

/**
 * @brief Reads pending serial characters and returns the current command,
 * stamped with @p nowMs.
 *
 *   s  stand            w  walk forward      r  walk rotating
 *   x  walk, no input   g  next gait         f  toggle fixed stride
 *   z  sleep            c  calibrate         +/-  body height
 */
MovementCommand serialCommand(uint32_t nowMs);

#endif // SPOODER_SERIAL_COMMANDS
#endif // SERIAL_COMMANDS_H
