#ifndef MOVEMENT_H
#define MOVEMENT_H

// Public movement API. A controller builds MovementCommand values; movement
// owns every body, gait and leg state transition. Call movementTick once per
// main-loop iteration: it advances by elapsed time and always returns.
#include <stdint.h>
#include "movement_command.h"

/**
 * @brief Starts the servo drivers and puts the body into Initialise.
 */
void setupMovement();

/**
 * @brief Advances movement to @p nowMs (millis()) and writes servo output.
 */
void movementTick(uint32_t nowMs, const MovementCommand &command);

#endif // MOVEMENT_H
