#ifndef SERVO_OUTPUT_H
#define SERVO_OUTPUT_H

// Servo output on the Servo 2040's 18 PIO-driven outputs (GPIO0-17).
#include "movement/body.h"

namespace servo_output
{

/**
 * @brief Creates the servo cluster on PIO 0. All outputs start off.
 * @return false if the cluster could not claim its PIO/DMA resources.
 */
bool setup();

/**
 * @brief While inhibited (power protection), no output produces a pulse,
 * whatever the body requests.
 */
void setInhibited(bool inhibited);

/**
 * @brief Applies one movement step: writes the joints the body updated and
 * switches output on or off as the body and protection require. All changes
 * take effect in the same frame.
 */
void apply(const body::BodyState &body);

bool outputsOn();

} // namespace servo_output

#endif // SERVO_OUTPUT_H
