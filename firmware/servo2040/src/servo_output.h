#ifndef SERVO_OUTPUT_H
#define SERVO_OUTPUT_H

// Servo output on the Servo 2040's 18 PIO-driven outputs (GPIO0-17).
#include "movement/body.h"
#include "movement/joint_calibration.h"

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
 * @brief Applies one movement step: converts every joint the body has set
 * through @p calibration (so a calibration change moves a joint even when
 * its angle did not change) and switches output on or off as the body and
 * protection require. All changes take effect in the same frame.
 */
void apply(const body::BodyState &body, const joint_calibration::Table &calibration);

/**
 * @brief Switches output off immediately (the body has stopped). Output comes
 * back only through apply() after the body restarts.
 */
void stop();

bool outputsOn();

/**
 * @brief The last pulse set per output, in µs (servo output order; 0 = never
 * set since power-up). Produced while outputsOn().
 */
const uint16_t *pulses();

} // namespace servo_output

#endif // SERVO_OUTPUT_H
