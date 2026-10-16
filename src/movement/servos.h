#ifndef SERVOS_H
#define SERVOS_H

// Servo output on the two PCA9685 drivers. The only code that talks to them.
#include "kinematics.h"

namespace servos
{

/**
 * @brief Starts both drivers at a 50 Hz servo frame rate.
 */
void setup();

/**
 * @brief Writes one leg's joint angles. Switches output back on first if it is off.
 */
void writeLeg(int leg, const kinematics::JointAngles &angles);

/**
 * @brief Switches all eighteen channels off (joints unpowered) or back on.
 * If any channel fails to switch off, outputsEnabled() stays true so the
 * caller retries.
 */
void setOutputsEnabled(bool enabled);

bool outputsEnabled();

} // namespace servos

#endif // SERVOS_H
