#ifndef JOINT_CALIBRATION_H
#define JOINT_CALIBRATION_H

// Per-joint calibration: the angle offset added by the IK, the pulse widths
// for 0° and 180°, and the direction. The robot keeps two tables: requested
// (the latest accepted values, after clamping) and applied (what the IK and
// servo output use), which follows requested at a bounded rate.
// Hardware-free: compiled for the board and for host tests.
#include <stdint.h>
#include "kinematics.h"
#include "vectors.h"

namespace joint_calibration
{

// One joint per servo output, in servoOutputIndex order.
constexpr int JOINT_COUNT = kinematics::LEG_COUNT * kinematics::JOINTS_PER_LEG;

// Guards on a requested joint.
constexpr float MAX_OFFSET_FROM_DEFAULT_DEG = 30;
constexpr float MIN_PULSE_US = 400;
constexpr float MAX_PULSE_US = 2600;
constexpr float MIN_PULSE_SPAN_US = 1000; // between the 0° and 180° pulses

// Fastest a calibration change may move a joint's output, in degrees through
// the joint's current calibration.
constexpr float MAX_RATE_DEG_PER_S = 60;

struct Joint
{
  float offsetDeg;
  // Whole microseconds in the defaults and in requested tables; applied
  // values move through fractions while they follow a change.
  float pulse0Us;
  float pulse180Us;
  bool reversed;
};

struct Table
{
  Joint joints[JOINT_COUNT];
};

// The compiled defaults (calibration_defaults.cpp, fixed format).
extern const Joint DEFAULTS[JOINT_COUNT];

/**
 * @brief Fills @p t with the compiled defaults.
 */
void loadDefaults(Table &t);

/**
 * @brief Clamps a requested joint to its safe range: the offset within
 * MAX_OFFSET_FROM_DEFAULT_DEG of @p defaults, each pulse within
 * MIN_PULSE_US..MAX_PULSE_US, and the pulses at least MIN_PULSE_SPAN_US apart
 * (the 180° pulse moves away from the 0° pulse, on its own side; if it would
 * leave the pulse range, it stops at the limit and the 0° pulse makes room).
 * @return true if any value changed
 */
bool clampJoint(Joint &j, const Joint &defaults);

/**
 * @brief Moves @p applied towards @p requested by @p dtS seconds' worth of
 * change, so no joint's output moves faster than MAX_RATE_DEG_PER_S. Per
 * joint, the offset converges first, then the pulse end points. A change of
 * direction is approached as the equivalent swapped end points, so the
 * output never jumps; the direction flips only once the output is the same
 * either way.
 */
void step(Table &applied, const Table &requested, float dtS);

/**
 * @brief The pulse for a joint angle: clamped to 0–180°, mirrored to 180° minus
 * the angle when reversed, and interpolated between the joint's end points.
 */
uint16_t toPulse(const Joint &j, float angleDeg);

/**
 * @brief A leg's (coxa, femur, tibia) offsets, for kinematics::solveLeg.
 */
Vector3 legOffsets(const Table &t, int leg);

} // namespace joint_calibration

#endif // JOINT_CALIBRATION_H
