#ifndef KINEMATICS_H
#define KINEMATICS_H

// Hardware-free leg inverse kinematics. Compiled for the board and for host tests.
#include <stdint.h>
#include "vectors.h"

namespace kinematics
{

// Link lengths in millimetres.
constexpr float COXA_LENGTH = 41;
constexpr float FEMUR_LENGTH = 116;
constexpr float TIBIA_LENGTH = 183;
// Furthest accepted foot target from the coxa pivot.
constexpr float LEG_REACH = COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH;

constexpr int LEG_COUNT = 6;

// Per-leg calibration offsets in degrees (coxa, femur, tibia), indexed by leg.
// Inherited from upstream Aecert's Hexapod V1; not yet measured on this robot.
extern const Vector3 LEG_OFFSETS[LEG_COUNT];

struct JointAngles
{
  float coxa;
  float femur;
  float tibia;
};

/**
 * @brief True when solveLeg would accept the target: finite and within
 * LEG_REACH of the coxa pivot. Cheap; no trigonometry.
 */
bool isValidTarget(Vector3 target);

/**
 * @brief Solves the joint angles for a foot target in the leg's own frame
 * (origin at the coxa pivot, x outward, z up, millimetres).
 *
 * @return false, leaving @p out untouched, when the target is not finite or
 *         lies further than LEG_REACH from the coxa pivot.
 */
bool solveLeg(Vector3 target, Vector3 offsets, JointAngles &out);

/**
 * @brief Converts a joint angle to a servo pulse width: 0° → 500 µs,
 * 180° → 2500 µs. Angles outside 0–180° are clamped.
 */
uint16_t angleToMicroseconds(float angle);

} // namespace kinematics

#endif // KINEMATICS_H
