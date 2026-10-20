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
// Furthest accepted distance from the femur joint to the foot. Near the
// straight knee the joint angles change without bound for a small foot
// movement; this margin keeps that rate to about 2.5°/mm.
constexpr float STRAIGHT_KNEE_MARGIN_MM = 4;
constexpr float MAX_KNEE_SPAN = FEMUR_LENGTH + TIBIA_LENGTH - STRAIGHT_KNEE_MARGIN_MM;

constexpr int LEG_COUNT = 6;
constexpr int JOINTS_PER_LEG = 3; // coxa, femur, tibia

/**
 * @brief Servo output (0-based; output 1 on the board is index 0) driving a
 * joint: legs in order, coxa, femur, tibia within each leg.
 */
constexpr int servoOutputIndex(int leg, int joint)
{
  return leg * JOINTS_PER_LEG + joint;
}

struct JointAngles
{
  float coxa;
  float femur;
  float tibia;
};

/**
 * @brief True when solveLeg would accept the target: finite, and at most
 * MAX_KNEE_SPAN from the femur joint. Cheap; no trigonometry.
 */
bool isValidTarget(Vector3 target);

/**
 * @brief Solves the joint angles for a foot target in the leg's own frame
 * (origin at the coxa pivot, x outward, z up, millimetres), adding the
 * leg's (coxa, femur, tibia) calibration offsets in degrees.
 *
 * @return false, leaving @p out untouched, when the target is not finite or
 *         lies further than MAX_KNEE_SPAN from the femur joint.
 */
bool solveLeg(Vector3 target, Vector3 offsets, JointAngles &out);

/**
 * @brief As solveLeg, but first moves @p target, as little as needed, so
 * that every joint's servo angle (offset included) lies within 0-180°: the
 * foot's direction into the coxa's travel, its distance from the femur joint
 * into the tibia's, then its elevation about the femur joint into the
 * femur's. The move is continuous in the target, so a path crossing a limit
 * follows it instead of jumping. A target within range is solved exactly as
 * solveLeg solves it, and left unchanged.
 *
 * @return false, leaving @p target and @p out untouched, when solveLeg
 *         would reject the target.
 */
bool solveLegInRange(Vector3 &target, Vector3 offsets, JointAngles &out);

} // namespace kinematics

#endif // KINEMATICS_H
