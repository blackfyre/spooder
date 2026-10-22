#ifndef CENTRE_OF_GRAVITY_H
#define CENTRE_OF_GRAVITY_H

// The robot's estimated centre of gravity: point masses for the body, the
// battery and each leg's servos and links, placed by forward kinematics of
// the joint angles on the leg mounts, and the support margin of the feet on
// the ground. A static estimate (no accelerations, no foot forces).
// Hardware-free: compiled for host tests only; the desktop's legview has a
// copy, pinned by test/golden/centre_of_gravity.txt.
//
// Body frame: x right, y forward, z up, millimetres, the coxa pivots at z = 0.
#include "kinematics.h"
#include "vectors.h"

namespace centre_of_gravity
{

using kinematics::LEG_COUNT;

// Masses in grams. The servo and the battery are weighed; the rest are
// estimates from typical parts until the rebuilt robot is weighed.
constexpr float BODY_G = 420;        // frame, Servo 2040, BEC, wiring; no battery
constexpr float BATTERY_G = 284;     // 2S 5000 mAh, weighed
constexpr float SERVO_G = 55;        // MG996R with horn and screws, weighed; one per joint
constexpr float COXA_LINK_G = 30;    // halfway along the coxa
constexpr float FEMUR_LINK_G = 45;   // halfway along the femur
constexpr float TIBIA_LINK_G = 50;   // TIBIA_LINK_AT of the way from the knee to the foot
constexpr float TIBIA_LINK_AT = 0.4f;

// Where the body's and the battery's masses sit.
const Vector3 BODY_AT(0, 0, 0);
const Vector3 BATTERY_AT(0, 0, 0);

// Schematic coxa pivots (the desktop legs pane's), until the body geometry
// is measured (add-gestures part two). The mount angles are gait's.
const Vector2 PIVOTS[LEG_COUNT] = {
    Vector2(-70, 120), Vector2(-90, 0), Vector2(-70, -120), Vector2(70, -120), Vector2(90, 0), Vector2(70, 120)};

// A foot higher than this above the lowest foot is off the ground.
constexpr float CONTACT_MM = 10;

// A leg's joints in the body frame: coxa pivot, coxa end (femur joint), knee
// and foot.
struct LegPoints
{
  Vector3 points[4];
};

/**
 * @brief Forward kinematics of a leg's commanded angles (before calibration
 * offsets, as solveLeg returns them), placed on its mount.
 */
LegPoints legInBody(int leg, kinematics::JointAngles a);

/** @brief A leg-frame point in the body frame. */
Vector3 toBody(int leg, Vector3 p);

struct Estimate
{
  Vector3 centre; // body frame
  float massG;
};

/** @brief The centre of gravity of the robot with @p legs' angles. */
Estimate estimate(const kinematics::JointAngles legs[LEG_COUNT], bool battery);

/**
 * @brief How far @p point's ground projection lies inside the support
 * polygon, the convex hull of the feet (body frame) within CONTACT_MM of the lowest:
 * the distance to its nearest edge, negative outside.
 *
 * @return false, leaving @p margin untouched, with fewer than three feet on
 *         the ground.
 */
bool supportMargin(const Vector3 feet[LEG_COUNT], Vector3 point, float &margin);

/**
 * @brief The support polygon: the hull of the feet (body frame) on the ground,
 * anticlockwise seen from above, into @p hull (room for LEG_COUNT).
 *
 * @return its corner count; 0 with fewer than three feet on the ground.
 */
int supportHull(const Vector3 feet[LEG_COUNT], Vector2 hull[LEG_COUNT]);

} // namespace centre_of_gravity

#endif // CENTRE_OF_GRAVITY_H
