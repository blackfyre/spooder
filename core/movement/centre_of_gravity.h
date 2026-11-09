#ifndef CENTRE_OF_GRAVITY_H
#define CENTRE_OF_GRAVITY_H

// The robot's estimated centre of gravity: point masses for the body, the
// battery and each leg's servos and links, placed by forward kinematics of
// the joint angles on the leg mounts (body_geometry), and the support margin of the feet on
// the ground. A static estimate (no accelerations, no foot forces).
// Hardware-free: compiled for host tests only; the desktop's legview has a
// copy, pinned by test/golden/centre_of_gravity.txt.
//
// Body frame: x right, y forward, z up, millimetres, the coxa pivots at z = 0.
#include "body_geometry.h"
#include "kinematics.h"
#include "vectors.h"

namespace centre_of_gravity
{

using kinematics::LEG_COUNT;

// Masses in grams, all weighed but the links' split. The coxa servo sits on
// the frame at the coxa pivot; it is counted with each leg's masses, at the
// pivot, which places it the same, so BODY_G is the frame as weighed with
// its six coxa servos (CENTRE_G) less those servos. A leg (the coxa bracket outward: femur
// and tibia servos and the links) weighed LEG_G; the links share what the
// two servos leave of it, in the earlier estimates' ratio, where along each
// link unknown.
constexpr float CENTRE_G = 827;    // frame, Servo 2040, BEC, wiring and the six coxa servos, weighed
constexpr float BODY_G = 497;      // CENTRE_G less the six coxa servos; no battery
constexpr float BATTERY_G = 284;   // 2S 5000 mAh, weighed
constexpr float SERVO_G = 55;      // MG996R with horn and screws, weighed; one per joint
constexpr float LEG_G = 303;       // one leg outward of the coxa servo, weighed
constexpr float COXA_LINK_G = 46;  // halfway along the coxa
constexpr float FEMUR_LINK_G = 70; // halfway along the femur
constexpr float TIBIA_LINK_G = 77; // TIBIA_LINK_AT of the way from the knee to the foot
static_assert(BODY_G + 6 * SERVO_G == CENTRE_G, "the body is the weighed centre less its coxa servos");
static_assert(2 * SERVO_G + COXA_LINK_G + FEMUR_LINK_G + TIBIA_LINK_G == LEG_G, "the links share the weighed leg");
constexpr float TIBIA_LINK_AT = 0.4f;

// Where the body's and the battery's masses sit.
const Vector3 BODY_AT(0, 0, 0);
const Vector3 BATTERY_AT(0, 0, 0);

// A foot higher than this above the lowest foot is off the ground.
constexpr float CONTACT_MM = 10;

// The underside (body_geometry::UNDERSIDE_Z) is on the ground when it lies no
// higher than this above the lowest foot.
constexpr float UNDERSIDE_CONTACT_MM = 1;

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

/**
 * @brief Whether the level body rests on its underside: no foot (body frame)
 * lies more than UNDERSIDE_CONTACT_MM below body_geometry::UNDERSIDE_Z, so
 * the frame, not the feet, carries the body (add-gestures design D3). The
 * frame touches the ground under the coxa pivots (body_geometry::PIVOTS).
 */
bool undersideGrounded(const Vector3 feet[LEG_COUNT]);

} // namespace centre_of_gravity

#endif // CENTRE_OF_GRAVITY_H
