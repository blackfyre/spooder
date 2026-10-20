#ifndef GAIT_H
#define GAIT_H

// Hardware-free gait maths: gait table, leg phase and foot paths.
// Compiled for the board and for host tests.
#include <stdint.h>
#include "vectors.h"
#include "kinematics.h"

namespace gait
{

using kinematics::LEG_COUNT;

// Gait cycle length in progress units.
constexpr float CYCLE_POINTS = 1000;

enum class Gait : uint8_t
{
  Tri,
  Wave,
  Ripple,
  Bi,
  Quad,
  Hop,
};
constexpr int GAIT_COUNT = 6;

struct GaitParams
{
  float phaseOffset[LEG_COUNT]; // cycle start per leg, in progress units
  float pushFraction;           // share of the cycle the foot is on the ground
  float speedMultiplier;
  float strideLengthMultiplier;
  float liftHeightMultiplier;
  float maxStrideLength;
  float maxSpeed;
};

enum class LegPhase : uint8_t
{
  Propelling,
  Lifting,
  Standing,
  Reset,
};

// Per-leg cycle state. The cycle start point is captured whenever the leg
// changes phase, so each path segment starts where the foot actually is.
// A leg that enters a phase part-way (entering Walk, a gait change) runs
// that segment from where it entered (segmentStart, a cycle position 0..1)
// to the phase's end, so the foot does not jump onto the path.
struct LegCycle
{
  LegPhase phase;
  Vector3 start;
  float segmentStart = 0;
};

// Body-level inputs to the foot path, already smoothed and scaled.
struct GaitInput
{
  Vector2 translate;         // -100..100 per axis
  float rotate;              // -100..100
  float fixedStride;         // share of the fixed stride (constant stride/rotation length, input sets
                             // direction only) blended with the dynamic one: 0 dynamic, 1 fixed
  float distanceFromGround;  // foot height relative to the coxa pivot
  float speedScale;          // global speed multiplier
  float rotationScale;       // global rotation multiplier
};

constexpr float STANCE_DISTANCE = 190;   // foot distance from the coxa pivot when standing
constexpr float LIFT_HEIGHT = 130;
constexpr float LAND_HEIGHT = 70;
constexpr float STRIDE_OVERSHOOT = 10;
constexpr float LEG_PLACEMENT_ANGLE = 56; // degrees
constexpr float FIXED_STRIDE_LENGTH = 70;
// Below this combined input the walking maths is undefined (0/0); legs hold.
constexpr float MIN_INPUT = 0.01f;

bool isValidGait(int value);

/**
 * @brief Direction of @p leg's frame x axis in the body frame (x right, y
 * forward), in degrees anticlockwise from +x. The leg frame's y axis is its
 * x axis turned 90° clockwise. Matches the placement the gait rotates
 * strides by.
 */
float mountAngleDeg(int leg);
const GaitParams &params(Gait g);

constexpr int MAX_BEZIER_POINTS = 5;

/**
 * @brief Evaluates a Bézier curve with @p numPoints (at most
 * MAX_BEZIER_POINTS) control points at t (0..1).
 */
Vector3 bezier(const Vector3 *points, int numPoints, float t);

/**
 * @brief Combined input weight; walking is defined only above MIN_INPUT.
 */
float inputWeight(const GaitInput &in);

/**
 * @brief Gait progress per second for the given input, in progress units.
 */
float progressRate(const GaitParams &p, const GaitInput &in);

/**
 * @brief Updates the leg's phase and segment start for cycle position t
 * without evaluating the foot path. Call after every integration sub-step so
 * a phase shorter than a tick is not skipped. No-op below MIN_INPUT.
 */
void trackPhase(int leg, float t, const GaitParams &p, const GaitInput &in,
                LegCycle &cycle, Vector3 current);

/**
 * @brief Foot target for one leg at cycle position t (0..1).
 *
 * Updates @p cycle when the leg changes phase. Returns @p current unchanged
 * when the input weight is below MIN_INPUT.
 */
Vector3 footTarget(int leg, float t, const GaitParams &p, const GaitInput &in,
                   LegCycle &cycle, Vector3 current);

} // namespace gait

#endif // GAIT_H
