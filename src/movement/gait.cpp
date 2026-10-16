#include "gait.h"

namespace gait
{

// Legs 0-2 are on one side, 3-5 on the other.
static const float STRIDE_MULTIPLIER[LEG_COUNT] = {1, 1, 1, -1, -1, -1};
// Front / middle / rear placement relative to the body.
static const float ROTATION_MULTIPLIER[LEG_COUNT] = {-1, 0, 1, -1, 0, 1};

constexpr float SIXTH = CYCLE_POINTS / 6;
constexpr float THIRD = CYCLE_POINTS / 3;
constexpr float HALF = CYCLE_POINTS / 2;

static const GaitParams GAITS[GAIT_COUNT] = {
    // Tri
    {{0, HALF, 0, HALF, 0, HALF}, 3.1f / 6.0f, 1, 1.2f, 1.1f, 240, 200},
    // Wave
    {{0, SIXTH, SIXTH * 2, SIXTH * 5, SIXTH * 4, SIXTH * 3}, 5.0f / 6.0f, 0.40f, 2, 1.3f, 150, 160},
    // Ripple
    {{0, SIXTH * 4, SIXTH * 2, SIXTH * 5, SIXTH, SIXTH * 3}, 3.2f / 6.0f, 1, 1.3f, 1, 220, 200},
    // Bi
    {{0, THIRD, THIRD * 2, 0, THIRD, THIRD * 2}, 2.1f / 6.0f, 4, 1, 1.8f, 230, 130},
    // Quad
    {{0, THIRD, THIRD * 2, 0, THIRD, THIRD * 2}, 4.1f / 6.0f, 1, 1.2f, 1.8f, 220, 200},
    // Hop
    {{0, 0, 0, 0, 0, 0}, 3.0f / 6.0f, 1, 1.6f, 2.5f, 240, 200},
};

// The original per-loop rates were tuned for a 20 ms reference step.
constexpr float REFERENCE_STEPS_PER_SECOND = 50;

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

static int binomialCoefficient(int n, int k)
{
  int result = 1;
  for (int i = 1; i <= k; i++)
  {
    result *= (n - (k - i));
    result /= i;
  }
  return result;
}

bool isValidGait(int value)
{
  return value >= 0 && value < GAIT_COUNT;
}

const GaitParams &params(Gait g)
{
  return GAITS[static_cast<int>(g)];
}

Vector3 bezier(const Vector3 *points, int numPoints, float t)
{
  Vector3 pos;
  for (int i = 0; i < numPoints; i++)
  {
    float b = binomialCoefficient(numPoints - 1, i) * powf(1 - t, numPoints - 1 - i) * powf(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
    pos.z += b * points[i].z;
  }
  return pos;
}

float inputWeight(const GaitInput &in)
{
  return fabsf(in.translate.magnitude()) + fabsf(in.rotate);
}

float progressRate(const GaitParams &p, const GaitInput &in)
{
  float perStep = fmaxf(in.translate.magnitude(), fabsf(in.rotate)) * p.speedMultiplier * in.speedScale;
  perStep = clampf(perStep, 0, p.maxSpeed * in.speedScale);
  return perStep * REFERENCE_STEPS_PER_SECOND;
}

// Stride geometry for one leg, shared by phase tracking and path evaluation.
struct Stride
{
  float forwardAmount;
  float turnAmount;
  float weightSum;
  Vector2 v;
  float rotateStrideLength;
  float ground;
  float lift;
  float side;
  int placement;
  Vector2 pivot;
};

static Stride strideFor(int leg, const GaitParams &p, const GaitInput &in)
{
  Stride st;
  st.forwardAmount = in.translate.magnitude();
  st.turnAmount = in.rotate;
  st.weightSum = fabsf(st.forwardAmount) + fabsf(st.turnAmount);
  st.rotateStrideLength = in.rotate * in.rotationScale;
  st.v = in.translate;

  if (in.fixedStride)
  {
    st.v.normalize();
    st.v = st.v * FIXED_STRIDE_LENGTH;
    st.rotateStrideLength = st.rotateStrideLength < 0 ? -FIXED_STRIDE_LENGTH : FIXED_STRIDE_LENGTH;
  }

  st.v = st.v * Vector2(1, p.strideLengthMultiplier);
  st.v.y = clampf(st.v.y, -p.maxStrideLength / 2, p.maxStrideLength / 2);
  st.v = st.v * in.speedScale;

  st.ground = in.distanceFromGround;
  st.lift = LIFT_HEIGHT * p.liftHeightMultiplier;
  st.side = STRIDE_MULTIPLIER[leg];
  st.placement = (int)(LEG_PLACEMENT_ANGLE * ROTATION_MULTIPLIER[leg]);
  st.pivot = Vector2(STANCE_DISTANCE, 0);
  return st;
}

static Vector3 blend(const Stride &st, Vector3 straightPoint, Vector3 rotatePoint)
{
  return (straightPoint * fabsf(st.forwardAmount) + rotatePoint * fabsf(st.turnAmount)) / st.weightSum;
}

// End of the ground path (start of the swing).
static Vector3 propelEnd(const Stride &st)
{
  return blend(st,
               Vector3(st.v.x * st.side + STANCE_DISTANCE, -st.v.y * st.side, st.ground).rotate(st.placement, st.pivot),
               Vector3(STANCE_DISTANCE, st.rotateStrideLength, st.ground));
}

// End of the swing path (start of the next ground phase).
static Vector3 liftEnd(const Stride &st)
{
  return blend(st,
               Vector3(-st.v.x * st.side + STANCE_DISTANCE, st.v.y * st.side, st.ground).rotate(st.placement, st.pivot),
               Vector3(STANCE_DISTANCE, -st.rotateStrideLength, st.ground));
}

// Updates the leg's phase and segment start for cycle position t. A leg
// changing between ground and swing starts the new segment from the end of
// the previous one rather than from wherever the last step left it, so the
// path does not depend on step size.
static void trackPhase(float t, const GaitParams &p, const Stride &st, LegCycle &cycle, Vector3 current)
{
  if (t < p.pushFraction)
  {
    if (cycle.phase == LegPhase::Lifting)
      cycle.start = liftEnd(st);
    else if (cycle.phase != LegPhase::Propelling)
      cycle.start = current;
    cycle.phase = LegPhase::Propelling;
  }
  else
  {
    if (cycle.phase == LegPhase::Propelling)
      cycle.start = propelEnd(st);
    else if (cycle.phase != LegPhase::Lifting)
      cycle.start = current;
    cycle.phase = LegPhase::Lifting;
  }
}

void trackPhase(int leg, float t, const GaitParams &p, const GaitInput &in,
                LegCycle &cycle, Vector3 current)
{
  Stride st = strideFor(leg, p, in);
  if (st.weightSum < MIN_INPUT)
    return;
  trackPhase(t, p, st, cycle, current);
}

Vector3 footTarget(int leg, float t, const GaitParams &p, const GaitInput &in,
                   LegCycle &cycle, Vector3 current)
{
  Stride st = strideFor(leg, p, in);
  if (st.weightSum < MIN_INPUT)
    return current;

  trackPhase(t, p, st, cycle, current);

  Vector3 straight[4];
  Vector3 rotation[5];
  Vector3 straightPoint;
  Vector3 rotatePoint;

  if (cycle.phase == LegPhase::Propelling)
  {
    // Propelling: foot on the ground.
    float s = mapFloat(t, 0, p.pushFraction, 0, 1);

    straight[0] = cycle.start;
    straight[1] = Vector3(st.v.x * st.side + STANCE_DISTANCE, -st.v.y * st.side, st.ground).rotate(st.placement, st.pivot);
    straightPoint = bezier(straight, 2, s);

    rotation[0] = cycle.start;
    rotation[1] = Vector3(STANCE_DISTANCE + 40, 0, st.ground);
    rotation[2] = Vector3(STANCE_DISTANCE, st.rotateStrideLength, st.ground);
    rotatePoint = bezier(rotation, 3, s);
  }
  else
  {
    // Lifting: swing the foot to the next stride start.
    float s = mapFloat(t, p.pushFraction, 1, 0, 1);

    straight[0] = cycle.start;
    straight[1] = cycle.start + Vector3(0, 0, st.lift);
    straight[2] = Vector3(-st.v.x * st.side + STANCE_DISTANCE, (st.v.y + STRIDE_OVERSHOOT) * st.side, st.ground + LAND_HEIGHT).rotate(st.placement, st.pivot);
    straight[3] = Vector3(-st.v.x * st.side + STANCE_DISTANCE, st.v.y * st.side, st.ground).rotate(st.placement, st.pivot);
    straightPoint = bezier(straight, 4, s);

    rotation[0] = cycle.start;
    rotation[1] = cycle.start + Vector3(0, 0, st.lift);
    rotation[2] = Vector3(STANCE_DISTANCE + 40, 0, st.ground + st.lift);
    rotation[3] = Vector3(STANCE_DISTANCE, -(st.rotateStrideLength + STRIDE_OVERSHOOT), st.ground + LAND_HEIGHT);
    rotation[4] = Vector3(STANCE_DISTANCE, -st.rotateStrideLength, st.ground);
    rotatePoint = bezier(rotation, 5, s);
  }

  return blend(st, straightPoint, rotatePoint);
}

} // namespace gait
