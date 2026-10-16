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

Vector3 footTarget(int leg, float t, const GaitParams &p, const GaitInput &in,
                   LegCycle &cycle, Vector3 current)
{
  float forwardAmount = in.translate.magnitude();
  float turnAmount = in.rotate;
  float weightSum = fabsf(forwardAmount) + fabsf(turnAmount);
  if (weightSum < MIN_INPUT)
    return current;

  float rotateStrideLength = in.rotate * in.rotationScale;
  Vector2 v = in.translate;

  if (in.fixedStride)
  {
    v.normalize();
    v = v * FIXED_STRIDE_LENGTH;
    rotateStrideLength = rotateStrideLength < 0 ? -FIXED_STRIDE_LENGTH : FIXED_STRIDE_LENGTH;
  }

  v = v * Vector2(1, p.strideLengthMultiplier);
  v.y = clampf(v.y, -p.maxStrideLength / 2, p.maxStrideLength / 2);
  v = v * in.speedScale;

  const float ground = in.distanceFromGround;
  const float lift = LIFT_HEIGHT * p.liftHeightMultiplier;
  const float stride = STRIDE_MULTIPLIER[leg];
  const int placement = (int)(LEG_PLACEMENT_ANGLE * ROTATION_MULTIPLIER[leg]);
  const Vector2 pivot(STANCE_DISTANCE, 0);

  // End points of each phase's path. A leg changing phase starts the new
  // segment from the end of the previous one rather than from wherever the
  // last step left it, so the path does not depend on step size.
  auto blend = [&](Vector3 straightEnd, Vector3 rotateEnd) {
    return (straightEnd * fabsf(forwardAmount) + rotateEnd * fabsf(turnAmount)) / weightSum;
  };
  const Vector3 propelEnd = blend(
      Vector3(v.x * stride + STANCE_DISTANCE, -v.y * stride, ground).rotate(placement, pivot),
      Vector3(STANCE_DISTANCE, rotateStrideLength, ground));
  const Vector3 liftEnd = blend(
      Vector3(-v.x * stride + STANCE_DISTANCE, v.y * stride, ground).rotate(placement, pivot),
      Vector3(STANCE_DISTANCE, -rotateStrideLength, ground));

  Vector3 straight[4];
  Vector3 rotation[5];
  Vector3 straightPoint;
  Vector3 rotatePoint;

  if (t < p.pushFraction)
  {
    // Propelling: foot on the ground.
    if (cycle.phase == LegPhase::Lifting)
      cycle.start = liftEnd;
    else if (cycle.phase != LegPhase::Propelling)
      cycle.start = current;
    cycle.phase = LegPhase::Propelling;

    float s = mapFloat(t, 0, p.pushFraction, 0, 1);

    straight[0] = cycle.start;
    straight[1] = Vector3(v.x * stride + STANCE_DISTANCE, -v.y * stride, ground).rotate(placement, pivot);
    straightPoint = bezier(straight, 2, s);

    rotation[0] = cycle.start;
    rotation[1] = Vector3(STANCE_DISTANCE + 40, 0, ground);
    rotation[2] = Vector3(STANCE_DISTANCE, rotateStrideLength, ground);
    rotatePoint = bezier(rotation, 3, s);
  }
  else
  {
    // Lifting: swing the foot to the next stride start.
    if (cycle.phase == LegPhase::Propelling)
      cycle.start = propelEnd;
    else if (cycle.phase != LegPhase::Lifting)
      cycle.start = current;
    cycle.phase = LegPhase::Lifting;

    float s = mapFloat(t, p.pushFraction, 1, 0, 1);

    straight[0] = cycle.start;
    straight[1] = cycle.start + Vector3(0, 0, lift);
    straight[2] = Vector3(-v.x * stride + STANCE_DISTANCE, (v.y + STRIDE_OVERSHOOT) * stride, ground + LAND_HEIGHT).rotate(placement, pivot);
    straight[3] = Vector3(-v.x * stride + STANCE_DISTANCE, v.y * stride, ground).rotate(placement, pivot);
    straightPoint = bezier(straight, 4, s);

    rotation[0] = cycle.start;
    rotation[1] = cycle.start + Vector3(0, 0, lift);
    rotation[2] = Vector3(STANCE_DISTANCE + 40, 0, ground + lift);
    rotation[3] = Vector3(STANCE_DISTANCE, -(rotateStrideLength + STRIDE_OVERSHOOT), ground + LAND_HEIGHT);
    rotation[4] = Vector3(STANCE_DISTANCE, -rotateStrideLength, ground);
    rotatePoint = bezier(rotation, 5, s);
  }

  return (straightPoint * fabsf(forwardAmount) + rotatePoint * fabsf(turnAmount)) / weightSum;
}

} // namespace gait
