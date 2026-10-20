#include "gait.h"
#include "kinematics.h"

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
  // De Casteljau: repeated interpolation, no powers.
  Vector3 p[MAX_BEZIER_POINTS];
  for (int i = 0; i < numPoints; i++)
    p[i] = points[i];
  for (int n = numPoints - 1; n > 0; n--)
    for (int i = 0; i < n; i++)
      p[i] = p[i] + (p[i + 1] - p[i]) * t;
  return p[0];
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
  int leg;
};

static int placementFor(int leg)
{
  return (int)(LEG_PLACEMENT_ANGLE * ROTATION_MULTIPLIER[leg]);
}

// Vector3::rotate about the stance pivot by a leg's placement angle, with the
// angle's cosine and sine computed once: the same arithmetic (and integer
// truncation), without trigonometry on every foot target.
struct Placement
{
  int angle;
  float cosA, sinA;
};

static const Placement &placement(int leg)
{
  static Placement table[LEG_COUNT];
  static bool ready = false;
  if (!ready)
  {
    for (int i = 0; i < LEG_COUNT; i++)
    {
      int a = placementFor(i);
      float rad = a * (float)M_PI / 180.0f;
      table[i] = Placement{a, cosf(rad), sinf(rad)};
    }
    ready = true;
  }
  return table[leg];
}

static Vector3 placed(Vector3 v, int leg)
{
  const Placement &pl = placement(leg);
  if (pl.angle == 0)
    return v;
  float px = v.x - STANCE_DISTANCE, py = v.y;
  int xr = px * pl.cosA - py * pl.sinA;
  int yr = px * pl.sinA + py * pl.cosA;
  return Vector3(xr + STANCE_DISTANCE, yr, v.z);
}

// The ground stroke (propel end minus lift end) of each motion alone, in the
// leg's XY plane: the straight stroke rotated by the leg's placement, and the
// rotation stroke along the leg frame's Y axis.
static Vector2 straightStroke(int leg, Vector2 v)
{
  float side = STRIDE_MULTIPLIER[leg];
  const Placement &pl = placement(leg);
  float x = 2 * v.x * side, y = -2 * v.y * side;
  return Vector2(x * pl.cosA - y * pl.sinA, x * pl.sinA + y * pl.cosA);
}

static Vector2 rotationStroke(float rotateStrideLength)
{
  return Vector2(0, 2 * rotateStrideLength);
}

// Reachability shrink for strokes: down to about 13 % of the stroke at worst.
constexpr float MIN_REACH_SCALE = 0.13f;
// The placement rotation truncates to whole millimetres, which moves a point
// by less than this; stroke ends keep it from the reach limit.
constexpr float REACH_TRUNCATION_MARGIN_MM = 1.5f;

// The largest s >= 0 with |p + s d| <= r, or 0 when p itself is beyond r.
static float largestWithin(Vector2 p, Vector2 d, float r)
{
  float c = p.x * p.x + p.y * p.y - r * r;
  if (c > 0)
    return 0;
  float a = d.x * d.x + d.y * d.y;
  if (a <= 0)
    return INFINITY;
  float b = p.x * d.x + p.y * d.y;
  return (-b + sqrtf(b * b - a * c)) / a;
}

// The largest factor, from @p scale down to MIN_REACH_SCALE of it, at which
// every leg's blended stroke ends (both motions amplified by total / own
// weight) are reachable. A stroke end is the stance point plus the factor
// times a direction, and the reach at the ground's height is a circle about
// the coxa pivot, so each end's limit is the root of a quadratic: exact and
// smooth in the input, with no search.
static float reachScale(const Stride &st, float f, float t, float scale, float ground)
{
  const float maxSpan = kinematics::MAX_KNEE_SPAN;
  if (fabsf(ground) >= maxSpan)
    return scale * MIN_REACH_SCALE;
  float r = kinematics::COXA_LENGTH + sqrtf(maxSpan * maxSpan - ground * ground) - REACH_TRUNCATION_MARGIN_MM;
  // A motion with no input has no stroke (and no weight).
  Vector2 v = f >= MIN_INPUT ? st.v * (st.weightSum / f) : Vector2();
  float rot = t >= MIN_INPUT ? st.rotateStrideLength * st.weightSum / t : 0;
  float wS = f / st.weightSum, wR = t / st.weightSum;
  const Vector2 stance(STANCE_DISTANCE, 0);
  float limit = scale;
  // Alone, the straight stroke's ends on legs 3-5 mirror those on legs 0-2
  // (same placement, opposite side), and the rotation stroke's are the same
  // on every leg: fewer ends to solve.
  int legs = wR == 0 ? LEG_COUNT / 2 : (wS == 0 ? 1 : LEG_COUNT);
  for (int leg = 0; leg < legs; leg++)
  {
    const Placement &pl = placement(leg);
    float side = STRIDE_MULTIPLIER[leg];
    for (int end = -1; end <= 1; end += 2) // lift end, propel end
    {
      // The straight end's offset from the stance point, rotated by the
      // placement, blended with the rotation end's.
      float dx = end * v.x * side, dy = -end * v.y * side;
      Vector2 d((dx * pl.cosA - dy * pl.sinA) * wS, (dx * pl.sinA + dy * pl.cosA) * wS + end * rot * wR);
      limit = fminf(limit, largestWithin(stance, d, r));
    }
  }
  return fmaxf(limit, scale * MIN_REACH_SCALE);
}

// The reach factor of one motion of @p st on its own (the other's input zero).
static float aloneReachScale(Stride st, bool straight, float ground)
{
  float f = straight ? fabsf(st.forwardAmount) : 0, t = straight ? 0 : fabsf(st.turnAmount);
  st.weightSum = f + t;
  return reachScale(st, f, t, 1, ground);
}

float mountAngleDeg(int leg)
{
  return (leg < LEG_COUNT / 2 ? 180.0f : 0.0f) + placementFor(leg);
}

// The parts of the stride every leg shares.
static Stride commonStride(const GaitParams &p, const GaitInput &in)
{
  Stride st;
  st.forwardAmount = in.translate.magnitude();
  st.turnAmount = in.rotate;
  st.weightSum = fabsf(st.forwardAmount) + fabsf(st.turnAmount);
  st.rotateStrideLength = in.rotate * in.rotationScale;
  st.v = in.translate;

  if (in.fixedStride > 0)
  {
    // Blended, so a change of stride mode (eased by the body) shortens or
    // lengthens the stride gradually.
    Vector2 fixed = in.translate;
    fixed.normalize();
    fixed = fixed * FIXED_STRIDE_LENGTH;
    float fixedRotate = st.rotateStrideLength < 0 ? -FIXED_STRIDE_LENGTH : FIXED_STRIDE_LENGTH;
    float k = clampf(in.fixedStride, 0, 1);
    st.v = st.v * (1 - k) + fixed * k;
    st.rotateStrideLength = st.rotateStrideLength * (1 - k) + fixedRotate * k;
  }

  st.v = st.v * Vector2(1, p.strideLengthMultiplier);
  st.v.y = clampf(st.v.y, -p.maxStrideLength / 2, p.maxStrideLength / 2);
  st.v = st.v * in.speedScale;

  // Superposition: footTarget blends the two paths with weights that sum to
  // one, so with both inputs each motion is amplified by total / own weight,
  // and the blended stroke is the sum of the two full strokes. The rotation
  // path's bulge stays a weighted average, and blend() lifts a combined swing
  // to the higher of the two paths. Positive rotate turns the body
  // anticlockwise: its stroke moves the left feet forward and the right feet
  // back. When the combined stroke of
  // any leg exceeds the gait's stride limit (or the longer single stroke,
  // so single-motion strokes are never shortened), both motions shrink by
  // the same factor on every leg, which keeps the arc and its legs in step.
  float f = fabsf(st.forwardAmount), t = fabsf(st.turnAmount);
  bool combined = f >= MIN_INPUT && t >= MIN_INPUT;
  float scale = 1;
  if (combined)
  {
    // The single strokes as each motion alone would take them, shortened
    // where they would not reach.
    float straightReach = aloneReachScale(st, true, in.distanceFromGround);
    float rotationReach = aloneReachScale(st, false, in.distanceFromGround);
    // The placement rotation keeps lengths: every leg's single strokes are
    // as long as each other.
    Vector2 r = rotationStroke(st.rotateStrideLength);
    float limit = fmaxf(p.maxStrideLength, fmaxf(2 * st.v.magnitude() * straightReach, fabsf(r.y) * rotationReach));
    for (int i = 0; i < LEG_COUNT; i++)
    {
      float sum = (straightStroke(i, st.v) + r).magnitude();
      if (sum > limit)
        scale = fminf(scale, limit / sum);
    }
  }
  // Shrink further, still together, until every leg's stroke ends are
  // reachable: the ends lie farthest out, a long stride at an extreme
  // height may not reach, and each motion alone may reach while their sum
  // does not.
  if (st.weightSum >= MIN_INPUT)
    scale = reachScale(st, f, t, scale, in.distanceFromGround);
  if (combined)
  {
    st.v = st.v * (scale * st.weightSum / f);
    st.rotateStrideLength *= scale * st.weightSum / t;
  }
  else
  {
    st.v = st.v * scale;
    st.rotateStrideLength *= scale;
  }

  st.ground = in.distanceFromGround;
  st.lift = LIFT_HEIGHT * p.liftHeightMultiplier;
  return st;
}

static bool sameInput(const GaitInput &a, const GaitInput &b)
{
  return a.translate.x == b.translate.x && a.translate.y == b.translate.y && a.rotate == b.rotate &&
         a.fixedStride == b.fixedStride && a.distanceFromGround == b.distanceFromGround &&
         a.speedScale == b.speedScale && a.rotationScale == b.rotationScale;
}

static Stride strideFor(int leg, const GaitParams &p, const GaitInput &in)
{
  // Every leg of a step shares the stride: computed once for the same input.
  static const GaitParams *lastParams = nullptr;
  static GaitInput lastInput;
  static Stride last;
  if (lastParams != &p || !sameInput(lastInput, in))
  {
    last = commonStride(p, in);
    lastParams = &p;
    lastInput = in;
  }
  Stride st = last;
  st.side = STRIDE_MULTIPLIER[leg];
  st.leg = leg;
  return st;
}

static Vector3 blend(const Stride &st, Vector3 straightPoint, Vector3 rotatePoint)
{
  Vector3 p = (straightPoint * fabsf(st.forwardAmount) + rotatePoint * fabsf(st.turnAmount)) / st.weightSum;
  // With both motions, the swing lifts as high as the higher of the two.
  if (fabsf(st.forwardAmount) >= MIN_INPUT && fabsf(st.turnAmount) >= MIN_INPUT)
    p.z = fmaxf(straightPoint.z, rotatePoint.z);
  return p;
}

// End of the ground path (start of the swing).
static Vector3 propelEnd(const Stride &st)
{
  return blend(st,
               placed(Vector3(st.v.x * st.side + STANCE_DISTANCE, -st.v.y * st.side, st.ground), st.leg),
               Vector3(STANCE_DISTANCE, st.rotateStrideLength, st.ground));
}

// End of the swing path (start of the next ground phase).
static Vector3 liftEnd(const Stride &st)
{
  return blend(st,
               placed(Vector3(-st.v.x * st.side + STANCE_DISTANCE, st.v.y * st.side, st.ground), st.leg),
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
    {
      cycle.start = liftEnd(st);
      cycle.segmentStart = 0;
    }
    else if (cycle.phase != LegPhase::Propelling)
    {
      cycle.start = current;
      cycle.segmentStart = t;
    }
    cycle.phase = LegPhase::Propelling;
  }
  else
  {
    if (cycle.phase == LegPhase::Propelling)
    {
      cycle.start = propelEnd(st);
      cycle.segmentStart = p.pushFraction;
    }
    else if (cycle.phase != LegPhase::Lifting)
    {
      cycle.start = current;
      cycle.segmentStart = t;
    }
    cycle.phase = LegPhase::Lifting;
  }
}

void trackPhase(int leg, float t, const GaitParams &p, const GaitInput &in,
                LegCycle &cycle, Vector3 current)
{
  if (inputWeight(in) < MIN_INPUT)
    return;
  // The stride is needed only where the leg changes phase.
  LegPhase now = t < p.pushFraction ? LegPhase::Propelling : LegPhase::Lifting;
  if (cycle.phase == now)
    return;
  trackPhase(t, p, strideFor(leg, p, in), cycle, current);
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
    float s = mapFloat(t, cycle.segmentStart, p.pushFraction, 0, 1);

    straight[0] = cycle.start;
    straight[1] = placed(Vector3(st.v.x * st.side + STANCE_DISTANCE, -st.v.y * st.side, st.ground), st.leg);
    straightPoint = bezier(straight, 2, s);

    rotation[0] = cycle.start;
    rotation[1] = Vector3(STANCE_DISTANCE + 40, 0, st.ground);
    rotation[2] = Vector3(STANCE_DISTANCE, st.rotateStrideLength, st.ground);
    rotatePoint = bezier(rotation, 3, s);
  }
  else
  {
    // Lifting: swing the foot to the next stride start.
    float s = mapFloat(t, cycle.segmentStart, 1, 0, 1);

    straight[0] = cycle.start;
    straight[1] = cycle.start + Vector3(0, 0, st.lift);
    straight[2] = placed(Vector3(-st.v.x * st.side + STANCE_DISTANCE, (st.v.y + STRIDE_OVERSHOOT) * st.side, st.ground + LAND_HEIGHT), st.leg);
    straight[3] = placed(Vector3(-st.v.x * st.side + STANCE_DISTANCE, st.v.y * st.side, st.ground), st.leg);
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
