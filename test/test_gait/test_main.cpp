#include <unity.h>
#include "movement/gait.h"
#include "movement/kinematics.h"
#include <math.h>

using namespace gait;

static GaitInput input(Vector2 translate, float rotate, bool fixedStride = false)
{
  GaitInput in;
  in.translate = translate;
  in.rotate = rotate;
  in.fixedStride = fixedStride;
  in.distanceFromGround = 0;
  in.speedScale = 0.54f;
  in.rotationScale = 0.55f;
  return in;
}

static LegCycle freshCycle()
{
  return LegCycle{LegPhase::Reset, Vector3()};
}

static const Vector3 STANDING(STANCE_DISTANCE, 0, 0);

// The nominal path at cycle position t: enters t's phase at its start first,
// as a walking leg does (a leg entering part-way starts from its foot).
static Vector3 onPath(int leg, float t, const GaitParams &p, const GaitInput &in, LegCycle &cycle, Vector3 current)
{
  current = footTarget(leg, t < p.pushFraction ? 0 : p.pushFraction, p, in, cycle, current);
  return footTarget(leg, t, p, in, cycle, current);
}

void setUp() {}
void tearDown() {}

void test_bezier_end_points()
{
  Vector3 pts[4] = {Vector3(0, 0, 0), Vector3(10, 50, 0), Vector3(40, 50, 10), Vector3(100, 0, 20)};
  TEST_ASSERT_TRUE(bezier(pts, 4, 0) == pts[0]);
  Vector3 end = bezier(pts, 4, 1);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, end.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, end.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 20, end.z);
}

void test_gait_validity_and_phase_offsets()
{
  TEST_ASSERT_TRUE(isValidGait(0));
  TEST_ASSERT_TRUE(isValidGait(GAIT_COUNT - 1));
  TEST_ASSERT_FALSE(isValidGait(GAIT_COUNT));
  TEST_ASSERT_FALSE(isValidGait(-1));

  const GaitParams &tri = params(Gait::Tri);
  TEST_ASSERT_EQUAL_FLOAT(0, tri.phaseOffset[0]);
  TEST_ASSERT_EQUAL_FLOAT(CYCLE_POINTS / 2, tri.phaseOffset[1]);

  const GaitParams &wave = params(Gait::Wave);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, CYCLE_POINTS * 5 / 6, wave.phaseOffset[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 5.0f / 6.0f, wave.pushFraction);
}

void test_zero_input_holds_current_position()
{
  LegCycle cycle = freshCycle();
  Vector3 current(200, 10, -5);
  Vector3 p = footTarget(0, 0.3f, params(Gait::Tri), input(Vector2(0, 0), 0), cycle, current);
  TEST_ASSERT_TRUE(p == current);
  TEST_ASSERT_EQUAL_FLOAT(0, progressRate(params(Gait::Tri), input(Vector2(0, 0), 0)));
}

void test_rotate_only_input_is_finite_through_cycle()
{
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    LegCycle cycle = freshCycle();
    Vector3 current = STANDING;
    for (float t = 0; t < 1; t += 0.05f)
    {
      current = footTarget(leg, t, params(Gait::Ripple), input(Vector2(0, 0), 60), cycle, current);
      TEST_ASSERT_TRUE(current.isFinite());
    }
  }
}

void test_translate_and_rotate_input_is_finite_for_every_gait()
{
  for (int g = 0; g < GAIT_COUNT; g++)
  {
    LegCycle cycle = freshCycle();
    Vector3 current = STANDING;
    for (float t = 0; t < 1; t += 0.05f)
    {
      current = footTarget(2, t, params(static_cast<Gait>(g)), input(Vector2(40, -70), -30), cycle, current);
      TEST_ASSERT_TRUE(current.isFinite());
    }
  }
}

// End of the ground phase: the foot reaches the stride end point.
static Vector3 strideEnd(Vector2 translate, bool fixedStride)
{
  const GaitParams &p = params(Gait::Tri);
  LegCycle cycle = freshCycle();
  return onPath(1, p.pushFraction - 1e-4f, p, input(translate, 0, fixedStride), cycle, STANDING);
}

void test_fixed_stride_ignores_magnitude()
{
  Vector3 small = strideEnd(Vector2(0, 30), true);
  Vector3 large = strideEnd(Vector2(0, 90), true);
  TEST_ASSERT_FLOAT_WITHIN(0.5, small.x, large.x);
  TEST_ASSERT_FLOAT_WITHIN(0.5, small.y, large.y);
}

void test_dynamic_stride_scales_with_magnitude()
{
  Vector3 small = strideEnd(Vector2(0, 30), false);
  Vector3 large = strideEnd(Vector2(0, 90), false);
  TEST_ASSERT_TRUE(fabsf(large.y - STANDING.y) > fabsf(small.y - STANDING.y) + 5);
}

void test_cycle_start_captured_on_phase_change()
{
  const GaitParams &p = params(Gait::Tri);
  LegCycle cycle = freshCycle();
  Vector3 current(195, 5, 0);
  footTarget(0, 0.1f, p, input(Vector2(0, 50), 0), cycle, current);
  TEST_ASSERT_TRUE(cycle.phase == LegPhase::Propelling);
  TEST_ASSERT_TRUE(cycle.start == current);

  // Propelling -> Lifting starts the swing from the end of the ground path,
  // not from wherever the last step left the foot.
  LegCycle reference = freshCycle();
  Vector3 propelEnd = onPath(0, p.pushFraction - 1e-6f, p, input(Vector2(0, 50), 0), reference, current);
  Vector3 later(180, -20, 0);
  footTarget(0, 0.9f, p, input(Vector2(0, 50), 0), cycle, later);
  TEST_ASSERT_TRUE(cycle.phase == LegPhase::Lifting);
  TEST_ASSERT_FLOAT_WITHIN(0.05, 0, cycle.start.distanceTo(propelEnd));
}

void test_entering_mid_swing_starts_from_the_foot()
{
  // A leg entering its cycle part-way through the swing (entering Walk, a
  // gait change) starts from where its foot is and lands at the swing's end.
  const GaitParams &p = params(Gait::Bi);
  GaitInput in = input(Vector2(0, 50), 0);
  float entry = (p.pushFraction + 1) / 2;
  LegCycle cycle = freshCycle();
  Vector3 first = footTarget(2, entry, p, in, cycle, STANDING);
  TEST_ASSERT_TRUE(cycle.phase == LegPhase::Lifting);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, first.distanceTo(STANDING));
  // Small steps of the cycle move the foot by small amounts.
  Vector3 at = first;
  for (int i = 1; i <= 100; i++)
  {
    Vector3 next = footTarget(2, entry + (1 - 1e-6f - entry) * i / 100, p, in, cycle, at);
    TEST_ASSERT_TRUE(next.distanceTo(at) < 10);
    at = next;
  }
  LegCycle reference = freshCycle();
  TEST_ASSERT_FLOAT_WITHIN(0.05, 0, at.distanceTo(onPath(2, 1 - 1e-6f, p, in, reference, STANDING)));
}

// --- steering: superposed translate and rotate --------------------------------

// The leg's ground stroke (propel end minus lift end) in its XY plane.
static Vector2 groundStroke(int leg, const GaitParams &p, const GaitInput &in)
{
  LegCycle a = freshCycle(), b = freshCycle();
  Vector3 propel = onPath(leg, p.pushFraction - 1e-6f, p, in, a, STANDING);
  Vector3 lift = onPath(leg, 1 - 1e-6f, p, in, b, STANDING);
  return Vector2(propel.x - lift.x, propel.y - lift.y);
}

static bool cycleReachable(int leg, const GaitParams &p, const GaitInput &in)
{
  LegCycle cycle = freshCycle();
  Vector3 current = STANDING;
  for (int i = 0; i <= 200; i++)
  {
    current = footTarget(leg, i / 200.0f * 0.9999f, p, in, cycle, current);
    if (!kinematics::isValidTarget(current))
      return false;
  }
  return true;
}

static GaitInput steering(Vector2 translate, float rotate, float ground)
{
  GaitInput in = input(translate, rotate);
  in.distanceFromGround = ground;
  in.speedScale = 1.1f; // speed dial 100
  return in;
}

void test_turning_while_walking_keeps_the_forward_stroke()
{
  // Each leg's combined stroke is its straight stroke plus its rotation
  // stroke: adding a turn no longer takes stride away from walking.
  for (int g = 0; g < GAIT_COUNT; g++)
  {
    const GaitParams &p = params((Gait)g);
    for (int leg = 0; leg < LEG_COUNT; leg++)
    {
      Vector2 walk = groundStroke(leg, p, input(Vector2(0, 60), 0));
      Vector2 turn = groundStroke(leg, p, input(Vector2(0, 0), 30));
      Vector2 both = groundStroke(leg, p, input(Vector2(0, 60), 30));
      // Within the integer truncation of the placement rotation.
      TEST_ASSERT_FLOAT_WITHIN(3, walk.x + turn.x, both.x);
      TEST_ASSERT_FLOAT_WITHIN(3, walk.y + turn.y, both.y);
    }
  }
}

void test_positive_rotate_turns_the_body_anticlockwise()
{
  // Walking forward moves every grounded foot backward relative to the body.
  // A positive rotate moves the left feet (legs 1-3) forward and the right
  // feet back, so the body's right side advances: an anticlockwise turn.
  const GaitParams &p = params(Gait::Tri);
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    Vector2 forward = groundStroke(leg, p, input(Vector2(0, 50), 0));
    Vector2 turn = groundStroke(leg, p, input(Vector2(0, 0), 50));
    bool left = leg < 3;
    float along = forward.x * turn.x + forward.y * turn.y; // > 0: foot moves backward too
    TEST_ASSERT_TRUE_MESSAGE(left ? along < 0 : along > 0, "rotation direction");
  }
}

void test_combined_swing_lifts_to_the_higher_path()
{
  const GaitParams &p = params(Gait::Tri);
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    float t = (p.pushFraction + 1) / 2; // mid-swing
    LegCycle a = freshCycle(), b = freshCycle(), c = freshCycle();
    float walk = footTarget(leg, t, p, input(Vector2(0, 60), 0), a, STANDING).z;
    float turn = footTarget(leg, t, p, input(Vector2(0, 0), 30), b, STANDING).z;
    float both = footTarget(leg, t, p, input(Vector2(0, 60), 30), c, STANDING).z;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, fmaxf(walk, turn), both);
  }
}

void test_single_motions_are_unchanged_by_steering()
{
  // A rotate amount below the walking threshold leaves the straight path as
  // it was, and the same for a tiny translate on the rotation path.
  const GaitParams &p = params(Gait::Tri);
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    const float TS[] = {0.1f, 0.4f, 0.7f, 0.95f};
    for (float t : TS)
    {
      LegCycle a = freshCycle(), b = freshCycle();
      Vector3 walk = footTarget(leg, t, p, input(Vector2(20, 70), 0), a, STANDING);
      Vector3 nearly = footTarget(leg, t, p, input(Vector2(20, 70), MIN_INPUT / 2), b, STANDING);
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, walk.distanceTo(nearly));
    }
  }
}

// Whether a single motion's stroke at full speed is its full length, not
// shortened for reach: it is twice the stroke at half the speed (and, for a
// dynamic stride, half the rotation scale). A fixed rotation does not scale.
static bool fullLength(const GaitParams &p, GaitInput in)
{
  bool rotating = in.translate.magnitude() < MIN_INPUT;
  if (rotating && in.fixedStride)
    return true;
  GaitInput half = in;
  half.speedScale /= 2;
  half.rotationScale /= 2;
  for (int leg = 0; leg < LEG_COUNT; leg++)
    if (fabsf(groundStroke(leg, p, in).magnitude() - 2 * groundStroke(leg, p, half).magnitude()) > 1)
      return false;
  return true;
}

void test_combined_stroke_stays_within_the_stride_limit_and_reachable()
{
  const Vector2 directions[] = {Vector2(0, 100), Vector2(0, -100), Vector2(100, 0), Vector2(70.7f, 70.7f)};
  const float grounds[] = {-60, -60 - 1.7f * 50, -60 - 1.7f * 100}; // body height 0, 50, 100
  const float ROTATES[] = {-100, -40, 40, 100};
  const bool FIXED[] = {false, true};
  for (int g = 0; g < GAIT_COUNT; g++)
  {
    const GaitParams &p = params((Gait)g);
    for (Vector2 d : directions)
      for (float rotate : ROTATES)
        for (float ground : grounds)
          for (bool fixed : FIXED)
          {
            GaitInput both = steering(d, rotate, ground), walk = steering(d, 0, ground), turn = steering(Vector2(), rotate, ground);
            both.fixedStride = walk.fixedStride = turn.fixedStride = fixed;
            float scale = 1;
            bool full = fullLength(p, walk) && fullLength(p, turn);
            for (int leg = 0; leg < LEG_COUNT; leg++)
            {
              Vector2 s = groundStroke(leg, p, walk), r = groundStroke(leg, p, turn), c = groundStroke(leg, p, both);
              float limit = fmaxf(p.maxStrideLength, fmaxf(s.magnitude(), r.magnitude()));
              TEST_ASSERT_TRUE_MESSAGE(c.magnitude() <= limit + 3, "combined stroke above the limit");
              // The combined stroke is the sum, scaled by one common factor:
              // the same on every leg, so the arc keeps its shape. Checked
              // where neither single stroke is shortened for reach, so the
              // sum is of the full strokes.
              Vector2 sum = s + r;
              if (full && sum.magnitude() > 10)
              {
                float k = c.magnitude() / sum.magnitude();
                if (leg == 0)
                  scale = k;
                TEST_ASSERT_FLOAT_WITHIN(0.05f, scale, k);
                TEST_ASSERT_FLOAT_WITHIN(3 + 0.02f * sum.magnitude(), sum.x * k, c.x);
                TEST_ASSERT_FLOAT_WITHIN(3 + 0.02f * sum.magnitude(), sum.y * k, c.y);
              }
              // Where each motion alone reaches every point of its cycle, so
              // does their combination.
              if (cycleReachable(leg, p, walk) && cycleReachable(leg, p, turn))
                TEST_ASSERT_TRUE_MESSAGE(cycleReachable(leg, p, both), "unreachable combined target");
            }
          }
  }
}

void test_long_strides_shrink_to_reach()
{
  // Full forward input at dial 100 and body height 100: the full stride's
  // ends would need a nearly straight knee, so the stroke shortens to reach.
  for (int g = 0; g < GAIT_COUNT; g++)
  {
    const GaitParams &p = params((Gait)g);
    GaitInput high = steering(Vector2(0, 100), 0, -60 - 1.7f * 100), low = steering(Vector2(0, 100), 0, -60);
    for (int leg = 0; leg < LEG_COUNT; leg++)
    {
      TEST_ASSERT_TRUE(cycleReachable(leg, p, high));
      TEST_ASSERT_TRUE(groundStroke(leg, p, high).magnitude() < groundStroke(leg, p, low).magnitude() - 10);
    }
  }
}

// --- static stability ---------------------------------------------------------

// Schematic coxa pivots (the desktop legs pane's and test_gesture's), until the
// body geometry is measured (add-gestures part two).
static const Vector2 PIVOTS[LEG_COUNT] = {
    Vector2(-70, 120), Vector2(-90, 0), Vector2(-70, -120), Vector2(70, -120), Vector2(90, 0), Vector2(70, 120)};

// A leg-frame foot in the body frame (x right, y forward).
static Vector2 toBody(int leg, Vector3 foot)
{
  float a = mountAngleDeg(leg) * (float)M_PI / 180.0f;
  Vector2 x(cosf(a), sinf(a)), y(sinf(a), -cosf(a));
  return Vector2(PIVOTS[leg].x + foot.x * x.x + foot.y * y.x, PIVOTS[leg].y + foot.x * x.y + foot.y * y.y);
}

// How far the body centre (the body frame's origin) lies inside the polygon
// of the feet on the ground: the smallest distance to an edge, negative
// outside. A foot more than 1 mm above @p ground is off it; with fewer than
// three on it there is no polygon.
static float supportMargin(const Vector3 *feet, float ground)
{
  // Anticlockwise around the body: front right, front left, middle left,
  // rear left, rear right, middle right.
  const int ORDER[] = {5, 0, 1, 2, 3, 4};
  Vector2 poly[LEG_COUNT];
  int n = 0;
  for (int leg : ORDER)
    if (feet[leg].z <= ground + 1)
      poly[n++] = toBody(leg, feet[leg]);
  if (n < 3)
    return -1e9f;
  float margin = 1e9f;
  for (int i = 0; i < n; i++)
  {
    Vector2 a = poly[i], b = poly[(i + 1) % n];
    float ex = b.x - a.x, ey = b.y - a.y;
    margin = fminf(margin, (ex * (0 - a.y) - ey * (0 - a.x)) / sqrtf(ex * ex + ey * ey));
  }
  return margin;
}

// The smallest support margin through a walking cycle, every leg at its
// gait's phase offset, after one cycle that puts the feet on their paths.
static float worstSupportMargin(const GaitParams &p, const GaitInput &in)
{
  const int STEPS = 500;
  LegCycle cycles[LEG_COUNT];
  Vector3 feet[LEG_COUNT];
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    cycles[leg] = freshCycle();
    feet[leg] = Vector3(STANCE_DISTANCE, 0, in.distanceFromGround);
  }
  float worst = 1e9f;
  for (int n = 0; n < 2 * STEPS; n++)
  {
    for (int leg = 0; leg < LEG_COUNT; leg++)
    {
      float t = fmodf(n * CYCLE_POINTS / STEPS + p.phaseOffset[leg], CYCLE_POINTS) / CYCLE_POINTS;
      feet[leg] = footTarget(leg, t, p, in, cycles[leg], feet[leg]);
    }
    if (n >= STEPS)
      worst = fminf(worst, supportMargin(feet, in.distanceFromGround));
  }
  return worst;
}

void test_walking_keeps_the_body_over_its_support()
{
  // Tri, Wave and Quad always have three or more feet down around the body
  // centre. Ripple, Bi and Hop are not statically stable: Ripple's phases
  // leave front left, middle right and rear right down at times, a triangle
  // with the body centre on its diagonal edge; Bi has only two feet down at
  // times and Hop none. The pivots are schematic until the body geometry is
  // measured (add-gestures part two), so this guards the gait table's
  // intent, not the physical margin.
  const Gait STABLE[] = {Gait::Tri, Gait::Wave, Gait::Quad};
  const float MARGIN_MM = 20;
  const Vector2 directions[] = {Vector2(0, 100), Vector2(0, -100), Vector2(100, 0), Vector2(-100, 0), Vector2(70.7f, 70.7f), Vector2()};
  const float ROTATES[] = {0, -100, -40, 40, 100};
  const float grounds[] = {-60, -60 - 1.7f * 100}; // body height 0 and 100
  for (Gait g : STABLE)
    for (Vector2 d : directions)
      for (float rotate : ROTATES)
        for (float ground : grounds)
        {
          if (d.magnitude() < 1 && rotate == 0)
            continue;
          float margin = worstSupportMargin(params(g), steering(d, rotate, ground));
          TEST_ASSERT_TRUE_MESSAGE(margin >= MARGIN_MM, "body outside its support");
        }
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_bezier_end_points);
  RUN_TEST(test_gait_validity_and_phase_offsets);
  RUN_TEST(test_zero_input_holds_current_position);
  RUN_TEST(test_rotate_only_input_is_finite_through_cycle);
  RUN_TEST(test_translate_and_rotate_input_is_finite_for_every_gait);
  RUN_TEST(test_fixed_stride_ignores_magnitude);
  RUN_TEST(test_dynamic_stride_scales_with_magnitude);
  RUN_TEST(test_cycle_start_captured_on_phase_change);
  RUN_TEST(test_entering_mid_swing_starts_from_the_foot);
  RUN_TEST(test_long_strides_shrink_to_reach);
  RUN_TEST(test_turning_while_walking_keeps_the_forward_stroke);
  RUN_TEST(test_positive_rotate_turns_the_body_anticlockwise);
  RUN_TEST(test_combined_swing_lifts_to_the_higher_path);
  RUN_TEST(test_single_motions_are_unchanged_by_steering);
  RUN_TEST(test_combined_stroke_stays_within_the_stride_limit_and_reachable);
  RUN_TEST(test_walking_keeps_the_body_over_its_support);
  return UNITY_END();
}
