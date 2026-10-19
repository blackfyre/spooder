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
  return footTarget(1, p.pushFraction - 1e-4f, p, input(translate, 0, fixedStride), cycle, STANDING);
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
  Vector3 propelEnd = footTarget(0, p.pushFraction - 1e-6f, p, input(Vector2(0, 50), 0), reference, current);
  Vector3 later(180, -20, 0);
  footTarget(0, 0.9f, p, input(Vector2(0, 50), 0), cycle, later);
  TEST_ASSERT_TRUE(cycle.phase == LegPhase::Lifting);
  TEST_ASSERT_FLOAT_WITHIN(0.05, 0, cycle.start.distanceTo(propelEnd));
}

// --- steering: superposed translate and rotate --------------------------------

// The leg's ground stroke (propel end minus lift end) in its XY plane.
static Vector2 groundStroke(int leg, const GaitParams &p, const GaitInput &in)
{
  LegCycle a = freshCycle(), b = freshCycle();
  Vector3 propel = footTarget(leg, p.pushFraction - 1e-6f, p, in, a, STANDING);
  Vector3 lift = footTarget(leg, 1 - 1e-6f, p, in, b, STANDING);
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
            for (int leg = 0; leg < LEG_COUNT; leg++)
            {
              Vector2 s = groundStroke(leg, p, walk), r = groundStroke(leg, p, turn), c = groundStroke(leg, p, both);
              float limit = fmaxf(p.maxStrideLength, fmaxf(s.magnitude(), r.magnitude()));
              TEST_ASSERT_TRUE_MESSAGE(c.magnitude() <= limit + 3, "combined stroke above the limit");
              // The combined stroke is the sum, scaled by one common factor:
              // the same on every leg, so the arc keeps its shape.
              Vector2 sum = s + r;
              if (sum.magnitude() > 10)
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
  RUN_TEST(test_turning_while_walking_keeps_the_forward_stroke);
  RUN_TEST(test_positive_rotate_turns_the_body_anticlockwise);
  RUN_TEST(test_combined_swing_lifts_to_the_higher_path);
  RUN_TEST(test_single_motions_are_unchanged_by_steering);
  RUN_TEST(test_combined_stroke_stays_within_the_stride_limit_and_reachable);
  return UNITY_END();
}
