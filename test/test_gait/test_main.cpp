#include <unity.h>
#include "movement/gait.h"

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
  return UNITY_END();
}
