#include <unity.h>
#include "movement/kinematics.h"

using namespace kinematics;

static const Vector3 NO_OFFSETS(0, 0, 0);

void setUp() {}
void tearDown() {}

void test_calibration_pose_gives_right_angles()
{
  JointAngles a;
  TEST_ASSERT_TRUE(solveLeg(Vector3(224, 0, 116), NO_OFFSETS, a));
  TEST_ASSERT_FLOAT_WITHIN(0.5, 0, a.coxa);
  TEST_ASSERT_FLOAT_WITHIN(0.5, 90, a.femur);
  TEST_ASSERT_FLOAT_WITHIN(0.5, 90, a.tibia);
}

void test_offsets_are_added()
{
  JointAngles a;
  TEST_ASSERT_TRUE(solveLeg(Vector3(224, 0, 116), Vector3(90, 75, -18), a));
  TEST_ASSERT_FLOAT_WITHIN(0.5, 90, a.coxa);
  TEST_ASSERT_FLOAT_WITHIN(0.5, 165, a.femur);
  TEST_ASSERT_FLOAT_WITHIN(0.5, 72, a.tibia);
}

void test_unreachable_target_is_rejected_and_output_untouched()
{
  JointAngles a = {1, 2, 3};
  TEST_ASSERT_FALSE(solveLeg(Vector3(360, 0, 0), NO_OFFSETS, a));
  TEST_ASSERT_EQUAL_FLOAT(1, a.coxa);
  TEST_ASSERT_EQUAL_FLOAT(2, a.femur);
  TEST_ASSERT_EQUAL_FLOAT(3, a.tibia);
}

void test_target_at_reach_is_accepted()
{
  JointAngles a;
  TEST_ASSERT_TRUE(solveLeg(Vector3(LEG_REACH, 0, 0), NO_OFFSETS, a));
}

void test_nan_target_is_rejected()
{
  JointAngles a;
  TEST_ASSERT_FALSE(solveLeg(Vector3(NAN, 0, 0), NO_OFFSETS, a));
  TEST_ASSERT_FALSE(solveLeg(Vector3(200, 0, INFINITY), NO_OFFSETS, a));
}

void test_angle_to_microseconds_range_and_clamp()
{
  TEST_ASSERT_EQUAL_UINT16(500, angleToMicroseconds(0));
  TEST_ASSERT_EQUAL_UINT16(1500, angleToMicroseconds(90));
  TEST_ASSERT_EQUAL_UINT16(2500, angleToMicroseconds(180));
  TEST_ASSERT_EQUAL_UINT16(500, angleToMicroseconds(-20));
  TEST_ASSERT_EQUAL_UINT16(2500, angleToMicroseconds(200));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_calibration_pose_gives_right_angles);
  RUN_TEST(test_offsets_are_added);
  RUN_TEST(test_unreachable_target_is_rejected_and_output_untouched);
  RUN_TEST(test_target_at_reach_is_accepted);
  RUN_TEST(test_nan_target_is_rejected);
  RUN_TEST(test_angle_to_microseconds_range_and_clamp);
  return UNITY_END();
}
