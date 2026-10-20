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
  TEST_ASSERT_TRUE(solveLeg(Vector3(COXA_LENGTH + MAX_KNEE_SPAN, 0, 0), NO_OFFSETS, a));
}

void test_target_near_the_straight_knee_is_rejected()
{
  // Within the old 340 mm from the coxa pivot, but the knee would be (nearly)
  // straight: the joint angles change without bound there.
  JointAngles a = {1, 2, 3};
  TEST_ASSERT_FALSE(solveLeg(Vector3(COXA_LENGTH + MAX_KNEE_SPAN + 1, 0, 0), NO_OFFSETS, a));
  TEST_ASSERT_FALSE(solveLeg(Vector3(200, 0, -250), NO_OFFSETS, a)); // 296 mm from the femur joint, 320 mm from the coxa pivot
  TEST_ASSERT_EQUAL_FLOAT(2, a.femur);
}

static bool inTravel(const JointAngles &a)
{
  return a.coxa >= 0 && a.coxa <= 180 && a.femur >= 0 && a.femur <= 180 && a.tibia >= 0 && a.tibia <= 180;
}

void test_target_in_travel_is_solved_unchanged()
{
  const Vector3 offsets(90, 45, 0);
  Vector3 target(190, 20, -60);
  JointAngles plain, fitted;
  TEST_ASSERT_TRUE(solveLeg(target, offsets, plain));
  TEST_ASSERT_TRUE(inTravel(plain));
  Vector3 moved = target;
  TEST_ASSERT_TRUE(solveLegInRange(moved, offsets, fitted));
  TEST_ASSERT_TRUE(moved == target);
  TEST_ASSERT_EQUAL_FLOAT(plain.coxa, fitted.coxa);
  TEST_ASSERT_EQUAL_FLOAT(plain.femur, fitted.femur);
  TEST_ASSERT_EQUAL_FLOAT(plain.tibia, fitted.tibia);
}

void test_lift_past_the_femur_travel_is_fitted_continuously()
{
  // A femur offset of 89 deg: lifting the foot soon needs more than 180 deg.
  const Vector3 offsets(90, 89, -16);
  Vector3 last;
  bool first = true, fitted = false;
  for (int i = 0; i <= 200; i++)
  {
    Vector3 target(190, 0, -60 + i); // straight up, in 1 mm steps
    Vector3 foot = target;
    JointAngles a;
    TEST_ASSERT_TRUE(solveLegInRange(foot, offsets, a));
    TEST_ASSERT_TRUE(inTravel(a));
    JointAngles plain;
    solveLeg(target, offsets, plain);
    if (!inTravel(plain))
    {
      fitted = true;
      TEST_ASSERT_FLOAT_WITHIN(0.05f, 180, a.femur); // held at the limit
    }
    // The fitted foot reproduces its angles and never jumps.
    JointAngles check;
    TEST_ASSERT_TRUE(solveLeg(foot, offsets, check));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, a.femur, check.femur);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, a.tibia, check.tibia);
    if (!first)
      TEST_ASSERT_TRUE(foot.distanceTo(last) < 2);
    last = foot;
    first = false;
  }
  TEST_ASSERT_TRUE(fitted);
}

void test_straight_knee_past_the_tibia_travel_is_fitted()
{
  // A tibia offset of -26 deg: the knee may open to 154 deg only.
  const Vector3 offsets(90, 45, -26);
  Vector3 target(41 + 293, 0, 0);
  JointAngles plain, a;
  TEST_ASSERT_TRUE(solveLeg(target, offsets, plain));
  TEST_ASSERT_TRUE(plain.tibia < 0);
  TEST_ASSERT_TRUE(solveLegInRange(target, offsets, a));
  TEST_ASSERT_TRUE(inTravel(a));
  TEST_ASSERT_TRUE(target.x < 41 + 293); // drawn in along the same line
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, target.z);
}

void test_direction_past_the_coxa_travel_is_fitted()
{
  const Vector3 offsets(30, 45, 0); // the coxa reaches -30 deg at most
  Vector3 target(150, -150, -60);  // -45 deg
  JointAngles a;
  TEST_ASSERT_TRUE(solveLegInRange(target, offsets, a));
  TEST_ASSERT_TRUE(inTravel(a));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0, a.coxa);
}

void test_nan_target_is_rejected()
{
  JointAngles a;
  TEST_ASSERT_FALSE(solveLeg(Vector3(NAN, 0, 0), NO_OFFSETS, a));
  TEST_ASSERT_FALSE(solveLeg(Vector3(200, 0, INFINITY), NO_OFFSETS, a));
}

void test_femur_offset_scenario()
{
  // Spec scenario: a 5° femur offset in the calibration pose gives 95°.
  JointAngles a;
  TEST_ASSERT_TRUE(solveLeg(Vector3(224, 0, 116), Vector3(0, 5, 0), a));
  TEST_ASSERT_FLOAT_WITHIN(0.5, 95, a.femur);
}

void test_servo_output_mapping()
{
  // Spec scenario: leg 5's femur is servo output 14 (GPIO13).
  TEST_ASSERT_EQUAL_INT(13, servoOutputIndex(4, 1));
  TEST_ASSERT_EQUAL_INT(0, servoOutputIndex(0, 0));
  TEST_ASSERT_EQUAL_INT(17, servoOutputIndex(5, 2));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_calibration_pose_gives_right_angles);
  RUN_TEST(test_offsets_are_added);
  RUN_TEST(test_unreachable_target_is_rejected_and_output_untouched);
  RUN_TEST(test_target_at_reach_is_accepted);
  RUN_TEST(test_target_near_the_straight_knee_is_rejected);
  RUN_TEST(test_target_in_travel_is_solved_unchanged);
  RUN_TEST(test_lift_past_the_femur_travel_is_fitted_continuously);
  RUN_TEST(test_straight_knee_past_the_tibia_travel_is_fitted);
  RUN_TEST(test_direction_past_the_coxa_travel_is_fitted);
  RUN_TEST(test_nan_target_is_rejected);
  RUN_TEST(test_femur_offset_scenario);
  RUN_TEST(test_servo_output_mapping);
  return UNITY_END();
}
