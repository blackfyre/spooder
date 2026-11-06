// The servo and supply model (core/robot/bench_model): the current it draws
// with output off, idle, moving and capped, scaled by the load factor, and a
// fixed current.
#include <unity.h>
#include "robot/bench_model.h"

using namespace bench_model;

static body::BodyState pose;
static joint_calibration::Table calibration;
static Servos servos;
static Supply supply;

// Every joint at @p angleDeg, every leg updated, output enabled.
static void setPose(float angleDeg)
{
  pose = {};
  pose.outputsEnabled = true;
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    pose.angles[leg] = {angleDeg, angleDeg, angleDeg};
    pose.legUpdated[leg] = true;
  }
}

void setUp()
{
  // 500 us at 0 deg to 2500 us at 180 deg: 100/9 us per degree.
  for (int i = 0; i < OUTPUT_COUNT; i++)
    calibration.joints[i] = {0, 500, 2500, false};
  servos = Servos();
  supply = Supply();
  setPose(90);
}
void tearDown() {}

void test_zero_current_with_output_off()
{
  servos.apply(pose, calibration, kinematics::LEG_COUNT); // still inhibited
  TEST_ASSERT_FALSE(servos.on);
  protection::Reading r = supply.read(servos);
  TEST_ASSERT_EQUAL_FLOAT(0, r.currentA);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULT_SUPPLY_V, r.voltageV);

  servos.inhibited = false;
  servos.apply(pose, calibration, kinematics::LEG_COUNT);
  TEST_ASSERT_TRUE(servos.on);
  servos.stop();
  TEST_ASSERT_EQUAL_FLOAT(0, supply.read(servos).currentA);
  TEST_ASSERT_EQUAL_INT(0, servos.legsOn);
}

void test_idle_draw_per_powered_servo()
{
  servos.inhibited = false;
  servos.apply(pose, calibration, 2); // two legs released: six servos
  TEST_ASSERT_EQUAL_INT(6, servos.powered());
  TEST_ASSERT_EQUAL_UINT16(1500, servos.desiredUs[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 6 * IDLE_A_PER_SERVO, supply.read(servos).currentA);

  servos.apply(pose, calibration, kinematics::LEG_COUNT); // unchanged pose: no moving draw
  TEST_ASSERT_EQUAL_INT(OUTPUT_COUNT, servos.powered());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, OUTPUT_COUNT * IDLE_A_PER_SERVO, supply.read(servos).currentA);
}

void test_moving_draw_and_its_cap()
{
  servos.inhibited = false;
  servos.apply(pose, calibration, kinematics::LEG_COUNT);

  setPose(91.8f); // 20 us per joint in one tick
  servos.apply(pose, calibration, kinematics::LEG_COUNT);
  float moving = OUTPUT_COUNT * 20 * A_PER_US_PER_TICK;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, moving, servos.movingA);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, OUTPUT_COUNT * IDLE_A_PER_SERVO + moving, supply.read(servos).currentA);

  setPose(100.8f); // 100 us per joint: above the cap
  servos.apply(pose, calibration, kinematics::LEG_COUNT);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, OUTPUT_COUNT * MAX_MOVING_A_PER_SERVO, servos.movingA);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, OUTPUT_COUNT * (IDLE_A_PER_SERVO + MAX_MOVING_A_PER_SERVO),
                           supply.read(servos).currentA);
}

void test_load_factor_scales_the_draw()
{
  servos.inhibited = false;
  servos.apply(pose, calibration, kinematics::LEG_COUNT);
  setPose(91.8f);
  servos.apply(pose, calibration, kinematics::LEG_COUNT);
  float atLoad1 = supply.read(servos).currentA;
  supply.load = 2.5f;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.5f * atLoad1, supply.read(servos).currentA);
}

void test_fixed_current()
{
  supply.fixedCurrent = true;
  supply.fixedCurrentA = 3.25f;
  supply.voltageV = 5.5f;
  protection::Reading r = supply.read(servos); // output off
  TEST_ASSERT_EQUAL_FLOAT(3.25f, r.currentA);
  TEST_ASSERT_EQUAL_FLOAT(5.5f, r.voltageV);
  servos.inhibited = false;
  servos.apply(pose, calibration, kinematics::LEG_COUNT);
  TEST_ASSERT_EQUAL_FLOAT(3.25f, supply.read(servos).currentA);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_zero_current_with_output_off);
  RUN_TEST(test_idle_draw_per_powered_servo);
  RUN_TEST(test_moving_draw_and_its_cap);
  RUN_TEST(test_load_factor_scales_the_draw);
  RUN_TEST(test_fixed_current);
  return UNITY_END();
}
