// The estimated centre of gravity (core/movement/centre_of_gravity), and its
// golden vectors for the desktop calibrator's copy: a fixed set of poses,
// with and without the battery, compared with
// test/golden/centre_of_gravity.txt, which the Go leg view tests read
// (desktop/internal/legview). After a deliberate change to the model,
// regenerate the file with `SPOODER_WRITE_GOLDEN=1 mise run test` and review
// the diff.
#include <unity.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "movement/body.h"
#include "movement/centre_of_gravity.h"
#include "movement/kinematics.h"

using namespace centre_of_gravity;
using kinematics::JointAngles;

void setUp() {}
void tearDown() {}

static JointAngles solved(Vector3 foot)
{
  JointAngles a;
  TEST_ASSERT_TRUE_MESSAGE(kinematics::solveLeg(foot, Vector3(0, 0, 0), a), "foot out of reach");
  return a;
}

static void allAt(JointAngles legs[LEG_COUNT], Vector3 foot)
{
  for (int leg = 0; leg < LEG_COUNT; leg++)
    legs[leg] = solved(foot);
}

static void feetOf(const JointAngles legs[LEG_COUNT], Vector3 feet[LEG_COUNT])
{
  for (int leg = 0; leg < LEG_COUNT; leg++)
    feet[leg] = legInBody(leg, legs[leg]).points[3];
}

static const Vector3 STAND(190, 0, -145); // body height 50
static const Vector3 RAISED(230, 80, 40); // a foot lifted and reached forward

void test_forward_kinematics_inverts_the_solver()
{
  const Vector3 targets[] = {STAND, RAISED, Vector3(190, 60, -145), Vector3(150, -75, -145), body::CALIBRATION_POSE};
  for (int leg = 0; leg < LEG_COUNT; leg++)
    for (Vector3 t : targets)
    {
      Vector3 foot = legInBody(leg, solved(t)).points[3], want = toBody(leg, t);
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, foot.distanceTo(want));
    }
}

void test_symmetric_stand_is_on_the_centre_line()
{
  JointAngles legs[LEG_COUNT];
  allAt(legs, STAND);
  Estimate e = estimate(legs, false);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, e.centre.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, e.centre.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, BODY_G + LEG_COUNT * (3 * SERVO_G + COXA_LINK_G + FEMUR_LINK_G + TIBIA_LINK_G), e.massG);
  Vector3 feet[LEG_COUNT];
  feetOf(legs, feet);
  float margin;
  TEST_ASSERT_TRUE(supportMargin(feet, e.centre, margin));
  TEST_ASSERT_TRUE(margin >= 20);
}

// The leg's own mass centre: its servos at the joints, its links along them.
static Vector3 legMassCentre(int leg, JointAngles a)
{
  const Vector3 *p = legInBody(leg, a).points;
  Vector3 moment = (p[0] + p[1] + p[2]) * SERVO_G + interpolate(p[0], p[1], 0.5f) * COXA_LINK_G +
                   interpolate(p[1], p[2], 0.5f) * FEMUR_LINK_G + interpolate(p[2], p[3], TIBIA_LINK_AT) * TIBIA_LINK_G;
  return moment / (3 * SERVO_G + COXA_LINK_G + FEMUR_LINK_G + TIBIA_LINK_G);
}

void test_lifted_leg_moves_the_centre_with_its_mass()
{
  // Raising a foot bends the knee up and in, so the leg's mass, and the
  // robot's centre with it, moves by the leg's share of the total.
  JointAngles legs[LEG_COUNT];
  allAt(legs, STAND);
  Estimate standing = estimate(legs, false);
  Vector3 before = legMassCentre(0, legs[0]);
  legs[0] = solved(RAISED);
  Estimate e = estimate(legs, false);
  float share = (3 * SERVO_G + COXA_LINK_G + FEMUR_LINK_G + TIBIA_LINK_G) / e.massG;
  Vector3 want = standing.centre + (legMassCentre(0, legs[0]) - before) * share;
  TEST_ASSERT_TRUE(e.centre.distanceTo(standing.centre) > 1);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, e.centre.distanceTo(want));
  Vector3 feet[LEG_COUNT];
  feetOf(legs, feet);
  Vector2 hull[LEG_COUNT];
  TEST_ASSERT_EQUAL_INT(5, supportHull(feet, hull));
}

void test_battery_adds_its_mass_towards_its_position()
{
  JointAngles legs[LEG_COUNT];
  allAt(legs, STAND);
  legs[0] = solved(RAISED);
  Estimate without = estimate(legs, false), with = estimate(legs, true);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, BATTERY_G, with.massG - without.massG);
  TEST_ASSERT_TRUE(with.centre.distanceTo(BATTERY_AT) < without.centre.distanceTo(BATTERY_AT));
}

void test_fewer_than_three_feet_give_no_margin()
{
  Vector3 feet[LEG_COUNT];
  for (int leg = 0; leg < LEG_COUNT; leg++)
    feet[leg] = Vector3(leg * 10.0f, 0, leg < 2 ? -100 : -50);
  float margin = 123;
  TEST_ASSERT_FALSE(supportMargin(feet, Vector3(), margin));
  TEST_ASSERT_EQUAL_FLOAT(123, margin);
  // Three feet down but in one line: no polygon either.
  feet[2].z = -100;
  TEST_ASSERT_FALSE(supportMargin(feet, Vector3(), margin));
}

void test_hull_drops_inner_feet_and_signs_the_margin()
{
  // A 200 mm square with two feet inside it; one foot is lifted 11 mm.
  Vector3 feet[LEG_COUNT] = {Vector3(-100, -100, 0), Vector3(100, -100, 5), Vector3(100, 100, 0),
                             Vector3(-100, 100, 0), Vector3(0, 50, 0), Vector3(150, 0, 11)};
  Vector2 hull[LEG_COUNT];
  TEST_ASSERT_EQUAL_INT(4, supportHull(feet, hull));
  float margin;
  TEST_ASSERT_TRUE(supportMargin(feet, Vector3(0, 0, 80), margin));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100, margin);
  TEST_ASSERT_TRUE(supportMargin(feet, Vector3(130, 0, 0), margin));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -30, margin);
}

// --- golden vectors -----------------------------------------------------------

static const char *const HEADER =
    "# Spooder estimated centre of gravity: golden vectors.\n"
    "# Generated by test/test_centre_of_gravity from core/movement/centre_of_gravity;\n"
    "# regenerate with SPOODER_WRITE_GOLDEN=1 mise run test after a deliberate\n"
    "# change to the model (masses, mounts, kinematics).\n"
    "#\n"
    "# One pose per line, fields separated by single spaces:\n"
    "#   <name> <battery 0|1> <18 commanded angles> <x> <y> <z> <mass> <margin>\n"
    "# The angles are in degrees, leg by leg (coxa, femur, tibia), legs in servo\n"
    "# output order. The centre is in the body frame (x right, y forward, z up,\n"
    "# millimetres, coxa pivots at z = 0), the mass in grams, the margin in\n"
    "# millimetres, or none with fewer than three feet on the ground.\n"
    "# Lines starting with # are comments.\n";

struct Text
{
  char buf[8192];
  int len;
};

static void add(Text &t, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void add(Text &t, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  t.len += vsnprintf(t.buf + t.len, sizeof(t.buf) - t.len, fmt, ap);
  va_end(ap);
  TEST_ASSERT_TRUE_MESSAGE(t.len < (int)sizeof(t.buf), "golden text buffer too small");
}

struct Pose
{
  const char *name;
  Vector3 feet[LEG_COUNT]; // leg frame
};

static const Pose POSES[] = {
    {"stand_height_0", {Vector3(190, 0, -60), Vector3(190, 0, -60), Vector3(190, 0, -60), Vector3(190, 0, -60),
                        Vector3(190, 0, -60), Vector3(190, 0, -60)}},
    {"stand_height_50", {STAND, STAND, STAND, STAND, STAND, STAND}},
    {"stand_height_100", {Vector3(190, 0, -230), Vector3(190, 0, -230), Vector3(190, 0, -230),
                          Vector3(190, 0, -230), Vector3(190, 0, -230), Vector3(190, 0, -230)}},
    {"calibration", {body::CALIBRATION_POSE, body::CALIBRATION_POSE, body::CALIBRATION_POSE,
                     body::CALIBRATION_POSE, body::CALIBRATION_POSE, body::CALIBRATION_POSE}},
    {"front_left_raised", {RAISED, STAND, STAND, STAND, STAND, STAND}},
    {"walking", {Vector3(190, 60, -145), Vector3(190, 20, -100), Vector3(150, -75, -145), Vector3(190, -60, -145),
                 Vector3(230, 70, -120), Vector3(190, 20, -145)}},
    {"tripod_up", {Vector3(190, 20, -100), Vector3(190, -40, -145), Vector3(190, 20, -100), Vector3(190, -40, -145),
                   Vector3(190, 20, -100), Vector3(190, -40, -145)}},
};

// A value rounding to zero at three decimals, as exactly 0: the sign of a
// tiny residue differs between compilers and maths libraries, and "-0.000"
// would make the golden file depend on them.
static float printable(float v)
{
  return fabsf(v) < 0.0005f ? 0.0f : v;
}

static void buildGolden(Text &t)
{
  add(t, "%s", HEADER);
  for (const Pose &pose : POSES)
    for (int battery = 0; battery < 2; battery++)
    {
      JointAngles legs[LEG_COUNT];
      for (int leg = 0; leg < LEG_COUNT; leg++)
        legs[leg] = solved(pose.feet[leg]);
      Estimate e = estimate(legs, battery);
      add(t, "%s %d", pose.name, battery);
      for (const JointAngles &a : legs)
        add(t, " %.4f %.4f %.4f", a.coxa, a.femur, a.tibia);
      add(t, " %.3f %.3f %.3f %.1f", printable(e.centre.x), printable(e.centre.y), printable(e.centre.z), e.massG);
      Vector3 feet[LEG_COUNT];
      feetOf(legs, feet);
      float margin;
      if (supportMargin(feet, e.centre, margin))
        add(t, " %.3f\n", printable(margin));
      else
        add(t, " none\n");
    }
}

static void goldenPath(char *out, int size)
{
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_centre_of_gravity/");
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*stest/golden/centre_of_gravity.txt", prefix, file);
}

void test_golden_vectors()
{
  static Text t;
  t = {};
  buildGolden(t);

  char path[512];
  goldenPath(path, sizeof(path));
  const char *write = getenv("SPOODER_WRITE_GOLDEN");
  if (write && write[0] == '1')
  {
    FILE *f = fopen(path, "wb");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "cannot write test/golden/centre_of_gravity.txt");
    fwrite(t.buf, 1, t.len, f);
    fclose(f);
  }

  static char golden[sizeof(t.buf)];
  FILE *f = fopen(path, "rb");
  TEST_ASSERT_NOT_NULL_MESSAGE(f, "cannot read test/golden/centre_of_gravity.txt");
  int n = (int)fread(golden, 1, sizeof(golden) - 1, f);
  fclose(f);
  golden[n] = 0;
  TEST_ASSERT_EQUAL_STRING(golden, t.buf);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_forward_kinematics_inverts_the_solver);
  RUN_TEST(test_symmetric_stand_is_on_the_centre_line);
  RUN_TEST(test_lifted_leg_moves_the_centre_with_its_mass);
  RUN_TEST(test_battery_adds_its_mass_towards_its_position);
  RUN_TEST(test_fewer_than_three_feet_give_no_margin);
  RUN_TEST(test_hull_drops_inner_feet_and_signs_the_margin);
  RUN_TEST(test_golden_vectors);
  return UNITY_END();
}
