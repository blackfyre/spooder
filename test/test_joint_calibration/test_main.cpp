#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "movement/joint_calibration.h"

using namespace joint_calibration;

void setUp() {}
void tearDown() {}

static Joint joint(float offset, float p0, float p180, bool reversed = false)
{
  Joint j;
  j.offsetDeg = offset;
  j.pulse0Us = p0;
  j.pulse180Us = p180;
  j.reversed = reversed;
  return j;
}

static const Joint DEFAULT_RANGE = joint(0, 500, 2500);

// The output for an angle without rounding: what the joint is driven to,
// including the offset the IK adds to the geometric angle.
static float outputUs(const Joint &j, float geometricDeg)
{
  float a = geometricDeg + j.offsetDeg;
  a = a < 0 ? 0 : (a > 180 ? 180 : a);
  if (j.reversed)
    a = 180 - a;
  return j.pulse0Us + (j.pulse180Us - j.pulse0Us) * a / 180.0f;
}

// The largest output change a step of dtS may make: 60°/s through the
// calibration at the start of the step (never below the minimum span).
static float allowedUs(const Joint &before, float dtS)
{
  float span = fabsf(before.pulse180Us - before.pulse0Us);
  if (span < MIN_PULSE_SPAN_US)
    span = MIN_PULSE_SPAN_US;
  return MAX_RATE_DEG_PER_S * dtS * span / 180.0f + 1e-3f;
}

static bool sameJoint(const Joint &a, const Joint &b)
{
  return a.offsetDeg == b.offsetDeg && a.pulse0Us == b.pulse0Us && a.pulse180Us == b.pulse180Us &&
         a.reversed == b.reversed;
}

// Steps joint 0 from `from` to `to` every dtS, checking the output rate at
// several angles. Returns the time taken to converge (or -1 after 30 s).
static float follow(Joint from, Joint to, float dtS)
{
  Table applied, requested;
  loadDefaults(applied);
  loadDefaults(requested);
  applied.joints[0] = from;
  requested.joints[0] = to;
  const float angles[] = {-30, 0, 15, 45, 90, 135, 165, 200};
  float t = 0;
  while (t < 30)
  {
    Joint before = applied.joints[0];
    step(applied, requested, dtS);
    t += dtS;
    const Joint &after = applied.joints[0];
    for (float g : angles)
    {
      float moved = fabsf(outputUs(after, g) - outputUs(before, g));
      if (moved > allowedUs(before, dtS))
      {
        char msg[96];
        snprintf(msg, sizeof(msg), "output moved %.2f us at %.0f deg in one step", moved, g);
        TEST_FAIL_MESSAGE(msg);
      }
    }
    for (int i = 1; i < JOINT_COUNT; i++)
      TEST_ASSERT_TRUE(sameJoint(DEFAULTS[i], applied.joints[i]));
    if (sameJoint(applied.joints[0], to))
      return t;
  }
  return -1;
}

// --- defaults -----------------------------------------------------------------

void test_defaults_are_the_former_leg_offsets()
{
  const float former[6][3] = {{90, 75, -18}, {93, 75, -15}, {93, 75, -18},
                              {87, 80, -26}, {85, 89, -16}, {93, 85, -24}};
  Table t;
  loadDefaults(t);
  for (int leg = 0; leg < 6; leg++)
  {
    Vector3 o = legOffsets(t, leg);
    TEST_ASSERT_EQUAL_FLOAT(former[leg][0], o.x);
    TEST_ASSERT_EQUAL_FLOAT(former[leg][1], o.y);
    TEST_ASSERT_EQUAL_FLOAT(former[leg][2], o.z);
  }
  for (int i = 0; i < JOINT_COUNT; i++)
  {
    TEST_ASSERT_EQUAL_FLOAT(500, DEFAULTS[i].pulse0Us);
    TEST_ASSERT_EQUAL_FLOAT(2500, DEFAULTS[i].pulse180Us);
    TEST_ASSERT_FALSE(DEFAULTS[i].reversed);
  }
}

void test_load_defaults_and_leg_offsets()
{
  Table t;
  loadDefaults(t);
  t.joints[13].offsetDeg = 42; // leg 5 femur
  Vector3 o = legOffsets(t, 4);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULTS[12].offsetDeg, o.x);
  TEST_ASSERT_EQUAL_FLOAT(42, o.y);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULTS[14].offsetDeg, o.z);
}

// --- pulse conversion -----------------------------------------------------------

void test_default_range_and_angle_clamping()
{
  TEST_ASSERT_EQUAL_UINT16(500, toPulse(DEFAULT_RANGE, 0));
  TEST_ASSERT_EQUAL_UINT16(1500, toPulse(DEFAULT_RANGE, 90));
  TEST_ASSERT_EQUAL_UINT16(2500, toPulse(DEFAULT_RANGE, 180));
  TEST_ASSERT_EQUAL_UINT16(500, toPulse(DEFAULT_RANGE, -20));
  // Spec scenario: 200° with the default calibration gives 2500 µs.
  TEST_ASSERT_EQUAL_UINT16(2500, toPulse(DEFAULT_RANGE, 200));
}

void test_calibrated_pulse_range()
{
  // Spec scenario: 600/2400 µs at 90° gives 1500 µs.
  TEST_ASSERT_EQUAL_UINT16(1500, toPulse(joint(0, 600, 2400), 90));
  TEST_ASSERT_EQUAL_UINT16(600, toPulse(joint(0, 600, 2400), 0));
  // End points in falling order work the same way.
  TEST_ASSERT_EQUAL_UINT16(2400, toPulse(joint(0, 2400, 600), 0));
  TEST_ASSERT_EQUAL_UINT16(1050, toPulse(joint(0, 2400, 600), 135));
}

void test_reversed_joint()
{
  // Spec scenario: reversed, default range, 30° gives 2167 µs (150°).
  TEST_ASSERT_EQUAL_UINT16(2167, toPulse(joint(0, 500, 2500, true), 30));
  TEST_ASSERT_EQUAL_UINT16(2500, toPulse(joint(0, 500, 2500, true), -10));
  TEST_ASSERT_EQUAL_UINT16(500, toPulse(joint(0, 500, 2500, true), 190));
}

void test_non_finite_angle_stays_defined()
{
  TEST_ASSERT_EQUAL_UINT16(1500, toPulse(DEFAULT_RANGE, NAN));
}

// --- guards ---------------------------------------------------------------------

void test_offset_clamped_to_thirty_degrees_of_default()
{
  // Spec scenario: 45° above the default becomes the default plus 30°.
  Joint d = joint(75, 500, 2500);
  Joint j = joint(120, 500, 2500);
  TEST_ASSERT_TRUE(clampJoint(j, d));
  TEST_ASSERT_EQUAL_FLOAT(105, j.offsetDeg);
  j = joint(20, 500, 2500);
  TEST_ASSERT_TRUE(clampJoint(j, d));
  TEST_ASSERT_EQUAL_FLOAT(45, j.offsetDeg);
  j = joint(104.5f, 500, 2500);
  TEST_ASSERT_FALSE(clampJoint(j, d));
  TEST_ASSERT_EQUAL_FLOAT(104.5f, j.offsetDeg);
}

void test_pulses_clamped_to_range()
{
  Joint j = joint(0, 300, 2700);
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(400, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(2600, j.pulse180Us);
  j = joint(0, 2700, 0);
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(2600, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(400, j.pulse180Us);
}

void test_minimum_span_moves_the_180_pulse()
{
  Joint j = joint(0, 1500, 1800);
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(1500, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(2500, j.pulse180Us);
  j = joint(0, 1500, 1200); // falling order is kept
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(1500, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(500, j.pulse180Us);
  j = joint(0, 1500, 1500); // equal pulses count as rising
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(2500, j.pulse180Us);
  j = joint(0, 600, 1600); // exactly the minimum span
  TEST_ASSERT_FALSE(clampJoint(j, DEFAULT_RANGE));
}

void test_minimum_span_at_the_range_limits()
{
  Joint j = joint(0, 2000, 2100);
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(1600, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(2600, j.pulse180Us);
  j = joint(0, 800, 700);
  TEST_ASSERT_TRUE(clampJoint(j, DEFAULT_RANGE));
  TEST_ASSERT_EQUAL_FLOAT(1400, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(400, j.pulse180Us);
}

// --- rate limit -------------------------------------------------------------------

void test_offset_change_takes_at_least_a_third_of_a_second()
{
  // Spec scenario: a 20° change takes at least 1/3 s, and converges.
  float t = follow(joint(75, 500, 2500), joint(95, 500, 2500), 0.02f);
  TEST_ASSERT_TRUE(t >= 1.0f / 3.0f - 1e-4f);
  TEST_ASSERT_TRUE(t < 0.4f);
  t = follow(joint(75, 500, 2500), joint(55, 500, 2500), 0.005f);
  TEST_ASSERT_TRUE(t >= 1.0f / 3.0f - 1e-4f);
  TEST_ASSERT_TRUE(t < 0.4f);
}

void test_pulse_change_is_rate_limited_and_converges()
{
  TEST_ASSERT_TRUE(follow(DEFAULT_RANGE, joint(0, 700, 2300), 0.02f) > 0.29f);
  TEST_ASSERT_TRUE(follow(DEFAULT_RANGE, joint(0, 1500, 2600), 0.1f) > 0);
  // A narrow requested span moves no faster through the narrower range.
  TEST_ASSERT_TRUE(follow(joint(0, 400, 2600), joint(0, 1000, 2000), 0.02f) > 0);
}

void test_offset_converges_before_pulses_move()
{
  Table applied, requested;
  loadDefaults(applied);
  loadDefaults(requested);
  requested.joints[0].offsetDeg += 10;
  requested.joints[0].pulse180Us = 2300;
  step(applied, requested, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULTS[0].offsetDeg + 6, applied.joints[0].offsetDeg);
  TEST_ASSERT_EQUAL_FLOAT(2500, applied.joints[0].pulse180Us);
  TEST_ASSERT_TRUE(follow(joint(0, 500, 2500), joint(10, 500, 2300), 0.02f) > 0);
}

void test_direction_change_never_jumps()
{
  TEST_ASSERT_TRUE(follow(DEFAULT_RANGE, joint(0, 500, 2500, true), 0.02f) > 1.0f);
  TEST_ASSERT_TRUE(follow(joint(5, 600, 2400, true), joint(-5, 700, 2300), 0.02f) > 0);
  TEST_ASSERT_TRUE(follow(joint(0, 500, 2500), joint(0, 2500, 500, true), 0.02f) > 0);
}

void test_zero_or_negative_dt_changes_nothing()
{
  Table applied, requested;
  loadDefaults(applied);
  loadDefaults(requested);
  requested.joints[3].offsetDeg += 10;
  step(applied, requested, 0);
  step(applied, requested, -1);
  step(applied, requested, NAN);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULTS[3].offsetDeg, applied.joints[3].offsetDeg);
}

// --- compiled-defaults file -------------------------------------------------------

// Repository root, from this file's path (test/test_joint_calibration/...).
static void repoPath(const char *relative, char *out, int size)
{
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_joint_calibration/");
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*s%s", prefix, file, relative);
}

static int readFile(const char *relative, char *buf, int size)
{
  char path[512];
  repoPath(relative, path, sizeof(path));
  FILE *f = fopen(path, "rb");
  if (!f)
    return -1;
  int n = (int)fread(buf, 1, size - 1, f);
  fclose(f);
  buf[n] = 0;
  return n;
}

// The reference exporter: the compiled defaults as the complete text of the
// defaults file (design D6). Front-end exporters must produce the same text.
static int exportDefaults(const Joint *table, char *out, int size)
{
  static const char *const JOINT_NAMES[] = {"coxa", "femur", "tibia"};
  int n = snprintf(out, size,
                   "// Spooder joint calibration defaults, in a fixed format: an export from a\n"
                   "// calibration front-end replaces this whole file. Do not reformat it.\n"
                   "//\n"
                   "// One line per joint, in servo output order (leg 1 coxa, femur, tibia, then\n"
                   "// leg 2 ...):\n"
                   "//   {offset in degrees (two decimals), pulse at 0 degrees in us,\n"
                   "//    pulse at 180 degrees in us, reversed},\n"
                   "// test/golden/calibration_defaults.cpp must equal this file.\n"
                   "#include \"joint_calibration.h\"\n"
                   "\n"
                   "namespace joint_calibration\n"
                   "{\n"
                   "\n"
                   "const joint_calibration::Joint DEFAULTS[18] = {\n");
  for (int i = 0; i < JOINT_COUNT; i++)
    n += snprintf(out + n, size - n, "  {%.2f, %d, %d, %s}, // leg %d %s\n", table[i].offsetDeg,
                  (int)table[i].pulse0Us, (int)table[i].pulse180Us, table[i].reversed ? "true" : "false",
                  i / 3 + 1, JOINT_NAMES[i % 3]);
  n += snprintf(out + n, size - n, "};\n\n} // namespace joint_calibration\n");
  return n;
}

void test_golden_defaults_equal_the_committed_file()
{
  static char golden[4096], committed[4096];
  TEST_ASSERT_TRUE_MESSAGE(readFile("test/golden/calibration_defaults.cpp", golden, sizeof(golden)) > 0,
                           "cannot read test/golden/calibration_defaults.cpp");
  TEST_ASSERT_TRUE_MESSAGE(readFile("core/movement/calibration_defaults.cpp", committed, sizeof(committed)) > 0,
                           "cannot read core/movement/calibration_defaults.cpp");
  TEST_ASSERT_EQUAL_STRING(golden, committed);
}

void test_export_of_the_defaults_reproduces_the_golden_file()
{
  static char golden[4096], exported[4096];
  TEST_ASSERT_TRUE(readFile("test/golden/calibration_defaults.cpp", golden, sizeof(golden)) > 0);
  exportDefaults(DEFAULTS, exported, sizeof(exported));
  TEST_ASSERT_EQUAL_STRING(golden, exported);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_defaults_are_the_former_leg_offsets);
  RUN_TEST(test_load_defaults_and_leg_offsets);
  RUN_TEST(test_default_range_and_angle_clamping);
  RUN_TEST(test_calibrated_pulse_range);
  RUN_TEST(test_reversed_joint);
  RUN_TEST(test_non_finite_angle_stays_defined);
  RUN_TEST(test_offset_clamped_to_thirty_degrees_of_default);
  RUN_TEST(test_pulses_clamped_to_range);
  RUN_TEST(test_minimum_span_moves_the_180_pulse);
  RUN_TEST(test_minimum_span_at_the_range_limits);
  RUN_TEST(test_offset_change_takes_at_least_a_third_of_a_second);
  RUN_TEST(test_pulse_change_is_rate_limited_and_converges);
  RUN_TEST(test_offset_converges_before_pulses_move);
  RUN_TEST(test_direction_change_never_jumps);
  RUN_TEST(test_zero_or_negative_dt_changes_nothing);
  RUN_TEST(test_golden_defaults_equal_the_committed_file);
  RUN_TEST(test_export_of_the_defaults_reproduces_the_golden_file);
  return UNITY_END();
}
