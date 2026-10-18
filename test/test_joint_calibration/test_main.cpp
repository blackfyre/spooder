#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "movement/body.h"
#include "movement/joint_calibration.h"
#include "movement/kinematics.h"
#include "protocol/link_receiver.h"
#include "setup/calibration_export.h"

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
  applied.joints[0] = joint(75, 500, 2500);
  requested.joints[0] = joint(85, 500, 2300);
  step(applied, requested, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(81, applied.joints[0].offsetDeg);
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

// The format fixture: a table that exercises the format (signs, fractions,
// three-digit offsets, falling end points, reversed joints) and its export,
// test/golden/calibration_defaults.cpp. Neither changes with calibration; the
// portal's and the desktop calibrator's exporter tests hold the same table.
static const Joint FIXTURE[JOINT_COUNT] = {
    {-0.50f, 2450, 610, true},   {0.00f, 500, 2500, false},    {-29.99f, 2500, 500, false},
    {76.50f, 612, 2390, true},   {105.25f, 400, 2600, false},  {-18.07f, 1000, 2000, false},
    {90.01f, 520, 2480, false},  {-120.40f, 2600, 1600, true}, {7.00f, 1400, 400, false},
    {87.33f, 505, 2495, false},  {80.10f, 498, 2502, true},    {-26.00f, 700, 2300, false},
    {85.99f, 1500, 2500, false}, {-0.01f, 500, 1500, false},   {-16.50f, 2400, 600, true},
    {179.99f, 450, 2550, false}, {-180.00f, 2550, 450, true},  {12.34f, 640, 2360, false},
};

static const char *const JOINT_NAMES[] = {"coxa", "femur", "tibia"};

// Repository root, from this file's path (test/test_joint_calibration/...).
static void repoPath(const char *relative, char *out, int size)
{
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_joint_calibration/");
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*s%s", prefix, file, relative);
}

static int readPath(const char *path, char *buf, int size)
{
  FILE *f = fopen(path, "rb");
  if (!f)
    return -1;
  int n = (int)fread(buf, 1, size - 1, f);
  fclose(f);
  buf[n] = 0;
  return n;
}

static int readFile(const char *relative, char *buf, int size)
{
  char path[512];
  repoPath(relative, path, sizeof(path));
  return readPath(path, buf, size);
}

// The reference exporter: a table as the complete text of the defaults file
// (design D6). Front-end exporters must produce the same text.
static int exportDefaults(const Joint *table, char *out, int size)
{
  int n = snprintf(out, size,
                   "// Spooder joint calibration defaults, in a fixed format: an export from a\n"
                   "// calibration front-end replaces this whole file. Do not reformat it.\n"
                   "//\n"
                   "// One line per joint, in servo output order (leg 1 coxa, femur, tibia, then\n"
                   "// leg 2 ...):\n"
                   "//   {offset in degrees (two decimals), pulse at 0 degrees in us,\n"
                   "//    pulse at 180 degrees in us, reversed},\n"
                   "// Its format is pinned by a fixture, test/golden/calibration_defaults.cpp.\n"
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

// Reads the 18 joint lines of a defaults file, which must name the joints in
// output order. The rest of the text is checked by the round trip.
static bool parseDefaults(const char *text, Joint out[JOINT_COUNT])
{
  const char *p = strstr(text, "const joint_calibration::Joint DEFAULTS[18] = {\n");
  if (p == nullptr)
    return false;
  for (int i = 0; i < JOINT_COUNT; i++)
  {
    p = strchr(p, '\n');
    if (p == nullptr)
      return false;
    p++;
    float offset;
    int p0, p180, leg;
    char reversed[6], name[6];
    if (sscanf(p, "  {%f, %d, %d, %5[a-z]}, // leg %d %5[a-z]", &offset, &p0, &p180, reversed, &leg, name) != 6)
      return false;
    if (leg != i / 3 + 1 || strcmp(name, JOINT_NAMES[i % 3]) != 0)
      return false;
    if (strcmp(reversed, "true") != 0 && strcmp(reversed, "false") != 0)
      return false;
    out[i] = joint(offset, (float)p0, (float)p180, strcmp(reversed, "true") == 0);
  }
  return true;
}

// The format check: the reference export of the fixture is the golden file
// (the portal's and the desktop calibrator's exporters check the same in
// their own tests), and the golden file parses back to the fixture.
static void checkFormatFixture()
{
  static char golden[4096], exported[4096];
  TEST_ASSERT_TRUE_MESSAGE(readFile("test/golden/calibration_defaults.cpp", golden, sizeof(golden)) > 0,
                           "cannot read test/golden/calibration_defaults.cpp");
  TEST_ASSERT_EQUAL_INT((int)strlen(golden), exportDefaults(FIXTURE, exported, sizeof(exported)));
  TEST_ASSERT_EQUAL_STRING(golden, exported);
  Joint parsed[JOINT_COUNT];
  TEST_ASSERT_TRUE(parseDefaults(golden, parsed));
  for (int i = 0; i < JOINT_COUNT; i++)
    TEST_ASSERT_TRUE(sameJoint(FIXTURE[i], parsed[i]));
}

// The IK angle of joint @p i (output order) at the calibration pose, before
// its offset: 0 for a coxa (straight out from the mount), 90 for a femur
// (straight up) and a tibia (horizontal).
static float calibrationPoseAngle(int i)
{
  kinematics::JointAngles a;
  TEST_ASSERT_TRUE(kinematics::solveLeg(body::CALIBRATION_POSE, Vector3(0, 0, 0), a));
  return i % 3 == 0 ? a.coxa : i % 3 == 1 ? a.femur : a.tibia;
}

// The checks on a defaults file at @p path, whatever its values: it holds 18
// joints in output order, each within the guards' hard limits (pulses in
// 400-2600 us, at least 1000 us apart; the offset such that the servo reaches
// the calibration pose, its angle there within 0-180 degrees, which also keeps
// the default plus or minus any guard test's change within the wire's int16
// centidegrees), and the reference and the portal's exporters both reproduce
// it byte for byte (export(parse(file)) == file).
static void checkDefaultsFile(const char *path, Joint parsed[JOINT_COUNT])
{
  static char text[4096], exported[4096];
  TEST_ASSERT_TRUE_MESSAGE(readPath(path, text, sizeof(text)) > 0, path);
  TEST_ASSERT_TRUE_MESSAGE(parseDefaults(text, parsed), "the defaults file does not parse");
  for (int i = 0; i < JOINT_COUNT; i++)
  {
    Joint j = parsed[i];
    TEST_ASSERT_FALSE_MESSAGE(clampJoint(j, parsed[i]), "a joint outside the guards' hard limits");
    float servo = calibrationPoseAngle(i) + parsed[i].offsetDeg;
    TEST_ASSERT_TRUE_MESSAGE(servo >= -0.005f && servo <= 180.005f, "a servo that cannot reach the calibration pose");
  }
  exportDefaults(parsed, exported, sizeof(exported));
  TEST_ASSERT_EQUAL_STRING(text, exported);
  Table t;
  for (int i = 0; i < JOINT_COUNT; i++)
    t.joints[i] = parsed[i];
  link_receiver::Receiver r;
  protocol::CalibrationReport rep = link_receiver::makeReport(r, t);
  TEST_ASSERT_TRUE(calibration_export::write(rep.joints, exported, calibration_export::MAX_TEXT_SIZE) > 0);
  TEST_ASSERT_EQUAL_STRING(text, exported);
}

// An offset moved by @p deg towards zero, so it stays on the wire.
static float towardsZero(float offsetDeg, float deg)
{
  return offsetDeg < 0 ? offsetDeg + deg : offsetDeg - deg;
}

// Equal as the file and the wire carry them: centidegrees and whole pulses.
static bool sameAsWritten(const Joint &a, const Joint &b)
{
  return lroundf(a.offsetDeg * 100) == lroundf(b.offsetDeg * 100) && a.pulse0Us == b.pulse0Us &&
         a.pulse180Us == b.pulse180Us && a.reversed == b.reversed;
}

void test_calibration_pose_angles_before_offsets()
{
  // The reachability check relies on these; the skill names them.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, calibrationPoseAngle(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90, calibrationPoseAngle(1));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90, calibrationPoseAngle(2));
}

void test_export_of_the_fixture_is_the_golden_file()
{
  checkFormatFixture();
}

void test_committed_defaults_file_is_valid_and_compiled()
{
  char path[512];
  repoPath("core/movement/calibration_defaults.cpp", path, sizeof(path));
  Joint parsed[JOINT_COUNT];
  checkDefaultsFile(path, parsed);
  for (int i = 0; i < JOINT_COUNT; i++)
    TEST_ASSERT_TRUE_MESSAGE(sameAsWritten(parsed[i], DEFAULTS[i]), "the file differs from the compiled DEFAULTS");
}

void test_an_exported_calibration_passes_every_check()
{
  // A calibration export (the portal's exporter, from a report) written over
  // a copy of the defaults file: the copy passes the defaults checks, and the
  // format check does not depend on it.
  Table t;
  loadDefaults(t);
  t.joints[0] = joint(towardsZero(t.joints[0].offsetDeg, 12.34f), 612, 2390, !t.joints[0].reversed);
  t.joints[8] = joint(towardsZero(t.joints[8].offsetDeg, 0.5f), 2500, 500);
  t.joints[17] = joint(towardsZero(t.joints[17].offsetDeg, 7.07f), 400, 2600);
  link_receiver::Receiver r;
  protocol::CalibrationReport rep = link_receiver::makeReport(r, t);
  static char text[calibration_export::MAX_TEXT_SIZE];
  int n = calibration_export::write(rep.joints, text, sizeof(text));
  TEST_ASSERT_TRUE(n > 0);

  const char *dir = getenv("TMPDIR");
  char path[512];
  snprintf(path, sizeof(path), "%s/spooder_calibration_defaults_XXXXXX", dir && *dir ? dir : "/tmp");
  int fd = mkstemp(path);
  TEST_ASSERT_TRUE_MESSAGE(fd >= 0, "cannot create a temporary file");
  FILE *f = fdopen(fd, "wb");
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_EQUAL_INT(n, (int)fwrite(text, 1, n, f));
  fclose(f);

  Joint parsed[JOINT_COUNT];
  checkDefaultsFile(path, parsed);
  remove(path);
  for (int i = 0; i < JOINT_COUNT; i++)
    TEST_ASSERT_TRUE(sameAsWritten(parsed[i], t.joints[i]));
  TEST_ASSERT_FALSE(sameAsWritten(parsed[0], DEFAULTS[0]));
  checkFormatFixture();
}

int main()
{
  UNITY_BEGIN();
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
  RUN_TEST(test_calibration_pose_angles_before_offsets);
  RUN_TEST(test_export_of_the_fixture_is_the_golden_file);
  RUN_TEST(test_committed_defaults_file_is_valid_and_compiled);
  RUN_TEST(test_an_exported_calibration_passes_every_check);
  return UNITY_END();
}
