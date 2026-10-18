#include <unity.h>
#include <stdio.h>
#include <string.h>
#include "movement/joint_calibration.h"
#include "protocol/link_receiver.h"
#include "setup/calibration_export.h"

void setUp() {}
void tearDown() {}

// Repository root, from this file's path (test/test_calibration_export/...).
static int readFile(const char *relative, char *buf, int size)
{
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_calibration_export/");
  int prefix = cut ? (int)(cut - file) : 0;
  char path[512];
  snprintf(path, sizeof(path), "%.*s%s", prefix, file, relative);
  FILE *f = fopen(path, "rb");
  if (!f)
    return -1;
  int n = (int)fread(buf, 1, size - 1, f);
  fclose(f);
  buf[n] = 0;
  return n;
}

// The table as the robot reports it.
static protocol::CalibrationReport reportOf(const joint_calibration::Table &t)
{
  link_receiver::Receiver r;
  return link_receiver::makeReport(r, t);
}

void test_export_of_the_reported_defaults_is_the_golden_file()
{
  static char golden[4096], exported[calibration_export::MAX_TEXT_SIZE];
  TEST_ASSERT_TRUE_MESSAGE(readFile("test/golden/calibration_defaults.cpp", golden, sizeof(golden)) > 0,
                           "cannot read test/golden/calibration_defaults.cpp");
  joint_calibration::Table t;
  joint_calibration::loadDefaults(t);
  protocol::CalibrationReport rep = reportOf(t);
  int n = calibration_export::write(rep.joints, exported, sizeof(exported));
  TEST_ASSERT_EQUAL_INT((int)strlen(golden), n);
  TEST_ASSERT_EQUAL_STRING(golden, exported);
}

void test_export_of_a_changed_table()
{
  static char exported[calibration_export::MAX_TEXT_SIZE];
  joint_calibration::Table t;
  joint_calibration::loadDefaults(t);
  t.joints[4] = {76.5f, 612, 2390, true};    // leg 2 femur
  t.joints[17] = {-0.5f, 400, 2600, false};  // leg 6 tibia
  t.joints[0] = {-29.99f, 2500, 500, false}; // leg 1 coxa, end points swapped
  protocol::CalibrationReport rep = reportOf(t);
  TEST_ASSERT_TRUE(calibration_export::write(rep.joints, exported, sizeof(exported)) > 0);
  TEST_ASSERT_NOT_NULL(strstr(exported, "  {76.50, 612, 2390, true}, // leg 2 femur\n"));
  TEST_ASSERT_NOT_NULL(strstr(exported, "  {-0.50, 400, 2600, false}, // leg 6 tibia\n"));
  TEST_ASSERT_NOT_NULL(strstr(exported, "  {-29.99, 2500, 500, false}, // leg 1 coxa\n"));
  // Unchanged joints keep their lines.
  TEST_ASSERT_NOT_NULL(strstr(exported, "  {93.00, 500, 2500, false}, // leg 2 coxa\n"));
  // Still a complete file.
  TEST_ASSERT_NOT_NULL(strstr(exported, "const joint_calibration::Joint DEFAULTS[18] = {\n"));
  const char *end = "};\n\n} // namespace joint_calibration\n";
  size_t len = strlen(exported);
  TEST_ASSERT_EQUAL_STRING(end, exported + len - strlen(end));
}

void test_extreme_values_fit()
{
  static char exported[calibration_export::MAX_TEXT_SIZE];
  protocol::CalibrationJoint joints[protocol::CALIBRATION_JOINTS];
  for (int i = 0; i < protocol::CALIBRATION_JOINTS; i++)
    joints[i] = {-32768, 65535, 65535, protocol::CALIBRATION_REVERSED};
  int n = calibration_export::write(joints, exported, sizeof(exported));
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NOT_NULL(strstr(exported, "  {-327.68, 65535, 65535, true}, // leg 6 tibia\n"));
}

void test_too_small_a_buffer_gives_an_empty_text()
{
  char small[100];
  joint_calibration::Table t;
  joint_calibration::loadDefaults(t);
  protocol::CalibrationReport rep = reportOf(t);
  TEST_ASSERT_EQUAL_INT(-1, calibration_export::write(rep.joints, small, sizeof(small)));
  TEST_ASSERT_EQUAL_STRING("", small);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_export_of_the_reported_defaults_is_the_golden_file);
  RUN_TEST(test_export_of_a_changed_table);
  RUN_TEST(test_extreme_values_fit);
  RUN_TEST(test_too_small_a_buffer_gives_an_empty_text);
  return UNITY_END();
}
