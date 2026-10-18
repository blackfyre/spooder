#include "calibration_export.h"

#include <stdio.h>

namespace calibration_export
{

static const char HEADER[] = "// Spooder joint calibration defaults, in a fixed format: an export from a\n"
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
                             "const joint_calibration::Joint DEFAULTS[18] = {\n";
static const char FOOTER[] = "};\n\n} // namespace joint_calibration\n";
static const char *const JOINT_NAMES[] = {"coxa", "femur", "tibia"};

// Counts n characters just written at *used; false once the text no longer fits.
static bool append(int size, int *used, int n)
{
  if (n < 0 || n >= size - *used)
    return false;
  *used += n;
  return true;
}

int write(const protocol::CalibrationJoint joints[protocol::CALIBRATION_JOINTS], char *out, int size)
{
  static_assert(protocol::CALIBRATION_JOINTS == 18, "the defaults file holds 18 joints");
  if (out == nullptr || size <= 0)
    return -1;
  int used = 0;
  bool ok = append(size, &used, snprintf(out, size, "%s", HEADER));
  for (int i = 0; ok && i < protocol::CALIBRATION_JOINTS; i++)
  {
    const protocol::CalibrationJoint &j = joints[i];
    // Centidegrees as degrees with two decimals, from integers so there is
    // no rounding (and -0.50 keeps its sign).
    int32_t c = j.offsetCentideg;
    int32_t mag = c < 0 ? -c : c;
    ok = append(size, &used,
                snprintf(out + used, size - used, "  {%s%ld.%02ld, %u, %u, %s}, // leg %d %s\n", c < 0 ? "-" : "",
                         (long)(mag / 100), (long)(mag % 100), (unsigned)j.pulse0Us, (unsigned)j.pulse180Us,
                         (j.flags & protocol::CALIBRATION_REVERSED) ? "true" : "false", i / 3 + 1,
                         JOINT_NAMES[i % 3]));
  }
  ok = ok && append(size, &used, snprintf(out + used, size - used, "%s", FOOTER));
  if (!ok)
  {
    out[0] = 0;
    return -1;
  }
  return used;
}

} // namespace calibration_export
