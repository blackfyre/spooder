// Runs core/gamepad_mapping over test/golden/gamepad_mapping.txt, the
// hand-written steps the handheld's Go port (desktop/handheld/internal/padmap)
// is also tested against, and compares every field of every step's desired
// state. SPOODER_GAMEPAD_GOLDEN names another copy of the file (to check that
// a changed expected value fails the test).
#include <unity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gamepad_mapping/gamepad_mapping.h"

using namespace gamepad_mapping;

void setUp() {}
void tearDown() {}

struct Name
{
  const char *name;
  unsigned bit;
};

static const Name BUTTON_NAMES[] = {{"A", BUTTON_A},
                                    {"B", BUTTON_B},
                                    {"X", BUTTON_X},
                                    {"Y", BUTTON_Y},
                                    {"L1", BUTTON_L1},
                                    {"R1", BUTTON_R1},
                                    {"L2", BUTTON_L2},
                                    {"R2", BUTTON_R2},
                                    {"THUMB_L", BUTTON_THUMB_L},
                                    {"THUMB_R", BUTTON_THUMB_R}};
static const Name DPAD_NAMES[] = {{"UP", DPAD_UP}, {"DOWN", DPAD_DOWN}, {"RIGHT", DPAD_RIGHT}, {"LEFT", DPAD_LEFT}};
static const Name MISC_NAMES[] = {{"HOME", MISC_HOME}, {"BACK", MISC_BACK}};
static const Name MODE_NAMES[] = {{"Stand", (unsigned)RequestedMode::Stand},
                                  {"Walk", (unsigned)RequestedMode::Walk},
                                  {"Sleep", (unsigned)RequestedMode::Sleep},
                                  {"Calibrate", (unsigned)RequestedMode::Calibrate}};
static const Name STRIDE_NAMES[] = {{"Dynamic", (unsigned)StrideMode::Dynamic}, {"Fixed", (unsigned)StrideMode::Fixed}};
static const Name STICK_NAMES[] = {{"Steer", (unsigned)StickMode::Steer}, {"Crab", (unsigned)StickMode::Crab}};

template <size_t N> static bool lookup(const Name (&names)[N], const char *s, unsigned &out)
{
  for (const Name &n : names)
    if (strcmp(n.name, s) == 0)
    {
      out = n.bit;
      return true;
    }
  return false;
}

// '+'-joined names, or '-' for none.
template <size_t N> static bool parseBits(const Name (&names)[N], char *s, unsigned &out)
{
  out = 0;
  if (strcmp(s, "-") == 0)
    return true;
  for (char *save = nullptr, *tok = strtok_r(s, "+", &save); tok; tok = strtok_r(nullptr, "+", &save))
  {
    unsigned bit;
    if (!lookup(names, tok, bit) || (out & bit))
      return false;
    out |= bit;
  }
  return out != 0;
}

static bool parseInt(const char *s, long lo, long hi, long &out)
{
  char *end;
  out = strtol(s, &end, 10);
  return *s && !*end && out >= lo && out <= hi;
}

static const int FIELDS = 24; // 9 inputs, '=', 14 outputs

struct Report
{
  int steps;
  int failures;
  char first[256];
};

static void fail(Report &r, int line, const char *what)
{
  if (r.failures++ == 0)
    snprintf(r.first, sizeof(r.first), "line %d: %s", line, what);
}

// Checks one step line; returns false when it cannot be parsed.
static bool runStep(Mapper &m, char *text, int line, Report &r)
{
  char *f[FIELDS + 1];
  int n = 0;
  for (char *save = nullptr, *tok = strtok_r(text, " ", &save); tok && n <= FIELDS; tok = strtok_r(nullptr, " ", &save))
    f[n++] = tok;
  if (n != FIELDS || strcmp(f[9], "=") != 0)
    return false;

  long t, conn, ax, ay, arx, ary;
  unsigned buttons, dpad, misc;
  if (!parseInt(f[0], 0, 0xFFFFFFFFL, t) || !parseInt(f[1], 0, 1, conn) || !parseInt(f[2], -512, 512, ax) ||
      !parseInt(f[3], -512, 512, ay) || !parseInt(f[4], -512, 512, arx) || !parseInt(f[5], -512, 512, ary) ||
      !parseBits(BUTTON_NAMES, f[6], buttons) || !parseBits(DPAD_NAMES, f[7], dpad) ||
      !parseBits(MISC_NAMES, f[8], misc))
    return false;
  PadState pad = {};
  pad.connected = conn != 0;
  pad.axisX = (int32_t)ax;
  pad.axisY = (int32_t)ay;
  pad.axisRX = (int32_t)arx;
  pad.axisRY = (int32_t)ary;
  pad.buttons = (uint16_t)buttons;
  pad.dpad = (uint8_t)dpad;
  pad.misc = (uint8_t)misc;
  Desired d = update(m, pad, (uint32_t)t);
  r.steps++;

  unsigned mode, stride, stick;
  long v[11];
  static const long LO[11] = {-100, -100, -100, 0, 0, 0, 0, 0, 0, 0, 0};
  static const long HI[11] = {100, 100, 100, 100, 100, 255, 1, 255, 255, 255, 255};
  const char *idx[11] = {f[11], f[12], f[13], f[14], f[15], f[16], f[19], f[20], f[21], f[22], f[23]};
  if (!lookup(MODE_NAMES, f[10], mode) || !lookup(STRIDE_NAMES, f[17], stride) || !lookup(STICK_NAMES, f[18], stick))
    return false;
  for (int i = 0; i < 11; i++)
    if (!parseInt(idx[i], LO[i], HI[i], v[i]))
      return false;

  const struct
  {
    const char *name;
    long want, got;
  } checks[] = {{"mode", (long)mode, (long)d.mode},
                {"translateX", v[0], d.translateX},
                {"translateY", v[1], d.translateY},
                {"rotate", v[2], d.rotate},
                {"height", v[3], d.height},
                {"speed", v[4], d.speed},
                {"gait", v[5], d.gait},
                {"stride", (long)stride, (long)d.stride},
                {"stickMode", (long)stick, (long)d.stickMode},
                {"controllerConnected", v[6], d.controllerConnected ? 1 : 0},
                {"clearCounter", v[7], d.clearCounter},
                {"modeRequestCounter", v[8], d.modeRequestCounter},
                {"gesture", v[9], d.gesture},
                {"gestureRequestCounter", v[10], d.gestureRequestCounter}};
  for (const auto &c : checks)
    if (c.want != c.got)
    {
      char what[128];
      snprintf(what, sizeof(what), "%s expected %ld, got %ld", c.name, c.want, c.got);
      fail(r, line, what);
    }
  return true;
}

static void goldenPath(char *out, int size)
{
  const char *override = getenv("SPOODER_GAMEPAD_GOLDEN");
  if (override && override[0])
  {
    snprintf(out, size, "%s", override);
    return;
  }
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_gamepad_mapping_golden/");
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*stest/golden/gamepad_mapping.txt", prefix, file);
}

void test_golden_steps()
{
  char path[512];
  goldenPath(path, sizeof(path));
  FILE *fp = fopen(path, "rb");
  TEST_ASSERT_NOT_NULL_MESSAGE(fp, "cannot read test/golden/gamepad_mapping.txt");

  static Mapper m;
  bool inSequence = false;
  int sequences = 0;
  Report r = {};
  char text[512];
  for (int line = 1; fgets(text, sizeof(text), fp); line++)
  {
    size_t len = strlen(text);
    if (len && text[len - 1] == '\n')
      text[--len] = 0;
    if (len == 0 || text[0] == '#')
      continue;
    if (strncmp(text, "reset ", 6) == 0 && text[6])
    {
      m = Mapper();
      inSequence = true;
      sequences++;
      continue;
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "line %d: step before a reset", line);
    TEST_ASSERT_TRUE_MESSAGE(inSequence, msg);
    snprintf(msg, sizeof(msg), "line %d: malformed step", line);
    TEST_ASSERT_TRUE_MESSAGE(runStep(m, text, line, r), msg);
  }
  fclose(fp);

  // A truncated or emptied file must not pass.
  TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(8, sequences, "too few sequences");
  TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(100, r.steps, "too few steps");
  if (r.failures)
  {
    char msg[320];
    snprintf(msg, sizeof(msg), "%d mismatches; first: %s", r.failures, r.first);
    TEST_FAIL_MESSAGE(msg);
  }
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_golden_steps);
  return UNITY_END();
}
