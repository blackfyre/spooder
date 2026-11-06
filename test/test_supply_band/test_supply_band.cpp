// Runs core/status/supply_band over test/golden/supply_bands.txt, the
// hand-written cases the handheld's Go copy
// (desktop/handheld/internal/supplyband) is also tested against.
// SPOODER_SUPPLY_BANDS_GOLDEN names another copy of the file (to check that
// a changed band fails the test).
#include <unity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "status/supply_band.h"

using namespace supply_band;

void setUp() {}
void tearDown() {}

static void goldenPath(char *out, int size)
{
  const char *override = getenv("SPOODER_SUPPLY_BANDS_GOLDEN");
  if (override && override[0])
  {
    snprintf(out, size, "%s", override);
    return;
  }
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_supply_band/");
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*stest/golden/supply_bands.txt", prefix, file);
}

static bool parseBand(const char *s, Band &out)
{
  static const struct
  {
    const char *name;
    Band band;
  } names[] = {{"unknown", Band::Unknown}, {"ok", Band::Ok}, {"warning", Band::Warning}, {"fault", Band::Fault}};
  for (const auto &n : names)
    if (strcmp(n.name, s) == 0)
    {
      out = n.band;
      return true;
    }
  return false;
}

static bool parseU16(const char *s, uint16_t &out)
{
  char *end;
  long v = strtol(s, &end, 10);
  if (!*s || *end || v < 0 || v > 65535)
    return false;
  out = (uint16_t)v; // within 0..65535, checked above
  return true;
}

void test_golden_cases()
{
  char path[512];
  goldenPath(path, sizeof(path));
  FILE *fp = fopen(path, "rb");
  TEST_ASSERT_NOT_NULL_MESSAGE(fp, "cannot read test/golden/supply_bands.txt");

  Limits limits = {};
  bool haveLimits = false;
  int cases = 0, failures = 0;
  char text[256];
  for (int line = 1; fgets(text, sizeof(text), fp); line++)
  {
    size_t len = strlen(text);
    if (len && text[len - 1] == '\n')
      text[--len] = 0;
    if (len == 0 || text[0] == '#')
      continue;
    char *f[7];
    int n = 0;
    for (char *save = nullptr, *tok = strtok_r(text, " ", &save); tok && n < 7; tok = strtok_r(nullptr, " ", &save))
      f[n++] = tok;
    char msg[96];
    snprintf(msg, sizeof(msg), "line %d: malformed", line);
    if (n == 6 && strcmp(f[0], "limits") == 0)
    {
      bool ok = parseU16(f[1], limits.brownoutMv) && parseU16(f[2], limits.minMv) && parseU16(f[3], limits.maxMv) &&
                parseU16(f[4], limits.sustainedCurrentMa) && parseU16(f[5], limits.tripCurrentMa);
      TEST_ASSERT_TRUE_MESSAGE(ok, msg);
      haveLimits = true;
      continue;
    }
    uint16_t reading = 0;
    Band want = Band::Unknown;
    bool isVoltage = n == 3 && strcmp(f[0], "voltage") == 0;
    bool isCurrent = n == 3 && strcmp(f[0], "current") == 0;
    TEST_ASSERT_TRUE_MESSAGE((isVoltage || isCurrent) && haveLimits && parseU16(f[1], reading) && parseBand(f[2], want),
                             msg);
    Band got = isVoltage ? voltage(limits, reading) : current(limits, reading);
    cases++;
    if (got != want)
    {
      failures++;
      printf("line %d: %s %u: band %d, want %d\n", line, f[0], reading, (int)got, (int)want);
    }
  }
  fclose(fp);
  TEST_ASSERT_TRUE_MESSAGE(cases > 0, "no cases in test/golden/supply_bands.txt");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, failures, "bands differ from test/golden/supply_bands.txt");
}

// The limits are a diagnostics frame's, field by field.
void test_from_diagnostics()
{
  protocol::Diagnostics d = {};
  d.brownoutMv = 4800;
  d.minMv = 5200;
  d.maxMv = 6800;
  d.sustainedCurrentMa = 6000;
  d.tripCurrentMa = 9000;
  Limits l = fromDiagnostics(d);
  TEST_ASSERT_EQUAL_UINT16(4800, l.brownoutMv);
  TEST_ASSERT_EQUAL_UINT16(5200, l.minMv);
  TEST_ASSERT_EQUAL_UINT16(6800, l.maxMv);
  TEST_ASSERT_EQUAL_UINT16(6000, l.sustainedCurrentMa);
  TEST_ASSERT_EQUAL_UINT16(9000, l.tripCurrentMa);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_golden_cases);
  RUN_TEST(test_from_diagnostics);
  return UNITY_END();
}
