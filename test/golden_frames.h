#ifndef TEST_GOLDEN_FRAMES_H
#define TEST_GOLDEN_FRAMES_H

// Reads the valid frames of test/golden/protocol_v3.txt (name and bytes),
// for tests that need real frames of every type. Header-only: each test
// program that includes it gets its own copy.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "protocol/protocol.h"

namespace golden_frames
{

constexpr int MAX_FRAMES = 64;

struct Frame
{
  char name[48];
  uint8_t bytes[protocol::MAX_FRAME_SIZE];
  int length;
};

struct Set
{
  Frame frames[MAX_FRAMES];
  int count;
};

// The golden file, found from this test's own path (its last "test/test_").
inline void path(const char *file, char *out, int size)
{
  const char *cut = nullptr;
  for (const char *at = strstr(file, "test/test_"); at; at = strstr(at + 1, "test/test_"))
    cut = at;
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*stest/golden/protocol_v3.txt", prefix, file);
}

inline int hexDigit(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

/**
 * @brief Loads every frame line that is not "rejected".
 * @return false if the file cannot be read or a line does not parse
 */
inline bool load(const char *file, Set &s)
{
  char p[512];
  path(file, p, sizeof(p));
  FILE *f = fopen(p, "rb");
  if (!f)
    return false;
  s.count = 0;
  static char line[8192];
  bool ok = true;
  while (ok && fgets(line, sizeof(line), f))
  {
    if (line[0] == '#' || line[0] == '\n')
      continue;
    char *name = strtok(line, " \n");
    char *hex = strtok(nullptr, " \n");
    char *first = strtok(nullptr, " \n");
    if (!name || !hex || !first)
    {
      ok = false;
      break;
    }
    if (strcmp(first, "rejected") == 0)
      continue;
    int n = (int)strlen(hex);
    if (s.count == MAX_FRAMES || n % 2 != 0 || n / 2 > protocol::MAX_FRAME_SIZE)
    {
      ok = false;
      break;
    }
    Frame &fr = s.frames[s.count++];
    snprintf(fr.name, sizeof(fr.name), "%s", name);
    fr.length = n / 2;
    for (int i = 0; i < fr.length; i++)
    {
      int hi = hexDigit(hex[2 * i]), lo = hexDigit(hex[2 * i + 1]);
      if (hi < 0 || lo < 0)
        ok = false;
      fr.bytes[i] = (uint8_t)(hi << 4 | lo);
    }
  }
  fclose(f);
  return ok && s.count > 0;
}

} // namespace golden_frames

#endif // TEST_GOLDEN_FRAMES_H
