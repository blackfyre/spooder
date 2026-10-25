#include "build_identity.h"

namespace build_identity
{

namespace
{

const char *orUnknown(const char *id)
{
  return id == nullptr || id[0] == '\0' ? "unknown" : id;
}

char printable(char c)
{
  unsigned char b = (unsigned char)c;
  return b >= 0x20 && b <= 0x7E ? c : '?';
}

int length(const char *s)
{
  int n = 0;
  while (s[n] != '\0')
    n++;
  return n;
}

// Appends @p c while there is room for it and the NUL.
void put(char *out, int size, int &n, char c)
{
  if (n < size - 1)
    out[n++] = c;
}

void putText(char *out, int size, int &n, const char *s)
{
  for (; *s != '\0'; s++)
    put(out, size, n, printable(*s));
}

} // namespace

int banner(char *out, int size, const char *component, const char *id)
{
  if (out == nullptr || size <= 0)
    return 0;
  int n = 0;
  putText(out, size, n, "spooder ");
  putText(out, size, n, component == nullptr ? "" : component);
  putText(out, size, n, ": build ");
  id = orUnknown(id);
  for (int i = 0; i < MAX_ID_CHARS && id[i] != '\0'; i++)
    put(out, size, n, printable(id[i]));
  put(out, size, n, '\n');
  out[n] = '\0';
  return n;
}

void fit(char *out, int width, const char *id)
{
  if (out == nullptr)
    return;
  if (width <= 0)
  {
    out[0] = '\0';
    return;
  }
  id = orUnknown(id);
  int len = length(id);
  int n = 0;
  int from = 0;
  if (len > width)
  {
    // `..` then the last width - 2 characters (only dots when narrower).
    for (int i = 0; i < 2 && n < width; i++)
      out[n++] = '.';
    from = len - (width - n);
  }
  for (int i = from; i < len && n < width; i++)
    out[n++] = printable(id[i]);
  out[n] = '\0';
}

} // namespace build_identity
