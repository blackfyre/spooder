#include "access_point.h"

#include <stdio.h>

namespace access_point
{

void ssid(const uint8_t mac[6], char out[SSID_SIZE])
{
  snprintf(out, SSID_SIZE, "spooder-%02x%02x", mac[4], mac[5]);
}

void makePassword(uint32_t (*random)(void), char out[PASSWORD_LENGTH + 1])
{
  constexpr uint32_t ACCEPT_BELOW = 256 / PASSWORD_ALPHABET_SIZE * PASSWORD_ALPHABET_SIZE; // 248
  int n = 0;
  while (n < PASSWORD_LENGTH)
  {
    uint32_t word = random();
    for (int b = 0; b < 4 && n < PASSWORD_LENGTH; b++, word >>= 8)
    {
      uint32_t byte = word & 0xFF;
      if (byte < ACCEPT_BELOW)
        out[n++] = PASSWORD_ALPHABET[byte % PASSWORD_ALPHABET_SIZE];
    }
  }
  out[PASSWORD_LENGTH] = 0;
}

bool validPassword(const char *p)
{
  if (p == nullptr)
    return false;
  int n = 0;
  for (; p[n] != 0; n++)
  {
    if (n >= PASSWORD_LENGTH)
      return false;
    bool known = false;
    for (int a = 0; a < PASSWORD_ALPHABET_SIZE && !known; a++)
      known = p[n] == PASSWORD_ALPHABET[a];
    if (!known)
      return false;
  }
  return n == PASSWORD_LENGTH;
}

int joinCode(const char *ssid, const char *password, char *out, int size)
{
  if (out == nullptr || size <= 0)
    return -1;
  int n = snprintf(out, size, "WIFI:T:WPA;S:%s;P:%s;;", ssid, password);
  if (n < 0 || n >= size)
  {
    out[0] = 0;
    return -1;
  }
  return n;
}

} // namespace access_point
