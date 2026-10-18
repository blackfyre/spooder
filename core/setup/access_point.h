#ifndef ACCESS_POINT_H
#define ACCESS_POINT_H

// The setup-mode access point's credentials: the network name from the MAC,
// a random password (made once, then kept by the bridge), and the joining
// QR code's text.
// Hardware-free (snprintf only): the bridge supplies the MAC, the random
// source and the storage; host tests drive it directly.
#include <stdint.h>

namespace access_point
{

constexpr int SSID_SIZE = 13; // "spooder-" + 4 hex digits + NUL
constexpr int PASSWORD_LENGTH = 10;
// No characters that are easily confused (0/o, 1/l/i), and no upper case,
// so the password is easy to type from the screen.
constexpr char PASSWORD_ALPHABET[] = "abcdefghjkmnpqrstuvwxyz23456789";
constexpr int PASSWORD_ALPHABET_SIZE = sizeof(PASSWORD_ALPHABET) - 1; // 31
// "WIFI:T:WPA;S:" + SSID + ";P:" + password + ";;" + NUL
constexpr int JOIN_CODE_SIZE = 13 + (SSID_SIZE - 1) + 3 + PASSWORD_LENGTH + 2 + 1;

/**
 * @brief "spooder-" and the last four hex digits (lower case) of @p mac.
 */
void ssid(const uint8_t mac[6], char out[SSID_SIZE]);

/**
 * @brief A random password of PASSWORD_LENGTH characters from
 * PASSWORD_ALPHABET. Each random byte below the largest multiple of the
 * alphabet size picks one character; the rest are discarded, so every
 * character is equally likely.
 * @param random a source of uniformly random 32-bit words
 */
void makePassword(uint32_t (*random)(void), char out[PASSWORD_LENGTH + 1]);

/**
 * @brief True if @p p could have come from makePassword (a stored password
 * that does not is replaced).
 */
bool validPassword(const char *p);

/**
 * @brief The text of the joining QR code:
 * `WIFI:T:WPA;S:<ssid>;P:<password>;;`. Neither ever holds a character that
 * would need escaping.
 * @return the text length, or -1 if it does not fit (out is then empty)
 */
int joinCode(const char *ssid, const char *password, char *out, int size);

} // namespace access_point

#endif // ACCESS_POINT_H
