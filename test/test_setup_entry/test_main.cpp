// Setup-mode entry (button hold, boot decision) and the access point's
// credentials.
#include <unity.h>
#include <string.h>
#include "setup/access_point.h"
#include "setup/setup_entry.h"

void setUp() {}
void tearDown() {}

// --- setup_entry -------------------------------------------------------------

void test_hold_fires_once_after_one_second()
{
  setup_entry::Hold h;
  TEST_ASSERT_FALSE(setup_entry::update(h, true, 100));
  TEST_ASSERT_TRUE(setup_entry::held(h));
  TEST_ASSERT_FALSE(setup_entry::update(h, true, 100 + setup_entry::HOLD_MS - 1));
  TEST_ASSERT_TRUE(setup_entry::update(h, true, 100 + setup_entry::HOLD_MS));
  TEST_ASSERT_FALSE(setup_entry::update(h, true, 100 + setup_entry::HOLD_MS + 500));
}

void test_release_restarts_the_hold()
{
  setup_entry::Hold h;
  setup_entry::update(h, true, 0);
  TEST_ASSERT_FALSE(setup_entry::update(h, false, 900));
  TEST_ASSERT_FALSE(setup_entry::held(h));
  TEST_ASSERT_FALSE(setup_entry::update(h, true, 950));
  TEST_ASSERT_FALSE(setup_entry::update(h, true, 1500));
  TEST_ASSERT_TRUE(setup_entry::update(h, true, 1950));
}

void test_setup_mode_only_after_a_software_restart_with_the_magic()
{
  TEST_ASSERT_TRUE(setup_entry::startInSetupMode(setup_entry::MAGIC, true));
  // Power-up, the reset button, a watchdog or a panic: normal operation,
  // whatever the RTC memory holds.
  TEST_ASSERT_FALSE(setup_entry::startInSetupMode(setup_entry::MAGIC, false));
  TEST_ASSERT_FALSE(setup_entry::startInSetupMode(0, true));
  TEST_ASSERT_FALSE(setup_entry::startInSetupMode(setup_entry::MAGIC ^ 1, true));
  TEST_ASSERT_FALSE(setup_entry::startInSetupMode(0xFFFFFFFFu, false));
}

// --- access_point --------------------------------------------------------------

void test_ssid_uses_the_last_four_hex_digits_of_the_mac()
{
  const uint8_t mac[6] = {0x24, 0x0A, 0xC4, 0x12, 0xAB, 0x0F};
  char ssid[access_point::SSID_SIZE];
  access_point::ssid(mac, ssid);
  TEST_ASSERT_EQUAL_STRING("spooder-ab0f", ssid);
}

static uint32_t sequence[64];
static int sequenceAt;
static uint32_t fromSequence()
{
  return sequence[sequenceAt++ % 64];
}

void test_password_has_the_length_and_alphabet()
{
  for (int i = 0; i < 64; i++)
    sequence[i] = 0x9E3779B9u * (uint32_t)(i + 1);
  sequenceAt = 0;
  char p[access_point::PASSWORD_LENGTH + 1];
  access_point::makePassword(fromSequence, p);
  TEST_ASSERT_EQUAL_INT(access_point::PASSWORD_LENGTH, (int)strlen(p));
  TEST_ASSERT_TRUE(access_point::validPassword(p));
  for (const char *c = p; *c; c++)
    TEST_ASSERT_NULL(strchr("0o1liIO", *c));
}

void test_password_maps_bytes_without_bias()
{
  // Bytes 248..255 are discarded; 0..247 map to alphabet[byte % 31].
  sequence[0] = 0xFFFEFDF8u; // all four discarded
  sequence[1] = 0x1F1E0100u; // 0, 1, 30, 31 -> a, b, 9, a
  sequence[2] = 0x3E3D3C3Bu; // 59..62 -> 28..31 % 31 -> 7, 8, 9, a
  sequence[3] = 0xF7F7F7F7u; // 247 % 31 = 30 -> 9 (only two needed)
  sequenceAt = 0;
  char p[access_point::PASSWORD_LENGTH + 1];
  access_point::makePassword(fromSequence, p);
  TEST_ASSERT_EQUAL_STRING("ab9a789a99", p);
}

void test_every_character_is_reachable()
{
  bool seen[access_point::PASSWORD_ALPHABET_SIZE] = {};
  for (int i = 0; i < 64; i++)
    sequence[i] = (uint32_t)(4 * i) | (uint32_t)(4 * i + 1) << 8 | (uint32_t)(4 * i + 2) << 16 |
                  (uint32_t)(4 * i + 3) << 24;
  sequenceAt = 0;
  for (int k = 0; k < 8; k++)
  {
    char p[access_point::PASSWORD_LENGTH + 1];
    access_point::makePassword(fromSequence, p);
    for (const char *c = p; *c; c++)
      seen[strchr(access_point::PASSWORD_ALPHABET, *c) - access_point::PASSWORD_ALPHABET] = true;
  }
  for (int a = 0; a < access_point::PASSWORD_ALPHABET_SIZE; a++)
    TEST_ASSERT_TRUE(seen[a]);
}

void test_stored_password_validation()
{
  TEST_ASSERT_TRUE(access_point::validPassword("abcdefgh23"));
  TEST_ASSERT_FALSE(access_point::validPassword(nullptr));
  TEST_ASSERT_FALSE(access_point::validPassword(""));
  TEST_ASSERT_FALSE(access_point::validPassword("abcdefgh2"));   // short
  TEST_ASSERT_FALSE(access_point::validPassword("abcdefgh234")); // long
  TEST_ASSERT_FALSE(access_point::validPassword("abcdefgh2o"));  // ambiguous
  TEST_ASSERT_FALSE(access_point::validPassword("ABCDEFGH23"));
}

void test_join_code()
{
  char code[access_point::JOIN_CODE_SIZE];
  int n = access_point::joinCode("spooder-ab0f", "abcdefgh23", code, sizeof(code));
  TEST_ASSERT_EQUAL_STRING("WIFI:T:WPA;S:spooder-ab0f;P:abcdefgh23;;", code);
  TEST_ASSERT_EQUAL_INT((int)strlen(code), n);
  TEST_ASSERT_EQUAL_INT(access_point::JOIN_CODE_SIZE - 1, n);
  char small[10];
  TEST_ASSERT_EQUAL_INT(-1, access_point::joinCode("spooder-ab0f", "abcdefgh23", small, sizeof(small)));
  TEST_ASSERT_EQUAL_STRING("", small);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_hold_fires_once_after_one_second);
  RUN_TEST(test_release_restarts_the_hold);
  RUN_TEST(test_setup_mode_only_after_a_software_restart_with_the_magic);
  RUN_TEST(test_ssid_uses_the_last_four_hex_digits_of_the_mac);
  RUN_TEST(test_password_has_the_length_and_alphabet);
  RUN_TEST(test_password_maps_bytes_without_bias);
  RUN_TEST(test_every_character_is_reachable);
  RUN_TEST(test_stored_password_validation);
  RUN_TEST(test_join_code);
  return UNITY_END();
}
