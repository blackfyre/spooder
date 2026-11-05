// The drive mode: the boot decision and the switch notice's debounce.
#include <unity.h>
#include "wifi_drive/drive_mode.h"

using drive_mode::Mode;
using drive_mode::NOTICE_MS;

void setUp() {}
void tearDown() {}

void test_boot_decision_for_every_combination()
{
  TEST_ASSERT_TRUE(drive_mode::select(false, false) == Mode::Bluetooth); // open, or not fitted
  TEST_ASSERT_TRUE(drive_mode::select(false, true) == Mode::Wifi);       // closed to ground
  TEST_ASSERT_TRUE(drive_mode::select(true, false) == Mode::Setup);      // setup wins
  TEST_ASSERT_TRUE(drive_mode::select(true, true) == Mode::Setup);
}

void test_zero_value_is_bluetooth_drive()
{
  Mode m = {};
  TEST_ASSERT_TRUE(m == Mode::Bluetooth);
}

void test_notice_after_100_ms_not_99()
{
  drive_mode::Notice n;
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, false, 0));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, 1000)); // closed while running Bluetooth drive
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, 1000 + NOTICE_MS - 1));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, true, 1000 + NOTICE_MS));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, true, 5000));
}

void test_notice_in_wifi_drive_when_the_switch_opens()
{
  drive_mode::Notice n;
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Wifi, true, 0));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Wifi, false, 10));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Wifi, false, 10 + NOTICE_MS));
}

void test_bounce_shorter_than_100_ms_shows_nothing()
{
  drive_mode::Notice n;
  uint32_t t = 0;
  // Repeated 60 ms bounces, each followed by a return: never 100 ms in a row.
  for (int i = 0; i < 10; i++)
  {
    TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, t));
    TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, t + 60));
    TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, false, t + 70));
    t += 80;
  }
  // A 99 ms change that returns shows nothing either.
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, t));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, t + NOTICE_MS - 1));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, false, t + NOTICE_MS));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, t + NOTICE_MS + 50));
}

void test_notice_clears_once_the_switch_is_back_for_100_ms()
{
  drive_mode::Notice n;
  drive_mode::update(n, Mode::Bluetooth, true, 0);
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, true, NOTICE_MS));
  // Back, with a bounce first: the notice stays.
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, false, 200));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, true, 250));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, false, 300));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, false, 300 + NOTICE_MS - 1));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, false, 300 + NOTICE_MS));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, false, 2000));
}

void test_setup_mode_ignores_the_switch()
{
  drive_mode::Notice n;
  for (uint32_t t = 0; t <= 1000; t += 20)
  {
    TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Setup, true, t));
    TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Setup, false, t + 10));
  }
}

void test_debounce_across_the_millisecond_wrap()
{
  drive_mode::Notice n;
  uint32_t start = 0xFFFFFFFFu - 40;
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, start));
  TEST_ASSERT_FALSE(drive_mode::update(n, Mode::Bluetooth, true, start + NOTICE_MS - 1));
  TEST_ASSERT_TRUE(drive_mode::update(n, Mode::Bluetooth, true, start + NOTICE_MS));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_boot_decision_for_every_combination);
  RUN_TEST(test_zero_value_is_bluetooth_drive);
  RUN_TEST(test_notice_after_100_ms_not_99);
  RUN_TEST(test_notice_in_wifi_drive_when_the_switch_opens);
  RUN_TEST(test_bounce_shorter_than_100_ms_shows_nothing);
  RUN_TEST(test_notice_clears_once_the_switch_is_back_for_100_ms);
  RUN_TEST(test_setup_mode_ignores_the_switch);
  RUN_TEST(test_debounce_across_the_millisecond_wrap);
  return UNITY_END();
}
