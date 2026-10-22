#include <unity.h>
#include "status/page_button.h"

using namespace page_button;

static State s;
static uint32_t now;

void setUp()
{
  s = State();
  now = 1000;
  update(s, false, now); // released when sampling starts
}
void tearDown() {}

// Holds the button for @p ms, sampled every 10 ms, then releases it.
static bool press(uint32_t ms)
{
  bool changed = false;
  for (uint32_t t = 0; t < ms; t += 10)
    changed |= update(s, true, now + t);
  now += ms;
  changed |= update(s, false, now);
  now += 100;
  update(s, false, now);
  return changed;
}

void test_starts_on_the_drive_page()
{
  TEST_ASSERT_EQUAL_INT((int)Page::Drive, (int)s.page);
}

// The spec's cycling scenario.
void test_three_presses_cycle_back_to_drive()
{
  TEST_ASSERT_TRUE(press(100));
  TEST_ASSERT_EQUAL_INT((int)Page::Power, (int)s.page);
  TEST_ASSERT_TRUE(press(100));
  TEST_ASSERT_EQUAL_INT((int)Page::Link, (int)s.page);
  TEST_ASSERT_TRUE(press(100));
  TEST_ASSERT_EQUAL_INT((int)Page::Drive, (int)s.page);
}

void test_counts_on_release_not_on_press()
{
  for (uint32_t t = 0; t < 200; t += 10)
    TEST_ASSERT_FALSE(update(s, true, now + t));
  TEST_ASSERT_EQUAL_INT((int)Page::Drive, (int)s.page);
  TEST_ASSERT_TRUE(update(s, false, now + 200));
}

void test_bounce_is_ignored()
{
  TEST_ASSERT_FALSE(press(20));
  TEST_ASSERT_EQUAL_INT((int)Page::Drive, (int)s.page);
}

void test_shortest_press_counts()
{
  update(s, true, now);
  TEST_ASSERT_TRUE(update(s, false, now + MIN_PRESS_MS));
}

// A press begun during the start-up sequence (already down at the first
// sample) never counts, however short what remains of it.
void test_press_from_start_up_is_ignored()
{
  s = State();
  TEST_ASSERT_FALSE(update(s, true, now));
  TEST_ASSERT_FALSE(update(s, true, now + 50));
  TEST_ASSERT_FALSE(update(s, false, now + 100));
  TEST_ASSERT_EQUAL_INT((int)Page::Drive, (int)s.page);
  now += 200;
  TEST_ASSERT_TRUE(press(100)); // the next press counts
}

void test_long_hold_is_ignored()
{
  TEST_ASSERT_FALSE(press(MAX_PRESS_MS));
  TEST_ASSERT_FALSE(press(5000));
  TEST_ASSERT_EQUAL_INT((int)Page::Drive, (int)s.page);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_starts_on_the_drive_page);
  RUN_TEST(test_three_presses_cycle_back_to_drive);
  RUN_TEST(test_counts_on_release_not_on_press);
  RUN_TEST(test_bounce_is_ignored);
  RUN_TEST(test_shortest_press_counts);
  RUN_TEST(test_press_from_start_up_is_ignored);
  RUN_TEST(test_long_hold_is_ignored);
  return UNITY_END();
}
