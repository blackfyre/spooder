#include <unity.h>
#include "protection/output_hold.h"

using namespace protection;

static OutputHold h;

void setUp() { h = OutputHold(); }
void tearDown() {}

void test_normal_operation_follows_body()
{
  TEST_ASSERT_TRUE(outputAllowed(h, false, true, 0));
  TEST_ASSERT_FALSE(outputAllowed(h, false, false, 0)); // Sleep folded
  TEST_ASSERT_TRUE(outputAllowed(h, false, true, 1));   // woke
}

void test_clear_mid_fold_keeps_output_off()
{
  outputAllowed(h, false, true, 0);
  TEST_ASSERT_FALSE(outputAllowed(h, true, true, 0)); // overcurrent while active
  // Cleared and settled while the body is still folding (still wants output).
  for (int i = 0; i < 20; i++)
    TEST_ASSERT_FALSE(outputAllowed(h, false, true, 0));
}

void test_output_returns_after_a_wake()
{
  outputAllowed(h, false, true, 0);
  outputAllowed(h, true, true, 0);    // inhibited
  outputAllowed(h, false, true, 0);   // cleared, still folding: off
  TEST_ASSERT_FALSE(outputAllowed(h, false, false, 0)); // fold done, asleep
  TEST_ASSERT_TRUE(outputAllowed(h, false, true, 1));   // fresh request woke it
}

void test_fold_and_wake_within_one_step_releases()
{
  // The adapter never sees outputsEnabled go false: the fold completed and the
  // body woke inside the same step. The wake count still changed.
  outputAllowed(h, false, true, 4);
  outputAllowed(h, true, true, 4);
  outputAllowed(h, false, true, 4);
  TEST_ASSERT_TRUE(outputAllowed(h, false, true, 5));
}

void test_wake_while_still_inhibited_does_not_release()
{
  outputAllowed(h, false, true, 0);
  outputAllowed(h, true, true, 0);
  TEST_ASSERT_FALSE(outputAllowed(h, true, true, 1)); // woke, but inhibited again
  TEST_ASSERT_FALSE(outputAllowed(h, false, true, 1)); // needs a wake after the inhibit
  TEST_ASSERT_TRUE(outputAllowed(h, false, true, 2));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_normal_operation_follows_body);
  RUN_TEST(test_clear_mid_fold_keeps_output_off);
  RUN_TEST(test_output_returns_after_a_wake);
  RUN_TEST(test_fold_and_wake_within_one_step_releases);
  RUN_TEST(test_wake_while_still_inhibited_does_not_release);
  return UNITY_END();
}
