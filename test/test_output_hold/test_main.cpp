#include <unity.h>
#include "protection/output_hold.h"

using namespace protection;

static OutputHold h;

void setUp() { h = OutputHold(); }
void tearDown() {}

void test_boot_after_supply_gate_allows_output()
{
  // Output inhibited while the supply settles, then the body starts.
  TEST_ASSERT_FALSE(outputAllowed(h, true, false));
  TEST_ASSERT_FALSE(outputAllowed(h, true, false));
  TEST_ASSERT_TRUE(outputAllowed(h, false, true)); // body start is a wake
}

void test_normal_operation_follows_body()
{
  outputAllowed(h, false, true);
  TEST_ASSERT_TRUE(outputAllowed(h, false, true));
  TEST_ASSERT_FALSE(outputAllowed(h, false, false)); // Sleep folded
  TEST_ASSERT_TRUE(outputAllowed(h, false, true));   // woke
}

void test_clear_mid_fold_keeps_output_off()
{
  outputAllowed(h, false, true);
  TEST_ASSERT_FALSE(outputAllowed(h, true, true)); // overcurrent while active
  // Cleared and settled while the body is still folding (still wants output).
  for (int i = 0; i < 20; i++)
    TEST_ASSERT_FALSE(outputAllowed(h, false, true));
}

void test_output_returns_only_at_a_wake()
{
  outputAllowed(h, false, true);
  outputAllowed(h, true, true);    // inhibited
  outputAllowed(h, false, true);   // cleared, still folding: off
  TEST_ASSERT_FALSE(outputAllowed(h, false, false)); // fold done
  TEST_ASSERT_FALSE(outputAllowed(h, false, false)); // asleep, awaiting a request
  TEST_ASSERT_TRUE(outputAllowed(h, false, true));   // fresh request wakes it
}

void test_wake_while_still_inhibited_does_not_release()
{
  outputAllowed(h, false, true);
  outputAllowed(h, true, true);
  outputAllowed(h, true, false);
  TEST_ASSERT_FALSE(outputAllowed(h, true, true)); // wake edge, but still inhibited
  TEST_ASSERT_FALSE(outputAllowed(h, false, true)); // edge already consumed: no release
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_boot_after_supply_gate_allows_output);
  RUN_TEST(test_normal_operation_follows_body);
  RUN_TEST(test_clear_mid_fold_keeps_output_off);
  RUN_TEST(test_output_returns_only_at_a_wake);
  RUN_TEST(test_wake_while_still_inhibited_does_not_release);
  return UNITY_END();
}
