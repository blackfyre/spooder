#include <unity.h>
#include "protection/supervisor.h"

using namespace protection;

static Supervisor sv;

static Result result(bool allowOutput, uint16_t faults, uint16_t newly = 0)
{
  Result r = {};
  r.allowOutput = allowOutput;
  r.forceSleep = faults != 0;
  r.faults = faults;
  r.newlyLatched = newly;
  return r;
}

static const Result SETTLING = result(false, NONE);
static const Result HEALTHY = result(true, NONE);

void setUp() { sv = Supervisor(); }
void tearDown() {}

void test_boot_starts_body_once_supply_settles()
{
  Decision d = supervise(sv, SETTLING, RequestedMode::Stand, 0);
  TEST_ASSERT_FALSE(d.runBody);
  d = supervise(sv, HEALTHY, RequestedMode::Stand, 0);
  TEST_ASSERT_TRUE(d.startBody);
  TEST_ASSERT_TRUE(d.runBody);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Stand);
  d = supervise(sv, HEALTHY, RequestedMode::Stand, 0);
  TEST_ASSERT_FALSE(d.startBody);
  TEST_ASSERT_TRUE(d.runBody);
}

void test_fault_before_startup_needs_a_request_after_clear()
{
  // Overvoltage latches before the supply was ever accepted.
  supervise(sv, result(false, OVERVOLTAGE, OVERVOLTAGE), RequestedMode::Stand, 0);
  supervise(sv, result(false, OVERVOLTAGE), RequestedMode::Stand, 0);
  // Cleared and settled: still no movement without a new request.
  for (int i = 0; i < 10; i++)
  {
    Decision d = supervise(sv, HEALTHY, RequestedMode::Stand, 0);
    TEST_ASSERT_FALSE(d.runBody);
  }
  Decision d = supervise(sv, HEALTHY, RequestedMode::Stand, 1);
  TEST_ASSERT_TRUE(d.startBody);
}

void test_inhibiting_fault_stops_the_body()
{
  supervise(sv, HEALTHY, RequestedMode::Walk, 0);
  Decision d = supervise(sv, result(false, OVERCURRENT, OVERCURRENT), RequestedMode::Walk, 0);
  TEST_ASSERT_TRUE(d.stopBody);
  TEST_ASSERT_FALSE(d.runBody);
}

void test_clear_alone_does_not_restart()
{
  supervise(sv, HEALTHY, RequestedMode::Walk, 0);
  supervise(sv, result(false, OVERCURRENT, OVERCURRENT), RequestedMode::Walk, 0);
  supervise(sv, SETTLING, RequestedMode::Walk, 0); // cleared, supply settling
  for (int i = 0; i < 10; i++)
    TEST_ASSERT_FALSE(supervise(sv, HEALTHY, RequestedMode::Walk, 0).runBody);
}

void test_request_during_settling_restarts_once_allowed()
{
  supervise(sv, HEALTHY, RequestedMode::Walk, 0);
  supervise(sv, result(false, OVERCURRENT, OVERCURRENT), RequestedMode::Walk, 0);
  supervise(sv, SETTLING, RequestedMode::Walk, 0);
  supervise(sv, SETTLING, RequestedMode::Stand, 1); // request while still settling
  TEST_ASSERT_FALSE(supervise(sv, SETTLING, RequestedMode::Stand, 1).runBody);
  Decision d = supervise(sv, HEALTHY, RequestedMode::Stand, 1);
  TEST_ASSERT_TRUE(d.startBody);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Stand);
}

void test_sleep_request_does_not_restart_stopped_body()
{
  supervise(sv, HEALTHY, RequestedMode::Walk, 0);
  supervise(sv, result(false, OVERCURRENT, OVERCURRENT), RequestedMode::Walk, 0);
  supervise(sv, SETTLING, RequestedMode::Walk, 0);
  // Cleared and settled; the operator presses Sleep: a new request, but Sleep.
  for (int i = 0; i < 5; i++)
    TEST_ASSERT_FALSE(supervise(sv, HEALTHY, RequestedMode::Sleep, 1).runBody);
  // A later request to move does restart it.
  TEST_ASSERT_TRUE(supervise(sv, HEALTHY, RequestedMode::Stand, 2).startBody);
}

void test_sleep_at_boot_does_not_start_body()
{
  TEST_ASSERT_FALSE(supervise(sv, HEALTHY, RequestedMode::Sleep, 0).runBody);
  TEST_ASSERT_TRUE(supervise(sv, HEALTHY, RequestedMode::Stand, 1).startBody);
}

void test_sleep_only_fault_keeps_body_running_asleep()
{
  supervise(sv, HEALTHY, RequestedMode::Walk, 0);
  Decision d = supervise(sv, result(true, UNDERVOLTAGE, UNDERVOLTAGE), RequestedMode::Walk, 0);
  TEST_ASSERT_TRUE(d.runBody);
  TEST_ASSERT_FALSE(d.stopBody);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  d = supervise(sv, HEALTHY, RequestedMode::Walk, 0); // cleared, same request
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  d = supervise(sv, HEALTHY, RequestedMode::Walk, 1); // new request
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Walk);
  TEST_ASSERT_FALSE(d.startBody); // woke through Sleep, not restarted
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_boot_starts_body_once_supply_settles);
  RUN_TEST(test_fault_before_startup_needs_a_request_after_clear);
  RUN_TEST(test_inhibiting_fault_stops_the_body);
  RUN_TEST(test_clear_alone_does_not_restart);
  RUN_TEST(test_request_during_settling_restarts_once_allowed);
  RUN_TEST(test_sleep_request_does_not_restart_stopped_body);
  RUN_TEST(test_sleep_at_boot_does_not_start_body);
  RUN_TEST(test_sleep_only_fault_keeps_body_running_asleep);
  return UNITY_END();
}
