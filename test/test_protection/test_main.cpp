#include <unity.h>
#include "protection/protection.h"

using namespace protection;

// Representative limits (design D6 example for a regulated 6.0 V supply).
static const Limits LIMITS = {
    9.0f,  // tripCurrentA
    6.0f,  // sustainedCurrentA
    2000,  // sustainedWindowMs
    5.2f,  // minV
    200,   // undervoltDebounceMs
    4.8f,  // brownoutV
    6.8f,  // maxV
};

static State s;
static uint32_t now;
static Result last;

static void reset()
{
  init(s, LIMITS);
  now = 1000;
  last = step(s, Reading{0, 0}, now, false, false);
}

// Feeds the same reading every 20 ms for @p durationMs.
static void feed(float currentA, float voltageV, uint32_t durationMs, bool clear = false, bool estop = false)
{
  for (uint32_t t = 0; t < durationMs; t += 20)
  {
    now += 20;
    last = step(s, Reading{currentA, voltageV}, now, clear, estop);
  }
}

// Supply valid long enough for the gate to open.
static void powerUp()
{
  reset();
  feed(0.2f, 6.0f, 600);
  TEST_ASSERT_TRUE(last.allowOutput);
}

void setUp() {}
void tearDown() {}

// --- 2.1 power-on gate ------------------------------------------------------

void test_usb_only_keeps_output_off_without_faults()
{
  reset();
  feed(0, 0.02f, 5000);
  TEST_ASSERT_FALSE(last.allowOutput);
  TEST_ASSERT_FALSE(last.supplyValid);
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  TEST_ASSERT_FALSE(last.forceSleep);
}

void test_gate_opens_after_500ms_of_valid_supply()
{
  reset();
  feed(0.1f, 6.0f, 480);
  TEST_ASSERT_FALSE(last.allowOutput);
  feed(0.1f, 6.0f, 40);
  TEST_ASSERT_TRUE(last.allowOutput);
}

void test_gate_needs_continuous_validity()
{
  reset();
  feed(0.1f, 6.0f, 400);
  feed(0.1f, 3.0f, 20); // supply wobbles while connecting
  feed(0.1f, 6.0f, 400);
  TEST_ASSERT_FALSE(last.allowOutput);
  feed(0.1f, 6.0f, 120);
  TEST_ASSERT_TRUE(last.allowOutput);
}

// --- 2.2 trips --------------------------------------------------------------

void test_overcurrent_inhibits_on_the_same_step()
{
  powerUp();
  now += 20;
  last = step(s, Reading{9.5f, 6.0f}, now, false, false);
  TEST_ASSERT_FALSE(last.allowOutput);
  TEST_ASSERT_TRUE(last.forceSleep);
  TEST_ASSERT_EQUAL_UINT16(OVERCURRENT, last.faults);
  TEST_ASSERT_EQUAL_UINT16(OVERCURRENT, last.newlyLatched);
  // Reported once: not newly latched again on the next step.
  now += 20;
  last = step(s, Reading{9.5f, 6.0f}, now, false, false);
  TEST_ASSERT_EQUAL_UINT16(NONE, last.newlyLatched);
}

void test_sustained_current_puts_body_to_sleep()
{
  powerUp();
  feed(7.0f, 6.0f, 2100);
  TEST_ASSERT_TRUE(last.faults & SUSTAINED_CURRENT);
  TEST_ASSERT_TRUE(last.forceSleep);
  TEST_ASSERT_TRUE(last.allowOutput); // Sleep folds the legs before output goes off
}

void test_short_peak_does_not_trip()
{
  powerUp();
  feed(1.0f, 6.0f, 2000);
  feed(8.5f, 6.0f, 400); // below the trip current, well above sustained
  feed(1.0f, 6.0f, 2000);
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
}

void test_sustained_threshold_uses_exactly_the_window()
{
  // After a long idle, a pulse whose true 2 s average is just below 6 A must
  // not trip, and one just above must.
  powerUp();
  feed(0, 6.0f, 3000);
  feed(8.5f, 6.0f, 1400); // 2 s average 5.95 A
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 5.95f, last.averageCurrentA);

  powerUp();
  feed(0, 6.0f, 3000);
  feed(8.5f, 6.0f, 1440); // 2 s average 6.12 A
  TEST_ASSERT_TRUE(last.faults & SUSTAINED_CURRENT);
}

void test_sustained_window_must_fill_before_tripping()
{
  powerUp(); // 600 ms of low current so far
  feed(7.0f, 6.0f, 1000);
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults); // window not yet representative of 7 A
}

void test_undervoltage_debounced()
{
  powerUp();
  feed(0.5f, 5.0f, 180);
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  feed(0.5f, 5.0f, 40);
  TEST_ASSERT_EQUAL_UINT16(UNDERVOLTAGE, last.faults);
  TEST_ASSERT_TRUE(last.forceSleep);
  TEST_ASSERT_TRUE(last.allowOutput);
}

void test_transient_sag_tolerated()
{
  powerUp();
  for (int i = 0; i < 10; i++)
  {
    feed(0.5f, 5.0f, 100); // shorter than the 200 ms debounce
    feed(0.5f, 6.0f, 100);
  }
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  TEST_ASSERT_TRUE(last.allowOutput);
}

void test_brownout_inhibits_immediately()
{
  powerUp();
  now += 20;
  last = step(s, Reading{0.5f, 4.5f}, now, false, false);
  TEST_ASSERT_TRUE(last.faults & BROWNOUT);
  TEST_ASSERT_FALSE(last.allowOutput);
}

void test_overvoltage_inhibits_immediately_even_before_gate()
{
  reset();
  now += 20;
  last = step(s, Reading{0, 7.2f}, now, false, false);
  TEST_ASSERT_EQUAL_UINT16(OVERVOLTAGE, last.faults);
  TEST_ASSERT_FALSE(last.allowOutput);
}

// --- 2.3 clearing -----------------------------------------------------------

void test_clear_while_condition_persists_keeps_fault()
{
  powerUp();
  feed(0.5f, 5.0f, 300);
  TEST_ASSERT_EQUAL_UINT16(UNDERVOLTAGE, last.faults);
  feed(0.5f, 5.0f, 20, true);
  TEST_ASSERT_EQUAL_UINT16(UNDERVOLTAGE, last.faults);
  TEST_ASSERT_TRUE(last.forceSleep);
}

void test_clear_after_recovery()
{
  powerUp();
  feed(0.5f, 5.0f, 300);
  feed(0.5f, 6.0f, 100);
  TEST_ASSERT_EQUAL_UINT16(UNDERVOLTAGE, last.faults); // latched until cleared
  TEST_ASSERT_TRUE(last.forceSleep);
  feed(0.5f, 6.0f, 20, true);
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  TEST_ASSERT_FALSE(last.forceSleep);
  TEST_ASSERT_TRUE(last.allowOutput); // undervoltage never closed the gate
}

void test_inhibiting_fault_needs_fresh_settle_after_clear()
{
  powerUp();
  now += 20;
  last = step(s, Reading{10.0f, 6.0f}, now, false, false);
  TEST_ASSERT_FALSE(last.allowOutput);
  feed(0.2f, 6.0f, 20, true); // cleared: current back to normal
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  TEST_ASSERT_FALSE(last.allowOutput);
  feed(0.2f, 6.0f, 520);
  TEST_ASSERT_TRUE(last.allowOutput);
}

void test_clear_is_selective()
{
  powerUp();
  feed(0.5f, 5.0f, 300); // undervoltage latches
  now += 20;
  last = step(s, Reading{9.5f, 5.0f}, now, false, false); // and an overcurrent spike
  TEST_ASSERT_EQUAL_UINT16(UNDERVOLTAGE | OVERCURRENT, last.faults);
  feed(0.5f, 5.0f, 20, true); // current recovered, voltage did not
  TEST_ASSERT_EQUAL_UINT16(UNDERVOLTAGE, last.faults);
}

// --- emergency stop ---------------------------------------------------------

void test_emergency_stop_inhibits_on_the_same_step()
{
  powerUp();
  now += 20;
  last = step(s, Reading{0.2f, 6.0f}, now, false, true);
  TEST_ASSERT_FALSE(last.allowOutput);
  TEST_ASSERT_TRUE(last.forceSleep);
  TEST_ASSERT_EQUAL_UINT16(EMERGENCY_STOP, last.faults);
  TEST_ASSERT_EQUAL_UINT16(EMERGENCY_STOP, last.newlyLatched);
}

void test_emergency_stop_latches_before_the_supply_settles()
{
  reset();
  feed(0.2f, 6.0f, 100, false, true);
  TEST_ASSERT_EQUAL_UINT16(EMERGENCY_STOP, last.faults);
  feed(0.2f, 6.0f, 1000);
  TEST_ASSERT_FALSE(last.allowOutput);
}

void test_emergency_stop_release_without_clear_keeps_output_off()
{
  powerUp();
  feed(0.2f, 6.0f, 40, false, true);
  feed(0.2f, 6.0f, 2000); // released, no clear
  TEST_ASSERT_FALSE(last.allowOutput);
  TEST_ASSERT_EQUAL_UINT16(EMERGENCY_STOP, last.faults);
}

void test_clear_while_emergency_stop_held_keeps_fault()
{
  powerUp();
  feed(0.2f, 6.0f, 40, false, true);
  feed(0.2f, 6.0f, 200, true, true); // clear requested while still held
  TEST_ASSERT_EQUAL_UINT16(EMERGENCY_STOP, last.faults);
  TEST_ASSERT_FALSE(last.allowOutput);
}

void test_clear_after_release_restores_output_after_settling()
{
  powerUp();
  feed(0.2f, 6.0f, 40, false, true);
  feed(0.2f, 6.0f, 40);       // released
  feed(0.2f, 6.0f, 20, true); // one clear request
  TEST_ASSERT_EQUAL_UINT16(NONE, last.faults);
  TEST_ASSERT_FALSE(last.allowOutput); // the gate needs a fresh settle
  feed(0.2f, 6.0f, SUPPLY_SETTLE_MS + 20);
  TEST_ASSERT_TRUE(last.allowOutput);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_usb_only_keeps_output_off_without_faults);
  RUN_TEST(test_gate_opens_after_500ms_of_valid_supply);
  RUN_TEST(test_gate_needs_continuous_validity);
  RUN_TEST(test_overcurrent_inhibits_on_the_same_step);
  RUN_TEST(test_sustained_current_puts_body_to_sleep);
  RUN_TEST(test_short_peak_does_not_trip);
  RUN_TEST(test_sustained_threshold_uses_exactly_the_window);
  RUN_TEST(test_sustained_window_must_fill_before_tripping);
  RUN_TEST(test_undervoltage_debounced);
  RUN_TEST(test_transient_sag_tolerated);
  RUN_TEST(test_brownout_inhibits_immediately);
  RUN_TEST(test_overvoltage_inhibits_immediately_even_before_gate);
  RUN_TEST(test_clear_while_condition_persists_keeps_fault);
  RUN_TEST(test_clear_after_recovery);
  RUN_TEST(test_inhibiting_fault_needs_fresh_settle_after_clear);
  RUN_TEST(test_clear_is_selective);
  RUN_TEST(test_emergency_stop_inhibits_on_the_same_step);
  RUN_TEST(test_emergency_stop_latches_before_the_supply_settles);
  RUN_TEST(test_emergency_stop_release_without_clear_keeps_output_off);
  RUN_TEST(test_clear_while_emergency_stop_held_keeps_fault);
  RUN_TEST(test_clear_after_release_restores_output_after_settling);
  return UNITY_END();
}
