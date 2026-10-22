#include <unity.h>
#include "status/peak_hold.h"

using namespace peak_hold;

static Hold h;

void setUp() { h = Hold(); }
void tearDown() {}

// Feeds @p reading every 20 ms from @p fromMs to before @p toMs.
static void feed(Extreme e, float rate, uint32_t fromMs, uint32_t toMs, float reading)
{
  float margin = e == Extreme::Max ? CURRENT_WARN_MARGIN_MA : VOLTAGE_WARN_MARGIN_MV;
  for (uint32_t t = fromMs; t < toMs; t += 20)
    update(h, e, rate, margin, t, true, reading);
}

// The spec's stand-surge scenario.
void test_current_peak_holds_then_falls()
{
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 0, true, 4300);
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 20, 3000, 400);
  TEST_ASSERT_EQUAL_FLOAT(4300, h.value); // held for 3 s
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 3000, 4020, 400);
  TEST_ASSERT_FLOAT_WITHIN(1, 2300, h.value); // one second of 2 A/s
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 4020, 6000, 400);
  TEST_ASSERT_EQUAL_FLOAT(400, h.value); // met the reading
}

// The spec's supply-sag scenario.
void test_voltage_low_holds_then_rises()
{
  feed(Extreme::Min, VOLTAGE_RISE_MV_PER_S, 0, 100, 6000);
  update(h, Extreme::Min, VOLTAGE_RISE_MV_PER_S, VOLTAGE_WARN_MARGIN_MV, 100, true, 5500);
  feed(Extreme::Min, VOLTAGE_RISE_MV_PER_S, 120, 3100, 6000);
  TEST_ASSERT_EQUAL_FLOAT(5500, h.value);
  feed(Extreme::Min, VOLTAGE_RISE_MV_PER_S, 3100, 4120, 6000);
  TEST_ASSERT_EQUAL_FLOAT(6000, h.value); // 0.5 V in a second
}

void test_new_extreme_moves_at_once_and_restarts_the_hold()
{
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 0, true, 3000);
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 20, 3500, 500); // falling by now
  TEST_ASSERT_TRUE(h.value < 3000);
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 3500, true, 5000);
  TEST_ASSERT_EQUAL_FLOAT(5000, h.value);
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 3520, 6500, 500);
  TEST_ASSERT_EQUAL_FLOAT(5000, h.value); // the new hold
}

void test_lost_link_clears_and_restarts()
{
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 0, true, 4000);
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 100, false, 0);
  TEST_ASSERT_FALSE(h.present);
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 2000, true, 300);
  TEST_ASSERT_TRUE(h.present);
  TEST_ASSERT_EQUAL_FLOAT(300, h.value);
}

void test_sparse_updates_fall_by_elapsed_time()
{
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 0, true, 4300);
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 4000, true, 400); // one call, 1 s past the hold
  TEST_ASSERT_FLOAT_WITHIN(1, 2300, h.value);
}

// The spec's warning scenarios.
void test_new_high_current_warns_for_a_second()
{
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 0, 1000, 400);
  TEST_ASSERT_FALSE(h.warning); // steady: no new extreme
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, 1000, true, 4300);
  TEST_ASSERT_TRUE(h.warning);
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 1020, 1980, 400);
  TEST_ASSERT_TRUE(h.warning);
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 1980, 2100, 400);
  TEST_ASSERT_FALSE(h.warning); // a second after the last new high
}

void test_noise_within_the_margin_does_not_warn()
{
  // The held value has eased back to the reading; wobbles of 50 mA follow.
  for (uint32_t t = 0; t < 10000; t += 20)
  {
    update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, t, true, 400 + (t / 20 % 2) * 50);
    TEST_ASSERT_FALSE(h.warning);
  }
}

void test_rising_current_keeps_warning()
{
  feed(Extreme::Max, CURRENT_FALL_MA_PER_S, 0, 500, 400);
  uint32_t t = 500;
  for (float ma = 600; ma <= 3000; ma += 30, t += 100) // past the margin, then 30 mA a frame
  {
    update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, t, true, ma);
    TEST_ASSERT_TRUE(h.warning);
  }
}

void test_deeper_sag_warns_and_the_link_loss_clears_it()
{
  feed(Extreme::Min, VOLTAGE_RISE_MV_PER_S, 0, 500, 6000);
  update(h, Extreme::Min, VOLTAGE_RISE_MV_PER_S, VOLTAGE_WARN_MARGIN_MV, 500, true, 5960); // 40 mV: noise
  TEST_ASSERT_FALSE(h.warning);
  update(h, Extreme::Min, VOLTAGE_RISE_MV_PER_S, VOLTAGE_WARN_MARGIN_MV, 520, true, 5600);
  TEST_ASSERT_TRUE(h.warning);
  update(h, Extreme::Min, VOLTAGE_RISE_MV_PER_S, VOLTAGE_WARN_MARGIN_MV, 540, false, 0);
  TEST_ASSERT_FALSE(h.warning);
}

void test_millisecond_counter_wrap()
{
  uint32_t t0 = 0xFFFFFFFFu - 1000;
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, t0, true, 4300);
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, t0 + 2000, true, 400);
  TEST_ASSERT_EQUAL_FLOAT(4300, h.value);
  update(h, Extreme::Max, CURRENT_FALL_MA_PER_S, CURRENT_WARN_MARGIN_MA, t0 + 4000, true, 400);
  TEST_ASSERT_FLOAT_WITHIN(1, 2300, h.value);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_current_peak_holds_then_falls);
  RUN_TEST(test_voltage_low_holds_then_rises);
  RUN_TEST(test_new_extreme_moves_at_once_and_restarts_the_hold);
  RUN_TEST(test_lost_link_clears_and_restarts);
  RUN_TEST(test_sparse_updates_fall_by_elapsed_time);
  RUN_TEST(test_new_high_current_warns_for_a_second);
  RUN_TEST(test_noise_within_the_margin_does_not_warn);
  RUN_TEST(test_rising_current_keeps_warning);
  RUN_TEST(test_deeper_sag_warns_and_the_link_loss_clears_it);
  RUN_TEST(test_millisecond_counter_wrap);
  return UNITY_END();
}
