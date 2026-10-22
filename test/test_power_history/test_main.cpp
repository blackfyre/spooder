#include <unity.h>
#include "status/power_history.h"

using namespace power_history;

static History h;

void setUp() { h = History(); }
void tearDown() {}

// Feeds one reading per 20 ms tick for @p ms, starting at @p fromMs.
static uint32_t feed(uint32_t fromMs, uint32_t ms, bool fresh, uint16_t mv, uint16_t ma)
{
  for (uint32_t done = 0; done < ms; done += 20)
    record(h, fromMs + done, fresh, mv, ma);
  return fromMs + ms;
}

void test_one_sample_per_100_ms()
{
  feed(0, 1000, true, 6000, 500);
  TEST_ASSERT_EQUAL_INT(10, h.count);
  Sample s;
  TEST_ASSERT_TRUE(newest(h, s));
  TEST_ASSERT_EQUAL_UINT16(6000, s.voltageMv);
  TEST_ASSERT_EQUAL_UINT16(500, s.currentMa);
}

void test_empty_history_has_nothing()
{
  Sample s;
  uint16_t v, vMax, a;
  TEST_ASSERT_FALSE(newest(h, s));
  TEST_ASSERT_FALSE(extremes(h, v, vMax, a));
}

// The spec's supply-sag scenario: a sag twenty seconds ago is the window's
// lowest voltage and highest current.
void test_extremes_of_the_window()
{
  uint32_t t = feed(0, 10000, true, 6050, 400);
  t = feed(t, 200, true, 5650, 4300);
  feed(t, 20000, true, 6050, 400);
  uint16_t v = 0, vMax = 0, a = 0;
  TEST_ASSERT_TRUE(extremes(h, v, vMax, a));
  TEST_ASSERT_EQUAL_UINT16(5650, v);
  TEST_ASSERT_EQUAL_UINT16(6050, vMax);
  TEST_ASSERT_EQUAL_UINT16(4300, a);
}

void test_old_samples_leave_the_window()
{
  uint32_t t = feed(0, 200, true, 5000, 9000);
  feed(t, WINDOW_MS + 1000, true, 6000, 300);
  uint16_t v = 0, vMax = 0, a = 0;
  TEST_ASSERT_TRUE(extremes(h, v, vMax, a));
  TEST_ASSERT_EQUAL_UINT16(6000, v);
  TEST_ASSERT_EQUAL_UINT16(300, a);
  TEST_ASSERT_EQUAL_INT(SAMPLE_COUNT, h.count);
}

// The spec's link-lost scenario: ten seconds without a link show as a gap.
void test_lost_link_is_a_gap()
{
  uint32_t t = feed(0, 20000, true, 6000, 500);
  t = feed(t, 10000, false, 0, 0);
  feed(t, 20000, true, 6000, 500);
  Column cols[60]; // one column per second
  columns(h, 60, cols);
  // 50 s recorded: columns 0-9 unrecorded, 10-29 present, 30-39 lost,
  // 40-59 present.
  int gaps = 0, present = 0;
  for (const Column &c : cols)
    (c.present ? present : gaps)++;
  TEST_ASSERT_EQUAL_INT(10 + 10, gaps); // 10 s unrecorded + 10 s lost
  TEST_ASSERT_EQUAL_INT(40, present);
  TEST_ASSERT_FALSE(cols[0].present);   // before the first sample
  TEST_ASSERT_TRUE(cols[59].present);   // the newest
  TEST_ASSERT_FALSE(cols[35].present);  // inside the lost 10 s
  TEST_ASSERT_TRUE(cols[25].present);
}

void test_newest_sample_while_lost_is_a_gap()
{
  uint32_t t = feed(0, 1000, true, 6000, 500);
  feed(t, 200, false, 0, 0);
  Sample s;
  TEST_ASSERT_FALSE(newest(h, s));
  uint16_t v, vMax, a;
  TEST_ASSERT_TRUE(extremes(h, v, vMax, a)); // the earlier samples still count
}

void test_columns_hold_the_range_they_cover()
{
  feed(0, WINDOW_MS, true, 6000, 500);
  record(h, WINDOW_MS, true, 5200, 7000); // the newest sample
  Column cols[120];
  columns(h, 120, cols);
  TEST_ASSERT_TRUE(cols[119].present);
  TEST_ASSERT_EQUAL_UINT16(5200, cols[119].minVoltageMv);
  TEST_ASSERT_EQUAL_UINT16(6000, cols[119].maxVoltageMv);
  TEST_ASSERT_EQUAL_UINT16(500, cols[119].minCurrentMa);
  TEST_ASSERT_EQUAL_UINT16(7000, cols[119].maxCurrentMa);
  TEST_ASSERT_EQUAL_UINT16(500, cols[118].maxCurrentMa); // only the last column has it
}

void test_late_call_fills_the_missed_samples()
{
  record(h, 0, true, 6000, 500);
  record(h, 450, true, 6100, 600); // 100, 200, 300, 400 were due
  TEST_ASSERT_EQUAL_INT(5, h.count);
  Sample s;
  TEST_ASSERT_TRUE(newest(h, s));
  TEST_ASSERT_EQUAL_UINT16(6100, s.voltageMv);
}

// A long stall does not invent readings: the missed time is a gap, the
// newest samples take the reading.
void test_long_pause_is_a_gap_not_a_reading()
{
  record(h, 0, true, 6000, 500);
  record(h, 2000, true, 5000, 9000); // 20 samples due: 100..2000 ms
  TEST_ASSERT_EQUAL_INT(21, h.count);
  static Column cols[SAMPLE_COUNT]; // one column per sample
  columns(h, SAMPLE_COUNT, cols);
  int present = 0;
  for (const Column &c : cols)
    present += c.present;
  TEST_ASSERT_EQUAL_INT(1 + BACKFILL_SAMPLES, present); // the first, and 1600..2000 ms
  TEST_ASSERT_FALSE(cols[SAMPLE_COUNT - 1 - BACKFILL_SAMPLES].present);
  TEST_ASSERT_TRUE(cols[SAMPLE_COUNT - 1].present);
}

void test_pause_longer_than_the_window_fills_it_once()
{
  record(h, 0, true, 6000, 500);
  record(h, 10 * WINDOW_MS, false, 0, 0);
  TEST_ASSERT_EQUAL_INT(SAMPLE_COUNT, h.count);
  uint16_t v, vMax, a;
  TEST_ASSERT_FALSE(extremes(h, v, vMax, a));
  record(h, 10 * WINDOW_MS + SAMPLE_MS, true, 6000, 500);
  Sample s;
  TEST_ASSERT_TRUE(newest(h, s));
}

void test_millisecond_counter_wrap()
{
  uint32_t start = 0xFFFFFFFFu - 250;
  feed(start, 1000, true, 6000, 500);
  TEST_ASSERT_EQUAL_INT(10, h.count);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_one_sample_per_100_ms);
  RUN_TEST(test_empty_history_has_nothing);
  RUN_TEST(test_extremes_of_the_window);
  RUN_TEST(test_old_samples_leave_the_window);
  RUN_TEST(test_lost_link_is_a_gap);
  RUN_TEST(test_newest_sample_while_lost_is_a_gap);
  RUN_TEST(test_columns_hold_the_range_they_cover);
  RUN_TEST(test_late_call_fills_the_missed_samples);
  RUN_TEST(test_long_pause_is_a_gap_not_a_reading);
  RUN_TEST(test_pause_longer_than_the_window_fills_it_once);
  RUN_TEST(test_millisecond_counter_wrap);
  return UNITY_END();
}
