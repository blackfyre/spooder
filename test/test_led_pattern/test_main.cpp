#include <unity.h>
#include "status/led_pattern.h"
#include "protection/protection.h"

using namespace led_pattern;

static Rgb out[LED_COUNT];

static State healthy(BodyMode mode)
{
  State s = {};
  s.nowMs = 0;
  s.faults = 0;
  s.supplyAccepted = true;
  s.bodyRunning = true;
  s.mode = mode;
  s.gait = 0;
  s.currentA = 1.0f;
  s.tripCurrentA = 9.0f;
  s.sustainedCurrentA = 6.0f;
  s.input = Input::Serial;
  return s;
}

static bool lit(const Rgb &c) { return c.r || c.g || c.b; }
static int litCount(int from, int to)
{
  int n = 0;
  for (int i = from; i < to; i++)
    if (lit(out[i]))
      n++;
  return n;
}
static bool isRedOnly(const Rgb &c) { return c.r > 0 && c.g == 0 && c.b == 0; }

void setUp() {}
void tearDown() {}

void test_fault_overrides_everything_and_is_brighter()
{
  State s = healthy(BodyMode::Walk);
  s.faults = protection::OVERCURRENT;
  compute(s, out);
  TEST_ASSERT_EQUAL_INT(LED_COUNT, litCount(0, LED_COUNT));
  // Overcurrent is white in its first phase, at fault brightness.
  TEST_ASSERT_TRUE(out[0].r == out[0].g && out[0].g == out[0].b);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)(255 * FAULT_BRIGHTNESS), out[0].r);
  s.nowMs = 500; // alternate phase: red
  compute(s, out);
  TEST_ASSERT_TRUE(isRedOnly(out[3]));
}

void test_each_fault_has_a_distinct_colour()
{
  const uint16_t faults[] = {protection::OVERCURRENT, protection::BROWNOUT, protection::OVERVOLTAGE,
                             protection::SUSTAINED_CURRENT, protection::UNDERVOLTAGE, protection::EMERGENCY_STOP};
  Rgb seen[6];
  for (int i = 0; i < 6; i++)
  {
    State s = healthy(BodyMode::Stand);
    s.faults = faults[i];
    compute(s, out);
    seen[i] = out[0];
    for (int j = 0; j < i; j++)
      TEST_ASSERT_FALSE(seen[i] == seen[j]);
  }
}

void test_emergency_stop_is_steady_red_and_wins()
{
  State s = healthy(BodyMode::Walk);
  s.faults = protection::EMERGENCY_STOP | protection::OVERCURRENT;
  for (uint32_t t = 0; t < 2000; t += 100)
  {
    s.nowMs = t;
    compute(s, out);
    for (int i = 0; i < LED_COUNT; i++)
      TEST_ASSERT_TRUE(isRedOnly(out[i]));
  }
}

void test_waiting_for_supply_pulses_amber_on_all_leds()
{
  State s = healthy(BodyMode::Initialise);
  s.supplyAccepted = false;
  s.bodyRunning = false;
  compute(s, out);
  TEST_ASSERT_EQUAL_INT(LED_COUNT, litCount(0, LED_COUNT));
  TEST_ASSERT_TRUE(out[0].r > out[0].g && out[0].b == 0);
  Rgb dim = out[0];
  s.nowMs = 750; // peak of the pulse
  compute(s, out);
  TEST_ASSERT_TRUE(out[0].r > dim.r);
}

void test_stopped_body_breathes_blue_with_input_indicator()
{
  State s = healthy(BodyMode::Stand);
  s.bodyRunning = false;
  s.nowMs = 1500; // peak
  compute(s, out);
  for (int i = 0; i < MAIN_LEDS; i++)
    TEST_ASSERT_TRUE(out[i].b > out[i].r && out[i].b > out[i].g);
  TEST_ASSERT_TRUE(lit(out[INPUT_LED]));
}

void test_non_walking_modes_show_mode_count_in_green()
{
  const BodyMode modes[] = {BodyMode::Initialise, BodyMode::Stand, BodyMode::Sleep, BodyMode::Calibrate};
  const int expected[] = {1, 2, 4, 5};
  for (int i = 0; i < 4; i++)
  {
    compute(healthy(modes[i]), out);
    TEST_ASSERT_EQUAL_INT(expected[i], litCount(0, MAIN_LEDS));
    TEST_ASSERT_TRUE(out[0].g > 0 && out[0].r == 0 && out[0].b == 0);
  }
}

void test_walking_load_bar_tracks_current()
{
  TEST_ASSERT_EQUAL_INT(1, loadSegments(0, 9));
  TEST_ASSERT_EQUAL_INT(1, loadSegments(1.0f, 9));
  TEST_ASSERT_EQUAL_INT(3, loadSegments(5.0f, 9));
  TEST_ASSERT_EQUAL_INT(5, loadSegments(9.0f, 9));
  TEST_ASSERT_EQUAL_INT(5, loadSegments(20.0f, 9));

  State s = healthy(BodyMode::Walk);
  s.currentA = 5.0f;
  compute(s, out);
  TEST_ASSERT_EQUAL_INT(3, litCount(0, MAIN_LEDS));
}

void test_perform_shows_the_load_bar_like_walking()
{
  State s = healthy(BodyMode::Perform);
  s.currentA = 5.0f;
  compute(s, out);
  TEST_ASSERT_EQUAL_INT(3, litCount(0, MAIN_LEDS));
}

void test_walking_bar_turns_red_past_sustained_limit()
{
  State s = healthy(BodyMode::Walk);
  s.currentA = 9.0f; // full bar
  compute(s, out);
  // Segments of 1.8 A: midpoints 0.9, 2.7, 4.5 below 6 A; 6.3 and 8.1 above.
  TEST_ASSERT_FALSE(isRedOnly(out[2]));
  TEST_ASSERT_TRUE(isRedOnly(out[3]));
  TEST_ASSERT_TRUE(isRedOnly(out[4]));
}

void test_walking_bar_uses_the_gait_colour()
{
  State s = healthy(BodyMode::Walk);
  s.gait = 2; // Ripple
  compute(s, out);
  Rgb ripple = out[0];
  s.gait = 0; // Tri
  compute(s, out);
  TEST_ASSERT_FALSE(ripple == out[0]);
  for (int a = 0; a < 6; a++)
    for (int b = 0; b < a; b++)
      TEST_ASSERT_FALSE(gaitColour(a) == gaitColour(b));
}

void test_input_indicator_states()
{
  State s = healthy(BodyMode::Stand);
  s.input = Input::Serial;
  compute(s, out);
  Rgb serial = out[INPUT_LED];
  TEST_ASSERT_TRUE(serial.r == serial.g && serial.g == serial.b && serial.r > 0);

  s.input = Input::LinkConnected;
  compute(s, out);
  TEST_ASSERT_TRUE(out[INPUT_LED].g > 0 && out[INPUT_LED].b > 0 && out[INPUT_LED].r == 0);

  s.input = Input::LinkNoGamepad;
  s.nowMs = 0;
  compute(s, out);
  TEST_ASSERT_TRUE(out[INPUT_LED].r > 0 && out[INPUT_LED].b > 0 && out[INPUT_LED].g == 0);
  s.nowMs = 600;
  compute(s, out);
  TEST_ASSERT_FALSE(lit(out[INPUT_LED])); // blinking

  s.input = Input::LinkStale;
  s.nowMs = 0;
  compute(s, out);
  TEST_ASSERT_TRUE(lit(out[INPUT_LED]));
  s.nowMs = 130;
  compute(s, out);
  TEST_ASSERT_FALSE(lit(out[INPUT_LED])); // fast blink

  s.input = Input::UsbConnected;
  compute(s, out);
  TEST_ASSERT_TRUE(out[INPUT_LED].b > 0 && out[INPUT_LED].r == 0);
  Rgb usb = out[INPUT_LED];
  s.input = Input::LinkConnected;
  compute(s, out);
  TEST_ASSERT_FALSE(usb == out[INPUT_LED]);

  // A conflict alternates between two lit colours, never off.
  s.input = Input::SourceConflict;
  s.nowMs = 0;
  compute(s, out);
  Rgb first = out[INPUT_LED];
  s.nowMs = 130;
  compute(s, out);
  TEST_ASSERT_TRUE(lit(first) && lit(out[INPUT_LED]));
  TEST_ASSERT_FALSE(first == out[INPUT_LED]);
}

void test_normal_brightness_is_dim()
{
  compute(healthy(BodyMode::Calibrate), out);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)(255 * NORMAL_BRIGHTNESS), out[0].g);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_fault_overrides_everything_and_is_brighter);
  RUN_TEST(test_each_fault_has_a_distinct_colour);
  RUN_TEST(test_emergency_stop_is_steady_red_and_wins);
  RUN_TEST(test_waiting_for_supply_pulses_amber_on_all_leds);
  RUN_TEST(test_stopped_body_breathes_blue_with_input_indicator);
  RUN_TEST(test_non_walking_modes_show_mode_count_in_green);
  RUN_TEST(test_walking_load_bar_tracks_current);
  RUN_TEST(test_walking_bar_turns_red_past_sustained_limit);
  RUN_TEST(test_walking_bar_uses_the_gait_colour);
  RUN_TEST(test_input_indicator_states);
  RUN_TEST(test_normal_brightness_is_dim);
  RUN_TEST(test_perform_shows_the_load_bar_like_walking);
  return UNITY_END();
}
