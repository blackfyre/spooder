#include <unity.h>
#include <math.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "movement/gait.h"

using namespace gamepad_mapping;

static Mapper m;
static PadState pad;
static uint32_t now;
static Desired last;

void setUp()
{
  m = Mapper();
  pad = PadState();
  pad.connected = true;
  now = 1000;
  last = update(m, pad, now);
}
void tearDown() {}

static Desired tick(uint32_t dtMs = 20)
{
  now += dtMs;
  last = update(m, pad, now);
  return last;
}

static void pressButton(uint16_t b)
{
  pad.buttons |= b;
  tick();
  pad.buttons &= ~b;
  tick();
}

static void pressDpad(uint8_t b)
{
  pad.dpad |= b;
  tick();
  pad.dpad &= ~b;
  tick();
}

static void pressMisc(uint8_t b)
{
  pad.misc |= b;
  tick();
  pad.misc &= ~b;
  tick();
}

// Holds the given misc and button bits for @p ms, then releases them.
static void hold(uint8_t misc, uint16_t buttons, uint32_t ms)
{
  pad.misc |= misc;
  pad.buttons |= buttons;
  tick();
  for (uint32_t t = 20; t <= ms; t += 20)
    tick();
  pad.misc &= ~misc;
  pad.buttons &= ~buttons;
  tick();
}

static int pct(int fraction1000)
{
  return (int)lroundf((float)fraction1000 / 1000.0f * AXIS_MAX);
}

// --- Defaults and connection ---

void test_connected_starts_in_stand_with_defaults()
{
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
  TEST_ASSERT_TRUE(last.controllerConnected);
  TEST_ASSERT_EQUAL(0, last.height);
  TEST_ASSERT_EQUAL(44, last.speed);
  TEST_ASSERT_EQUAL(0, last.gait);
  TEST_ASSERT_TRUE(last.stride == StrideMode::Dynamic);
}

void test_disconnect_requests_sleep_with_zero_input()
{
  pressMisc(MISC_HOME); // walking
  pad.axisX = 300;
  pad.axisY = -400;
  pad.axisRX = 500;
  TEST_ASSERT_TRUE(tick().mode == RequestedMode::Walk);
  pad.connected = false;
  Desired d = tick();
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  TEST_ASSERT_FALSE(d.controllerConnected);
  TEST_ASSERT_EQUAL(0, d.translateX);
  TEST_ASSERT_EQUAL(0, d.translateY);
  TEST_ASSERT_EQUAL(0, d.rotate);
}

void test_reconnect_starts_in_stand_and_keeps_settings()
{
  pressDpad(DPAD_UP);
  pressDpad(DPAD_LEFT);
  pressButton(BUTTON_R1);
  pressButton(BUTTON_X);
  pressMisc(MISC_HOME); // Walk
  pad.connected = false;
  tick();
  pad.connected = true;
  Desired d = tick();
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Stand);
  TEST_ASSERT_EQUAL(10, d.height);
  TEST_ASSERT_EQUAL(34, d.speed);
  TEST_ASSERT_EQUAL(1, d.gait);
  TEST_ASSERT_TRUE(d.stride == StrideMode::Fixed);
}

void test_stick_mode_survives_reconnect()
{
  pressButton(BUTTON_B);
  pad.connected = false;
  tick();
  pad.connected = true;
  TEST_ASSERT_TRUE(tick().stickMode == StickMode::Crab);
}

void test_reconnect_clears_sleep()
{
  pressButton(BUTTON_Y);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Sleep);
  pad.connected = false;
  tick();
  pad.connected = true;
  TEST_ASSERT_TRUE(tick().mode == RequestedMode::Stand);
}

void test_button_held_through_reconnect_is_not_a_press()
{
  pad.connected = false;
  tick();
  pad.connected = true;
  pad.buttons = BUTTON_R1;
  tick();
  tick();
  TEST_ASSERT_EQUAL(0, last.gait);
}

// --- Mapping table ---

void test_home_toggles_stand_and_walk()
{
  uint8_t seq = last.modeRequestCounter;
  pressMisc(MISC_HOME);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Walk);
  pressMisc(MISC_HOME);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
  TEST_ASSERT_EQUAL((uint8_t)(seq + 2), last.modeRequestCounter);
}

void test_holding_home_toggles_once()
{
  pad.misc = MISC_HOME;
  for (int i = 0; i < 100; i++)
    tick();
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Walk);
}

void test_y_toggles_sleep_and_returns_to_previous_mode()
{
  pressMisc(MISC_HOME); // Walk
  uint8_t seq = last.modeRequestCounter;
  pressButton(BUTTON_Y);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Sleep);
  pressButton(BUTTON_Y);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Walk);
  TEST_ASSERT_EQUAL((uint8_t)(seq + 2), last.modeRequestCounter);
}

void test_calibrate_combo_held_one_second_toggles()
{
  uint8_t seq = last.modeRequestCounter;
  hold(MISC_BACK | MISC_HOME, 0, COMBO_HOLD_MS);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Calibrate);
  TEST_ASSERT_EQUAL((uint8_t)(seq + 1), last.modeRequestCounter);
  hold(MISC_BACK | MISC_HOME, 0, COMBO_HOLD_MS);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
}

void test_calibrate_needs_a_deliberate_hold()
{
  uint8_t seq = last.modeRequestCounter;
  hold(MISC_BACK | MISC_HOME, 0, 500);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
  TEST_ASSERT_EQUAL(seq, last.modeRequestCounter);
}

void test_calibrate_combo_fires_once_per_hold()
{
  hold(MISC_BACK | MISC_HOME, 0, 3 * COMBO_HOLD_MS);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Calibrate);
}

void test_leaving_calibrate_returns_to_stand_even_from_walk()
{
  pressMisc(MISC_HOME); // Walk
  hold(MISC_BACK | MISC_HOME, 0, COMBO_HOLD_MS);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Calibrate);
  hold(MISC_BACK | MISC_HOME, 0, COMBO_HOLD_MS);
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
}

void test_combination_has_priority_over_home_toggle()
{
  // miscBack first, then miscHome: the Home press is part of the combination.
  pad.misc = MISC_BACK;
  tick();
  pad.misc = MISC_BACK | MISC_HOME;
  tick();
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
  // Released early: neither the toggle nor Calibrate.
  pad.misc = 0;
  tick();
  TEST_ASSERT_TRUE(last.mode == RequestedMode::Stand);
}

void test_shoulders_change_gait_and_wrap()
{
  pressButton(BUTTON_R1);
  TEST_ASSERT_EQUAL(1, last.gait);
  pressButton(BUTTON_L1);
  pressButton(BUTTON_L1);
  TEST_ASSERT_EQUAL(gait::GAIT_COUNT - 1, last.gait);
  pressButton(BUTTON_R1);
  TEST_ASSERT_EQUAL(0, last.gait);
}

void test_dpad_steps_height_and_speed_within_range()
{
  pressDpad(DPAD_UP);
  TEST_ASSERT_EQUAL(10, last.height);
  pressDpad(DPAD_DOWN);
  pressDpad(DPAD_DOWN);
  TEST_ASSERT_EQUAL(0, last.height);
  for (int i = 0; i < 12; i++)
    pressDpad(DPAD_UP);
  TEST_ASSERT_EQUAL(100, last.height);

  pressDpad(DPAD_RIGHT);
  TEST_ASSERT_EQUAL(54, last.speed);
  for (int i = 0; i < 10; i++)
    pressDpad(DPAD_RIGHT);
  TEST_ASSERT_EQUAL(100, last.speed);
  for (int i = 0; i < 12; i++)
    pressDpad(DPAD_LEFT);
  TEST_ASSERT_EQUAL(0, last.speed);
}

void test_x_toggles_stride()
{
  pressButton(BUTTON_X);
  TEST_ASSERT_TRUE(last.stride == StrideMode::Fixed);
  pressButton(BUTTON_X);
  TEST_ASSERT_TRUE(last.stride == StrideMode::Dynamic);
}

void test_held_buttons_do_not_repeat()
{
  pad.buttons = BUTTON_R1 | BUTTON_X;
  pad.dpad = DPAD_UP;
  for (int i = 0; i < 100; i++)
    tick();
  TEST_ASSERT_EQUAL(1, last.gait);
  TEST_ASSERT_EQUAL(10, last.height);
  TEST_ASSERT_TRUE(last.stride == StrideMode::Fixed);
}

void test_thumbs_held_one_second_request_one_clear()
{
  uint8_t c = last.clearCounter;
  hold(0, BUTTON_THUMB_L | BUTTON_THUMB_R, 500);
  TEST_ASSERT_EQUAL(c, last.clearCounter);
  hold(0, BUTTON_THUMB_L | BUTTON_THUMB_R, 3 * COMBO_HOLD_MS);
  TEST_ASSERT_EQUAL((uint8_t)(c + 1), last.clearCounter);
}

void test_clear_does_not_change_mode_request_counter()
{
  uint8_t seq = last.modeRequestCounter;
  hold(0, BUTTON_THUMB_L | BUTTON_THUMB_R, COMBO_HOLD_MS);
  TEST_ASSERT_EQUAL(seq, last.modeRequestCounter);
}

void test_left_stick_translates_with_y_forward()
{
  pressButton(BUTTON_B); // crab: the left stick translates
  pad.axisY = -511; // Bluepad32: up is negative
  Desired d = tick();
  TEST_ASSERT_EQUAL(100, d.translateY);
  TEST_ASSERT_EQUAL(0, d.translateX);
  pad.axisY = 0;
  pad.axisX = 512;
  d = tick();
  TEST_ASSERT_EQUAL(100, d.translateX);
}

void test_translate_magnitude_is_clamped()
{
  // Every direction at and beyond full deflection stays within 100.
  for (int deg = 0; deg < 360; deg++)
  {
    float a = (float)deg * 3.14159265f / 180.0f;
    for (float r = 1.0f; r <= 1.5f; r += 0.25f)
    {
      int8_t x, y;
      stickToVector((int32_t)lroundf(cosf(a) * r * AXIS_MAX), (int32_t)lroundf(sinf(a) * r * AXIS_MAX), x, y);
      int sq = x * x + y * y;
      TEST_ASSERT_TRUE_MESSAGE(sq <= 100 * 100, "magnitude above 100");
      TEST_ASSERT_TRUE(sq >= 98 * 98);
    }
  }
}

void test_right_stick_x_rotates_and_y_is_unused()
{
  pad.axisRX = -511; // left: anticlockwise, positive
  pad.axisRY = 512;
  Desired d = tick();
  TEST_ASSERT_EQUAL(100, d.rotate);
  TEST_ASSERT_EQUAL(0, d.translateX);
  TEST_ASSERT_EQUAL(0, d.translateY);
}

// --- Stick modes ---

void test_steer_is_the_default_and_b_toggles_crab()
{
  TEST_ASSERT_TRUE(last.stickMode == StickMode::Steer);
  pressButton(BUTTON_B);
  TEST_ASSERT_TRUE(last.stickMode == StickMode::Crab);
  pad.buttons |= BUTTON_B; // held: one toggle per press
  for (int i = 0; i < 10; i++)
    tick();
  TEST_ASSERT_TRUE(last.stickMode == StickMode::Steer);
}

void test_steering_while_walking()
{
  // Left stick forward and to the left: forward, no strafe, and the same
  // turn as the right stick to the left: anticlockwise, a positive rotate.
  pad.axisY = -pct(700);
  pad.axisX = -pct(700);
  Desired d = tick();
  TEST_ASSERT_EQUAL(0, d.translateX);
  TEST_ASSERT_TRUE(d.translateY > 50);
  TEST_ASSERT_TRUE(d.rotate > 50);
  TEST_ASSERT_EQUAL(d.translateY, d.rotate); // a 45 degree stick
}

void test_steering_while_reversing_is_mirrored()
{
  pad.axisY = pct(700); // back
  pad.axisX = -pct(700); // left
  Desired d = tick();
  TEST_ASSERT_EQUAL(0, d.translateX);
  TEST_ASSERT_TRUE(d.translateY < -50);
  TEST_ASSERT_TRUE(d.rotate < -50); // the rear swings right: clockwise
}

void test_steering_sideways_turns_on_the_spot()
{
  // Left stick X alone turns, as right stick X does; a slightly backward
  // stick within the reverse threshold does not flip it.
  pad.axisX = -511;
  TEST_ASSERT_TRUE(tick().rotate == 100);
  pad.axisY = pct(80); // a little back: no reversing, no mirroring
  Desired d = tick();
  TEST_ASSERT_EQUAL(0, d.translateY);
  TEST_ASSERT_TRUE(d.rotate > 90);
  pad.axisY = pct(200); // clearly back: reverse, mirrored
  d = tick();
  TEST_ASSERT_TRUE(d.translateY < -REVERSE_THRESHOLD);
  TEST_ASSERT_TRUE(d.rotate < 0);
}

void test_right_stick_adds_rotation_in_both_modes_and_clamps()
{
  pad.axisY = -pct(700);
  pad.axisX = pct(700); // steer right: clockwise, negative
  pad.axisRX = 512;
  TEST_ASSERT_EQUAL(-100, tick().rotate);
  pad.axisRX = -pct(300); // right stick left takes some of it back
  Desired d = tick();
  TEST_ASSERT_TRUE(d.rotate < 0 && d.rotate > -d.translateY);

  pad = PadState();
  pad.connected = true;
  tick();
  pressButton(BUTTON_B); // crab
  pad.axisX = pct(700);
  pad.axisRX = pct(500);
  d = tick();
  TEST_ASSERT_TRUE(d.translateX > 50);
  TEST_ASSERT_TRUE(d.rotate < -40 && d.rotate > -60);
}

// --- Dead zone ---

void test_stick_inside_dead_zone_is_zero()
{
  pressButton(BUTTON_B); // crab, so left stick X would translate
  pad.axisX = pct(50); // 5 %
  Desired d = tick();
  TEST_ASSERT_EQUAL(0, d.translateX);
  pad.axisX = pct(35);
  pad.axisY = -pct(35); // radius about 4.9 %
  d = tick();
  TEST_ASSERT_EQUAL(0, d.translateX);
  TEST_ASSERT_EQUAL(0, d.translateY);
  pad.axisRX = pct(79);
  TEST_ASSERT_EQUAL(0, tick().rotate);
}

void test_output_continuous_at_dead_zone_edge()
{
  int8_t x, y;
  // From 7 % to 10 % in fine steps: output starts at zero and never jumps by
  // more than one unit between adjacent steps.
  int prev = 0;
  for (int f = 70; f <= 100; f++)
  {
    stickToVector((int32_t)lroundf(f / 1000.0f * AXIS_MAX), 0, x, y);
    TEST_ASSERT_TRUE(x >= prev);
    TEST_ASSERT_TRUE(x - prev <= 1);
    prev = x;
  }
  stickToVector((int32_t)lroundf(0.081f * AXIS_MAX), 0, x, y);
  TEST_ASSERT_TRUE(x <= 1);
  stickToVector((int32_t)lroundf(0.10f * AXIS_MAX), 0, x, y);
  TEST_ASSERT_TRUE(x >= 1 && x <= 3);
}

// --- Command conversion ---

void test_to_command_sets_flags_and_fields()
{
  pressMisc(MISC_HOME);
  protocol::Command c = toCommand(last, true);
  TEST_ASSERT_EQUAL((uint8_t)RequestedMode::Walk, c.mode);
  TEST_ASSERT_EQUAL(protocol::FLAG_CONTROLLER_CONNECTED | protocol::FLAG_EMERGENCY_STOP, c.flags);
  TEST_ASSERT_EQUAL(last.modeRequestCounter, c.modeRequestCounter);
  pad.connected = false;
  tick();
  c = toCommand(last, false);
  TEST_ASSERT_EQUAL(0, c.flags);
  TEST_ASSERT_EQUAL((uint8_t)RequestedMode::Sleep, c.mode);
}

// --- Gesture layer ---

void test_l2_layer_requests_gestures_once_per_press()
{
  pad.buttons |= BUTTON_L2;
  tick();
  const struct
  {
    uint16_t button;
    uint8_t gesture;
  } cases[] = {{BUTTON_A, protocol::GESTURE_SAY_HI}, {BUTTON_B, protocol::GESTURE_WIGGLE},
               {BUTTON_X, protocol::GESTURE_DANCE},  {BUTTON_Y, protocol::GESTURE_ATTACK},
               {BUTTON_R2, protocol::GESTURE_BALL}};
  uint8_t counter = last.gestureRequestCounter;
  for (const auto &c : cases)
  {
    pad.buttons |= c.button;
    for (int i = 0; i < 5; i++) // held: still one request
      tick();
    pad.buttons &= ~c.button;
    tick();
    counter++;
    TEST_ASSERT_EQUAL(counter, last.gestureRequestCounter);
    TEST_ASSERT_EQUAL(c.gesture, last.gesture);
    protocol::Command cmd = toCommand(last, false);
    TEST_ASSERT_EQUAL(c.gesture, cmd.gesture);
    TEST_ASSERT_EQUAL(counter, cmd.gestureRequestCounter);
  }
}

void test_l2_layer_suppresses_the_buttons_own_functions()
{
  pad.buttons |= BUTTON_L2;
  tick();
  Desired before = last;
  pressButton(BUTTON_Y); // attack, not Sleep
  pressButton(BUTTON_X); // dance, not the stride
  pressButton(BUTTON_B); // wiggle, not the stick mode
  TEST_ASSERT_TRUE(last.mode == before.mode);
  TEST_ASSERT_TRUE(last.stride == before.stride);
  TEST_ASSERT_TRUE(last.stickMode == before.stickMode);
  TEST_ASSERT_EQUAL(before.modeRequestCounter, last.modeRequestCounter);
  // Released, the buttons are themselves again.
  pad.buttons &= ~BUTTON_L2;
  tick();
  pressButton(BUTTON_B);
  TEST_ASSERT_TRUE(last.stickMode != before.stickMode);
  TEST_ASSERT_EQUAL(before.gestureRequestCounter + 3, last.gestureRequestCounter);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_connected_starts_in_stand_with_defaults);
  RUN_TEST(test_disconnect_requests_sleep_with_zero_input);
  RUN_TEST(test_reconnect_starts_in_stand_and_keeps_settings);
  RUN_TEST(test_reconnect_clears_sleep);
  RUN_TEST(test_button_held_through_reconnect_is_not_a_press);
  RUN_TEST(test_home_toggles_stand_and_walk);
  RUN_TEST(test_holding_home_toggles_once);
  RUN_TEST(test_y_toggles_sleep_and_returns_to_previous_mode);
  RUN_TEST(test_calibrate_combo_held_one_second_toggles);
  RUN_TEST(test_calibrate_needs_a_deliberate_hold);
  RUN_TEST(test_calibrate_combo_fires_once_per_hold);
  RUN_TEST(test_leaving_calibrate_returns_to_stand_even_from_walk);
  RUN_TEST(test_combination_has_priority_over_home_toggle);
  RUN_TEST(test_shoulders_change_gait_and_wrap);
  RUN_TEST(test_dpad_steps_height_and_speed_within_range);
  RUN_TEST(test_x_toggles_stride);
  RUN_TEST(test_held_buttons_do_not_repeat);
  RUN_TEST(test_thumbs_held_one_second_request_one_clear);
  RUN_TEST(test_clear_does_not_change_mode_request_counter);
  RUN_TEST(test_left_stick_translates_with_y_forward);
  RUN_TEST(test_translate_magnitude_is_clamped);
  RUN_TEST(test_right_stick_x_rotates_and_y_is_unused);
  RUN_TEST(test_steer_is_the_default_and_b_toggles_crab);
  RUN_TEST(test_steering_while_walking);
  RUN_TEST(test_steering_while_reversing_is_mirrored);
  RUN_TEST(test_steering_sideways_turns_on_the_spot);
  RUN_TEST(test_right_stick_adds_rotation_in_both_modes_and_clamps);
  RUN_TEST(test_stick_mode_survives_reconnect);
  RUN_TEST(test_stick_inside_dead_zone_is_zero);
  RUN_TEST(test_output_continuous_at_dead_zone_edge);
  RUN_TEST(test_to_command_sets_flags_and_fields);
  RUN_TEST(test_l2_layer_requests_gestures_once_per_press);
  RUN_TEST(test_l2_layer_suppresses_the_buttons_own_functions);
  return UNITY_END();
}
