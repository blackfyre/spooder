#include <unity.h>
#include "setup/portal_controller.h"

using namespace portal_controller;

void setUp() {}
void tearDown() {}

void test_no_poll_means_no_controller()
{
  Controller c;
  gamepad_mapping::Desired d = desired(c, 5000);
  TEST_ASSERT_FALSE(d.controllerConnected);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
}

void test_connected_within_one_second_of_a_poll()
{
  Controller c;
  poll(c, 10000);
  TEST_ASSERT_TRUE(desired(c, 10000).controllerConnected);
  TEST_ASSERT_TRUE(desired(c, 10000 + CONNECTED_WINDOW_MS).controllerConnected);
  TEST_ASSERT_FALSE(desired(c, 10000 + CONNECTED_WINDOW_MS + 1).controllerConnected);
  poll(c, 12000);
  TEST_ASSERT_TRUE(desired(c, 12500).controllerConnected);
}

void test_connected_across_the_millisecond_wrap()
{
  Controller c;
  poll(c, 0xFFFFFF00u);
  TEST_ASSERT_TRUE(desired(c, 0x00000100u).controllerConnected); // 512 ms later
  TEST_ASSERT_FALSE(desired(c, 0x00000300u).controllerConnected);
}

void test_mode_buttons_advance_the_counter()
{
  Controller c;
  poll(c, 0);
  uint8_t before = desired(c, 0).modeRequestCounter;
  TEST_ASSERT_TRUE(requestMode(c, RequestedMode::Calibrate, 10));
  gamepad_mapping::Desired d = desired(c, 10);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Calibrate);
  TEST_ASSERT_EQUAL_UINT8(before + 1, d.modeRequestCounter);
  TEST_ASSERT_TRUE(requestMode(c, RequestedMode::Stand, 20));
  d = desired(c, 20);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Stand);
  TEST_ASSERT_EQUAL_UINT8(before + 2, d.modeRequestCounter);
  TEST_ASSERT_TRUE(requestMode(c, RequestedMode::Sleep, 30));
  d = desired(c, 30);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_UINT8(before + 3, d.modeRequestCounter);
  // Pressing the same button again is another request.
  TEST_ASSERT_TRUE(requestMode(c, RequestedMode::Sleep, 40));
  TEST_ASSERT_EQUAL_UINT8(before + 4, desired(c, 40).modeRequestCounter);
}

void test_walk_is_not_a_portal_mode()
{
  Controller c;
  poll(c, 0);
  TEST_ASSERT_FALSE(requestMode(c, RequestedMode::Walk, 10));
  gamepad_mapping::Desired d = desired(c, 10);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_UINT8(0, d.modeRequestCounter);
}

void test_a_press_counts_as_a_poll()
{
  Controller c;
  requestMode(c, RequestedMode::Stand, 100);
  TEST_ASSERT_TRUE(desired(c, 100).controllerConnected);
  requestClear(c, 1050);
  TEST_ASSERT_TRUE(desired(c, 2000).controllerConnected);
}

void test_translate_and_rotate_are_always_zero()
{
  Controller c;
  const RequestedMode modes[] = {RequestedMode::Stand, RequestedMode::Calibrate, RequestedMode::Sleep};
  for (int i = 0; i < 3; i++)
  {
    requestMode(c, modes[i], 100u * i);
    gamepad_mapping::Desired d = desired(c, 100u * i);
    TEST_ASSERT_EQUAL_INT8(0, d.translateX);
    TEST_ASSERT_EQUAL_INT8(0, d.translateY);
    TEST_ASSERT_EQUAL_INT8(0, d.rotate);
    protocol::Command cmd = gamepad_mapping::toCommand(d, false);
    TEST_ASSERT_EQUAL_INT8(0, cmd.translateX);
    TEST_ASSERT_EQUAL_INT8(0, cmd.translateY);
    TEST_ASSERT_EQUAL_INT8(0, cmd.rotate);
    TEST_ASSERT_EQUAL_UINT8(protocol::FLAG_CONTROLLER_CONNECTED, cmd.flags);
  }
}

void test_disconnect_requests_sleep_until_the_next_press()
{
  Controller c;
  requestMode(c, RequestedMode::Calibrate, 0);
  TEST_ASSERT_TRUE(desired(c, 500).mode == RequestedMode::Calibrate);
  gamepad_mapping::Desired d = desired(c, 1500);
  TEST_ASSERT_FALSE(d.controllerConnected);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  protocol::Command cmd = gamepad_mapping::toCommand(d, false);
  TEST_ASSERT_EQUAL_UINT8(0, cmd.flags & protocol::FLAG_CONTROLLER_CONNECTED);
  // The page comes back: still Sleep, and no new request.
  uint8_t counter = d.modeRequestCounter;
  poll(c, 2000);
  d = desired(c, 2000);
  TEST_ASSERT_TRUE(d.controllerConnected);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_UINT8(counter, d.modeRequestCounter);
  requestMode(c, RequestedMode::Calibrate, 2100);
  TEST_ASSERT_TRUE(desired(c, 2100).mode == RequestedMode::Calibrate);
}

void test_clear_advances_only_the_clear_counter()
{
  Controller c;
  poll(c, 0);
  gamepad_mapping::Desired before = desired(c, 0);
  requestClear(c, 10);
  gamepad_mapping::Desired d = desired(c, 10);
  TEST_ASSERT_EQUAL_UINT8(before.clearCounter + 1, d.clearCounter);
  TEST_ASSERT_EQUAL_UINT8(before.modeRequestCounter, d.modeRequestCounter);
  TEST_ASSERT_TRUE(d.mode == before.mode);
}

void test_dials_are_the_gamepad_defaults()
{
  Controller c;
  poll(c, 0);
  gamepad_mapping::Desired d = desired(c, 0);
  gamepad_mapping::Mapper m;
  TEST_ASSERT_EQUAL_UINT8(m.speed, d.speed);
  TEST_ASSERT_EQUAL_UINT8(m.height, d.height);
  TEST_ASSERT_EQUAL_UINT8(m.gait, d.gait);
  TEST_ASSERT_TRUE(d.stride == m.stride);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_no_poll_means_no_controller);
  RUN_TEST(test_connected_within_one_second_of_a_poll);
  RUN_TEST(test_connected_across_the_millisecond_wrap);
  RUN_TEST(test_mode_buttons_advance_the_counter);
  RUN_TEST(test_walk_is_not_a_portal_mode);
  RUN_TEST(test_a_press_counts_as_a_poll);
  RUN_TEST(test_translate_and_rotate_are_always_zero);
  RUN_TEST(test_disconnect_requests_sleep_until_the_next_press);
  RUN_TEST(test_clear_advances_only_the_clear_counter);
  RUN_TEST(test_dials_are_the_gamepad_defaults);
  return UNITY_END();
}
