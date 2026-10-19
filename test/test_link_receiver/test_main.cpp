#include <unity.h>
#include "protocol/link_receiver.h"

using namespace link_receiver;

static Receiver r;
static uint32_t now;

void setUp()
{
  r = Receiver();
  now = 5000;
}
void tearDown() {}

static protocol::Command frame(RequestedMode mode, uint8_t clear = 0, uint8_t modeSeq = 0, bool connected = true,
                               bool estop = false)
{
  protocol::Command c = {};
  c.mode = (uint8_t)mode;
  c.speed = 44;
  c.flags = (connected ? protocol::FLAG_CONTROLLER_CONNECTED : 0) | (estop ? protocol::FLAG_EMERGENCY_STOP : 0);
  c.clearCounter = clear;
  c.modeRequestCounter = modeSeq;
  return c;
}

// One frame per 20 ms tick.
static Input tick(const protocol::Command &c)
{
  now += 20;
  accept(r, c, now);
  return take(r);
}

void test_no_frame_yet_requests_sleep_and_stale()
{
  Input in = take(r);
  TEST_ASSERT_FALSE(in.linkSeen);
  TEST_ASSERT_TRUE(in.command.mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_UINT32(0, in.command.issuedAtMs);
  TEST_ASSERT_FALSE(in.controllerConnected);
  TEST_ASSERT_FALSE(in.clearRequested);
}

void test_command_is_stamped_with_arrival_time()
{
  Input in = tick(frame(RequestedMode::Walk));
  TEST_ASSERT_TRUE(in.linkSeen);
  TEST_ASSERT_TRUE(in.command.mode == RequestedMode::Walk);
  TEST_ASSERT_EQUAL_UINT32(now, in.command.issuedAtMs);
  // No new frame: the stamp stays at the last arrival, so it can go stale.
  uint32_t arrived = now;
  now += 1500;
  in = take(r);
  TEST_ASSERT_EQUAL_UINT32(arrived, in.command.issuedAtMs);
}

void test_newest_frame_wins()
{
  accept(r, frame(RequestedMode::Walk), now);
  accept(r, frame(RequestedMode::Stand), now + 1);
  TEST_ASSERT_TRUE(take(r).command.mode == RequestedMode::Stand);
}

void test_lost_frame_loses_nothing()
{
  // Full state in every frame: skipping one changes nothing.
  tick(frame(RequestedMode::Stand, 0, 0));
  tick(frame(RequestedMode::Walk, 0, 1));
  Input a = tick(frame(RequestedMode::Walk, 1, 1));

  Receiver other;
  uint32_t t = 5020;
  accept(other, frame(RequestedMode::Stand, 0, 0), t);
  take(other);
  t += 40; // the Walk frame is lost
  accept(other, frame(RequestedMode::Walk, 1, 1), t);
  Input b = take(other);
  TEST_ASSERT_TRUE(a.command.mode == b.command.mode);
  TEST_ASSERT_EQUAL_UINT32(a.requestSeq, b.requestSeq);
  TEST_ASSERT_EQUAL(a.clearRequested, b.clearRequested);
}

void test_disconnected_controller_requests_sleep()
{
  Input in = tick(frame(RequestedMode::Walk, 0, 0, false));
  TEST_ASSERT_TRUE(in.command.mode == RequestedMode::Sleep);
  TEST_ASSERT_FALSE(in.controllerConnected);
}

void test_unknown_mode_requests_sleep()
{
  protocol::Command c = frame(RequestedMode::Walk);
  c.mode = 9;
  TEST_ASSERT_TRUE(tick(c).command.mode == RequestedMode::Sleep);
}

void test_out_of_range_values_are_clamped()
{
  protocol::Command c = frame(RequestedMode::Walk);
  c.translateX = 127;
  c.translateY = 127;
  c.rotate = -128;
  c.height = 250;
  c.speed = 101;
  c.stride = 7;
  Input in = tick(c);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, in.command.translate.magnitude());
  TEST_ASSERT_EQUAL_FLOAT(-100.0f, in.command.rotate);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, in.command.height);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, in.command.speed);
  TEST_ASSERT_TRUE(in.command.stride == StrideMode::Dynamic);
}

void test_emergency_stop_flag_passes_through()
{
  TEST_ASSERT_TRUE(tick(frame(RequestedMode::Walk, 0, 0, true, true)).emergencyStop);
  TEST_ASSERT_FALSE(tick(frame(RequestedMode::Walk)).emergencyStop);
}

void test_repeated_clear_counter_issues_no_clear()
{
  tick(frame(RequestedMode::Stand, 4));
  for (int i = 0; i < 50; i++)
    TEST_ASSERT_FALSE(tick(frame(RequestedMode::Stand, 4)).clearRequested);
}

void test_incremented_clear_counter_issues_exactly_one_clear()
{
  tick(frame(RequestedMode::Stand, 4));
  TEST_ASSERT_TRUE(tick(frame(RequestedMode::Stand, 5)).clearRequested);
  TEST_ASSERT_FALSE(tick(frame(RequestedMode::Stand, 5)).clearRequested);
  // Wraps.
  tick(frame(RequestedMode::Stand, 255));
  TEST_ASSERT_TRUE(tick(frame(RequestedMode::Stand, 0)).clearRequested);
}

void test_clear_is_held_until_taken()
{
  accept(r, frame(RequestedMode::Stand, 4), now);
  accept(r, frame(RequestedMode::Stand, 5), now + 20);
  accept(r, frame(RequestedMode::Stand, 5), now + 40);
  TEST_ASSERT_TRUE(take(r).clearRequested);
  TEST_ASSERT_FALSE(take(r).clearRequested);
}

void test_mode_request_counter_advances_request_seq()
{
  uint32_t seq = tick(frame(RequestedMode::Stand, 0, 7)).requestSeq;
  TEST_ASSERT_EQUAL_UINT32(seq, tick(frame(RequestedMode::Stand, 0, 7)).requestSeq);
  TEST_ASSERT_EQUAL_UINT32(seq + 1, tick(frame(RequestedMode::Walk, 0, 8)).requestSeq);
}

void test_first_frame_sets_baseline_only()
{
  Input in = tick(frame(RequestedMode::Walk, 9, 3));
  TEST_ASSERT_FALSE(in.clearRequested);
  TEST_ASSERT_EQUAL_UINT32(0, in.requestSeq);
}

void test_bridge_restart_after_gap_neither_clears_nor_requests()
{
  tick(frame(RequestedMode::Walk, 9, 3));
  uint32_t seq = tick(frame(RequestedMode::Walk, 9, 3)).requestSeq;
  now += LINK_GAP_MS; // the bridge reboots
  Input in = tick(frame(RequestedMode::Stand, 0, 0));
  TEST_ASSERT_FALSE(in.clearRequested);
  TEST_ASSERT_EQUAL_UINT32(seq, in.requestSeq);
  // Later changes count again.
  TEST_ASSERT_TRUE(tick(frame(RequestedMode::Stand, 1, 0)).clearRequested);
  TEST_ASSERT_EQUAL_UINT32(seq + 1, tick(frame(RequestedMode::Walk, 1, 1)).requestSeq);
}

void test_short_gap_keeps_counting()
{
  tick(frame(RequestedMode::Stand, 1, 1));
  now += LINK_GAP_MS - 40; // a few lost frames
  TEST_ASSERT_TRUE(tick(frame(RequestedMode::Stand, 2, 1)).clearRequested);
}

// Spec "Calibration pose": commanded 0/90/90 per leg, servo angles with the
// offsets the IK added, pulses as given.
void test_pose_in_calibration_pose()
{
  static body::BodyState b = {};
  joint_calibration::Table t;
  joint_calibration::loadDefaults(t);
  uint16_t pulses[protocol::CALIBRATION_JOINTS];
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    b.offsets[leg] = joint_calibration::legOffsets(t, leg);
    TEST_ASSERT_TRUE(kinematics::solveLeg(body::CALIBRATION_POSE, b.offsets[leg], b.angles[leg]));
  }
  for (int i = 0; i < protocol::CALIBRATION_JOINTS; i++)
    pulses[i] = (uint16_t)(1000 + i);
  protocol::Pose p = makePose(b, pulses, true);
  TEST_ASSERT_EQUAL_HEX8(protocol::POSE_OUTPUTS_ON, p.flags);
  const int16_t want[3] = {0, 9000, 9000};
  for (int i = 0; i < protocol::CALIBRATION_JOINTS; i++)
  {
    TEST_ASSERT_INT_WITHIN(1, want[i % 3], p.joints[i].commandedCentideg);
    int16_t offset = (int16_t)lroundf(t.joints[i].offsetDeg * 100);
    TEST_ASSERT_INT_WITHIN(1, want[i % 3] + offset, p.joints[i].servoCentideg);
    TEST_ASSERT_EQUAL_UINT16(1000 + i, p.joints[i].pulseUs);
  }
  TEST_ASSERT_EQUAL_HEX8(0, makePose(b, pulses, false).flags);
}

// Rounded to the nearest centidegree, saturated to the i16 range.
void test_pose_rounds_and_saturates()
{
  static body::BodyState b = {};
  uint16_t pulses[protocol::CALIBRATION_JOINTS] = {};
  b.angles[0] = {12.346f, -12.346f, 400.0f};
  b.offsets[0] = Vector3(0.0f, 0.0f, 0.0f);
  b.angles[1] = {10.0f, 0.0f, -400.0f};
  b.offsets[1] = Vector3(-400.0f, 0.0f, 0.0f);
  protocol::Pose p = makePose(b, pulses, false);
  TEST_ASSERT_EQUAL_INT16(1235, p.joints[0].commandedCentideg);
  TEST_ASSERT_EQUAL_INT16(-1235, p.joints[1].servoCentideg);
  TEST_ASSERT_EQUAL_INT16(32767, p.joints[2].servoCentideg);
  TEST_ASSERT_EQUAL_INT16(32767, p.joints[3].commandedCentideg); // 10 - (-400)
  TEST_ASSERT_EQUAL_INT16(1000, p.joints[3].servoCentideg);
  TEST_ASSERT_EQUAL_INT16(-32768, p.joints[5].servoCentideg);
  TEST_ASSERT_EQUAL_UINT16(0, p.joints[17].pulseUs);
}

// --- Gesture requests ---

static protocol::Command gestureFrame(uint8_t gesture, uint8_t counter)
{
  protocol::Command c = frame(RequestedMode::Stand);
  c.gesture = gesture;
  c.gestureRequestCounter = counter;
  return c;
}

void test_each_new_gesture_counter_is_one_request()
{
  Input in = tick(gestureFrame(0, 0));
  uint32_t seq = in.gestureSeq;
  in = tick(gestureFrame(protocol::GESTURE_WIGGLE, 1));
  TEST_ASSERT_EQUAL_UINT32(seq + 1, in.gestureSeq);
  TEST_ASSERT_EQUAL(protocol::GESTURE_WIGGLE, in.gesture);
  // A held button: the same counter in later frames requests nothing more.
  for (int i = 0; i < 10; i++)
    in = tick(gestureFrame(protocol::GESTURE_WIGGLE, 1));
  TEST_ASSERT_EQUAL_UINT32(seq + 1, in.gestureSeq);
  in = tick(gestureFrame(protocol::GESTURE_SAY_HI, 2));
  TEST_ASSERT_EQUAL_UINT32(seq + 2, in.gestureSeq);
  TEST_ASSERT_EQUAL(protocol::GESTURE_SAY_HI, in.gesture);
}

void test_gesture_counter_after_a_link_gap_is_only_a_baseline()
{
  Input in = tick(gestureFrame(0, 3));
  uint32_t seq = in.gestureSeq;
  now += LINK_GAP_MS;
  in = tick(gestureFrame(protocol::GESTURE_SAY_HI, 0)); // a restarted bridge
  TEST_ASSERT_EQUAL_UINT32(seq, in.gestureSeq);
  in = tick(gestureFrame(protocol::GESTURE_SAY_HI, 1));
  TEST_ASSERT_EQUAL_UINT32(seq + 1, in.gestureSeq);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_pose_in_calibration_pose);
  RUN_TEST(test_pose_rounds_and_saturates);
  RUN_TEST(test_no_frame_yet_requests_sleep_and_stale);
  RUN_TEST(test_command_is_stamped_with_arrival_time);
  RUN_TEST(test_newest_frame_wins);
  RUN_TEST(test_lost_frame_loses_nothing);
  RUN_TEST(test_disconnected_controller_requests_sleep);
  RUN_TEST(test_unknown_mode_requests_sleep);
  RUN_TEST(test_out_of_range_values_are_clamped);
  RUN_TEST(test_emergency_stop_flag_passes_through);
  RUN_TEST(test_repeated_clear_counter_issues_no_clear);
  RUN_TEST(test_incremented_clear_counter_issues_exactly_one_clear);
  RUN_TEST(test_clear_is_held_until_taken);
  RUN_TEST(test_mode_request_counter_advances_request_seq);
  RUN_TEST(test_first_frame_sets_baseline_only);
  RUN_TEST(test_bridge_restart_after_gap_neither_clears_nor_requests);
  RUN_TEST(test_short_gap_keeps_counting);
  RUN_TEST(test_each_new_gesture_counter_is_one_request);
  RUN_TEST(test_gesture_counter_after_a_link_gap_is_only_a_baseline);
  return UNITY_END();
}
