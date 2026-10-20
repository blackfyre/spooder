#include <unity.h>
#include <vector>
#include "gamepad_mapping/rumble.h"
#include "protection/protection.h"
#include "protocol/protocol.h"

using namespace rumble;

static State s;
static Inputs in;

void setUp()
{
  s = State();
  in = Inputs();
  in.nowMs = 10000;
  in.gamepadConnected = true;
  in.telemetryFresh = true;
}
void tearDown() {}

static Request step(uint32_t dtMs = 20)
{
  in.nowMs += dtMs;
  return update(s, in);
}

// Steps for @p ms and returns every rumble played, in order.
static std::vector<Request> all(uint32_t ms)
{
  std::vector<Request> out;
  for (uint32_t t = 0; t < ms; t += 20)
  {
    Request r = step();
    if (r.play)
      out.push_back(r);
  }
  return out;
}

// Steps for @p ms and returns the first rumble played, if any.
static Request run(uint32_t ms)
{
  Request first = {false, Event::None, 0, 0, 0};
  for (uint32_t t = 0; t < ms; t += 20)
  {
    Request r = step();
    if (r.play && !first.play)
      first = r;
  }
  return first;
}

void test_new_fault_rumbles_long_and_strong_once()
{
  step(); // baseline
  in.faults = protection::OVERCURRENT;
  Request r = step();
  TEST_ASSERT_TRUE(r.play);
  TEST_ASSERT_TRUE(r.event == Event::FaultLatched);
  TEST_ASSERT_EQUAL_UINT16(600, r.durationMs);
  TEST_ASSERT_EQUAL_UINT8(255, r.strong);
  TEST_ASSERT_FALSE(run(3000).play); // still latched: no repeat
}

void test_faults_present_on_connect_do_not_rumble()
{
  in.faults = protection::UNDERVOLTAGE;
  step(); // the gamepad connects with a fault latched
  TEST_ASSERT_FALSE(run(500).play);
  // A further fault does.
  in.faults |= protection::OVERCURRENT;
  TEST_ASSERT_TRUE(step().event == Event::FaultLatched);
}

void test_the_emergency_stop_fault_bit_alone_does_not_count_as_a_fault()
{
  step();
  in.faults = protection::EMERGENCY_STOP; // another source's e-stop
  TEST_ASSERT_FALSE(step().play);
}

void test_bridge_estop_on_and_off()
{
  step();
  in.emergencyStop = true;
  Request r = step();
  TEST_ASSERT_TRUE(r.event == Event::EstopOn);
  TEST_ASSERT_EQUAL_UINT16(250, r.durationMs);
  in.emergencyStop = false;
  r = run(1200);
  TEST_ASSERT_TRUE(r.event == Event::EstopOff);
}

void test_link_lost_and_regained()
{
  step();
  in.telemetryFresh = false;
  TEST_ASSERT_TRUE(step().event == Event::LinkLost);
  in.telemetryFresh = true;
  in.faults = protection::OVERCURRENT; // latched while the link was down
  Request r = run(1200);
  TEST_ASSERT_TRUE(r.event == Event::LinkRegained);
  TEST_ASSERT_FALSE(run(500).play); // the fault was the baseline
}

void test_refused_gesture_taps_after_600_ms()
{
  step();
  in.gesture = protocol::GESTURE_SAY_HI;
  in.gestureRequestCounter = 1;
  Request r = step();
  TEST_ASSERT_FALSE(r.play);
  uint32_t t = 0;
  while (!(r = step()).play && t < 2000)
    t += 20;
  TEST_ASSERT_TRUE(r.event == Event::GestureRefused);
  TEST_ASSERT_TRUE(t + 40 >= GESTURE_REFUSED_MS && t <= GESTURE_REFUSED_MS + 40);
}

void test_played_gesture_does_not_tap()
{
  step();
  in.gesture = protocol::GESTURE_WIGGLE;
  in.gestureRequestCounter = 1;
  step();
  in.playingGesture = protocol::GESTURE_WIGGLE; // the robot plays it
  TEST_ASSERT_FALSE(run(1500).play);
}

void test_more_important_event_replaces_a_lighter_rumble_but_not_the_reverse()
{
  step();
  in.emergencyStop = true;
  TEST_ASSERT_TRUE(step().event == Event::EstopOn);
  in.faults = protection::OVERCURRENT; // more important: plays at once
  TEST_ASSERT_TRUE(step().event == Event::FaultLatched);
  in.telemetryFresh = false; // less important, while the fault rumble runs
  TEST_ASSERT_FALSE(step().play);
  std::vector<Request> rs = all(800); // it plays once the fault rumble ends
  TEST_ASSERT_EQUAL(1, (int)rs.size());
  TEST_ASSERT_TRUE(rs[0].event == Event::LinkLost);
}

void test_nothing_without_a_gamepad()
{
  in.gamepadConnected = false;
  step();
  in.emergencyStop = true;
  in.faults = protection::OVERCURRENT;
  TEST_ASSERT_FALSE(run(500).play);
}

void test_the_same_event_does_not_repeat_within_a_second()
{
  step();
  in.faults = protection::OVERCURRENT;
  TEST_ASSERT_TRUE(step().event == Event::FaultLatched);
  in.faults = 0; // cleared, and latched again 400 ms later: no rumble
  all(380);
  in.faults = protection::OVERCURRENT;
  TEST_ASSERT_EQUAL(0, (int)all(500).size());
  in.faults = 0; // and again, past the second: it rumbles
  all(400);
  in.faults = protection::OVERCURRENT;
  std::vector<Request> later = all(200);
  TEST_ASSERT_EQUAL(1, (int)later.size());
  TEST_ASSERT_TRUE(later[0].event == Event::FaultLatched);
}

void test_a_lighter_event_plays_after_the_stronger_rumble()
{
  step();
  in.faults = protection::OVERCURRENT;
  TEST_ASSERT_TRUE(step().event == Event::FaultLatched);
  in.emergencyStop = true; // during the 600 ms fault rumble
  std::vector<Request> rs = all(800);
  TEST_ASSERT_EQUAL(1, (int)rs.size());
  TEST_ASSERT_TRUE(rs[0].event == Event::EstopOn);
}

void test_simultaneous_events_play_strongest_first()
{
  step();
  in.faults = protection::OVERCURRENT;
  in.emergencyStop = true;
  Request first = step();
  TEST_ASSERT_TRUE(first.event == Event::FaultLatched);
  std::vector<Request> rest = all(800);
  TEST_ASSERT_EQUAL(1, (int)rest.size());
  TEST_ASSERT_TRUE(rest[0].event == Event::EstopOn);
}

// Every rumble played over @p ms, with the time it started.
struct Played
{
  Event event;
  uint32_t atMs;
};
static std::vector<Played> timed(uint32_t ms)
{
  std::vector<Played> out;
  for (uint32_t t = 0; t < ms; t += 20)
  {
    Request r = step();
    if (r.play)
      out.push_back({r.event, in.nowMs});
  }
  return out;
}

void test_waiting_events_play_in_turn()
{
  step();
  in.faults = protection::OVERCURRENT;
  TEST_ASSERT_TRUE(step().event == Event::FaultLatched);
  uint32_t faultAt = in.nowMs;
  in.telemetryFresh = false; // LinkLost waits behind the fault rumble...
  step();
  in.telemetryFresh = true; // ...and LinkRegained behind both
  std::vector<Played> rs = timed(2000);
  TEST_ASSERT_EQUAL(2, (int)rs.size());
  TEST_ASSERT_TRUE(rs[0].event == Event::LinkLost);
  TEST_ASSERT_TRUE(rs[0].atMs >= faultAt + 600 && rs[0].atMs <= faultAt + 620);
  TEST_ASSERT_TRUE(rs[1].event == Event::LinkRegained);
  TEST_ASSERT_TRUE(rs[1].atMs >= rs[0].atMs + 400 && rs[1].atMs <= rs[0].atMs + 420);
}

void test_a_waiting_stronger_event_plays_before_a_new_lighter_one()
{
  step();
  in.faults = protection::OVERCURRENT;
  TEST_ASSERT_TRUE(step().event == Event::FaultLatched);
  in.emergencyStop = true; // waits behind the fault rumble
  timed(560);
  in.telemetryFresh = false; // a lighter event, just as the fault rumble ends
  std::vector<Played> rs = timed(1000);
  TEST_ASSERT_TRUE(rs.size() >= 2);
  TEST_ASSERT_TRUE(rs[0].event == Event::EstopOn);
  TEST_ASSERT_TRUE(rs[1].event == Event::LinkLost);
}

void test_the_repeat_limit_expires_across_a_timer_wrap()
{
  in.nowMs = 0xFFFFFF00u; // just before the millisecond counter wraps
  step();
  in.emergencyStop = true;
  TEST_ASSERT_TRUE(step().event == Event::EstopOn);
  uint32_t playedAt = in.nowMs;
  in.emergencyStop = false;
  all(1500); // the cooldown passes
  // A full counter cycle later, 500 ms after the old timestamp: without the
  // cooldown expiring it would look like a repeat within the second.
  in.nowMs = playedAt + 500 - 20;
  in.emergencyStop = true;
  TEST_ASSERT_TRUE(step().event == Event::EstopOn);
}

void test_a_disconnect_forgets_waiting_events()
{
  step();
  in.faults = protection::OVERCURRENT;
  step();
  in.emergencyStop = true; // waiting behind the fault rumble
  step();
  in.gamepadConnected = false;
  step();
  in.gamepadConnected = true;
  TEST_ASSERT_EQUAL(0, (int)all(1500).size()); // the new connection's baseline
}

void test_the_same_gesture_already_playing_is_refused()
{
  step();
  in.playingGesture = protocol::GESTURE_WIGGLE; // a wiggle plays
  in.gesture = protocol::GESTURE_WIGGLE;
  in.gestureRequestCounter = 1; // and another is requested: the robot drops it
  std::vector<Request> rs = all(1000);
  TEST_ASSERT_EQUAL(1, (int)rs.size());
  TEST_ASSERT_TRUE(rs[0].event == Event::GestureRefused);
}

void test_a_gesture_requested_while_the_link_is_down_is_not_refused()
{
  step();
  in.telemetryFresh = false;
  TEST_ASSERT_TRUE(step().event == Event::LinkLost);
  in.gesture = protocol::GESTURE_SAY_HI;
  in.gestureRequestCounter = 1;
  TEST_ASSERT_EQUAL(0, (int)all(1500).size());
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_new_fault_rumbles_long_and_strong_once);
  RUN_TEST(test_faults_present_on_connect_do_not_rumble);
  RUN_TEST(test_the_emergency_stop_fault_bit_alone_does_not_count_as_a_fault);
  RUN_TEST(test_bridge_estop_on_and_off);
  RUN_TEST(test_link_lost_and_regained);
  RUN_TEST(test_refused_gesture_taps_after_600_ms);
  RUN_TEST(test_played_gesture_does_not_tap);
  RUN_TEST(test_more_important_event_replaces_a_lighter_rumble_but_not_the_reverse);
  RUN_TEST(test_nothing_without_a_gamepad);
  RUN_TEST(test_the_same_event_does_not_repeat_within_a_second);
  RUN_TEST(test_a_lighter_event_plays_after_the_stronger_rumble);
  RUN_TEST(test_simultaneous_events_play_strongest_first);
  RUN_TEST(test_waiting_events_play_in_turn);
  RUN_TEST(test_a_waiting_stronger_event_plays_before_a_new_lighter_one);
  RUN_TEST(test_the_repeat_limit_expires_across_a_timer_wrap);
  RUN_TEST(test_a_disconnect_forgets_waiting_events);
  RUN_TEST(test_the_same_gesture_already_playing_is_refused);
  RUN_TEST(test_a_gesture_requested_while_the_link_is_down_is_not_refused);
  return UNITY_END();
}
