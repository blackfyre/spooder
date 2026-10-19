#include <unity.h>
#include "protocol/link_receiver.h"

using namespace link_receiver;

static Arbiter a;
static uint32_t now;

void setUp()
{
  a = Arbiter();
  now = 10000;
}
void tearDown() {}

// A link's input with a command that arrived `ageMs` ago.
static Input link(bool connected, RequestedMode mode = RequestedMode::Walk, uint32_t requestSeq = 0,
                  uint32_t ageMs = 0)
{
  Input in = {};
  in.linkSeen = true;
  in.controllerConnected = connected;
  in.command.mode = connected ? mode : RequestedMode::Sleep;
  in.command.issuedAtMs = now - ageMs;
  in.requestSeq = requestSeq;
  return in;
}

static Input absent()
{
  Input in = {};
  in.command.mode = RequestedMode::Sleep;
  in.command.issuedAtMs = 0;
  return in;
}

static Arbitration tick(const Input &bridge, const Input &usb)
{
  now += 20;
  return arbitrate(a, bridge, usb, now);
}

static void assertSleep(const Arbitration &out)
{
  TEST_ASSERT_TRUE(out.input.command.mode == RequestedMode::Sleep);
  TEST_ASSERT_TRUE(isFresh(out.input, now));
  TEST_ASSERT_FALSE(out.input.controllerConnected);
}

// --- the arbitration rule, row by row ------------------------------------------------

void test_both_connected_and_fresh_is_a_conflict()
{
  // Spec scenario: both claim control: Sleep, and telemetry reports it.
  now += 20;
  Arbitration out = arbitrate(a, link(true, RequestedMode::Walk), link(true, RequestedMode::Calibrate), now);
  TEST_ASSERT_TRUE(out.source == Source::Conflict);
  assertSleep(out);
  TEST_ASSERT_EQUAL_HEX8(protocol::TELEMETRY_SOURCE_CONFLICT, out.telemetryFlags);
}

void test_bridge_alone_is_in_control()
{
  now += 20;
  Arbitration out = arbitrate(a, link(true, RequestedMode::Walk), link(false), now);
  TEST_ASSERT_TRUE(out.source == Source::Bridge);
  TEST_ASSERT_TRUE(out.input.command.mode == RequestedMode::Walk);
  TEST_ASSERT_TRUE(out.input.controllerConnected);
  TEST_ASSERT_EQUAL_HEX8(protocol::TELEMETRY_BRIDGE_IN_CONTROL, out.telemetryFlags);
}

void test_usb_alone_is_in_control()
{
  // Spec scenario: no bridge, a USB host requests Calibrate: the robot follows.
  now += 20;
  Arbitration out = arbitrate(a, absent(), link(true, RequestedMode::Calibrate), now);
  TEST_ASSERT_TRUE(out.source == Source::Usb);
  TEST_ASSERT_TRUE(out.input.command.mode == RequestedMode::Calibrate);
  TEST_ASSERT_EQUAL_UINT32(now, out.input.command.issuedAtMs);
  TEST_ASSERT_EQUAL_HEX8(protocol::TELEMETRY_USB_IN_CONTROL, out.telemetryFlags);
}

void test_no_source_requests_sleep()
{
  now += 20;
  Arbitration out = arbitrate(a, link(false), link(false), now);
  TEST_ASSERT_TRUE(out.source == Source::None);
  assertSleep(out);
  TEST_ASSERT_EQUAL_HEX8(0, out.telemetryFlags);
  TEST_ASSERT_TRUE(out.input.linkSeen);

  out = tick(absent(), absent());
  TEST_ASSERT_TRUE(out.source == Source::None);
  assertSleep(out);
  TEST_ASSERT_FALSE(out.input.linkSeen);
}

void test_stale_link_does_not_claim_control()
{
  // Spec (USB staleness): a host unplugged mid-command stops counting after 1 s.
  now += 20;
  Arbitration out = arbitrate(a, absent(), link(true, RequestedMode::Walk, 0, body::COMMAND_TIMEOUT_MS), now);
  TEST_ASSERT_TRUE(out.source == Source::Usb);
  out = arbitrate(a, absent(), link(true, RequestedMode::Walk, 0, body::COMMAND_TIMEOUT_MS + 1), now);
  TEST_ASSERT_TRUE(out.source == Source::None);
  assertSleep(out);
  // A stale bridge does not block a fresh USB host, and makes no conflict.
  out = arbitrate(a, link(true, RequestedMode::Walk, 0, 1500), link(true, RequestedMode::Stand), now);
  TEST_ASSERT_TRUE(out.source == Source::Usb);
  TEST_ASSERT_TRUE(out.input.command.mode == RequestedMode::Stand);
}

void test_conflict_ends_when_one_stops()
{
  tick(link(true), link(true));
  Arbitration out = tick(link(true, RequestedMode::Stand), link(false));
  TEST_ASSERT_TRUE(out.source == Source::Bridge);
  TEST_ASSERT_EQUAL_HEX8(protocol::TELEMETRY_BRIDGE_IN_CONTROL, out.telemetryFlags);
}

// --- request counters across a source switch --------------------------------------------

void test_request_seq_follows_the_source_in_control()
{
  Arbitration out = tick(link(true, RequestedMode::Walk, 5), absent());
  uint32_t base = out.input.requestSeq;
  out = tick(link(true, RequestedMode::Walk, 6), absent());
  TEST_ASSERT_EQUAL_UINT32(base + 1, out.input.requestSeq);

  // The USB link is up with its own, different counter; then the bridge lets
  // go and USB takes over.
  out = tick(link(true, RequestedMode::Walk, 6), link(false, RequestedMode::Sleep, 2));
  TEST_ASSERT_EQUAL_UINT32(base + 1, out.input.requestSeq);
  out = tick(link(false, RequestedMode::Sleep, 6), link(true, RequestedMode::Stand, 2));
  TEST_ASSERT_TRUE(out.source == Source::Usb);
  TEST_ASSERT_EQUAL_UINT32(base + 1, out.input.requestSeq); // a switch is not a request
  out = tick(link(false, RequestedMode::Sleep, 6), link(true, RequestedMode::Stand, 2));
  TEST_ASSERT_EQUAL_UINT32(base + 1, out.input.requestSeq);
  out = tick(link(false, RequestedMode::Sleep, 6), link(true, RequestedMode::Stand, 3));
  TEST_ASSERT_EQUAL_UINT32(base + 2, out.input.requestSeq);

  // The idle bridge's counter never counts.
  out = tick(link(false, RequestedMode::Sleep, 9), link(true, RequestedMode::Stand, 3));
  TEST_ASSERT_EQUAL_UINT32(base + 2, out.input.requestSeq);
  // Nor does a request made during a conflict.
  out = tick(link(true, RequestedMode::Walk, 10), link(true, RequestedMode::Stand, 4));
  TEST_ASSERT_EQUAL_UINT32(base + 2, out.input.requestSeq);
  // Back to the bridge alone: no new request until its counter moves.
  out = tick(link(true, RequestedMode::Walk, 10), link(false, RequestedMode::Sleep, 4));
  TEST_ASSERT_EQUAL_UINT32(base + 2, out.input.requestSeq);
  out = tick(link(true, RequestedMode::Walk, 11), link(false, RequestedMode::Sleep, 4));
  TEST_ASSERT_EQUAL_UINT32(base + 3, out.input.requestSeq);
}

void test_first_tick_sets_counter_baselines()
{
  Arbitration out = tick(link(true, RequestedMode::Walk, 40), link(false, RequestedMode::Sleep, 7));
  TEST_ASSERT_EQUAL_UINT32(0, out.input.requestSeq);
}

// --- emergency stop and fault clear ------------------------------------------------------

void test_emergency_stop_from_the_idle_source()
{
  // Spec scenario: the gamepad is in control and the USB host signals e-stop.
  Input usb = link(false);
  usb.emergencyStop = true;
  Arbitration out = tick(link(true), usb);
  TEST_ASSERT_TRUE(out.source == Source::Bridge);
  TEST_ASSERT_TRUE(out.input.emergencyStop);

  Input bridge = link(false);
  bridge.emergencyStop = true;
  out = tick(bridge, link(true));
  TEST_ASSERT_TRUE(out.input.emergencyStop);
  // Also from a link that has gone stale, and during a conflict.
  bridge = link(true, RequestedMode::Walk, 0, 5000);
  bridge.emergencyStop = true;
  TEST_ASSERT_TRUE(tick(bridge, absent()).input.emergencyStop);
  usb = link(true);
  usb.emergencyStop = true;
  TEST_ASSERT_TRUE(tick(link(true), usb).input.emergencyStop);
}

void test_clear_accepted_only_from_the_source_in_control()
{
  Input usb = link(false);
  usb.clearRequested = true;
  TEST_ASSERT_FALSE(tick(link(true), usb).input.clearRequested);

  Input bridge = link(true);
  bridge.clearRequested = true;
  TEST_ASSERT_TRUE(tick(bridge, link(false)).input.clearRequested);

  usb = link(true);
  usb.clearRequested = true;
  TEST_ASSERT_TRUE(tick(link(false), usb).input.clearRequested);
  // Not during a conflict, nor with no source.
  TEST_ASSERT_FALSE(tick(bridge, usb).input.clearRequested);
  usb.controllerConnected = false;
  TEST_ASSERT_FALSE(tick(absent(), usb).input.clearRequested);
}

void test_gesture_requests_count_only_from_the_source_in_control()
{
  Input bridge = link(true, RequestedMode::Stand);
  Input usb = link(false);
  Arbitration out = tick(bridge, usb);
  uint32_t seq = out.input.command.gestureSeq;
  // The idle USB link's request is ignored.
  usb.gestureSeq = 1;
  usb.gesture = protocol::GESTURE_BALL;
  out = tick(bridge, usb);
  TEST_ASSERT_EQUAL_UINT32(seq, out.input.command.gestureSeq);
  // The bridge's, in control, counts once and carries its id.
  bridge.gestureSeq = 1;
  bridge.gesture = protocol::GESTURE_WIGGLE;
  out = tick(bridge, usb);
  TEST_ASSERT_EQUAL_UINT32(seq + 1, out.input.command.gestureSeq);
  TEST_ASSERT_EQUAL(protocol::GESTURE_WIGGLE, out.input.command.gesture);
  out = tick(bridge, usb);
  TEST_ASSERT_EQUAL_UINT32(seq + 1, out.input.command.gestureSeq);
  // A change of source does not count as a request.
  bridge = link(false);
  usb = link(true, RequestedMode::Stand);
  usb.gestureSeq = 1;
  out = tick(bridge, usb);
  TEST_ASSERT_EQUAL_UINT32(seq + 1, out.input.command.gestureSeq);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_both_connected_and_fresh_is_a_conflict);
  RUN_TEST(test_bridge_alone_is_in_control);
  RUN_TEST(test_usb_alone_is_in_control);
  RUN_TEST(test_no_source_requests_sleep);
  RUN_TEST(test_stale_link_does_not_claim_control);
  RUN_TEST(test_conflict_ends_when_one_stops);
  RUN_TEST(test_request_seq_follows_the_source_in_control);
  RUN_TEST(test_first_tick_sets_counter_baselines);
  RUN_TEST(test_emergency_stop_from_the_idle_source);
  RUN_TEST(test_clear_accepted_only_from_the_source_in_control);
  RUN_TEST(test_gesture_requests_count_only_from_the_source_in_control);
  return UNITY_END();
}
