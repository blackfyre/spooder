// The handheld controller (WiFi drive): datagram validation, the rate
// window, the sender lock, the sequence rule, liveness, counter translation,
// the e-stop flag and the relay target. The scenarios of the wifi-drive
// spec delta.
#include <unity.h>
#include "wifi_drive/handheld_controller.h"

using namespace handheld_controller;

static Controller c;
static const Sender HANDHELD = {0xC0A80402u, 40000}; // 192.168.4.2
static const Sender OTHER = {0xC0A80403u, 40000};

void setUp()
{
  c = Controller();
}
void tearDown() {}

// The handheld's own state, as its frames carry it.
static protocol::Command pad()
{
  protocol::Command p = {};
  p.mode = (uint8_t)RequestedMode::Walk;
  p.translateY = 60;
  p.rotate = -20;
  p.height = 30;
  p.speed = 70;
  p.gait = 2;
  p.stride = (uint8_t)StrideMode::Fixed;
  p.flags = protocol::FLAG_CONTROLLER_CONNECTED;
  return p;
}

static Outcome send(const protocol::Command &p, uint8_t seq, uint32_t nowMs, Sender from = HANDHELD)
{
  uint8_t buf[protocol::COMMAND_FRAME_SIZE];
  int n = protocol::encodeCommand(p, seq, buf);
  return accept(c, buf, n, from, nowMs);
}

// A 50 Hz handheld from @p fromMs for @p frames frames, sequence from @p seq.
static void stream(const protocol::Command &p, uint8_t &seq, uint32_t &nowMs, int frames)
{
  for (int i = 0; i < frames; i++, nowMs += 20)
    TEST_ASSERT_TRUE(send(p, seq++, nowMs) == Outcome::Accepted);
}

static void assertNoCounterChange(uint8_t clear, uint8_t mode, uint8_t gesture, uint32_t nowMs)
{
  gamepad_mapping::Desired d = desired(c, nowMs);
  TEST_ASSERT_EQUAL_UINT8(clear, d.clearCounter);
  TEST_ASSERT_EQUAL_UINT8(mode, d.modeRequestCounter);
  TEST_ASSERT_EQUAL_UINT8(gesture, d.gestureRequestCounter);
}

// --- datagrams -----------------------------------------------------------------

void test_valid_frame_is_the_newest_desired_state()
{
  TEST_ASSERT_TRUE(send(pad(), 0, 1000) == Outcome::Accepted);
  gamepad_mapping::Desired d = desired(c, 1000);
  TEST_ASSERT_TRUE(d.controllerConnected);
  TEST_ASSERT_EQUAL_INT8(0, d.translateX);
  TEST_ASSERT_EQUAL_INT8(60, d.translateY);
  TEST_ASSERT_EQUAL_INT8(-20, d.rotate);
  TEST_ASSERT_EQUAL_UINT8(30, d.height);
  TEST_ASSERT_EQUAL_UINT8(70, d.speed);
  TEST_ASSERT_EQUAL_UINT8(2, d.gait);
  TEST_ASSERT_TRUE(d.stride == StrideMode::Fixed);
  // The first frame of a session requests nothing: Sleep until a mode request.
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_UINT32(1, count(c, Outcome::Accepted));

  protocol::Command p = pad();
  p.translateX = -100; // the bridge passes it through; the robot clamps
  p.translateY = 127;
  TEST_ASSERT_TRUE(send(p, 1, 1020) == Outcome::Accepted);
  d = desired(c, 1020);
  TEST_ASSERT_EQUAL_INT8(-100, d.translateX);
  TEST_ASSERT_EQUAL_INT8(127, d.translateY);
}

void test_corrupt_or_foreign_datagrams_are_discarded_and_counted()
{
  TEST_ASSERT_TRUE(send(pad(), 0, 0) == Outcome::Accepted);
  gamepad_mapping::Desired before = desired(c, 0);
  protocol::Command p = pad();
  p.translateY = -50;
  p.clearCounter = 9;
  uint8_t buf[protocol::MAX_FRAME_SIZE + 1];

  // Eight in one window, the ninth in the next, so none is rate-limited.
  // Bad CRC.
  int n = protocol::encodeCommand(p, 1, buf);
  buf[n - 1] ^= 0x01;
  TEST_ASSERT_TRUE(accept(c, buf, n, HANDHELD, 20) == Outcome::Malformed);
  // Another protocol version (the CRC recomputed, so only the version is wrong).
  n = protocol::encodeCommand(p, 1, buf);
  buf[1] = protocol::VERSION - 1;
  uint16_t crc = protocol::crc16(buf + 1, n - 3);
  buf[n - 2] = (uint8_t)(crc & 0xFF);
  buf[n - 1] = (uint8_t)(crc >> 8);
  TEST_ASSERT_TRUE(accept(c, buf, n, HANDHELD, 21) == Outcome::Malformed);
  // Another frame type: a telemetry frame, and a calibration set.
  protocol::Telemetry t = {};
  n = protocol::encodeTelemetry(t, 1, buf);
  TEST_ASSERT_TRUE(accept(c, buf, n, HANDHELD, 22) == Outcome::Malformed);
  protocol::CalibrationSet set = {};
  n = protocol::encodeCalibrationSet(set, 1, buf);
  TEST_ASSERT_TRUE(accept(c, buf, n, HANDHELD, 23) == Outcome::Malformed);
  // 22 bytes: a valid frame plus one byte.
  n = protocol::encodeCommand(p, 1, buf);
  buf[n] = 0x00;
  TEST_ASSERT_TRUE(accept(c, buf, n + 1, HANDHELD, 24) == Outcome::Malformed);
  // A truncated frame, an empty datagram, none at all.
  TEST_ASSERT_TRUE(accept(c, buf, n - 1, HANDHELD, 25) == Outcome::Malformed);
  TEST_ASSERT_TRUE(accept(c, buf, 0, HANDHELD, 26) == Outcome::Malformed);
  TEST_ASSERT_TRUE(accept(c, nullptr, n, HANDHELD, 27) == Outcome::Malformed);
  // 21 bytes of garbage before a command frame's start: not a frame.
  for (int i = 0; i < protocol::COMMAND_FRAME_SIZE; i++)
    buf[i] = (uint8_t)(i == 0 ? 0x00 : 0xA5);
  TEST_ASSERT_TRUE(accept(c, buf, protocol::COMMAND_FRAME_SIZE, HANDHELD, 40) == Outcome::Malformed);

  TEST_ASSERT_EQUAL_UINT32(9, count(c, Outcome::Malformed));
  gamepad_mapping::Desired after = desired(c, 40);
  TEST_ASSERT_EQUAL_INT8(before.translateY, after.translateY);
  TEST_ASSERT_EQUAL_UINT8(before.clearCounter, after.clearCounter);
  // The session's sequence is untouched: the next frame is still newer.
  TEST_ASSERT_TRUE(send(p, 1, 41) == Outcome::Accepted);
}

void test_reordered_datagram_is_stale()
{
  protocol::Command p = pad();
  TEST_ASSERT_TRUE(send(p, 40, 0) == Outcome::Accepted);
  TEST_ASSERT_TRUE(send(p, 42, 20) == Outcome::Accepted);
  p.translateY = -90;
  p.clearCounter = 1; // a late frame changes nothing
  TEST_ASSERT_TRUE(send(p, 41, 25) == Outcome::Stale);
  TEST_ASSERT_TRUE(send(p, 42, 26) == Outcome::Stale); // repeated
  TEST_ASSERT_EQUAL_UINT32(2, count(c, Outcome::Stale));
  gamepad_mapping::Desired d = desired(c, 30);
  TEST_ASSERT_EQUAL_INT8(60, d.translateY);
  TEST_ASSERT_EQUAL_UINT8(0, d.clearCounter);
}

void test_stale_frames_do_not_refresh_liveness()
{
  TEST_ASSERT_TRUE(send(pad(), 10, 0) == Outcome::Accepted);
  for (uint32_t t = 20; t <= 140; t += 20)
    TEST_ASSERT_TRUE(send(pad(), 9, t) == Outcome::Stale);
  TEST_ASSERT_TRUE(live(c, 150));
  TEST_ASSERT_FALSE(live(c, 151));
}

void test_sequence_wraps_from_255_to_0()
{
  protocol::Command p = pad();
  TEST_ASSERT_TRUE(send(p, 254, 0) == Outcome::Accepted);
  TEST_ASSERT_TRUE(send(p, 255, 20) == Outcome::Accepted);
  p.clearCounter = 1;
  TEST_ASSERT_TRUE(send(p, 0, 40) == Outcome::Accepted);
  TEST_ASSERT_EQUAL_UINT8(1, desired(c, 40).clearCounter); // the same session
}

void test_sequence_window_edges()
{
  // d = 8 is newer; d = 9 is a jump; d = 127 a jump; d = 128 stale.
  TEST_ASSERT_TRUE(send(pad(), 0, 0) == Outcome::Accepted);
  TEST_ASSERT_TRUE(send(pad(), 8, 10) == Outcome::Accepted);
  TEST_ASSERT_TRUE(send(pad(), 8 + 128, 20) == Outcome::Stale);
  TEST_ASSERT_TRUE(send(pad(), 8 + 127, 30) == Outcome::Jump);
  TEST_ASSERT_FALSE(live(c, 30));
  setUp();
  TEST_ASSERT_TRUE(send(pad(), 0, 0) == Outcome::Accepted);
  TEST_ASSERT_TRUE(send(pad(), 9, 10) == Outcome::Jump);
  TEST_ASSERT_FALSE(live(c, 10));
}

// The spec's sequence jump, and a restarted app on the same port.
void test_jump_ends_the_session_and_the_next_starts_in_sleep()
{
  protocol::Command p = pad();
  p.clearCounter = 5;
  p.modeRequestCounter = 7;
  p.gestureRequestCounter = 3;
  uint8_t seq = 190;
  uint32_t now = 0;
  stream(p, seq, now, 3);
  p.modeRequestCounter = 8; // a Walk request
  stream(p, seq, now, 8);   // up to seq 200
  TEST_ASSERT_EQUAL_UINT8(201, seq);
  TEST_ASSERT_TRUE(desired(c, now - 20).mode == RequestedMode::Walk);
  gamepad_mapping::Desired before = desired(c, now - 20);

  // The app restarts on the same port: counters from zero, sequence 0, within 150 ms.
  protocol::Command restarted = pad();
  TEST_ASSERT_TRUE(send(restarted, 0, now) == Outcome::Jump);
  TEST_ASSERT_EQUAL_UINT32(1, count(c, Outcome::Jump));
  TEST_ASSERT_FALSE(live(c, now));
  gamepad_mapping::Desired d = desired(c, now);
  TEST_ASSERT_FALSE(d.controllerConnected);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);

  // The following valid frame starts a new session: no counter change, Sleep.
  TEST_ASSERT_TRUE(send(restarted, 1, now + 20) == Outcome::Accepted);
  TEST_ASSERT_TRUE(live(c, now + 20));
  assertNoCounterChange(before.clearCounter, before.modeRequestCounter, before.gestureRequestCounter, now + 20);
  TEST_ASSERT_TRUE(desired(c, now + 20).mode == RequestedMode::Sleep);
  // Sleep stays until the handheld's mode-request counter next changes.
  TEST_ASSERT_TRUE(send(restarted, 2, now + 40) == Outcome::Accepted);
  TEST_ASSERT_TRUE(desired(c, now + 40).mode == RequestedMode::Sleep);
  restarted.modeRequestCounter = 1;
  restarted.mode = (uint8_t)RequestedMode::Stand;
  TEST_ASSERT_TRUE(send(restarted, 3, now + 60) == Outcome::Accepted);
  d = desired(c, now + 60);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Stand);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)(before.modeRequestCounter + 1), d.modeRequestCounter);
}

void test_window_budget_and_the_next_window()
{
  // Eight datagrams in the window [100, 120); the ninth is rate-limited.
  for (int i = 0; i < WINDOW_BUDGET; i++)
  {
    TEST_ASSERT_FALSE(budgetSpent(c, 100));
    TEST_ASSERT_TRUE(send(pad(), (uint8_t)i, 100 + (uint32_t)i) == Outcome::Accepted);
  }
  TEST_ASSERT_TRUE(budgetSpent(c, 119));
  TEST_ASSERT_TRUE(send(pad(), 8, 119) == Outcome::RateLimited);
  TEST_ASSERT_EQUAL_UINT32(1, count(c, Outcome::RateLimited));
  // The first of the next window is accepted.
  TEST_ASSERT_FALSE(budgetSpent(c, 120));
  TEST_ASSERT_TRUE(send(pad(), 8, 120) == Outcome::Accepted);
}

void test_malformed_datagrams_spend_the_budget()
{
  uint8_t junk[4] = {1, 2, 3, 4};
  for (int i = 0; i < WINDOW_BUDGET; i++)
    TEST_ASSERT_TRUE(accept(c, junk, 4, OTHER, 0) == Outcome::Malformed);
  TEST_ASSERT_TRUE(send(pad(), 0, 0) == Outcome::RateLimited);
}

// The spec's flood: 200 valid datagrams in one window, then silence.
void test_flood_takes_eight_and_lapses_after_the_last()
{
  int accepted = 0, limited = 0;
  for (int i = 0; i < 200; i++)
  {
    Outcome o = send(pad(), (uint8_t)(1 + i % 8), 40 + (uint32_t)(i / 10)); // within [40, 60)
    accepted += o == Outcome::Accepted;
    limited += o == Outcome::RateLimited;
  }
  TEST_ASSERT_LESS_OR_EQUAL_INT(WINDOW_BUDGET, accepted);
  TEST_ASSERT_GREATER_OR_EQUAL_INT(1, accepted);
  TEST_ASSERT_EQUAL_INT(200 - WINDOW_BUDGET, limited);
  // The last accepted datagram came at 40 at the latest + 19: the controller
  // is gone 151 ms after it.
  TEST_ASSERT_FALSE(desired(c, 59 + LIVENESS_MS + 1).controllerConnected);
}

// --- one handheld at a time ------------------------------------------------------

void test_second_sender_is_ignored_while_live()
{
  uint8_t seq = 0;
  uint32_t now = 0;
  stream(pad(), seq, now, 3);
  protocol::Command other = pad();
  other.translateY = -100;
  other.clearCounter = 1;
  for (int i = 0; i < 5; i++)
  {
    TEST_ASSERT_TRUE(send(pad(), seq, now) == Outcome::Accepted);
    TEST_ASSERT_TRUE(send(other, (uint8_t)(seq + i), now + 5, OTHER) == Outcome::OtherSender);
    seq++;
    now += 20;
  }
  TEST_ASSERT_EQUAL_UINT32(5, count(c, Outcome::OtherSender));
  gamepad_mapping::Desired d = desired(c, now);
  TEST_ASSERT_EQUAL_INT8(60, d.translateY);
  TEST_ASSERT_EQUAL_UINT8(0, d.clearCounter);
  // Another port on the same address is another sender too.
  Sender otherPort = {HANDHELD.address, (uint16_t)(HANDHELD.port + 1)};
  TEST_ASSERT_TRUE(send(other, seq, now, otherPort) == Outcome::OtherSender);
}

void test_handover_after_a_lapse()
{
  uint8_t seq = 0;
  uint32_t now = 0;
  stream(pad(), seq, now, 3); // last at 40
  protocol::Command other = pad();
  other.translateY = -100;
  TEST_ASSERT_TRUE(send(other, 77, 40 + LIVENESS_MS, OTHER) == Outcome::OtherSender);
  TEST_ASSERT_TRUE(send(other, 78, 40 + LIVENESS_MS + 1, OTHER) == Outcome::Accepted);
  TEST_ASSERT_EQUAL_INT8(-100, desired(c, 200).translateY);
  // Now the first handheld is the other sender.
  TEST_ASSERT_TRUE(send(pad(), seq, 200) == Outcome::OtherSender);
}

// --- liveness ----------------------------------------------------------------------

void test_handheld_goes_quiet()
{
  TEST_ASSERT_TRUE(send(pad(), 0, 1000) == Outcome::Accepted);
  TEST_ASSERT_TRUE(desired(c, 1000 + LIVENESS_MS).controllerConnected);
  gamepad_mapping::Desired d = desired(c, 1000 + LIVENESS_MS + 1);
  TEST_ASSERT_FALSE(d.controllerConnected);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_INT8(0, d.translateX);
  TEST_ASSERT_EQUAL_INT8(0, d.translateY);
  TEST_ASSERT_EQUAL_INT8(0, d.rotate);
  TEST_ASSERT_EQUAL_UINT8(0, d.height);
  TEST_ASSERT_EQUAL_UINT8(DEFAULT_SPEED, d.speed);
  TEST_ASSERT_EQUAL_UINT8(0, d.gait);
  TEST_ASSERT_TRUE(d.stride == StrideMode::Dynamic);
}

void test_six_losses_stay_live()
{
  protocol::Command p = pad();
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  // Six frames lost: the next accepted one is 140 ms and seven numbers later.
  TEST_ASSERT_TRUE(desired(c, 139).controllerConnected);
  p.clearCounter = 1;
  TEST_ASSERT_TRUE(send(p, 7, 140) == Outcome::Accepted);
  gamepad_mapping::Desired d = desired(c, 140);
  TEST_ASSERT_TRUE(d.controllerConnected);
  TEST_ASSERT_EQUAL_UINT8(1, d.clearCounter); // still the same session
}

void test_149_ms_gap_stays_in_the_session()
{
  protocol::Command p = pad();
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  uint32_t e = epoch(c, 0);
  p.clearCounter = 1;
  TEST_ASSERT_TRUE(send(p, 1, 149) == Outcome::Accepted);
  TEST_ASSERT_EQUAL_UINT32(e, epoch(c, 149));
  TEST_ASSERT_EQUAL_UINT8(1, desired(c, 149).clearCounter);
}

void test_151_ms_gap_starts_a_new_session()
{
  protocol::Command p = pad();
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  uint32_t e = epoch(c, 0);
  p.clearCounter = 1;
  p.modeRequestCounter = 1;
  TEST_ASSERT_TRUE(send(p, 1, 151) == Outcome::Accepted);
  TEST_ASSERT_NOT_EQUAL(e, epoch(c, 151));
  assertNoCounterChange(0, 0, 0, 151); // only the baselines of a new session
  TEST_ASSERT_TRUE(desired(c, 151).mode == RequestedMode::Sleep);
}

void test_handheld_releases_control()
{
  protocol::Command p = pad();
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  TEST_ASSERT_TRUE(state(c, 0) == State::InControl);
  p.flags = 0;
  TEST_ASSERT_TRUE(send(p, 1, 20) == Outcome::Accepted);
  TEST_ASSERT_TRUE(live(c, 20));
  TEST_ASSERT_FALSE(desired(c, 20).controllerConnected);
  TEST_ASSERT_TRUE(state(c, 20) == State::Idle);
  TEST_ASSERT_TRUE(state(c, 20 + LIVENESS_MS + 1) == State::None);
}

void test_handheld_returns_after_a_lapse_in_sleep()
{
  protocol::Command p = pad();
  p.modeRequestCounter = 4;
  uint8_t seq = 0;
  uint32_t now = 0;
  stream(p, seq, now, 2);
  p.modeRequestCounter = 5; // Walk
  stream(p, seq, now, 1);
  TEST_ASSERT_TRUE(desired(c, now).mode == RequestedMode::Walk);
  now += 500; // a lapse
  TEST_ASSERT_TRUE(desired(c, now).mode == RequestedMode::Sleep);
  // Back, its mode still Walk: Sleep until the mode-request counter changes.
  stream(p, seq, now, 5);
  TEST_ASSERT_TRUE(desired(c, now).mode == RequestedMode::Sleep);
  TEST_ASSERT_EQUAL_UINT8(1, desired(c, now).modeRequestCounter);
  p.modeRequestCounter = 6;
  stream(p, seq, now, 1);
  TEST_ASSERT_TRUE(desired(c, now).mode == RequestedMode::Walk);
  TEST_ASSERT_EQUAL_UINT8(2, desired(c, now).modeRequestCounter);
}

// --- counter translation -----------------------------------------------------------

void test_clear_mode_and_gesture_requests_advance_by_one()
{
  protocol::Command p = pad();
  p.clearCounter = 3;
  p.modeRequestCounter = 10;
  p.gestureRequestCounter = 20;
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  assertNoCounterChange(0, 0, 0, 0);

  p.clearCounter = 4; // the spec's clear
  TEST_ASSERT_TRUE(send(p, 1, 20) == Outcome::Accepted);
  assertNoCounterChange(1, 0, 0, 20);

  p.modeRequestCounter = 11;
  p.mode = (uint8_t)RequestedMode::Stand; // the spec's mode request
  TEST_ASSERT_TRUE(send(p, 2, 40) == Outcome::Accepted);
  gamepad_mapping::Desired d = desired(c, 40);
  TEST_ASSERT_TRUE(d.mode == RequestedMode::Stand);
  TEST_ASSERT_EQUAL_UINT8(1, d.modeRequestCounter);

  // Between mode requests the frame's mode does not change the request.
  p.mode = (uint8_t)RequestedMode::Walk;
  TEST_ASSERT_TRUE(send(p, 3, 60) == Outcome::Accepted);
  TEST_ASSERT_TRUE(desired(c, 60).mode == RequestedMode::Stand);

  p.gestureRequestCounter = 21;
  p.gesture = protocol::GESTURE_WIGGLE;
  TEST_ASSERT_TRUE(send(p, 4, 80) == Outcome::Accepted);
  d = desired(c, 80);
  TEST_ASSERT_EQUAL_UINT8(1, d.gestureRequestCounter);
  TEST_ASSERT_EQUAL_UINT8(protocol::GESTURE_WIGGLE, d.gesture);

  // A change of several (lost frames) is one request; so is a wrap.
  p.clearCounter = 9;
  TEST_ASSERT_TRUE(send(p, 5, 100) == Outcome::Accepted);
  TEST_ASSERT_EQUAL_UINT8(2, desired(c, 100).clearCounter);
  p.clearCounter = 255;
  TEST_ASSERT_TRUE(send(p, 6, 120) == Outcome::Accepted);
  p.clearCounter = 0;
  TEST_ASSERT_TRUE(send(p, 7, 140) == Outcome::Accepted);
  TEST_ASSERT_EQUAL_UINT8(4, desired(c, 140).clearCounter);
  // The bridge's counters wrap at 256 like any other.
  c.clearCounter = 255;
  p.clearCounter = 1;
  TEST_ASSERT_TRUE(send(p, 8, 160) == Outcome::Accepted);
  TEST_ASSERT_EQUAL_UINT8(0, desired(c, 160).clearCounter);
}

// The spec's restarted handheld app, from a new source port: its counters
// start from zero while the old session is live, then after it lapses.
void test_restarted_app_from_a_new_port()
{
  protocol::Command p = pad();
  p.clearCounter = 12;
  p.modeRequestCounter = 30;
  p.gestureRequestCounter = 40;
  uint8_t seq = 100;
  uint32_t now = 0;
  stream(p, seq, now, 2);
  p.modeRequestCounter = 31;
  stream(p, seq, now, 1); // one request: the bridge's mode counter is 1
  uint32_t last = now - 20;

  Sender newPort = {HANDHELD.address, 50001};
  protocol::Command restarted = pad(); // counters from zero
  uint8_t s = 0;
  for (uint32_t t = last + 20; t <= last + LIVENESS_MS; t += 20)
    TEST_ASSERT_TRUE(send(restarted, s++, t, newPort) == Outcome::OtherSender);
  assertNoCounterChange(0, 1, 0, last + LIVENESS_MS);
  // After the lapse its frames start a session with fresh baselines.
  for (uint32_t t = last + LIVENESS_MS + 10; t <= last + 400; t += 20)
    TEST_ASSERT_TRUE(send(restarted, s++, t, newPort) == Outcome::Accepted);
  assertNoCounterChange(0, 1, 0, last + 400);
  TEST_ASSERT_TRUE(desired(c, last + 400).mode == RequestedMode::Sleep);
}

// The spec's restarted app after a lapse, on the same port.
void test_restarted_app_after_a_lapse_on_the_same_port()
{
  protocol::Command p = pad();
  p.clearCounter = 3;
  p.modeRequestCounter = 3;
  p.gestureRequestCounter = 3;
  TEST_ASSERT_TRUE(send(p, 50, 0) == Outcome::Accepted);
  protocol::Command restarted = pad(); // zeros: each counter differs from 3
  TEST_ASSERT_TRUE(send(restarted, 51, 300) == Outcome::Accepted);
  TEST_ASSERT_TRUE(send(restarted, 52, 320) == Outcome::Accepted);
  assertNoCounterChange(0, 0, 0, 320);
}

// --- emergency stop --------------------------------------------------------------

void test_handheld_e_stop_is_carried()
{
  protocol::Command p = pad();
  p.flags |= protocol::FLAG_EMERGENCY_STOP;
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  TEST_ASSERT_TRUE(emergencyStop(c));
  protocol::Command out = gamepad_mapping::toCommand(desired(c, 0), false || emergencyStop(c));
  TEST_ASSERT_TRUE(out.flags & protocol::FLAG_EMERGENCY_STOP);
}

void test_bridge_button_with_the_handheld_flag_clear()
{
  TEST_ASSERT_TRUE(send(pad(), 0, 0) == Outcome::Accepted);
  TEST_ASSERT_FALSE(emergencyStop(c));
  bool button = true; // the link task ORs its GPIO35 toggle in
  protocol::Command out = gamepad_mapping::toCommand(desired(c, 0), button || emergencyStop(c));
  TEST_ASSERT_TRUE(out.flags & protocol::FLAG_EMERGENCY_STOP);
}

void test_e_stop_is_kept_through_a_lapse_until_a_frame_clears_it()
{
  protocol::Command p = pad();
  p.flags |= protocol::FLAG_EMERGENCY_STOP;
  TEST_ASSERT_TRUE(send(p, 0, 0) == Outcome::Accepted);
  TEST_ASSERT_FALSE(live(c, 1000));
  TEST_ASSERT_TRUE(emergencyStop(c));
  // An accepted frame without the flag clears it (here a new session's first).
  TEST_ASSERT_TRUE(send(pad(), 1, 1000) == Outcome::Accepted);
  TEST_ASSERT_FALSE(emergencyStop(c));
  p.flags |= protocol::FLAG_EMERGENCY_STOP;
  TEST_ASSERT_TRUE(send(p, 2, 1020) == Outcome::Accepted);
  // Rejected datagrams do not clear it: a stale one, a corrupt one.
  TEST_ASSERT_TRUE(send(pad(), 2, 1040) == Outcome::Stale);
  TEST_ASSERT_TRUE(emergencyStop(c));
  uint8_t bad[protocol::COMMAND_FRAME_SIZE];
  protocol::encodeCommand(pad(), 3, bad);
  bad[10] ^= 0xFF;
  TEST_ASSERT_TRUE(accept(c, bad, sizeof(bad), HANDHELD, 1060) == Outcome::Malformed);
  TEST_ASSERT_TRUE(emergencyStop(c));
}

// --- relay target and session epoch ------------------------------------------------

void test_relay_target_follows_the_session()
{
  Sender to = {};
  TEST_ASSERT_FALSE(relayTarget(c, 0, to)); // before any session
  uint32_t e0 = epoch(c, 0);
  TEST_ASSERT_TRUE(send(pad(), 0, 10) == Outcome::Accepted);
  uint32_t e1 = epoch(c, 10);
  TEST_ASSERT_NOT_EQUAL(e0, e1);
  TEST_ASSERT_TRUE(relayTarget(c, 10, to)); // during
  TEST_ASSERT_EQUAL_UINT32(HANDHELD.address, to.address);
  TEST_ASSERT_EQUAL_UINT16(HANDHELD.port, to.port);
  TEST_ASSERT_TRUE(relayTarget(c, 10 + LIVENESS_MS, to));
  TEST_ASSERT_FALSE(relayTarget(c, 10 + LIVENESS_MS + 1, to)); // after
  uint32_t e2 = epoch(c, 10 + LIVENESS_MS + 1);
  TEST_ASSERT_NOT_EQUAL(e1, e2);
  // A handover: the other sender, in a new epoch.
  TEST_ASSERT_TRUE(send(pad(), 5, 400, OTHER) == Outcome::Accepted);
  TEST_ASSERT_TRUE(relayTarget(c, 400, to));
  TEST_ASSERT_EQUAL_UINT32(OTHER.address, to.address);
  TEST_ASSERT_NOT_EQUAL(e2, epoch(c, 400));
}

void test_jump_ends_the_relay_at_once()
{
  TEST_ASSERT_TRUE(send(pad(), 0, 0) == Outcome::Accepted);
  uint32_t e = epoch(c, 0);
  TEST_ASSERT_TRUE(send(pad(), 100, 20) == Outcome::Jump);
  Sender to = {};
  TEST_ASSERT_FALSE(relayTarget(c, 20, to));
  TEST_ASSERT_NOT_EQUAL(e, epoch(c, 20));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_valid_frame_is_the_newest_desired_state);
  RUN_TEST(test_corrupt_or_foreign_datagrams_are_discarded_and_counted);
  RUN_TEST(test_reordered_datagram_is_stale);
  RUN_TEST(test_stale_frames_do_not_refresh_liveness);
  RUN_TEST(test_sequence_wraps_from_255_to_0);
  RUN_TEST(test_sequence_window_edges);
  RUN_TEST(test_jump_ends_the_session_and_the_next_starts_in_sleep);
  RUN_TEST(test_window_budget_and_the_next_window);
  RUN_TEST(test_malformed_datagrams_spend_the_budget);
  RUN_TEST(test_flood_takes_eight_and_lapses_after_the_last);
  RUN_TEST(test_second_sender_is_ignored_while_live);
  RUN_TEST(test_handover_after_a_lapse);
  RUN_TEST(test_handheld_goes_quiet);
  RUN_TEST(test_six_losses_stay_live);
  RUN_TEST(test_149_ms_gap_stays_in_the_session);
  RUN_TEST(test_151_ms_gap_starts_a_new_session);
  RUN_TEST(test_handheld_releases_control);
  RUN_TEST(test_handheld_returns_after_a_lapse_in_sleep);
  RUN_TEST(test_clear_mode_and_gesture_requests_advance_by_one);
  RUN_TEST(test_restarted_app_from_a_new_port);
  RUN_TEST(test_restarted_app_after_a_lapse_on_the_same_port);
  RUN_TEST(test_handheld_e_stop_is_carried);
  RUN_TEST(test_bridge_button_with_the_handheld_flag_clear);
  RUN_TEST(test_e_stop_is_kept_through_a_lapse_until_a_frame_clears_it);
  RUN_TEST(test_relay_target_follows_the_session);
  RUN_TEST(test_jump_ends_the_relay_at_once);
  return UNITY_END();
}
