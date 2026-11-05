// The relay queue (WiFi drive): the robot's frames, byte for byte and in
// order, in a ring of 8 that drops and counts rather than waits.
#include <unity.h>
#include <string.h>
#include "wifi_drive/relay_queue.h"
#include "../golden_frames.h"

static relay_queue::Queue q;
static golden_frames::Set golden;
static handheld_controller::Controller hc;

static const handheld_controller::Sender A = {0x0204a8c0, 40001}; // 192.168.4.2
static const handheld_controller::Sender B = {0x0304a8c0, 40002}; // 192.168.4.3

void setUp()
{
  q = relay_queue::Queue();
  hc = handheld_controller::Controller();
}
void tearDown() {}

void test_golden_frames_come_out_byte_for_byte_in_order()
{
  TEST_ASSERT_TRUE_MESSAGE(golden_frames::load(__FILE__, golden), "cannot read test/golden/protocol_v3.txt");
  // Every frame type is there, the largest (a calibration report) too.
  bool types[7] = {};
  for (int i = 0; i < golden.count; i++)
    if (golden.frames[i].bytes[2] < 7)
      types[golden.frames[i].bytes[2]] = true;
  for (int t = 1; t <= 6; t++)
    TEST_ASSERT_TRUE(types[t]);

  uint8_t out[protocol::MAX_FRAME_SIZE];
  for (int start = 0; start < golden.count; start += relay_queue::CAPACITY)
  {
    int end = start + relay_queue::CAPACITY < golden.count ? start + relay_queue::CAPACITY : golden.count;
    for (int i = start; i < end; i++)
      TEST_ASSERT_TRUE(relay_queue::push(q, golden.frames[i].bytes, golden.frames[i].length));
    for (int i = start; i < end; i++)
    {
      const golden_frames::Frame &g = golden.frames[i];
      memset(out, 0, sizeof(out));
      int n = relay_queue::take(q, out);
      TEST_ASSERT_EQUAL_INT_MESSAGE(g.length, n, g.name);
      TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(g.bytes, out, n, g.name);
    }
    TEST_ASSERT_EQUAL_INT(0, relay_queue::take(q, out));
  }
  TEST_ASSERT_EQUAL_UINT32(0, relay_queue::dropped(q));
}

void test_ninth_frame_is_dropped_and_counted()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(golden.count > relay_queue::CAPACITY);
  for (int i = 0; i < relay_queue::CAPACITY; i++)
    TEST_ASSERT_TRUE(relay_queue::push(q, golden.frames[i].bytes, golden.frames[i].length));
  const golden_frames::Frame &ninth = golden.frames[relay_queue::CAPACITY];
  TEST_ASSERT_FALSE(relay_queue::push(q, ninth.bytes, ninth.length));
  TEST_ASSERT_EQUAL_UINT32(1, relay_queue::dropped(q));
  TEST_ASSERT_EQUAL_INT(relay_queue::CAPACITY, relay_queue::size(q));
  // The first eight survive, in order.
  uint8_t out[protocol::MAX_FRAME_SIZE];
  for (int i = 0; i < relay_queue::CAPACITY; i++)
  {
    int n = relay_queue::take(q, out);
    TEST_ASSERT_EQUAL_INT(golden.frames[i].length, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(golden.frames[i].bytes, out, n);
  }
  TEST_ASSERT_EQUAL_INT(0, relay_queue::take(q, out));
}

void test_ring_wraps_around()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  uint8_t out[protocol::MAX_FRAME_SIZE];
  // Push and take past the ring's end several times.
  for (int i = 0; i < 3 * relay_queue::CAPACITY; i++)
  {
    const golden_frames::Frame &a = golden.frames[i % golden.count];
    const golden_frames::Frame &b = golden.frames[(i + 1) % golden.count];
    TEST_ASSERT_TRUE(relay_queue::push(q, a.bytes, a.length));
    TEST_ASSERT_TRUE(relay_queue::push(q, b.bytes, b.length));
    int n = relay_queue::take(q, out);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a.bytes, out, n);
    n = relay_queue::take(q, out);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(b.bytes, out, n);
  }
  TEST_ASSERT_EQUAL_INT(0, relay_queue::size(q));
}

void test_clear_leaves_nothing_for_the_next_take()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  for (int i = 0; i < 5; i++)
    relay_queue::push(q, golden.frames[i].bytes, golden.frames[i].length);
  relay_queue::clear(q);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  TEST_ASSERT_EQUAL_INT(0, relay_queue::size(q));
  TEST_ASSERT_EQUAL_INT(0, relay_queue::take(q, out));
  TEST_ASSERT_EQUAL_UINT32(0, relay_queue::dropped(q));
  // The ring works on after a clear.
  TEST_ASSERT_TRUE(relay_queue::push(q, golden.frames[6].bytes, golden.frames[6].length));
  int n = relay_queue::take(q, out);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(golden.frames[6].bytes, out, n);
}

void test_out_of_range_lengths_store_nothing()
{
  uint8_t bytes[protocol::MAX_FRAME_SIZE + 1] = {};
  TEST_ASSERT_FALSE(relay_queue::push(q, bytes, 0));
  TEST_ASSERT_FALSE(relay_queue::push(q, bytes, -1));
  TEST_ASSERT_FALSE(relay_queue::push(q, bytes, protocol::MAX_FRAME_SIZE + 1));
  TEST_ASSERT_FALSE(relay_queue::push(q, nullptr, 10));
  TEST_ASSERT_EQUAL_INT(0, relay_queue::size(q));
}

// --- the relay tied to the handheld's sessions ------------------------------------

static handheld_controller::Outcome send(uint8_t seq, uint32_t nowMs, handheld_controller::Sender from)
{
  protocol::Command p = {};
  p.mode = (uint8_t)RequestedMode::Sleep;
  p.flags = protocol::FLAG_CONTROLLER_CONNECTED;
  uint8_t buf[protocol::COMMAND_FRAME_SIZE];
  int n = protocol::encodeCommand(p, seq, buf);
  return handheld_controller::accept(hc, buf, n, from, nowMs);
}

// One link-task poll of the robot's bytes, starting at nowMs, that leaves
// the decoder empty.
static void poll(uint32_t nowMs)
{
  relay_queue::beginPoll(q, hc, nowMs);
  relay_queue::endPoll(q, true);
}

// One link-task poll as the bridge's receive() runs it: drain the bytes
// into the decoder, relay each frame, then the idle check and the poll's
// end. Returns the frames relayed.
static int receivePoll(protocol::Decoder &dec, const uint8_t *bytes, int length, uint32_t nowMs, bool idle)
{
  relay_queue::beginPoll(q, hc, nowMs);
  int relayed = 0;
  protocol::Frame f;
  for (int i = 0; i < length; i++)
  {
    dec.push(bytes[i]);
    while (dec.next(f))
      relayed += relay_queue::relayFor(q, hc, nowMs, dec.frameBytes(), dec.frameLength());
  }
  if (idle)
  {
    dec.idle();
    while (dec.next(f))
      relayed += relay_queue::relayFor(q, hc, nowMs, dec.frameBytes(), dec.frameLength());
  }
  relay_queue::endPoll(q, dec.pending() == 0);
  return relayed;
}

// Queues golden frames [first, first + count) for the handheld at nowMs.
static void relayGolden(int first, int count, uint32_t nowMs)
{
  for (int i = first; i < first + count; i++)
    TEST_ASSERT_TRUE(relay_queue::relayFor(q, hc, nowMs, golden.frames[i].bytes, golden.frames[i].length));
}

void test_nothing_is_relayed_without_a_session()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_FALSE(relay_queue::relayFor(q, hc, 100, golden.frames[0].bytes, golden.frames[0].length));
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 100, out, to));
}

void test_frames_go_to_the_live_sender_in_order()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(send(1, 100, A) == handheld_controller::Outcome::Accepted);
  poll(100);
  poll(105);
  relayGolden(0, 3, 105);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  for (int i = 0; i < 3; i++)
  {
    handheld_controller::Sender to = {};
    int n = relay_queue::takeFor(q, hc, 110, out, to);
    TEST_ASSERT_EQUAL_INT(golden.frames[i].length, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(golden.frames[i].bytes, out, n);
    TEST_ASSERT_EQUAL_UINT32(A.address, to.address);
    TEST_ASSERT_EQUAL_UINT16(A.port, to.port);
  }
}

// A's frames wait; A lapses and B locks between two takes: B gets none of
// A's frames, only those queued in its own session.
void test_handover_after_a_lapse_sends_none_of_the_waiting_frames()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(send(1, 100, A) == handheld_controller::Outcome::Accepted);
  poll(100);
  poll(110);
  relayGolden(0, 3, 110);
  // No call between: the lapse and B's lock fall between two link periods.
  TEST_ASSERT_TRUE(send(7, 100 + handheld_controller::LIVENESS_MS + 1, B) == handheld_controller::Outcome::Accepted);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 260, out, to));
  poll(260);
  poll(280);
  relayGolden(3, 1, 282);
  int n = relay_queue::takeFor(q, hc, 284, out, to);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(golden.frames[3].bytes, out, n);
  TEST_ASSERT_EQUAL_UINT16(B.port, to.port);
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 284, out, to));
}

// Bytes A's session left in the UART: the robot's frame arrives after one
// poll, A lapses and B locks before the next, which decodes it. It may have
// arrived during A's session, so B never gets it; the poll after relays.
void test_frame_decoded_in_the_poll_of_a_handover_is_dropped()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(send(1, 0, A) == handheld_controller::Outcome::Accepted);
  poll(0);
  poll(140); // the frame arrives at 149, after this poll's drain
  TEST_ASSERT_TRUE(send(7, handheld_controller::LIVENESS_MS + 2, B) == handheld_controller::Outcome::Accepted);
  relay_queue::beginPoll(q, hc, 160);
  TEST_ASSERT_FALSE(relay_queue::relayFor(q, hc, 160, golden.frames[0].bytes, golden.frames[0].length));
  relay_queue::endPoll(q, true);
  TEST_ASSERT_EQUAL_UINT32(1, relay_queue::dropped(q));
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 162, out, to));
  poll(180);
  relayGolden(1, 1, 180);
  int n = relay_queue::takeFor(q, hc, 182, out, to);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(golden.frames[1].bytes, out, n);
  TEST_ASSERT_EQUAL_UINT16(B.port, to.port);
}

// Bytes the decoder holds across polls: during A's session the robot's
// bytes are a false start (a calibration report's header, never finished)
// and then a whole telemetry frame. B locks; the next poll ends with the
// decoder still holding them, the one after abandons the false start at
// idle and finds A's telemetry: B gets none of it. A frame that arrives
// after the decoder was empty goes to B.
void test_frame_held_by_the_decoder_across_a_handover_is_dropped()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  const golden_frames::Frame *report = nullptr, *telemetry = nullptr;
  for (int i = 0; i < golden.count; i++)
  {
    if (golden.frames[i].bytes[2] == (uint8_t)protocol::FrameType::CalibrationReport && report == nullptr)
      report = &golden.frames[i];
    if (golden.frames[i].bytes[2] == (uint8_t)protocol::FrameType::Telemetry && telemetry == nullptr)
      telemetry = &golden.frames[i];
  }
  TEST_ASSERT_NOT_NULL(report);
  TEST_ASSERT_NOT_NULL(telemetry);
  uint8_t held[protocol::MAX_FRAME_SIZE];
  int n = 0;
  for (int i = 0; i < protocol::HEADER_SIZE; i++)
    held[n++] = report->bytes[i];
  for (int i = 0; i < telemetry->length; i++)
    held[n++] = telemetry->bytes[i];
  TEST_ASSERT_TRUE(n < report->length); // the false start stays incomplete

  protocol::Decoder dec;
  TEST_ASSERT_TRUE(send(1, 0, A) == handheld_controller::Outcome::Accepted);
  TEST_ASSERT_EQUAL_INT(0, receivePoll(dec, nullptr, 0, 0, false));
  TEST_ASSERT_EQUAL_INT(0, receivePoll(dec, held, n, 20, false)); // A's bytes, held
  TEST_ASSERT_TRUE(dec.pending() > 0);
  TEST_ASSERT_TRUE(send(7, handheld_controller::LIVENESS_MS + 2, B) == handheld_controller::Outcome::Accepted);
  TEST_ASSERT_EQUAL_INT(0, receivePoll(dec, nullptr, 0, 160, false));
  TEST_ASSERT_EQUAL_INT(0, receivePoll(dec, nullptr, 0, 180, true)); // A's telemetry decoded, dropped
  TEST_ASSERT_EQUAL_UINT32(1, relay_queue::dropped(q));
  TEST_ASSERT_EQUAL_INT(0, relay_queue::size(q));
  TEST_ASSERT_EQUAL_INT(1, receivePoll(dec, telemetry->bytes, telemetry->length, 200, false));
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  int len = relay_queue::takeFor(q, hc, 202, out, to);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(telemetry->bytes, out, len);
  TEST_ASSERT_EQUAL_UINT16(B.port, to.port);
}

// A session that starts or ends during a poll: that poll's later frames are
// not relayed either.
void test_session_change_during_a_poll_drops_its_frames()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  poll(0);
  TEST_ASSERT_TRUE(send(1, 5, A) == handheld_controller::Outcome::Accepted);
  TEST_ASSERT_FALSE(relay_queue::relayFor(q, hc, 6, golden.frames[0].bytes, golden.frames[0].length));
  TEST_ASSERT_EQUAL_INT(0, relay_queue::size(q));
}

// The same with a sequence jump ending A's session, A's next frame starting
// a new one at once: none of the frames of the old session go out.
void test_session_ended_by_a_jump_sends_none_of_the_waiting_frames()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(send(200, 100, A) == handheld_controller::Outcome::Accepted);
  poll(100);
  poll(105);
  relayGolden(0, 2, 105);
  TEST_ASSERT_TRUE(send(0, 110, A) == handheld_controller::Outcome::Jump);
  TEST_ASSERT_TRUE(send(1, 112, A) == handheld_controller::Outcome::Accepted);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 114, out, to));
}

// A lapse with no new session drops the waiting frames.
void test_lapse_drops_the_waiting_frames()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(send(1, 100, A) == handheld_controller::Outcome::Accepted);
  poll(100);
  poll(105);
  relayGolden(0, 2, 105);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 100 + handheld_controller::LIVENESS_MS + 1, out, to));
  TEST_ASSERT_EQUAL_INT(0, relay_queue::size(q));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_golden_frames_come_out_byte_for_byte_in_order);
  RUN_TEST(test_ninth_frame_is_dropped_and_counted);
  RUN_TEST(test_ring_wraps_around);
  RUN_TEST(test_clear_leaves_nothing_for_the_next_take);
  RUN_TEST(test_out_of_range_lengths_store_nothing);
  RUN_TEST(test_nothing_is_relayed_without_a_session);
  RUN_TEST(test_frames_go_to_the_live_sender_in_order);
  RUN_TEST(test_handover_after_a_lapse_sends_none_of_the_waiting_frames);
  RUN_TEST(test_session_ended_by_a_jump_sends_none_of_the_waiting_frames);
  RUN_TEST(test_lapse_drops_the_waiting_frames);
  RUN_TEST(test_frame_decoded_in_the_poll_of_a_handover_is_dropped);
  RUN_TEST(test_session_change_during_a_poll_drops_its_frames);
  RUN_TEST(test_frame_held_by_the_decoder_across_a_handover_is_dropped);
  return UNITY_END();
}
