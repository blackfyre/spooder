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
  relayGolden(0, 3, 110);
  // No call between: the lapse and B's lock fall between two link periods.
  TEST_ASSERT_TRUE(send(7, 100 + handheld_controller::LIVENESS_MS + 1, B) == handheld_controller::Outcome::Accepted);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to = {};
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 260, out, to));
  relayGolden(3, 1, 262);
  int n = relay_queue::takeFor(q, hc, 264, out, to);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(golden.frames[3].bytes, out, n);
  TEST_ASSERT_EQUAL_UINT16(B.port, to.port);
  TEST_ASSERT_EQUAL_INT(0, relay_queue::takeFor(q, hc, 264, out, to));
}

// The same with a sequence jump ending A's session, A's next frame starting
// a new one at once: none of the frames of the old session go out.
void test_session_ended_by_a_jump_sends_none_of_the_waiting_frames()
{
  TEST_ASSERT_TRUE(golden_frames::load(__FILE__, golden));
  TEST_ASSERT_TRUE(send(200, 100, A) == handheld_controller::Outcome::Accepted);
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
  return UNITY_END();
}
