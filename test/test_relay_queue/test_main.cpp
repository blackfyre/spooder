// The relay queue (WiFi drive): the robot's frames, byte for byte and in
// order, in a ring of 8 that drops and counts rather than waits.
#include <unity.h>
#include <string.h>
#include "wifi_drive/relay_queue.h"
#include "../golden_frames.h"

static relay_queue::Queue q;
static golden_frames::Set golden;

void setUp()
{
  q = relay_queue::Queue();
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

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_golden_frames_come_out_byte_for_byte_in_order);
  RUN_TEST(test_ninth_frame_is_dropped_and_counted);
  RUN_TEST(test_ring_wraps_around);
  RUN_TEST(test_clear_leaves_nothing_for_the_next_take);
  RUN_TEST(test_out_of_range_lengths_store_nothing);
  return UNITY_END();
}
