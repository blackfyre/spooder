#include "relay_queue.h"

namespace relay_queue
{

bool push(Queue &q, const uint8_t *bytes, int length)
{
  if (bytes == nullptr || length <= 0 || length > protocol::MAX_FRAME_SIZE)
    return false;
  if (q.count == CAPACITY)
  {
    q.dropped++;
    return false;
  }
  Entry &e = q.entries[(q.head + q.count) % CAPACITY];
  for (int i = 0; i < length; i++)
    e.bytes[i] = bytes[i];
  e.length = length;
  q.count++;
  return true;
}

int take(Queue &q, uint8_t *out)
{
  if (q.count == 0)
    return 0;
  const Entry &e = q.entries[q.head];
  for (int i = 0; i < e.length; i++)
    out[i] = e.bytes[i];
  q.head = (q.head + 1) % CAPACITY;
  q.count--;
  return e.length;
}

void clear(Queue &q)
{
  q.head = 0;
  q.count = 0;
}

// A change of session empties the queue.
static void sync(Queue &q, handheld_controller::Controller &c, uint32_t nowMs)
{
  uint32_t e = handheld_controller::epoch(c, nowMs);
  if (e != q.epoch)
  {
    clear(q);
    q.epoch = e;
  }
}

void beginPoll(Queue &q, handheld_controller::Controller &c, uint32_t nowMs)
{
  uint32_t e = handheld_controller::epoch(c, nowMs);
  if (e != q.pollEpoch)
    q.clean = false;
  q.pollEpoch = e;
}

void endPoll(Queue &q, bool decoderEmpty)
{
  // The UART was drained after this poll began, so with the decoder empty
  // too every byte still to come arrived after the poll began.
  if (decoderEmpty)
    q.clean = true;
}

bool relayFor(Queue &q, handheld_controller::Controller &c, uint32_t nowMs, const uint8_t *bytes, int length)
{
  sync(q, c, nowMs);
  if (!handheld_controller::live(c, nowMs))
    return false;
  // The bytes may have arrived before the session that is live now began
  // (or the session changed during the poll).
  if (!q.clean || q.epoch != q.pollEpoch)
  {
    q.dropped++;
    return false;
  }
  return push(q, bytes, length);
}

int takeFor(Queue &q, handheld_controller::Controller &c, uint32_t nowMs, uint8_t *out, handheld_controller::Sender &to)
{
  sync(q, c, nowMs);
  if (!handheld_controller::relayTarget(c, nowMs, to))
  {
    clear(q); // nothing goes out without a live session
    return 0;
  }
  return take(q, out);
}

int size(const Queue &q)
{
  return q.count;
}

uint32_t dropped(const Queue &q)
{
  return q.dropped;
}

} // namespace relay_queue
