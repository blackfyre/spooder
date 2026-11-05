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

int size(const Queue &q)
{
  return q.count;
}

uint32_t dropped(const Queue &q)
{
  return q.dropped;
}

} // namespace relay_queue
