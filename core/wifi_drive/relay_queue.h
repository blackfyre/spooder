#ifndef RELAY_QUEUE_H
#define RELAY_QUEUE_H

// The relay of the robot's frames to the handheld (WiFi drive): a fixed ring
// of CAPACITY raw frames between the bridge's link task, which pushes each
// frame it decodes, and its UDP task, which takes them and sends one per
// datagram. It never waits: a push to a full ring drops the frame and
// counts it, so the link task's 20 ms period never depends on the relay.
// The bridge clears it at every change of the handheld session
// (handheld_controller::epoch), so no frame reaches another session's
// sender.
// Hardware-free: the bridge calls it under its link lock; host tests drive
// it directly.
#include <stdint.h>
#include "protocol/protocol.h"

namespace relay_queue
{

constexpr int CAPACITY = 8;

struct Entry
{
  uint8_t bytes[protocol::MAX_FRAME_SIZE];
  int length;
};

struct Queue
{
  Entry entries[CAPACITY] = {};
  int head = 0;  // the oldest frame
  int count = 0; // frames waiting
  uint32_t dropped = 0;
};

/**
 * @brief Adds a copy of one frame of @p length bytes (1..MAX_FRAME_SIZE).
 * @return false if the ring was full (the frame is dropped and counted) or
 * the length is out of range (nothing is stored)
 */
bool push(Queue &q, const uint8_t *bytes, int length);

/**
 * @brief Moves the oldest frame into @p out (at least MAX_FRAME_SIZE bytes).
 * @return its length, or 0 when the ring is empty
 */
int take(Queue &q, uint8_t *out);

/**
 * @brief Drops every waiting frame (not counted as dropped).
 */
void clear(Queue &q);

int size(const Queue &q);
uint32_t dropped(const Queue &q);

} // namespace relay_queue

#endif // RELAY_QUEUE_H
