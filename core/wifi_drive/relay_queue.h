#ifndef RELAY_QUEUE_H
#define RELAY_QUEUE_H

// The relay of the robot's frames to the handheld (WiFi drive): a fixed ring
// of CAPACITY raw frames between the bridge's link task, which pushes each
// frame it decodes, and its UDP task, which takes them and sends one per
// datagram. It never waits: a push to a full ring drops the frame and
// counts it, so the link task's 20 ms period never depends on the relay.
// relayFor() and takeFor() tie it to the handheld's sessions: they clear it
// at every change of session (handheld_controller::epoch), queue only while
// a session is live and hand each frame out with the live session's sender,
// so no frame reaches another session's sender. The link task drains the
// UART once per poll, and a frame decoded in a poll may have arrived at any
// time since the previous one, and the decoder can hold bytes of an earlier
// poll. After a session change relayFor() drops (and counts) every frame
// until a poll of the new session ends with the decoder empty (endPoll()):
// from then on every byte arrived during that session, so a frame received
// during one session never reaches the next one's sender.
// Hardware-free: the bridge calls it under its link lock; host tests drive
// it directly.
#include <stdint.h>
#include "protocol/protocol.h"
#include "wifi_drive/handheld_controller.h"

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
  uint32_t epoch = 0; // the handheld session the waiting frames belong to
  // The session live at the start of the current poll, and whether a poll
  // of it has ended with the decoder empty.
  uint32_t pollEpoch = 0;
  bool clean = false;
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

/**
 * @brief Starts a poll of the robot's bytes at @p nowMs (the link task,
 * before it drains the UART): a session changed since the previous poll
 * stops the relay until endPoll() finds the decoder empty.
 */
void beginPoll(Queue &q, handheld_controller::Controller &c, uint32_t nowMs);

/**
 * @brief Ends the poll (the link task, after the drain and the idle check):
 * @p decoderEmpty, the decoder holding no bytes, lets the next poll's frames
 * go to the session of this one.
 */
void endPoll(Queue &q, bool decoderEmpty);

/**
 * @brief Queues one robot frame for the handheld of @p c at @p nowMs: after
 * clearing the queue if the session changed, and only while a session is
 * live. A frame decoded before the session's first clean poll end
 * (endPoll) is dropped and counted, as a frame that finds the queue full is.
 * @return false if nothing was queued (no live session, not yet clean,
 * full, bad length)
 */
bool relayFor(Queue &q, handheld_controller::Controller &c, uint32_t nowMs, const uint8_t *bytes, int length);

/**
 * @brief Moves the oldest frame for the handheld of @p c into @p out (at
 * least MAX_FRAME_SIZE bytes), with its live session's sender in @p to:
 * after clearing the queue if the session changed; with no live session the
 * waiting frames are dropped.
 * @return its length, or 0 when there is nothing to send
 */
int takeFor(Queue &q, handheld_controller::Controller &c, uint32_t nowMs, uint8_t *out,
            handheld_controller::Sender &to);

int size(const Queue &q);
uint32_t dropped(const Queue &q);

} // namespace relay_queue

#endif // RELAY_QUEUE_H
