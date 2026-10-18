#ifndef LINK_RECEIVER_H
#define LINK_RECEIVER_H

// Robot side of the link: turns decoded command frames into the robot's input
// for one tick. Hardware-free: compiled for the board and for host tests.
//
// - The newest valid frame wins; it is stamped with its arrival time, so the
//   body's stale-command rule puts the robot to sleep when frames stop.
// - A frame without a connected controller requests Sleep.
// - Each change of the fault-clear counter is one clear request; each change
//   of the mode-request counter advances requestSeq (for protection's resume
//   gate).
// - After a gap in the link (including power-up and a bridge restart) the
//   first frame only sets the baseline for both counters: a restarted bridge,
//   whose counters start again from zero, cannot clear faults or count as a
//   new mode request.
#include <stdint.h>
#include "movement/movement_command.h"
#include "protocol/protocol.h"

namespace link_receiver
{

// A gap this long between valid frames resets the counter baselines. Frames
// normally arrive every 20 ms.
constexpr uint32_t LINK_GAP_MS = 200;

struct Input
{
  MovementCommand command; // Sleep with issuedAtMs 0 until a frame arrives
  bool linkSeen;           // at least one valid frame since power-up
  bool controllerConnected;
  bool emergencyStop;
  bool clearRequested;     // a new clear-counter value since the last tick
  uint32_t requestSeq;     // advances with each new mode-request counter value
};

struct Receiver
{
  bool haveFrame = false;
  protocol::Command last = {};
  uint32_t lastArrivalMs = 0;
  uint8_t clearCounter = 0;
  uint8_t modeRequestCounter = 0;
  bool clearPending = false;
  uint32_t requestSeq = 0;
};

/**
 * @brief Records a valid command frame that arrived at @p nowMs.
 */
void accept(Receiver &r, const protocol::Command &c, uint32_t nowMs);

/**
 * @brief The input for this tick; consumes the pending clear request.
 */
Input take(Receiver &r);

/**
 * @brief Converts a frame's payload to a movement command (no stamping).
 * Unknown modes and a disconnected controller become Sleep.
 */
MovementCommand toMovementCommand(const protocol::Command &c);

} // namespace link_receiver

#endif // LINK_RECEIVER_H
