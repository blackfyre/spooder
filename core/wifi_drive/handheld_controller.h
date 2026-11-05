#ifndef HANDHELD_CONTROLLER_H
#define HANDHELD_CONTROLLER_H

// A WiFi handheld as the bridge's controller (WiFi drive only): UDP
// datagrams, each one command frame of the link protocol, become the same
// desired state a gamepad gives, so the bridge's command frames and the
// robot's rules are those of normal operation.
// - At most WINDOW_BUDGET datagrams per WINDOW_MS window (a fixed grid of
//   nowMs / WINDOW_MS), counted before any decoding.
// - A datagram counts only when it is exactly one command frame of
//   protocol::VERSION with a valid CRC.
// - One sender (address and port) at a time: the first sender of an
//   accepted frame is locked while its session is live.
// - Sequence numbers, d = seq - last accepted, modulo 256: 1..SEQ_WINDOW is
//   newer; 0 or 128..255 is stale and changes nothing; SEQ_WINDOW+1..127 is
//   a jump (the sender restarted) and ends the session at once.
// - Live while the newest accepted frame is no older than LIVENESS_MS.
// - The first frame of a session only sets the baselines of the handheld's
//   three counters; later changes each advance the bridge's own counter by
//   one, taking the frame's mode or gesture. A session starts in Sleep and
//   stays there until its first mode request.
// - The e-stop flag is the newest accepted frame's, kept through a lapse.
// Hardware-free: the bridge's UDP task feeds it and its link task reads it,
// both under the link lock; host tests drive it directly.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"

namespace handheld_controller
{

constexpr uint16_t PORT = 52040; // UDP, on the access point's 192.168.4.1
constexpr uint32_t WINDOW_MS = 20;
constexpr int WINDOW_BUDGET = 8;
constexpr uint8_t SEQ_WINDOW = 8;
// Six lost frames of a 50 Hz handheld leave 140 ms; seven leave 160 ms.
constexpr uint32_t LIVENESS_MS = 150;
// The desired state with no live session: a gamepad's defaults
// (gamepad_mapping::Mapper).
constexpr uint8_t DEFAULT_SPEED = 44;

struct Sender
{
  uint32_t address; // IPv4, as the bridge's socket gives it
  uint16_t port;
};

enum class Outcome : uint8_t
{
  Accepted,
  RateLimited, // beyond the window's budget, not decoded
  Malformed,   // not exactly one valid version 3 command frame
  OtherSender, // while another sender's session is live
  Stale,       // late or repeated
  Jump,        // the sequence jumped ahead: the session ended
};
constexpr int OUTCOME_COUNT = 6;

// What the screen shows of the handheld.
enum class State : uint8_t
{
  None,      // no live session
  Idle,      // live, its frames clear the controller-connected flag
  InControl, // live, with the flag set
};

struct Controller
{
  // The rate window.
  bool windowStarted = false;
  uint32_t window = 0; // nowMs / WINDOW_MS
  int windowUsed = 0;
  // The session.
  bool session = false;
  uint32_t epoch = 0; // advances at every session start and end
  Sender sender = {};
  uint8_t lastSeq = 0;
  uint32_t lastMs = 0;
  protocol::Command last = {}; // the newest accepted frame
  // The bridge's own state.
  RequestedMode mode = RequestedMode::Sleep;
  uint8_t clearCounter = 0;
  uint8_t modeRequestCounter = 0;
  uint8_t gesture = 0;
  uint8_t gestureRequestCounter = 0;
  bool emergencyStop = false;
  uint32_t counts[OUTCOME_COUNT] = {};
};

/**
 * @brief Takes one datagram of @p length bytes from @p sender, received at
 * @p nowMs, and counts its outcome.
 */
Outcome accept(Controller &c, const uint8_t *datagram, int length, Sender sender, uint32_t nowMs);

/**
 * @brief True when the window holding @p nowMs has no budget left: the UDP
 * task leaves further datagrams unread until the next window.
 */
bool budgetSpent(const Controller &c, uint32_t nowMs);

/**
 * @brief The UDP task's read policy at @p nowMs: 0 while the window holding
 * @p nowMs has budget left (read now), else the milliseconds to the next
 * window (leave the datagrams unread until then).
 */
uint32_t readDelayMs(const Controller &c, uint32_t nowMs);

/**
 * @brief Ends a session whose newest frame is older than LIVENESS_MS at
 * @p nowMs (no controller, Sleep, sender lock released). The other calls
 * do this themselves.
 */
void judge(Controller &c, uint32_t nowMs);

/**
 * @brief True while a session is live at @p nowMs.
 */
bool live(Controller &c, uint32_t nowMs);

/**
 * @brief The session epoch after judging liveness at @p nowMs: it changes
 * whenever a session starts or ends, so the relay clears its queue when it
 * sees a change and no frame reaches another session's sender.
 */
uint32_t epoch(Controller &c, uint32_t nowMs);

/**
 * @brief Where to relay the robot's frames: the locked sender while its
 * session is live.
 * @return false when no session is live (relay nothing)
 */
bool relayTarget(Controller &c, uint32_t nowMs, Sender &out);

/**
 * @brief The desired state at @p nowMs: the bridge's counters and mode, and,
 * while live, the newest frame's analogue fields and controller flag
 * (passed through: the robot clamps them). With no live session: Sleep, no
 * controller, zero translate and rotate, a gamepad's defaults for the rest.
 */
gamepad_mapping::Desired desired(Controller &c, uint32_t nowMs);

/**
 * @brief The handheld's e-stop flag, kept through a lapse until an accepted
 * frame clears it.
 */
bool emergencyStop(const Controller &c);

/**
 * @brief The handheld's state at @p nowMs, for the screen.
 */
State state(Controller &c, uint32_t nowMs);

/**
 * @brief Datagrams counted with @p o since start.
 */
uint32_t count(const Controller &c, Outcome o);

} // namespace handheld_controller

#endif // HANDHELD_CONTROLLER_H
