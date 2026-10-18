#ifndef CALIBRATION_SYNC_H
#define CALIBRATION_SYNC_H

// Reconciles the setup portal's joint edits with the robot's calibration
// reports (setup mode only). The robot decides every value; this only keeps
// the operator's wanted values until a report confirms them.
// - An edit goes into a pending entry per joint, holding the latest value.
// - next() hands out at most one calibration set per call (the link task
//   calls it once per command period): the oldest pending joint, with a new
//   request id. One set is in flight at a time, since a report carries only
//   the id of the last set the robot processed.
// - A report carrying the in-flight id confirms it: the entry is cleared and
//   the reported values (possibly clamped, or unchanged if rejected) are what
//   the portal shows. A joint edited again while in flight stays pending.
// - Without a confirming report within RESEND_MS, the same frame is sent
//   again with the same id, which the robot treats as a retransmission.
// - Edits are relative to the wanted value (pending, else reported), so they
//   need a report first.
// Hardware-free: the bridge's link task and HTTP handlers share one Sync
// under a lock; host tests drive it directly.
#include <stdint.h>
#include "protocol/protocol.h"

namespace calibration_sync
{

constexpr int JOINT_COUNT = protocol::CALIBRATION_JOINTS;
constexpr uint32_t RESEND_MS = 200;

enum class Field : uint8_t
{
  Offset,   // centidegrees
  Pulse0,   // microseconds
  Pulse180, // microseconds
  Reversed, // 0 or 1
};

struct Entry
{
  bool pending;
  uint32_t order; // when it became pending; lowest is oldest
  protocol::CalibrationJoint value;
};

struct Sync
{
  bool haveReport = false;
  protocol::CalibrationReport report = {}; // the newest: what the portal shows
  Entry entries[JOINT_COUNT] = {};
  uint32_t nextOrder = 0;

  bool inFlight = false;
  protocol::CalibrationSet sent = {}; // the set in flight
  uint32_t sentAtMs = 0;
  bool editedInFlight = false;        // its joint was edited after sending
  bool haveLastId = false;
  uint8_t lastId = 0;                 // the newest request id used
};

/**
 * @brief The value the operator wants for @p joint: the pending one, else
 * the reported one.
 * @return false without a report or for a bad joint
 */
bool wanted(const Sync &s, int joint, protocol::CalibrationJoint &out);

/**
 * @brief Changes one field of @p joint's wanted value: set to @p value, or,
 * if @p relative, moved by it. The result saturates to the field's wire
 * range; the robot applies its own clamps. Reversed is set to value != 0, or
 * toggled by a relative change other than zero.
 * @return false (nothing changed) without a report or for a bad joint
 */
bool change(Sync &s, int joint, Field field, int32_t value, bool relative);

/**
 * @brief True while @p joint has an unconfirmed edit.
 */
bool pending(const Sync &s, int joint);

/**
 * @brief The calibration set to send now, if any: a resend of the set in
 * flight once RESEND_MS has passed, else the oldest pending joint.
 * @return true if @p out should be sent
 */
bool next(Sync &s, uint32_t nowMs, protocol::CalibrationSet &out);

/**
 * @brief Takes a calibration report from the robot.
 */
void onReport(Sync &s, const protocol::CalibrationReport &r);

} // namespace calibration_sync

#endif // CALIBRATION_SYNC_H
