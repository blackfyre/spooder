#ifndef LINK_RECEIVER_H
#define LINK_RECEIVER_H

// Robot side of the links (the bridge on UART1, a computer on USB): turns
// decoded frames into the robot's input for one tick, guards calibration
// changes, and decides which link is in control.
// Hardware-free: compiled for the board and for host tests.
//
// Commands, per link (one Receiver each):
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
//
// Calibration sets are queued per link and processed once per tick through
// the guards (Calibrate with output allowed, then joint_calibration's clamps)
// into the requested table. A repeated request id on the same link is a
// retransmission: it is reported again and changes nothing.
//
// Arbitration: the link that reports a fresh, connected controller is in
// control. Both at once is a conflict, which requests Sleep. Each link's own
// mode-request counter advances the arbitrated requestSeq only while that
// link is in control, so a change of source never counts as a new request.
#include <stdint.h>
#include "movement/body.h"
#include "movement/joint_calibration.h"
#include "movement/movement_command.h"
#include "protocol/protocol.h"

namespace link_receiver
{

// A gap this long between valid frames resets the counter baselines. Frames
// normally arrive every 20 ms.
constexpr uint32_t LINK_GAP_MS = 200;
// Calibration sets held per link between ticks; more are dropped (the
// front-end resends anything not confirmed).
constexpr int MAX_PENDING_SETS = 4;
// Calibration reports: at least this often in Calibrate, and never more often
// than the minimum interval on one link.
constexpr uint32_t CALIBRATION_REPORT_PERIOD_MS = 1000;
constexpr uint32_t MIN_REPORT_INTERVAL_MS = 200;

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

  // Calibration sets waiting for processCalibration().
  protocol::CalibrationSet pendingSets[MAX_PENDING_SETS] = {};
  int pendingSetCount = 0;
  // The last set processed from this link, for retransmissions and reports.
  bool haveCalibrationRequest = false;
  uint8_t lastRequestId = 0;
  protocol::CalibrationResult lastResult = protocol::CalibrationResult::NoneYet;
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

/**
 * @brief True while @p in carries a command no older than
 * body::COMMAND_TIMEOUT_MS.
 */
bool isFresh(const Input &in, uint32_t nowMs);

/**
 * @brief Queues a valid calibration set frame (dropped if the queue is full).
 */
void acceptCalibration(Receiver &r, const protocol::CalibrationSet &c);

/**
 * @brief Processes the queued calibration sets into @p requested. A set is
 * rejected unless @p calibrating (the body is in Calibrate) and
 * @p allowOutput; otherwise it is clamped to the joint's safe range around
 * its compiled default.
 * @return the number of sets processed (each one warrants a report)
 */
int processCalibration(Receiver &r, joint_calibration::Table &requested, bool calibrating, bool allowOutput);

/**
 * @brief The report for this link: @p table, and the result of the last set
 * processed from the link.
 */
protocol::CalibrationReport makeReport(const Receiver &r, const joint_calibration::Table &table);

/**
 * @brief The pose frame for the USB host: per joint, the commanded angle
 * (the body's angle minus the offset its IK added), the servo angle (the
 * body's angle, which toPulse converts) and @p pulses (servo output order,
 * 0 = never set). Angles are rounded to centidegrees and saturated.
 */
protocol::Pose makePose(const body::BodyState &body, const uint16_t *pulses, bool outputsOn);

// When to send calibration reports on one link.
struct ReportSchedule
{
  bool due = false;
  bool sentOnce = false;
  uint32_t lastSentMs = 0;
};

/**
 * @brief Asks for a report (a calibration set was processed).
 */
void requestReport(ReportSchedule &s);

/**
 * @brief True if a report should be sent now: one was requested, or a period
 * has passed in Calibrate, and the minimum interval has passed. Call
 * reportSent() once it has been queued.
 */
bool reportDue(ReportSchedule &s, bool calibrating, uint32_t nowMs);
void reportSent(ReportSchedule &s, uint32_t nowMs);

enum class Source : uint8_t
{
  None,
  Bridge,
  Usb,
  Conflict, // both claim control: treated as no source
};

struct Arbiter
{
  bool started = false;
  uint32_t bridgeSeq = 0; // each link's requestSeq at the previous tick
  uint32_t usbSeq = 0;
  uint32_t requestSeq = 0;
};

struct Arbitration
{
  Input input;            // the tick's input; Sleep with no source or a conflict
  Source source;
  uint8_t telemetryFlags; // protocol::TELEMETRY_* source bits
};

/**
 * @brief Chooses this tick's input from the two links' inputs. The emergency
 * stop is either link's; a fault clear counts only from the source in
 * control.
 */
Arbitration arbitrate(Arbiter &a, const Input &bridge, const Input &usb, uint32_t nowMs);

} // namespace link_receiver

#endif // LINK_RECEIVER_H
