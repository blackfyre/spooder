#ifndef BRIDGE_LINK_H
#define BRIDGE_LINK_H

// Bridge side of the robot link on UART1: GPIO27 TX, GPIO26 RX, 115200 8N1.
// A dedicated task sends a command frame every 20 ms and decodes telemetry,
// so screen redraws in loop() cannot delay command frames.
//
// The boot mode (drive_mode::Mode), given to start(), decides where the
// commands come from. In Bluetooth drive loop() hands them over with
// setDesired().
//
// In setup mode the task takes its desired state from the
// portal controller, evaluated at each period so a lapsed portal heartbeat
// counts at once, and sends at most one calibration set per period after the
// command frame. The HTTP handlers reach both only through the portal
// functions below.
//
// In WiFi drive the task takes its desired state from the handheld
// controller, judged at each period so a stalled UDP task counts as a lapse
// at once, and queues every frame it decodes for the relay to the handheld. The UDP
// task reaches both only through the handheld functions below.
//
// The emergency stop the frames carry follows one rule for every mode
// (handheld_controller::commandStop): the GPIO35 toggle, and in WiFi drive
// also the handheld's flag. status() reports that same stop.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"
#include "setup/calibration_sync.h"
#include "status/supply_band.h"
#include "wifi_drive/handheld_controller.h"

namespace robot_link
{

constexpr uint32_t COMMAND_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 1000;

struct Status
{
  bool telemetryFresh;           // a valid telemetry frame within the timeout
  protocol::Telemetry telemetry; // the newest one (meaningful when fresh)
  uint32_t telemetryAgeMs;       // since the newest one arrived
  // The emergency stop the command frames carry (handheld_controller::
  // commandStop): the GPIO35 toggle, in WiFi drive or the handheld's flag.
  bool emergencyStop;
  uint32_t validFrames;
  uint32_t discardedFrames;
  // The newest calibration report (the robot sends them in Calibrate). The
  // bridge sends no calibration sets in normal operation.
  bool haveReport;
  uint32_t reportAtMs;
  protocol::CalibrationReport report;
  // The protection limits from the newest diagnostics frame since start,
  // kept while diagnostics stop (they change only with a robot build); all
  // zero, so not valid, until the first.
  supply_band::Limits supplyLimits;
};

/**
 * @brief Starts UART1 and the link task in boot mode @p mode, once. In
 * Bluetooth drive commands come from setDesired(); until the first, frames
 * request Sleep with no controller connected. In setup mode they come from
 * the portal controller, and calibration edits go out as calibration sets.
 * In WiFi drive they come from the handheld controller, and the robot's
 * frames are queued for the relay.
 */
void start(drive_mode::Mode mode);

/**
 * @brief Hands the newest desired state to the link task (normal operation).
 */
void setDesired(const gamepad_mapping::Desired &d);

// --- setup mode: the portal's side ------------------------------------------

/**
 * @brief A page poll (the portal heartbeat).
 */
void portalPoll();

/**
 * @brief A press of Stand, Calibrate or Sleep on the portal.
 * @return false for any other mode
 */
bool portalRequestMode(RequestedMode mode);

/**
 * @brief A press of Clear faults on the portal.
 */
void portalRequestClear();

/**
 * @brief A joint edit (see calibration_sync::change).
 * @return false without a report yet or for a bad joint
 */
bool portalChange(int joint, calibration_sync::Field field, int32_t value, bool relative);

struct PortalView
{
  bool connected;                   // a poll within the last second
  gamepad_mapping::Desired desired; // what the command frames request
  calibration_sync::Sync sync;      // the reported table and pending edits
};

/**
 * @brief Copies the portal state, for the HTTP handlers and the screen.
 */
void portalView(PortalView &out);

// --- WiFi drive: the UDP task's side ----------------------------------------

/**
 * @brief Hands one datagram from @p sender, just received, to the handheld
 * controller (handheld_controller::accept), stamped inside the lock.
 */
handheld_controller::Outcome handheldAccept(const uint8_t *datagram, int length, handheld_controller::Sender sender);

/**
 * @brief The UDP task's read policy (handheld_controller::readDelayMs): 0 to
 * read now, else the milliseconds until the next rate window.
 */
uint32_t handheldReadDelayMs(uint32_t nowMs);

/**
 * @brief Takes the oldest frame waiting for the relay, and where to send it.
 * With no live session the waiting frames are dropped.
 * @param out at least protocol::MAX_FRAME_SIZE bytes
 * @return its length, or 0 when there is nothing to send
 */
int handheldTakeRelay(uint8_t *out, handheld_controller::Sender &to);

struct HandheldView
{
  handheld_controller::State state;
  gamepad_mapping::Desired desired; // what the command frames request
  uint32_t counts[handheld_controller::OUTCOME_COUNT];
  uint32_t relayDropped; // robot frames dropped because the relay was full
};

/**
 * @brief Copies the handheld state, for the screen and the console.
 */
void handheldView(HandheldView &out);

/**
 * @brief The link state now, one snapshot under the lock: the emergency
 * stop with it is the one the next command frame carries. Freshness is judged inside the lock, against a
 * time taken there, so a telemetry frame stamped by the link task between
 * the caller's clock read and this call cannot look stale.
 */
Status status();

} // namespace robot_link

#endif // BRIDGE_LINK_H
