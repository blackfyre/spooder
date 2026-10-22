#ifndef BRIDGE_LINK_H
#define BRIDGE_LINK_H

// Bridge side of the robot link on UART1: GPIO27 TX, GPIO26 RX, 115200 8N1.
// A dedicated task sends a command frame every 20 ms and decodes telemetry,
// so screen redraws in loop() cannot delay command frames.
//
// In setup mode (startPortal) the task takes its desired state from the
// portal controller, evaluated at each period so a lapsed portal heartbeat
// counts at once, and sends at most one calibration set per period after the
// command frame. The HTTP handlers reach both only through the portal
// functions below.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"
#include "setup/calibration_sync.h"

namespace robot_link
{

constexpr uint32_t COMMAND_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 1000;

struct Status
{
  bool telemetryFresh;          // a valid telemetry frame within the timeout
  protocol::Telemetry telemetry; // the newest one (meaningful when fresh)
  uint32_t telemetryAgeMs;       // since the newest one arrived
  bool emergencyStop;           // the bridge's own e-stop toggle
  uint32_t validFrames;
  uint32_t discardedFrames;
  // The newest calibration report (the robot sends them in Calibrate). The
  // bridge sends no calibration sets in normal operation.
  bool haveReport;
  uint32_t reportAtMs;
  protocol::CalibrationReport report;
};

/**
 * @brief Starts UART1 and the link task. Until the first setDesired(), frames
 * request Sleep with no controller connected.
 */
void start();

/**
 * @brief Starts UART1 and the link task in setup mode: commands come from
 * the portal controller, and calibration edits go out as calibration sets.
 */
void startPortal();

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

/**
 * @brief The link state now. Freshness is judged inside the lock, against a
 * time taken there, so a telemetry frame stamped by the link task between
 * the caller's clock read and this call cannot look stale.
 */
Status status();

} // namespace robot_link

#endif // BRIDGE_LINK_H
