#ifndef BRIDGE_LINK_H
#define BRIDGE_LINK_H

// Bridge side of the robot link on UART1: GPIO27 TX, GPIO26 RX, 115200 8N1.
// A dedicated task sends a command frame every 20 ms and decodes telemetry,
// so screen redraws in loop() cannot delay command frames.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"

namespace robot_link
{

constexpr uint32_t COMMAND_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 1000;

struct Status
{
  bool telemetryFresh;          // a valid telemetry frame within the timeout
  protocol::Telemetry telemetry; // the newest one (meaningful when fresh)
  bool emergencyStop;           // the bridge's own e-stop toggle
  uint32_t validFrames;
  uint32_t discardedFrames;
};

/**
 * @brief Starts UART1 and the link task. Until the first setDesired(), frames
 * request Sleep with no controller connected.
 */
void start();

/**
 * @brief Hands the newest desired state to the link task.
 */
void setDesired(const gamepad_mapping::Desired &d);

/**
 * @brief The link state now. Freshness is judged inside the lock, against a
 * time taken there, so a telemetry frame stamped by the link task between
 * the caller's clock read and this call cannot look stale.
 */
Status status();

} // namespace robot_link

#endif // BRIDGE_LINK_H
