#ifndef PORTAL_CONTROLLER_H
#define PORTAL_CONTROLLER_H

// The setup portal as the bridge's controller (setup mode only): page polls
// and the page's buttons become the same desired state a gamepad gives, so
// the command frames and the robot's rules are those of normal operation.
// - Connected while the page has polled within CONNECTED_WINDOW_MS.
// - Stand, Calibrate and Sleep are mode requests: each sets the mode and
//   advances the mode-request counter.
// - Clear faults advances the fault-clear counter, like the gamepad's combo.
// - Translate and rotate are always zero.
// - A disconnect requests Sleep, and the mode stays Sleep until the next
//   mode press, so a returning page never resumes motion by itself.
// Hardware-free: the bridge feeds it from its HTTP handlers and reads it in
// the link task; host tests drive it directly.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"

namespace portal_controller
{

constexpr uint32_t CONNECTED_WINDOW_MS = 1000;
// The speed dial a gamepad starts with (gamepad_mapping::Mapper's default).
constexpr uint8_t SPEED = 44;

struct Controller
{
  bool polled = false;
  uint32_t lastPollMs = 0;
  RequestedMode mode = RequestedMode::Sleep;
  uint8_t modeRequestCounter = 0;
  uint8_t clearCounter = 0;
};

/**
 * @brief Records a page poll (the portal heartbeat) at @p nowMs.
 */
void poll(Controller &c, uint32_t nowMs);

/**
 * @brief A press of Stand, Calibrate or Sleep (other modes are ignored). It
 * counts as a poll too.
 * @return true if the press was taken
 */
bool requestMode(Controller &c, RequestedMode mode, uint32_t nowMs);

/**
 * @brief A press of Clear faults. It counts as a poll too.
 */
void requestClear(Controller &c, uint32_t nowMs);

/**
 * @brief True while the newest poll is no older than CONNECTED_WINDOW_MS.
 */
bool connected(const Controller &c, uint32_t nowMs);

/**
 * @brief The desired state at @p nowMs. A lapsed heartbeat turns the mode
 * into Sleep, which stays until the next mode press.
 */
gamepad_mapping::Desired desired(Controller &c, uint32_t nowMs);

} // namespace portal_controller

#endif // PORTAL_CONTROLLER_H
