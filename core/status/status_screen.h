#ifndef STATUS_SCREEN_H
#define STATUS_SCREEN_H

// The bridge's status screen as text lines: what to show for the gamepad,
// the link, the robot's telemetry and the requested settings.
// Hardware-free: the bridge draws the lines; host tests check the content.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"

namespace status_screen
{

constexpr int LINE_COUNT = 8;
constexpr int LINE_CHARS = 28; // fits 240 px at the screen font

enum class Tone : uint8_t
{
  Normal,
  Good,
  Warn,
  Bad,
};

struct Line
{
  char text[LINE_CHARS + 1];
  Tone tone;
};

struct Inputs
{
  bool gamepadConnected;
  bool emergencyStop;    // the bridge's own e-stop toggle
  gamepad_mapping::Desired desired;
  bool telemetryFresh;   // robot values are current only while fresh
  protocol::Telemetry telemetry;
};

void compose(const Inputs &in, Line out[LINE_COUNT]);

// The setup-mode screen: text on the left, the joining QR code (135 px) on
// the right. Lines hold at most SETUP_LINE_CHARS characters and must fit
// SETUP_TEXT_WIDTH_PX at the screen font.
constexpr int SETUP_LINE_CHARS = 14;
constexpr int SETUP_TEXT_WIDTH_PX = 100; // left of the 135 px QR code

struct SetupInputs
{
  const char *ssid;      // the access point's network name
  const char *password;
  const char *address;   // the portal's address
  bool portalConnected;  // a portal page polled within the last second
  bool emergencyStop;    // the bridge's own e-stop toggle
  bool telemetryFresh;
  protocol::Telemetry telemetry;
};

/**
 * @brief The setup screen: SETUP MODE, network name, password, address,
 * portal client, link state, e-stop and, while the link is up, the body mode.
 */
void composeSetup(const SetupInputs &in, Line out[LINE_COUNT]);

/**
 * @brief Short name of one protection fault bit, or nullptr if unknown.
 */
const char *faultName(uint16_t bit);

} // namespace status_screen

#endif // STATUS_SCREEN_H
