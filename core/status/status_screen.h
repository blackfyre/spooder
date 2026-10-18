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

/**
 * @brief Short name of one protection fault bit, or nullptr if unknown.
 */
const char *faultName(uint16_t bit);

} // namespace status_screen

#endif // STATUS_SCREEN_H
