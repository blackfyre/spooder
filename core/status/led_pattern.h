#ifndef LED_PATTERN_H
#define LED_PATTERN_H

// Computes what the six status LEDs show from the robot's state.
// Hardware-free: compiled for the board and for host tests.
//
// LEDs 0-4 carry the main indication, in priority order:
//   latched fault      all six alternate the fault's colour with red (bright)
//   waiting for supply all six pulse amber
//   body stopped       slow blue breathing (supply valid, waiting for a request)
//   walking            load bar: supply current against the trip limit, in the
//                      gait's colour; segments past the sustained limit are red
//   other modes        green, LED count = mode (1 Initialise ... 5 Calibrate)
// LED 5 shows the command source (except during faults and supply waiting).
#include <stdint.h>

namespace led_pattern
{

constexpr int LED_COUNT = 6;
constexpr int MAIN_LEDS = 5;    // LEDs 0-4
constexpr int INPUT_LED = 5;    // LED 5

enum class Input : uint8_t
{
  Serial,          // USB serial hook
  LinkConnected,   // bridge link up, gamepad connected
  LinkNoGamepad,   // bridge link up, no gamepad
  LinkStale,       // no valid frames from the bridge
};

enum class BodyMode : uint8_t // mirrors body::Mode order
{
  Initialise,
  Stand,
  Walk,
  Sleep,
  Calibrate,
};

struct State
{
  uint32_t nowMs;
  uint16_t faults;       // protection::Fault bits; 0 = none
  bool supplyAccepted;   // protection allows output
  bool bodyRunning;
  BodyMode mode;
  int gait;              // gait::Gait index
  float currentA;
  float tripCurrentA;
  float sustainedCurrentA;
  Input input;
};

struct Rgb
{
  uint8_t r, g, b;
  bool operator==(const Rgb &o) const { return r == o.r && g == o.g && b == o.b; }
};

// Brightness levels applied to the full-scale colours below.
constexpr float NORMAL_BRIGHTNESS = 0.08f;
constexpr float FAULT_BRIGHTNESS = 0.35f;

/**
 * @brief Fills @p out with the colour of each LED, brightness applied.
 */
void compute(const State &s, Rgb out[LED_COUNT]);

/**
 * @brief Full-scale colour used for a gait while walking (for tests and docs).
 */
Rgb gaitColour(int gait);

/**
 * @brief Number of load-bar segments lit for a current (1..MAIN_LEDS).
 */
int loadSegments(float currentA, float tripCurrentA);

} // namespace led_pattern

#endif // LED_PATTERN_H
