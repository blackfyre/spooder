#ifndef LED_PATTERN_H
#define LED_PATTERN_H

// Computes what the six status LEDs show from the robot's state.
// Hardware-free: compiled for the board and for host tests.
//
// LEDs 0-4 carry the main indication, in priority order:
//   latched fault      all six alternate the fault's colour with red (bright)
//                      (emergency stop: steady red)
//   waiting for supply all six pulse amber
//   body stopped       slow blue breathing (supply valid, waiting for a request)
//   walking            load bar: supply current against the trip limit, in the
//                      gait's colour; segments past the sustained limit are red
//   other modes        green, LED count = mode (1 Initialise ... 5 Calibrate)
// LED 5 shows the command source (except during faults and supply waiting).
// Bench simulation (core/robot/bench_sim) overlays LED 5, in every state:
//   window, no USB yet   amber blinking slowly (1 s), until a host has
//                        enumerated the board; a press now is refused
//   simulation offered   white blinking (250 ms), for the rest of the 3 s
//                        window
//   refused: supply      red alternating with amber (250 ms), for 3 s
//   refused: no USB host red alternating with blue (250 ms), for 3 s
//   simulating           violet for half of each second, LED 5's own content
//                        for the other half; not while the simulation supply
//                        fault (cyan with red on all six, first fault) is
//                        latched
#include <stdint.h>

namespace led_pattern
{

constexpr int LED_COUNT = 6;
constexpr int MAIN_LEDS = 5; // LEDs 0-4
constexpr int INPUT_LED = 5; // LED 5

enum class Input : uint8_t
{
  Serial,         // USB serial hook
  LinkConnected,  // bridge link up, gamepad connected: the bridge in control
  LinkNoGamepad,  // bridge link up, no gamepad
  LinkStale,      // no valid frames from the bridge
  UsbConnected,   // a USB host with a connected controller in control
  SourceConflict, // the bridge and a USB host both claim control: asleep
};

// The bench simulation's indication on LED 5 (see above).
enum class Bench : uint8_t
{
  None,
  Waiting,       // the 3 s window is open, no USB host yet: a press is refused
  Available,     // the 3 s window is open and a USB host is attached
  RefusedSupply, // entry refused: a servo supply was measured
  RefusedUsb,    // entry refused: no USB host attached
  Simulating,
};

enum class BodyMode : uint8_t // mirrors body::Mode order
{
  Initialise,
  Stand,
  Walk,
  Sleep,
  Calibrate,
  Perform,
};

struct State
{
  uint32_t nowMs;
  uint16_t faults;     // protection::Fault bits; 0 = none
  bool supplyAccepted; // protection allows output
  bool bodyRunning;
  BodyMode mode;
  int gait; // gait::Gait index
  float currentA;
  float tripCurrentA;
  float sustainedCurrentA;
  Input input;
  Bench bench;
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
