#ifndef VIRTUAL_ROBOT_SIM_H
#define VIRTUAL_ROBOT_SIM_H

// The simulated hardware around the shared tick: servo outputs, the supply
// and its current, and a simulated gamepad bridge.
#include <stdint.h>
#include <vector>
#include "movement/body.h"
#include "movement/joint_calibration.h"
#include "protection/protection.h"

namespace sim
{

constexpr int OUTPUT_COUNT = joint_calibration::JOINT_COUNT;
// Current model, per powered servo: an idle draw, plus a draw per microsecond
// of pulse change in one tick (moving), up to a cap (a servo at full effort);
// all scaled by the load factor. At load 1, all 18 at full effort stay below
// a 9 A trip; a higher load trips it.
constexpr float IDLE_A_PER_SERVO = 0.12f;
constexpr float A_PER_US_PER_TICK = 0.004f;
constexpr float MAX_MOVING_A_PER_SERVO = 0.35f;
constexpr float DEFAULT_SUPPLY_V = 6.0f;

// servo_output's rules, without pins: inhibited or disabled is off; only
// joints the body has set, on released legs, get a pulse; on is known after
// apply().
struct Servos
{
  uint16_t desiredUs[OUTPUT_COUNT] = {};
  bool legSet[kinematics::LEG_COUNT] = {};
  bool on = false;
  bool inhibited = true;
  int legsOn = 0; // legs released since output switched on
  float movingA = 0; // the moving draw of the last apply, while on (load 1)

  void apply(const body::BodyState &body, const joint_calibration::Table &calibration, int legsOn);
  void stop()
  {
    on = false;
    legsOn = 0;
  }
  int powered() const;
};

struct Supply
{
  float voltageV = DEFAULT_SUPPLY_V;
  float load = 1;
  bool fixedCurrent = false;
  float fixedCurrentA = 0;

  // The reading for the next tick, from the servos' last tick.
  protection::Reading read(const Servos &servos) const;
};

// A gamepad bridge on the bridge link, without a terminal: its command frames
// go into the bridge link's bytes, every tick, like the real bridge's.
struct Bridge
{
  enum class State
  {
    Off,     // no frames: the link is stale
    Idle,    // frames, no gamepad
    Gamepad, // a connected gamepad requesting Stand
  };
  State state = State::Off;
  bool estop = false;
  uint8_t clearCounter = 0;
  uint8_t modeCounter = 0;
  uint8_t seq = 0;
  int framesUntilRequest = -1; // a mode request after the baseline frames

  void setState(State s);
  void clear() { clearCounter++; }
  // Appends this tick's frame (if any) to @p out.
  void frame(std::vector<uint8_t> &out);
};

} // namespace sim

#endif // VIRTUAL_ROBOT_SIM_H
