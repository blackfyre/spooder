#ifndef VIRTUAL_ROBOT_SIM_H
#define VIRTUAL_ROBOT_SIM_H

// The simulated hardware around the shared tick: servo outputs and the supply
// (core/robot/bench_model), and a simulated gamepad bridge.
#include <stdint.h>
#include <vector>
#include "robot/bench_model.h"

namespace sim
{

// The servo and supply model, shared with the robot's bench simulation.
using bench_model::DEFAULT_SUPPLY_V;
using bench_model::Servos;
using bench_model::Supply;

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
