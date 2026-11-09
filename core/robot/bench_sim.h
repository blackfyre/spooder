#ifndef ROBOT_BENCH_SIM_H
#define ROBOT_BENCH_SIM_H

// Bench simulation: the robot on USB only, with no servo supply, driven end
// to end while no servo moves. Hardware-free: compiled for the board and for
// host tests.
//
// Entry: the firmware calls step() once per tick, before robot_tick::tick,
// with the USER button, the measured servo supply, whether real servo output
// is on and whether a USB host is attached. For WINDOW_MS from the first
// tick, the first press edge (a pressed sample after a released one) decides,
// once per boot:
//   refused, supply   the supply read ABSENT_SUPPLY_V or more on any tick
//   refused, no USB   no host has enumerated the board, or it suspended it
//   entered           otherwise; simulation then lasts until restart
// While no USB host is attached the window shows Waiting (a press would be
// refused), and Available once one is.
// The window also closes once real output has been on. A refusal is shown for
// REFUSED_SHOW_MS after the press. In simulation, a measured supply above the
// brown-out voltage latches the supply fault until restart. The state is an
// Entry in RAM, zeroed at every start-up; nothing else records it.
//
// The board boundary: BenchBoard implements robot_tick::Hardware's servo
// calls over the real servo port (hooks the firmware's board implements) and,
// from enterSimulation() on, over a bench_model::Servos, with no path left to
// a real pulse.
#include <stdint.h>
#include "robot/bench_model.h"
#include "robot/robot_tick.h"

namespace bench_sim
{

constexpr uint32_t WINDOW_MS = 3000;
constexpr uint32_t REFUSED_SHOW_MS = 3000;
// Below this on every tick since start-up, the servo supply is absent. Not
// the brown-out voltage: a flat battery, or USB 5 V on the servo rail through
// an uncut trace, is still a supply.
constexpr float ABSENT_SUPPLY_V = 1.0f;

enum class Phase : uint8_t
{
  Window,     // live, simulation offered
  Refused,    // live, the refusal's reason shown
  Live,       // live for the rest of the boot
  Simulating, // until restart
};

// One tick's samples.
struct Sample
{
  uint32_t nowMs;
  bool buttonPressed;    // the USER button's level (active low on the board)
  float measuredSupplyV; // the real servo supply's voltage sense
  bool realOutputOn;     // real servo output is on
  bool usbHost;          // a USB host has enumerated the board and not suspended it
  float brownoutV;       // protection's brown-out voltage
};

struct Entry
{
  bool started = false;
  uint32_t startMs = 0;
  bool seenReleased = false; // a released sample inside the window
  bool supplyAbsent = true;  // below ABSENT_SUPPLY_V on every tick so far
  Phase phase = Phase::Window;
  robot_tick::BenchEvent refusal = robot_tick::BenchEvent::None;
  uint32_t pressMs = 0;
  bool supplyFault = false; // latched in Simulating until restart
  bool usbHost = false;     // the latest sample's: the window's indication
};

struct Result
{
  Phase phase;
  robot_tick::BenchInputs bench; // for robot_tick::Inputs::bench
};

/**
 * @brief Advances @p e by one tick's samples (see the header comment).
 */
Result step(Entry &e, const Sample &sample);

// robot_tick::Hardware's servo calls, with the real servo port behind
// protected hooks. The firmware's board derives from it and implements the
// hooks and the non-servo calls.
class BenchBoard : public robot_tick::Hardware
{
public:
  void setInhibited(bool inhibited) final;
  void stopServos() final;
  void applyServos(const body::BodyState &body, const joint_calibration::Table &applied, int legsOn) final;
  bool servosOn() final;
  const uint16_t *servoPulses() final;

  /**
   * @brief Enters simulation for the rest of the boot: sets the flag, then
   * forces real output off once (realForceOff). From then on the servo calls
   * act on the model; the real port only ever hears inhibit and stop.
   */
  void enterSimulation();
  bool simulated() const { return simulated_; }
  const bench_model::Servos &model() const { return model_; }

protected:
  ~BenchBoard() = default;

  virtual void realSetInhibited(bool inhibited) = 0;
  virtual void realStop() = 0;
  virtual void realApply(const body::BodyState &body, const joint_calibration::Table &applied, int legsOn) = 0;
  virtual bool realOn() = 0;
  virtual const uint16_t *realPulses() = 0;
  // Stops and disables every output until restart (servo_output::forceOff).
  virtual void realForceOff() = 0;

private:
  bool simulated_ = false;
  bench_model::Servos model_;
};

} // namespace bench_sim

#endif // ROBOT_BENCH_SIM_H
