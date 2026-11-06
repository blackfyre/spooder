#ifndef ROBOT_BENCH_MODEL_H
#define ROBOT_BENCH_MODEL_H

// The modelled servo outputs and servo supply: servo_output's rules without
// pins, and the supply current they draw. Shared by the virtual robot and the
// robot's bench simulation.
#include <stdint.h>
#include "movement/body.h"
#include "movement/joint_calibration.h"
#include "protection/protection.h"

namespace bench_model
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
  int legsOn = 0;    // legs released since output switched on
  float movingA = 0; // the moving draw of the last apply, while on (load 1)

  void apply(const body::BodyState &body, const joint_calibration::Table &calibration, int released);
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

} // namespace bench_model

#endif // ROBOT_BENCH_MODEL_H
