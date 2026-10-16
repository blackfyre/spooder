#include "servo_output.h"

#include "servo2040.hpp"

using namespace servo;

namespace servo_output
{

constexpr int OUTPUT_COUNT = kinematics::LEG_COUNT * kinematics::JOINTS_PER_LEG;
static_assert(OUTPUT_COUNT == servo2040::NUM_SERVOS, "one output per joint");

static ServoCluster cluster(pio0, 0, servo2040::SERVO_1, OUTPUT_COUNT);

// Last pulse the body asked for, per output; 0 = never set. Kept while output
// is off, so switching on restores the commanded pose instead of ServoCluster's
// default mid-position.
static uint16_t desiredUs[OUTPUT_COUNT];
static bool on = false;
static bool inhibited = true;

bool setup()
{
  if (!cluster.init())
    return false;
  cluster.disable_all(); // no pulses until the body and protection allow them
  on = false;
  return true;
}

void setInhibited(bool value)
{
  inhibited = value;
}

static void switchOff()
{
  cluster.disable_all(false);
  on = false;
}

void apply(const body::BodyState &body)
{
  bool changed[OUTPUT_COUNT] = {};
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    if (!body.legUpdated[leg])
      continue;
    const kinematics::JointAngles &a = body.angles[leg];
    const float angles[kinematics::JOINTS_PER_LEG] = {a.coxa, a.femur, a.tibia};
    for (int j = 0; j < kinematics::JOINTS_PER_LEG; j++)
    {
      int i = kinematics::servoOutputIndex(leg, j);
      uint16_t us = kinematics::angleToMicroseconds(angles[j]);
      if (us != desiredUs[i])
      {
        desiredUs[i] = us;
        changed[i] = true;
      }
    }
  }

  bool wantOn = !inhibited && body.outputsEnabled;
  if (!wantOn)
  {
    if (on)
    {
      switchOff();
      cluster.load();
    }
    return;
  }

  // pulse() also enables a disabled output, so this both switches on and
  // updates. Outputs the body has never set stay disabled rather than going
  // to ServoCluster's mid-position.
  bool any = false;
  for (int i = 0; i < OUTPUT_COUNT; i++)
  {
    if (desiredUs[i] == 0 || (on && !changed[i]))
      continue;
    cluster.pulse(i, desiredUs[i], false);
    any = true;
  }
  on = true;
  if (any)
    cluster.load();
}

bool outputsOn()
{
  return on;
}

} // namespace servo_output
