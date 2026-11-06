#include "bench_model.h"

#include <math.h>

namespace bench_model
{

void Servos::apply(const body::BodyState &body, const joint_calibration::Table &calibration, int released)
{
  movingA = 0;
  bool wantOn = !inhibited && body.outputsEnabled;
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    if (body.legUpdated[leg])
      legSet[leg] = true;
    if (!legSet[leg])
      continue;
    const kinematics::JointAngles &a = body.angles[leg];
    const float angles[kinematics::JOINTS_PER_LEG] = {a.coxa, a.femur, a.tibia};
    for (int j = 0; j < kinematics::JOINTS_PER_LEG; j++)
    {
      int i = kinematics::servoOutputIndex(leg, j);
      uint16_t us = joint_calibration::toPulse(calibration.joints[i], angles[j]);
      if (on && wantOn && desiredUs[i] != 0 && leg < legsOn)
        movingA += fminf(fabsf((float)us - (float)desiredUs[i]) * A_PER_US_PER_TICK, MAX_MOVING_A_PER_SERVO);
      desiredUs[i] = us;
    }
  }
  on = wantOn;
  legsOn = on ? released : 0;
}

int Servos::powered() const
{
  if (!on)
    return 0;
  int n = 0;
  for (int i = 0; i < OUTPUT_COUNT; i++)
    n += desiredUs[i] != 0 && i / kinematics::JOINTS_PER_LEG < legsOn;
  return n;
}

protection::Reading Supply::read(const Servos &servos) const
{
  protection::Reading r;
  r.voltageV = voltageV;
  if (fixedCurrent)
    r.currentA = fixedCurrentA;
  else if (!servos.on)
    r.currentA = 0;
  else
    // powered() is at most OUTPUT_COUNT (18): exact as a float.
    r.currentA = load * ((float)servos.powered() * IDLE_A_PER_SERVO + servos.movingA);
  return r;
}

} // namespace bench_model
