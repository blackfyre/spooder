#include "sim.h"

#include <math.h>
#include "protocol/protocol.h"

namespace sim
{

void Servos::apply(const body::BodyState &body, const joint_calibration::Table &calibration)
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
      if (on && wantOn && desiredUs[i] != 0)
        movingA += fminf(fabsf((float)us - (float)desiredUs[i]) * A_PER_US_PER_TICK, MAX_MOVING_A_PER_SERVO);
      desiredUs[i] = us;
    }
  }
  on = wantOn;
}

int Servos::powered() const
{
  if (!on)
    return 0;
  int n = 0;
  for (uint16_t us : desiredUs)
    n += us != 0;
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
    r.currentA = load * (servos.powered() * IDLE_A_PER_SERVO + servos.movingA);
  return r;
}

void Bridge::setState(State s)
{
  if (s == State::Gamepad && state != State::Gamepad)
    framesUntilRequest = 3; // after the first frame has set the baselines
  state = s;
}

void Bridge::frame(std::vector<uint8_t> &out)
{
  if (state == State::Off)
    return;
  if (framesUntilRequest > 0 && --framesUntilRequest == 0)
  {
    modeCounter++;
    framesUntilRequest = -1;
  }
  protocol::Command c = {};
  c.mode = (uint8_t)(state == State::Gamepad ? RequestedMode::Stand : RequestedMode::Sleep);
  c.speed = 44;
  c.flags = (state == State::Gamepad ? protocol::FLAG_CONTROLLER_CONNECTED : 0) |
            (estop ? protocol::FLAG_EMERGENCY_STOP : 0);
  c.clearCounter = clearCounter;
  c.modeRequestCounter = modeCounter;
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCommand(c, seq++, frame);
  out.insert(out.end(), frame, frame + n);
}

} // namespace sim
