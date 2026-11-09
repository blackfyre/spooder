#include "sim.h"

#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"

namespace sim
{

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
  c.speed = START_SPEED;
  c.gait = gamepad_mapping::START_GAIT; // a gamepad's start, as the bridge sends it
  c.flags = (state == State::Gamepad ? protocol::FLAG_CONTROLLER_CONNECTED : 0) |
            (estop ? protocol::FLAG_EMERGENCY_STOP : 0);
  c.clearCounter = clearCounter;
  c.modeRequestCounter = modeCounter;
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCommand(c, seq++, frame);
  out.insert(out.end(), frame, frame + n);
}

} // namespace sim
