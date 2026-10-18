#include "link_receiver.h"

namespace link_receiver
{

MovementCommand toMovementCommand(const protocol::Command &c)
{
  MovementCommand m;
  bool connected = (c.flags & protocol::FLAG_CONTROLLER_CONNECTED) != 0;
  m.mode = c.mode <= (uint8_t)RequestedMode::Calibrate ? (RequestedMode)c.mode : RequestedMode::Sleep;
  if (!connected)
    m.mode = RequestedMode::Sleep;
  m.translate = Vector2(c.translateX, c.translateY);
  if (m.translate.magnitude() > 100)
  {
    m.translate.normalize();
    m.translate = m.translate * 100;
  }
  m.rotate = c.rotate < -100 ? -100 : (c.rotate > 100 ? 100 : c.rotate);
  m.height = c.height > 100 ? 100 : c.height;
  m.speed = c.speed > 100 ? 100 : c.speed;
  m.gait = c.gait; // the body ignores unknown gaits
  m.stride = c.stride == (uint8_t)StrideMode::Fixed ? StrideMode::Fixed : StrideMode::Dynamic;
  return m;
}

void accept(Receiver &r, const protocol::Command &c, uint32_t nowMs)
{
  bool afterGap = !r.haveFrame || nowMs - r.lastArrivalMs >= LINK_GAP_MS;
  if (!afterGap)
  {
    if (c.clearCounter != r.clearCounter)
      r.clearPending = true;
    if (c.modeRequestCounter != r.modeRequestCounter)
      r.requestSeq++;
  }
  r.clearCounter = c.clearCounter;
  r.modeRequestCounter = c.modeRequestCounter;
  r.last = c;
  r.lastArrivalMs = nowMs;
  r.haveFrame = true;
}

Input take(Receiver &r)
{
  Input in;
  in.linkSeen = r.haveFrame;
  in.requestSeq = r.requestSeq;
  in.clearRequested = r.clearPending;
  r.clearPending = false;
  if (!r.haveFrame)
  {
    in.command.mode = RequestedMode::Sleep;
    in.command.issuedAtMs = 0;
    in.controllerConnected = false;
    in.emergencyStop = false;
    return in;
  }
  in.command = toMovementCommand(r.last);
  in.command.issuedAtMs = r.lastArrivalMs;
  in.controllerConnected = (r.last.flags & protocol::FLAG_CONTROLLER_CONNECTED) != 0;
  in.emergencyStop = (r.last.flags & protocol::FLAG_EMERGENCY_STOP) != 0;
  return in;
}

} // namespace link_receiver
