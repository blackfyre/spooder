#include "handheld_controller.h"

namespace handheld_controller
{

static bool sameSender(const Sender &a, const Sender &b)
{
  return a.address == b.address && a.port == b.port;
}

static void endSession(Controller &c)
{
  c.session = false;
  c.epoch++;
  c.mode = RequestedMode::Sleep;
}

void judge(Controller &c, uint32_t nowMs)
{
  if (c.session && nowMs - c.lastMs > LIVENESS_MS)
    endSession(c);
}

bool live(Controller &c, uint32_t nowMs)
{
  judge(c, nowMs);
  return c.session;
}

uint32_t epoch(Controller &c, uint32_t nowMs)
{
  judge(c, nowMs);
  return c.epoch;
}

bool budgetSpent(const Controller &c, uint32_t nowMs)
{
  return c.windowStarted && c.window == nowMs / WINDOW_MS && c.windowUsed >= WINDOW_BUDGET;
}

uint32_t readDelayMs(const Controller &c, uint32_t nowMs)
{
  return budgetSpent(c, nowMs) ? WINDOW_MS - nowMs % WINDOW_MS : 0;
}

// Exactly one valid command frame of this protocol version, and nothing else.
static bool decodeCommand(const uint8_t *datagram, int length, protocol::Frame &f)
{
  if (datagram == nullptr || length != protocol::COMMAND_FRAME_SIZE)
    return false;
  protocol::Decoder dec;
  for (int i = 0; i < length; i++)
    dec.push(datagram[i]);
  if (!dec.next(f) || f.type != protocol::FrameType::Command)
    return false;
  // The frame is the whole datagram: a frame found after a byte of garbage
  // would be shorter than the datagram.
  return dec.frameLength() == length && dec.discardedFrames() == 0;
}

static Outcome judgeDatagram(Controller &c, const uint8_t *datagram, int length, Sender sender, uint32_t nowMs)
{
  uint32_t window = nowMs / WINDOW_MS;
  if (!c.windowStarted || c.window != window)
  {
    c.windowStarted = true;
    c.window = window;
    c.windowUsed = 0;
  }
  if (c.windowUsed >= WINDOW_BUDGET)
    return Outcome::RateLimited;
  c.windowUsed++;

  protocol::Frame f;
  if (!decodeCommand(datagram, length, f))
    return Outcome::Malformed;

  judge(c, nowMs);
  const protocol::Command &cmd = f.command;
  if (c.session)
  {
    if (!sameSender(sender, c.sender))
      return Outcome::OtherSender;
    uint8_t d = (uint8_t)(f.seq - c.lastSeq);
    if (d == 0 || d >= 128)
      return Outcome::Stale;
    if (d > SEQ_WINDOW)
    {
      endSession(c);
      return Outcome::Jump;
    }
    // Newer: translate the counters' changes.
    if (cmd.clearCounter != c.last.clearCounter)
      c.clearCounter++;
    if (cmd.modeRequestCounter != c.last.modeRequestCounter)
    {
      c.modeRequestCounter++;
      c.mode = (RequestedMode)cmd.mode;
    }
    if (cmd.gestureRequestCounter != c.last.gestureRequestCounter)
    {
      c.gestureRequestCounter++;
      c.gesture = cmd.gesture;
    }
  }
  else
  {
    // A new session: the frame only sets the baselines.
    c.session = true;
    c.epoch++;
    c.sender = sender;
    c.mode = RequestedMode::Sleep;
  }
  c.lastSeq = f.seq;
  c.lastMs = nowMs;
  c.last = cmd;
  c.emergencyStop = (cmd.flags & protocol::FLAG_EMERGENCY_STOP) != 0;
  return Outcome::Accepted;
}

Outcome accept(Controller &c, const uint8_t *datagram, int length, Sender sender, uint32_t nowMs)
{
  Outcome o = judgeDatagram(c, datagram, length, sender, nowMs);
  c.counts[(int)o]++;
  return o;
}

bool relayTarget(Controller &c, uint32_t nowMs, Sender &out)
{
  if (!live(c, nowMs))
    return false;
  out = c.sender;
  return true;
}

gamepad_mapping::Desired desired(Controller &c, uint32_t nowMs)
{
  bool on = live(c, nowMs);
  gamepad_mapping::Desired d = {};
  d.mode = c.mode;
  d.clearCounter = c.clearCounter;
  d.modeRequestCounter = c.modeRequestCounter;
  d.gesture = c.gesture;
  d.gestureRequestCounter = c.gestureRequestCounter;
  d.stickMode = gamepad_mapping::StickMode::Steer; // not sent: the handheld maps its own sticks
  if (!on)
  {
    d.speed = DEFAULT_SPEED;
    d.gait = gamepad_mapping::START_GAIT;
    d.stride = StrideMode::Dynamic;
    return d;
  }
  const protocol::Command &f = c.last;
  d.translateX = f.translateX;
  d.translateY = f.translateY;
  d.rotate = f.rotate;
  d.height = f.height;
  d.speed = f.speed;
  d.gait = f.gait;
  d.stride = (StrideMode)f.stride;
  d.controllerConnected = (f.flags & protocol::FLAG_CONTROLLER_CONNECTED) != 0;
  return d;
}

bool emergencyStop(const Controller &c)
{
  return c.emergencyStop;
}

State state(Controller &c, uint32_t nowMs)
{
  if (!live(c, nowMs))
    return State::None;
  return (c.last.flags & protocol::FLAG_CONTROLLER_CONNECTED) ? State::InControl : State::Idle;
}

uint32_t count(const Controller &c, Outcome o)
{
  return c.counts[(int)o];
}

} // namespace handheld_controller
