#include "robot_tick.h"
#include "movement/servo_profile.h"

#include <stdarg.h>
#include <stdio.h>

namespace robot_tick
{

// The LED pattern mirrors body::Mode by value.
static_assert((int)led_pattern::BodyMode::Walk == (int)body::Mode::Walk &&
                  (int)led_pattern::BodyMode::Calibrate == (int)body::Mode::Calibrate &&
                  (int)led_pattern::BodyMode::Perform == (int)body::Mode::Perform,
              "led_pattern::BodyMode must mirror body::Mode");

// One tick's operator input: from the bridge or USB link, whichever is in
// control, or (bring-up builds) the USB serial console keys.
struct TickInput
{
  MovementCommand command;
  uint32_t requestSeq; // advances on every explicit mode request
  bool clearFaults;
  bool emergencyStop;
  bool printReadings;
  led_pattern::Input source;
  uint8_t telemetryFlags; // protocol::TELEMETRY_* source bits
};

static void logf(Hardware &hw, const char *format, ...) __attribute__((format(printf, 2, 3)));

static void logf(Hardware &hw, const char *format, ...)
{
  char line[MAX_LOG_LINE];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof line, format, args);
  va_end(args);
  hw.log(line);
}

static bool isFresh(const MovementCommand &c, uint32_t now)
{
  return c.issuedAtMs != 0 && now - c.issuedAtMs <= body::COMMAND_TIMEOUT_MS;
}

static void handleFrames(LinkEndpoint &e, uint32_t nowMs)
{
  protocol::Frame f;
  while (e.decoder.next(f))
  {
    if (f.type == protocol::FrameType::Command)
      link_receiver::accept(e.receiver, f.command, nowMs);
    else if (f.type == protocol::FrameType::CalibrationSet)
      link_receiver::acceptCalibration(e.receiver, f.calibrationSet);
  }
}

// Decodes one link's bytes and returns this tick's input from it (the newest
// valid command). Calibration sets are queued for processCalibration().
static link_receiver::Input poll(LinkEndpoint &e, const uint8_t *bytes, int count, uint32_t nowMs)
{
  for (int i = 0; i < count; i++)
  {
    e.decoder.push(bytes[i]);
    handleFrames(e, nowMs);
  }
  // A tick (20 ms) without a byte is longer than any gap inside a frame, so
  // an incomplete candidate is a false start.
  if (count == 0)
  {
    e.decoder.idle();
    handleFrames(e, nowMs);
  }
  return link_receiver::take(e.receiver);
}

static TickInput readInput(RobotState &s, const Inputs &in, uint32_t now, Hardware &hw)
{
  TickInput t = {};
  if (s.inputMode == InputMode::Serial)
  {
    t.command = in.serial.command;
    t.requestSeq = in.serial.requestSeq;
    t.clearFaults = in.serial.clearFaults;
    t.printReadings = in.serial.printReadings;
    t.source = led_pattern::Input::Serial;
    return t;
  }
  LinkEndpoint &bridgeLink = s.links[(int)Link::Bridge];
  LinkEndpoint &usbLink = s.links[(int)Link::Usb];
  link_receiver::Input bridge = poll(bridgeLink, in.bytes[(int)Link::Bridge], in.byteCount[(int)Link::Bridge], now);
  link_receiver::Input usb = poll(usbLink, in.bytes[(int)Link::Usb], in.byteCount[(int)Link::Usb], now);
  link_receiver::Arbitration a = link_receiver::arbitrate(s.arbiter, bridge, usb, now);
  // The bridge link as every link's telemetry reports it.
  bool bridgeUp = bridge.linkSeen && now - bridge.command.issuedAtMs < link_receiver::LINK_GAP_MS;
  s.bridgeFlags = (bridgeUp ? protocol::TELEMETRY_BRIDGE_LINK_UP : 0) |
                  (bridgeUp && bridge.controllerConnected ? protocol::TELEMETRY_BRIDGE_GAMEPAD : 0);
  t.command = a.input.command;
  t.requestSeq = a.input.requestSeq;
  t.clearFaults = a.input.clearRequested;
  t.emergencyStop = a.input.emergencyStop; // from either link
  t.telemetryFlags = a.telemetryFlags;
  switch (a.source)
  {
  case link_receiver::Source::Bridge:
    t.source = led_pattern::Input::LinkConnected;
    break;
  case link_receiver::Source::Usb:
    t.source = led_pattern::Input::UsbConnected;
    break;
  case link_receiver::Source::Conflict:
    t.source = led_pattern::Input::SourceConflict;
    break;
  case link_receiver::Source::None:
    t.source = link_receiver::isFresh(bridge, now) ? led_pattern::Input::LinkNoGamepad : led_pattern::Input::LinkStale;
    break;
  }
  if (a.source != s.lastSource)
  {
    static const char *const NAMES[] = {"none", "bridge", "USB", "conflict (bridge and USB): sleeping"};
    logf(hw, "operator source: %s\n", NAMES[(int)a.source]);
    s.lastSource = a.source;
  }
  return t;
}

static uint16_t clampU16(float v)
{
  if (v <= 0)
    return 0;
  if (v >= 65535.0f)
    return 65535;
  return (uint16_t)(v + 0.5f);
}

// Encodes and sends a frame on one link; the sequence number advances only if
// it was sent.
static bool sendTelemetryOn(RobotState &s, Link link, const protocol::Telemetry &t, Hardware &hw)
{
  LinkEndpoint &e = s.links[(int)link];
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeTelemetry(t, e.txSeq, frame);
  if (!hw.sendFrame(link, frame, n))
    return false;
  e.txSeq++;
  return true;
}

static bool sendCalibrationReportOn(RobotState &s, Link link, Hardware &hw)
{
  LinkEndpoint &e = s.links[(int)link];
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCalibrationReport(link_receiver::makeReport(e.receiver, s.requested), e.txSeq, frame);
  if (!hw.sendFrame(link, frame, n))
    return false;
  e.txSeq++;
  return true;
}

// USB only: the bridge link has no pose.
static bool sendPose(RobotState &s, Hardware &hw)
{
  LinkEndpoint &e = s.links[(int)Link::Usb];
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodePose(link_receiver::makePose(s.body, hw.servoPulses(), hw.servosOn()), e.txSeq, frame);
  if (!hw.sendFrame(Link::Usb, frame, n))
    return false;
  e.txSeq++;
  return true;
}

static uint16_t clamp16(float v)
{
  return v <= 0 ? 0 : (v >= 65535 ? 65535 : (uint16_t)lroundf(v));
}

// The diagnostics for the window just ended; starts the next window.
static protocol::Diagnostics windowDiagnostics(RobotState &s, const protection::Result &p)
{
  const protection::Limits &l = s.protection.limits;
  protocol::Diagnostics d = {};
  d.motionSharePermille = clamp16(s.windowMotionShare * 1000);
  d.backstopTicks = (uint16_t)s.backstopTicks;
  d.tickMaxUs = clamp16((float)s.windowTickMaxUs);
  d.tickMeanUs = s.windowTicks ? clamp16((float)s.windowTickSumUs / s.windowTicks) : 0;
  d.averageCurrentMa = clamp16(p.averageCurrentA * 1000);
  d.tripCurrentMa = clamp16(l.tripCurrentA * 1000);
  d.sustainedCurrentMa = clamp16(l.sustainedCurrentA * 1000);
  d.minMv = clamp16(l.minV * 1000);
  d.maxMv = clamp16(l.maxV * 1000);
  d.brownoutMv = clamp16(l.brownoutV * 1000);
  d.budgetDegPerS = clamp16(s.jointBudgetDegPerS);
  d.ratedDegPerS = clamp16(s.backstopDegPerS);
  s.windowMotionShare = 1;
  s.windowTickMaxUs = s.windowTickSumUs = s.windowTicks = 0;
  return d;
}

static bool sendDiagnostics(RobotState &s, const protection::Result &p, Hardware &hw)
{
  LinkEndpoint &e = s.links[(int)Link::Usb];
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeDiagnostics(windowDiagnostics(s, p), e.txSeq, frame);
  if (!hw.sendFrame(Link::Usb, frame, n))
    return false;
  e.txSeq++;
  return true;
}

// @p gesture is the gesture being played, 0 when none or the body is stopped.
static void sendTelemetry(RobotState &s, const protection::Reading &r, const protection::Result &p, bool outputsOn,
                          uint8_t flags, uint8_t gesture, Hardware &hw)
{
  protocol::Telemetry t = {};
  t.bodyMode = (uint8_t)s.body.mode;
  t.gait = (uint8_t)s.body.currentGait;
  t.outputsOn = outputsOn;
  t.faults = p.faults;
  t.voltageMv = clampU16(r.voltageV * 1000.0f);
  t.currentMa = clampU16(r.currentA * 1000.0f);
  t.sensorsAvailable = 0; // filled by add-body-sensing
  t.flags = flags | s.bridgeFlags;
  t.gesture = gesture;
  const protocol::Decoder &bridge = s.links[(int)Link::Bridge].decoder;
  t.bridgeValidFrames = (uint16_t)bridge.validFrames();
  t.bridgeDiscardedFrames = (uint16_t)bridge.discardedFrames();
  // Each link reports its own frame counters.
  for (int i = 0; i < LINK_COUNT; i++) // the bridge, then USB
  {
    Link link = (Link)i;
    const protocol::Decoder &d = s.links[(int)link].decoder;
    t.validFrames = (uint16_t)d.validFrames();
    t.discardedFrames = (uint16_t)d.discardedFrames();
    sendTelemetryOn(s, link, t, hw);
  }
}

static led_pattern::State ledState(const RobotState &s, uint32_t now, const protection::Result &prot,
                                   bool bodyRunning, float currentA, led_pattern::Input source)
{
  led_pattern::State l;
  l.nowMs = now;
  l.faults = prot.faults;
  l.supplyAccepted = prot.allowOutput;
  l.bodyRunning = bodyRunning;
  l.mode = (led_pattern::BodyMode)s.body.mode;
  l.gait = (int)s.body.currentGait;
  l.currentA = currentA;
  l.tripCurrentA = s.tripCurrentA;
  l.sustainedCurrentA = s.sustainedCurrentA;
  l.input = source;
  return l;
}

static const char *faultName(uint16_t fault)
{
  switch (fault)
  {
  case protection::OVERCURRENT:
    return "overcurrent";
  case protection::SUSTAINED_CURRENT:
    return "sustained current";
  case protection::UNDERVOLTAGE:
    return "undervoltage";
  case protection::BROWNOUT:
    return "brown-out";
  case protection::OVERVOLTAGE:
    return "overvoltage";
  case protection::EMERGENCY_STOP:
    return "emergency stop";
  default:
    return "unknown";
  }
}

static void reportFaults(uint16_t faults, const protection::Reading &r, Hardware &hw)
{
  for (uint16_t bit = 1; bit != 0; bit <<= 1)
    if (faults & bit)
      logf(hw, "fault latched: %s (%.2f A, %.2f V)\n", faultName(bit), r.currentA, r.voltageV);
}

static void reportStatus(const RobotState &s, const protection::Reading &r, const protection::Result &p,
                         bool outputsOn, bool bodyRunning, Hardware &hw)
{
  logf(hw, "supply %.2f A (avg %.2f A), %.2f V%s; output %s; body %s, mode %d; faults 0x%02x\n", r.currentA,
       p.averageCurrentA, r.voltageV, p.supplyValid ? "" : " (out of range or absent)", outputsOn ? "on" : "off",
       bodyRunning ? "running" : "stopped", (int)s.body.mode, p.faults);
}

// The body state handed to the servos: the body's, with each joint's angle
// limited to the backstop speed since the last tick. Static: a BodyState is
// too large for the board's main stack.
static body::BodyState output;

static float stepTowards(float &value, float target, float maxStep)
{
  float behind = fabsf(target - value);
  if (behind <= maxStep)
  {
    value = target;
    return 0;
  }
  value += target > value ? maxStep : -maxStep;
  return behind - maxStep;
}

static const body::BodyState &limitOutput(RobotState &s, uint32_t elapsedMs)
{
  output = s.body;
  float maxStep = s.backstopDegPerS * (elapsedMs / 1000.0f);
  bool heldBack = false;
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    // A leg not yet switched on: its servos' positions are unknown, so its
    // first pulse goes straight to the command.
    if (leg >= s.legsOn)
    {
      s.outputKnown[leg] = false;
      continue;
    }
    kinematics::JointAngles &out = s.outputAngles[leg];
    const kinematics::JointAngles &target = s.body.angles[leg];
    if (!s.outputKnown[leg])
    {
      if (s.body.legUpdated[leg])
      {
        out = target;
        s.outputKnown[leg] = true;
      }
      continue;
    }
    // Initialise sets its poses as step changes, by design: not limited.
    if (s.backstopDegPerS <= 0 || s.body.mode == body::Mode::Initialise)
    {
      out = target;
      continue;
    }
    float *joints[kinematics::JOINTS_PER_LEG] = {&out.coxa, &out.femur, &out.tibia};
    const float targets[kinematics::JOINTS_PER_LEG] = {target.coxa, target.femur, target.tibia};
    bool moved = false;
    for (int j = 0; j < kinematics::JOINTS_PER_LEG; j++)
    {
      float before = *joints[j];
      float behind = stepTowards(*joints[j], targets[j], maxStep);
      moved = moved || *joints[j] != before;
      if (behind > 0)
      {
        heldBack = true;
        if (behind > s.backstopWorstDeg)
        {
          s.backstopWorstDeg = behind;
          s.backstopWorstJoint = kinematics::servoOutputIndex(leg, j);
        }
      }
    }
    // A leg held back keeps moving towards its target on later ticks.
    output.legUpdated[leg] = s.body.legUpdated[leg] || moved;
    output.angles[leg] = out;
  }
  if (heldBack)
    s.backstopTicks++;
  return output;
}

static void reportBackstop(RobotState &s, Hardware &hw)
{
  if (s.backstopTicks == s.backstopReported)
    return;
  static const char *const JOINT_NAMES[kinematics::JOINTS_PER_LEG] = {"coxa", "femur", "tibia"};
  int leg = s.backstopWorstJoint / kinematics::JOINTS_PER_LEG, joint = s.backstopWorstJoint % kinematics::JOINTS_PER_LEG;
  logf(hw, "output rate backstop: %lu ticks held back; most: leg %d %s, %.1f deg behind\n",
       (unsigned long)s.backstopTicks, leg, JOINT_NAMES[joint], s.backstopWorstDeg);
  s.backstopReported = s.backstopTicks;
  s.backstopWorstDeg = 0;
  s.backstopWorstJoint = -1;
}

static void reportRejections(const RobotState &s, Hardware &hw)
{
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    if (!s.body.newlyRejected[leg])
      continue;
    const Vector3 &t = s.body.rejectedTarget[leg];
    logf(hw, "point impossible to reach: (%.1f, %.1f, %.1f) leg %d\n", t.x, t.y, t.z, leg);
  }
}

void init(RobotState &s, InputMode mode, const protection::Limits &limits, uint32_t nowMs)
{
  // No whole-state assignment: a RobotState temporary would not fit the
  // board's 2 KB main stack. The rest is as default-constructed.
  s.inputMode = mode;
  s.tripCurrentA = limits.tripCurrentA;
  s.sustainedCurrentA = limits.sustainedCurrentA;
  protection::init(s.protection, limits);
  joint_calibration::loadDefaults(s.requested);
  joint_calibration::loadDefaults(s.applied);
  s.jointBudgetDegPerS = servo_profile::SELECTED.budgetDegPerS();
  s.backstopDegPerS = servo_profile::SELECTED.noLoadDegPerS;
  s.lastTickMs = nowMs;
}

void tick(RobotState &s, const Inputs &in, uint32_t now, Hardware &hw)
{
  const bool links = s.inputMode == InputMode::Links;
  if (!s.profileAnnounced)
  {
    s.profileAnnounced = true;
    logf(hw, "servo profile %s: joint speed budget %.0f deg/s\n", servo_profile::SELECTED.name, s.jointBudgetDegPerS);
  }
  if (s.previousMotionShare < s.windowMotionShare)
    s.windowMotionShare = s.previousMotionShare;
  s.previousMotionShare = 1;
  if (in.previousTickUs > 0)
  {
    if (in.previousTickUs > s.windowTickMaxUs)
      s.windowTickMaxUs = in.previousTickUs;
    s.windowTickSumUs += in.previousTickUs;
    s.windowTicks++;
  }
  uint32_t elapsedMs = now - s.lastTickMs;
  s.lastTickMs = now;
  if (elapsedMs > body::MAX_STEP_MS)
    elapsedMs = body::MAX_STEP_MS; // an overrun never makes one large calibration step

  // Input first, so an emergency stop in a frame that has arrived acts in
  // this tick. Both input sources are bounded and never block, so they
  // cannot delay the protection step that follows.
  TickInput input = readInput(s, in, now, hw);
  // Movement tells a new explicit mode request from a continuing one.
  input.command.modeRequestSeq = input.requestSeq;

  const protection::Reading &reading = in.power;
  protection::Result prot = protection::step(s.protection, reading, now, input.clearFaults, input.emergencyStop);
  hw.setInhibited(!prot.allowOutput);
  if (prot.newlyLatched)
    reportFaults(prot.newlyLatched, reading, hw);

  // A stale command never starts the body; the body itself sleeps on it.
  RequestedMode requested = isFresh(input.command, now) ? input.command.mode : RequestedMode::Sleep;

  // The supervisor decides whether the body runs: a fault that cuts output
  // stops it, and it restarts through the boot sequence only once output is
  // allowed and the operator has made a new request since the fault.
  protection::Decision run = protection::supervise(s.supervisor, prot, requested, input.requestSeq);
  if (run.stopBody)
  {
    hw.stopServos();
    logf(hw, "movement stopped: waiting for the fault to clear and a new request\n");
  }
  if (run.startBody)
  {
    body::init(s.body);
    logf(hw, "output allowed: starting movement\n");
  }
  // Calibration changes are guarded with this tick's output state and the
  // body's mode; applied values then move towards them at the rate limit.
  if (links)
  {
    bool calibrating = run.runBody && s.body.mode == body::Mode::Calibrate;
    int processed = 0;
    for (LinkEndpoint &e : s.links)
      processed += link_receiver::processCalibration(e.receiver, s.requested, calibrating, prot.allowOutput);
    if (processed > 0)
    {
      // Every link hears about every change.
      for (LinkEndpoint &e : s.links)
        link_receiver::requestReport(e.reports);
    }
  }
  joint_calibration::step(s.applied, s.requested, elapsedMs / 1000.0f);

  if (run.runBody)
  {
    MovementCommand cmd = input.command;
    cmd.mode = run.mode; // protection only rewrites the request
    for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
      body::setOffsets(s.body, leg, joint_calibration::legOffsets(s.applied, leg));
    s.body.jointBudgetDegPerS = s.jointBudgetDegPerS;
    body::step(s.body, cmd, now);
    s.previousMotionShare = s.body.motionScale;
    reportRejections(s, hw);
    // Output switches on in groups of legs, one group per tick. While it is
    // off, no leg's servo position is known.
    if (hw.servosOn())
      s.legsOn = s.legsOn + SWITCH_ON_LEGS_PER_TICK < kinematics::LEG_COUNT ? s.legsOn + SWITCH_ON_LEGS_PER_TICK
                                                                             : kinematics::LEG_COUNT;
    else
    {
      s.legsOn = SWITCH_ON_LEGS_PER_TICK;
      for (bool &known : s.outputKnown)
        known = false;
    }
    hw.applyServos(limitOutput(s, elapsedMs), s.applied, s.legsOn);
  }

  hw.showLeds(ledState(s, now, prot, run.runBody, reading.currentA, input.source));

  if (links)
  {
    if (++s.telemetryTick >= TELEMETRY_EVERY_TICKS)
    {
      s.telemetryTick = 0;
      sendTelemetry(s, reading, prot, hw.servosOn(), input.telemetryFlags,
                    run.runBody ? body::playingGesture(s.body) : 0, hw);
    }
    if (s.telemetryTick == POSE_TICK)
      sendPose(s, hw);
    if (s.telemetryTick == DIAGNOSTICS_TICK)
      sendDiagnostics(s, prot, hw);

    // In Calibrate, front-ends follow the table through these reports.
    bool calibrating = run.runBody && s.body.mode == body::Mode::Calibrate;
    for (int i = 0; i < LINK_COUNT; i++) // the bridge, then USB
    {
      Link link = (Link)i;
      link_receiver::ReportSchedule &r = s.links[(int)link].reports;
      if (link_receiver::reportDue(r, calibrating, now) && sendCalibrationReportOn(s, link, hw))
        link_receiver::reportSent(r, now);
    }
  }

  if (input.printReadings || now - s.lastStatusMs >= STATUS_PERIOD_MS)
  {
    reportStatus(s, reading, prot, hw.servosOn(), run.runBody, hw);
    reportBackstop(s, hw);
    s.lastStatusMs = now;
  }
}

} // namespace robot_tick
