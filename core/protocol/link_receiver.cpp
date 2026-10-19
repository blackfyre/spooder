#include "link_receiver.h"

#include <math.h>

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
    if (c.gestureRequestCounter != r.gestureCounter)
    {
      r.gestureSeq++;
      r.gesture = c.gesture;
    }
  }
  r.clearCounter = c.clearCounter;
  r.modeRequestCounter = c.modeRequestCounter;
  r.gestureCounter = c.gestureRequestCounter;
  r.last = c;
  r.lastArrivalMs = nowMs;
  r.haveFrame = true;
}

Input take(Receiver &r)
{
  Input in;
  in.linkSeen = r.haveFrame;
  in.requestSeq = r.requestSeq;
  in.gestureSeq = r.gestureSeq;
  in.gesture = r.gesture;
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

bool isFresh(const Input &in, uint32_t nowMs)
{
  return in.command.issuedAtMs != 0 && nowMs - in.command.issuedAtMs <= body::COMMAND_TIMEOUT_MS;
}

void acceptCalibration(Receiver &r, const protocol::CalibrationSet &c)
{
  if (r.pendingSetCount >= MAX_PENDING_SETS)
    return;
  r.pendingSets[r.pendingSetCount++] = c;
}

static protocol::CalibrationResult guard(const protocol::CalibrationSet &c, joint_calibration::Table &requested,
                                         bool calibrating, bool allowOutput)
{
  if (!calibrating || !allowOutput || c.joint >= joint_calibration::JOINT_COUNT)
    return protocol::CalibrationResult::Rejected;
  joint_calibration::Joint j;
  j.offsetDeg = c.offsetCentideg / 100.0f;
  j.pulse0Us = c.pulse0Us;
  j.pulse180Us = c.pulse180Us;
  j.reversed = (c.flags & protocol::CALIBRATION_REVERSED) != 0;
  bool clamped = joint_calibration::clampJoint(j, joint_calibration::DEFAULTS[c.joint]);
  requested.joints[c.joint] = j;
  return clamped ? protocol::CalibrationResult::Clamped : protocol::CalibrationResult::Accepted;
}

int processCalibration(Receiver &r, joint_calibration::Table &requested, bool calibrating, bool allowOutput)
{
  int processed = r.pendingSetCount;
  for (int i = 0; i < r.pendingSetCount; i++)
  {
    const protocol::CalibrationSet &c = r.pendingSets[i];
    if (r.haveCalibrationRequest && c.requestId == r.lastRequestId)
      continue; // retransmission: report again, change nothing
    r.lastResult = guard(c, requested, calibrating, allowOutput);
    r.lastRequestId = c.requestId;
    r.haveCalibrationRequest = true;
  }
  r.pendingSetCount = 0;
  return processed;
}

protocol::CalibrationReport makeReport(const Receiver &r, const joint_calibration::Table &table)
{
  static_assert(protocol::CALIBRATION_JOINTS == joint_calibration::JOINT_COUNT, "one report entry per joint");
  protocol::CalibrationReport rep = {};
  rep.lastRequestId = r.lastRequestId;
  rep.result = (uint8_t)r.lastResult;
  for (int i = 0; i < joint_calibration::JOINT_COUNT; i++)
  {
    const joint_calibration::Joint &j = table.joints[i];
    rep.joints[i].offsetCentideg = (int16_t)lroundf(j.offsetDeg * 100.0f);
    rep.joints[i].pulse0Us = (uint16_t)lroundf(j.pulse0Us);
    rep.joints[i].pulse180Us = (uint16_t)lroundf(j.pulse180Us);
    rep.joints[i].flags = j.reversed ? protocol::CALIBRATION_REVERSED : 0;
  }
  return rep;
}

static int16_t centidegrees(float deg)
{
  float v = deg * 100.0f;
  if (!(v > -32768.0f)) // also catches NaN
    return v != v ? 0 : -32768;
  if (v >= 32767.0f)
    return 32767;
  return (int16_t)lroundf(v);
}

protocol::Pose makePose(const body::BodyState &body, const uint16_t *pulses, bool outputsOn)
{
  static_assert(protocol::CALIBRATION_JOINTS == kinematics::LEG_COUNT * kinematics::JOINTS_PER_LEG,
                "one pose entry per joint");
  protocol::Pose p = {};
  p.flags = outputsOn ? protocol::POSE_OUTPUTS_ON : 0;
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    const kinematics::JointAngles &a = body.angles[leg];
    const Vector3 &o = body.offsets[leg];
    const float servo[kinematics::JOINTS_PER_LEG] = {a.coxa, a.femur, a.tibia};
    const float offset[kinematics::JOINTS_PER_LEG] = {o.x, o.y, o.z};
    for (int j = 0; j < kinematics::JOINTS_PER_LEG; j++)
    {
      int i = kinematics::servoOutputIndex(leg, j);
      p.joints[i].commandedCentideg = centidegrees(servo[j] - offset[j]);
      p.joints[i].servoCentideg = centidegrees(servo[j]);
      p.joints[i].pulseUs = pulses[i];
    }
  }
  return p;
}

void requestReport(ReportSchedule &s)
{
  s.due = true;
}

bool reportDue(ReportSchedule &s, bool calibrating, uint32_t nowMs)
{
  uint32_t since = nowMs - s.lastSentMs;
  if (calibrating && (!s.sentOnce || since >= CALIBRATION_REPORT_PERIOD_MS))
    s.due = true;
  return s.due && (!s.sentOnce || since >= MIN_REPORT_INTERVAL_MS);
}

void reportSent(ReportSchedule &s, uint32_t nowMs)
{
  s.due = false;
  s.sentOnce = true;
  s.lastSentMs = nowMs;
}

static bool claimsControl(const Input &in, uint32_t nowMs)
{
  return in.controllerConnected && isFresh(in, nowMs);
}

Arbitration arbitrate(Arbiter &a, const Input &bridge, const Input &usb, uint32_t nowMs)
{
  bool b = claimsControl(bridge, nowMs);
  bool u = claimsControl(usb, nowMs);
  Arbitration out;
  out.source = b && u ? Source::Conflict : (b ? Source::Bridge : (u ? Source::Usb : Source::None));

  // Only the source in control's own counter counts as a new request.
  bool bridgeAdvanced = a.started && bridge.requestSeq != a.bridgeSeq;
  bool usbAdvanced = a.started && usb.requestSeq != a.usbSeq;
  if ((out.source == Source::Bridge && bridgeAdvanced) || (out.source == Source::Usb && usbAdvanced))
    a.requestSeq++;
  bool bridgeGesture = a.started && bridge.gestureSeq != a.bridgeGestureSeq;
  bool usbGesture = a.started && usb.gestureSeq != a.usbGestureSeq;
  if (out.source == Source::Bridge && bridgeGesture)
  {
    a.gestureSeq++;
    a.gesture = bridge.gesture;
  }
  else if (out.source == Source::Usb && usbGesture)
  {
    a.gestureSeq++;
    a.gesture = usb.gesture;
  }
  a.bridgeSeq = bridge.requestSeq;
  a.usbSeq = usb.requestSeq;
  a.bridgeGestureSeq = bridge.gestureSeq;
  a.usbGestureSeq = usb.gestureSeq;
  a.started = true;

  switch (out.source)
  {
  case Source::Bridge:
    out.input = bridge;
    out.telemetryFlags = protocol::TELEMETRY_BRIDGE_IN_CONTROL;
    break;
  case Source::Usb:
    out.input = usb;
    out.telemetryFlags = protocol::TELEMETRY_USB_IN_CONTROL;
    break;
  default:
    // No source in control: a fresh Sleep request, and no fault clear.
    out.input.command = MovementCommand();
    out.input.command.mode = RequestedMode::Sleep;
    out.input.command.issuedAtMs = nowMs;
    out.input.controllerConnected = false;
    out.input.clearRequested = false;
    out.telemetryFlags = out.source == Source::Conflict ? protocol::TELEMETRY_SOURCE_CONFLICT : 0;
    break;
  }
  out.input.linkSeen = bridge.linkSeen || usb.linkSeen;
  out.input.emergencyStop = bridge.emergencyStop || usb.emergencyStop;
  out.input.requestSeq = a.requestSeq;
  out.input.gestureSeq = a.gestureSeq;
  out.input.gesture = a.gesture;
  out.input.command.gestureSeq = a.gestureSeq;
  out.input.command.gesture = a.gesture;
  return out;
}

} // namespace link_receiver
