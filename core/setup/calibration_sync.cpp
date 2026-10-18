#include "calibration_sync.h"

namespace calibration_sync
{

static bool validJoint(int joint)
{
  return joint >= 0 && joint < JOINT_COUNT;
}

bool wanted(const Sync &s, int joint, protocol::CalibrationJoint &out)
{
  if (!s.haveReport || !validJoint(joint))
    return false;
  out = s.entries[joint].pending ? s.entries[joint].value : s.report.joints[joint];
  return true;
}

// The new field value, saturated to lo..hi. Widened so a relative change
// cannot overflow.
static int32_t moved(int32_t base, int32_t value, bool relative, int32_t lo, int32_t hi)
{
  int64_t v = relative ? (int64_t)base + value : value;
  return v < lo ? lo : (v > hi ? hi : (int32_t)v);
}

bool change(Sync &s, int joint, Field field, int32_t value, bool relative)
{
  protocol::CalibrationJoint j;
  if (!wanted(s, joint, j))
    return false;
  switch (field)
  {
  case Field::Offset:
    j.offsetCentideg = (int16_t)moved(j.offsetCentideg, value, relative, INT16_MIN, INT16_MAX);
    break;
  case Field::Pulse0:
    j.pulse0Us = (uint16_t)moved(j.pulse0Us, value, relative, 0, UINT16_MAX);
    break;
  case Field::Pulse180:
    j.pulse180Us = (uint16_t)moved(j.pulse180Us, value, relative, 0, UINT16_MAX);
    break;
  case Field::Reversed:
  {
    bool reversed = (j.flags & protocol::CALIBRATION_REVERSED) != 0;
    bool want = relative ? (value != 0 ? !reversed : reversed) : value != 0;
    j.flags = want ? protocol::CALIBRATION_REVERSED : 0;
    break;
  }
  default:
    return false;
  }

  Entry &e = s.entries[joint];
  if (!e.pending)
  {
    e.pending = true;
    e.order = s.nextOrder++;
  }
  e.value = j;
  if (s.inFlight && s.sent.joint == joint)
    s.editedInFlight = true;
  return true;
}

bool pending(const Sync &s, int joint)
{
  return validJoint(joint) && s.entries[joint].pending;
}

bool next(Sync &s, uint32_t nowMs, protocol::CalibrationSet &out)
{
  if (s.inFlight)
  {
    if (nowMs - s.sentAtMs < RESEND_MS)
      return false;
    s.sentAtMs = nowMs;
    out = s.sent;
    return true;
  }

  int oldest = -1;
  for (int i = 0; i < JOINT_COUNT; i++)
  {
    // Orders compared by difference, so the counter may wrap.
    if (s.entries[i].pending &&
        (oldest < 0 || (int32_t)(s.entries[i].order - s.entries[oldest].order) < 0))
      oldest = i;
  }
  if (oldest < 0)
    return false;

  // A new id, never the one the robot last reported: it would take that as a
  // retransmission and change nothing.
  uint8_t id = (uint8_t)(s.lastId + 1);
  if (s.haveReport && id == s.report.lastRequestId)
    id++;
  s.lastId = id;
  s.haveLastId = true;

  const protocol::CalibrationJoint &v = s.entries[oldest].value;
  s.sent.requestId = id;
  s.sent.joint = (uint8_t)oldest;
  s.sent.offsetCentideg = v.offsetCentideg;
  s.sent.pulse0Us = v.pulse0Us;
  s.sent.pulse180Us = v.pulse180Us;
  s.sent.flags = v.flags;
  s.inFlight = true;
  s.sentAtMs = nowMs;
  s.editedInFlight = false;
  out = s.sent;
  return true;
}

void onReport(Sync &s, const protocol::CalibrationReport &r)
{
  if (!s.haveReport && !s.haveLastId)
  {
    // Continue after the robot's newest id rather than from zero.
    s.lastId = r.lastRequestId;
    s.haveLastId = true;
  }
  s.haveReport = true;
  s.report = r;
  if (s.inFlight && r.lastRequestId == s.sent.requestId &&
      r.result != (uint8_t)protocol::CalibrationResult::NoneYet)
  {
    s.inFlight = false;
    if (!s.editedInFlight)
      s.entries[s.sent.joint].pending = false;
    s.editedInFlight = false;
  }
}

} // namespace calibration_sync
