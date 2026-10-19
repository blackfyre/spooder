#include "protocol.h"

namespace protocol
{

uint16_t crc16(const uint8_t *data, int length)
{
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void put16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

static int payloadSize(uint8_t type)
{
  switch ((FrameType)type)
  {
  case FrameType::Command:
    return COMMAND_PAYLOAD_SIZE;
  case FrameType::Telemetry:
    return TELEMETRY_PAYLOAD_SIZE;
  case FrameType::CalibrationSet:
    return CALIBRATION_SET_PAYLOAD_SIZE;
  case FrameType::CalibrationReport:
    return CALIBRATION_REPORT_PAYLOAD_SIZE;
  case FrameType::Pose:
    return POSE_PAYLOAD_SIZE;
  }
  return -1;
}

static int finishFrame(FrameType type, uint8_t seq, uint8_t *out)
{
  int n = payloadSize((uint8_t)type);
  out[0] = START;
  out[1] = VERSION;
  out[2] = (uint8_t)type;
  out[3] = seq;
  out[4] = (uint8_t)n;
  put16(out + HEADER_SIZE + n, crc16(out + 1, HEADER_SIZE - 1 + n));
  return HEADER_SIZE + n + CRC_SIZE;
}

int encodeCommand(const Command &c, uint8_t seq, uint8_t *out)
{
  uint8_t *p = out + HEADER_SIZE;
  p[0] = c.mode;
  p[1] = (uint8_t)c.translateX;
  p[2] = (uint8_t)c.translateY;
  p[3] = (uint8_t)c.rotate;
  p[4] = c.height;
  p[5] = c.speed;
  p[6] = c.gait;
  p[7] = c.stride;
  p[8] = c.flags;
  p[9] = c.clearCounter;
  p[10] = c.modeRequestCounter;
  p[11] = c.gesture;
  p[12] = c.gestureRequestCounter;
  p[13] = 0; // reserved
  return finishFrame(FrameType::Command, seq, out);
}

int encodeTelemetry(const Telemetry &t, uint8_t seq, uint8_t *out)
{
  uint8_t *p = out + HEADER_SIZE;
  for (int i = 0; i < TELEMETRY_PAYLOAD_SIZE; i++)
    p[i] = 0; // reserved bytes stay zero
  p[0] = t.bodyMode;
  p[1] = t.gait;
  p[2] = t.outputsOn ? 1 : 0;
  put16(p + 3, t.faults);
  put16(p + 5, t.voltageMv);
  put16(p + 7, t.currentMa);
  put16(p + 9, t.validFrames);
  put16(p + 11, t.discardedFrames);
  p[13] = t.sensorsAvailable;
  put16(p + 14, t.tofMm);
  p[16] = t.tofStatus;
  put16(p + 17, (uint16_t)t.pitchCentideg);
  put16(p + 19, (uint16_t)t.rollCentideg);
  p[21] = t.flags;
  p[22] = t.gesture;
  return finishFrame(FrameType::Telemetry, seq, out);
}

int encodeCalibrationSet(const CalibrationSet &c, uint8_t seq, uint8_t *out)
{
  uint8_t *p = out + HEADER_SIZE;
  p[0] = c.requestId;
  p[1] = c.joint;
  put16(p + 2, (uint16_t)c.offsetCentideg);
  put16(p + 4, c.pulse0Us);
  put16(p + 6, c.pulse180Us);
  p[8] = c.flags;
  return finishFrame(FrameType::CalibrationSet, seq, out);
}

int encodeCalibrationReport(const CalibrationReport &r, uint8_t seq, uint8_t *out)
{
  uint8_t *p = out + HEADER_SIZE;
  p[0] = r.lastRequestId;
  p[1] = r.result;
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    uint8_t *j = p + 2 + i * CALIBRATION_JOINT_SIZE;
    put16(j, (uint16_t)r.joints[i].offsetCentideg);
    put16(j + 2, r.joints[i].pulse0Us);
    put16(j + 4, r.joints[i].pulse180Us);
    j[6] = r.joints[i].flags;
  }
  return finishFrame(FrameType::CalibrationReport, seq, out);
}

int encodePose(const Pose &p, uint8_t seq, uint8_t *out)
{
  uint8_t *b = out + HEADER_SIZE;
  b[0] = p.flags;
  b[1] = 0; // reserved
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    uint8_t *j = b + 2 + i * POSE_JOINT_SIZE;
    put16(j, (uint16_t)p.joints[i].commandedCentideg);
    put16(j + 2, (uint16_t)p.joints[i].servoCentideg);
    put16(j + 4, p.joints[i].pulseUs);
  }
  return finishFrame(FrameType::Pose, seq, out);
}

static void parseCommand(const uint8_t *p, Command &c)
{
  c.mode = p[0];
  c.translateX = (int8_t)p[1];
  c.translateY = (int8_t)p[2];
  c.rotate = (int8_t)p[3];
  c.height = p[4];
  c.speed = p[5];
  c.gait = p[6];
  c.stride = p[7];
  c.flags = p[8];
  c.clearCounter = p[9];
  c.modeRequestCounter = p[10];
  c.gesture = p[11];
  c.gestureRequestCounter = p[12];
}

static void parseTelemetry(const uint8_t *p, Telemetry &t)
{
  t.bodyMode = p[0];
  t.gait = p[1];
  t.outputsOn = p[2] != 0;
  t.faults = get16(p + 3);
  t.voltageMv = get16(p + 5);
  t.currentMa = get16(p + 7);
  t.validFrames = get16(p + 9);
  t.discardedFrames = get16(p + 11);
  t.sensorsAvailable = p[13];
  t.tofMm = get16(p + 14);
  t.tofStatus = p[16];
  t.pitchCentideg = (int16_t)get16(p + 17);
  t.rollCentideg = (int16_t)get16(p + 19);
  t.flags = p[21];
  t.gesture = p[22];
}

static void parseCalibrationSet(const uint8_t *p, CalibrationSet &c)
{
  c.requestId = p[0];
  c.joint = p[1];
  c.offsetCentideg = (int16_t)get16(p + 2);
  c.pulse0Us = get16(p + 4);
  c.pulse180Us = get16(p + 6);
  c.flags = p[8];
}

static void parseCalibrationReport(const uint8_t *p, CalibrationReport &r)
{
  r.lastRequestId = p[0];
  r.result = p[1];
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    const uint8_t *j = p + 2 + i * CALIBRATION_JOINT_SIZE;
    r.joints[i].offsetCentideg = (int16_t)get16(j);
    r.joints[i].pulse0Us = get16(j + 2);
    r.joints[i].pulse180Us = get16(j + 4);
    r.joints[i].flags = j[6];
  }
}

static void parsePose(const uint8_t *b, Pose &p)
{
  p.flags = b[0];
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    const uint8_t *j = b + 2 + i * POSE_JOINT_SIZE;
    p.joints[i].commandedCentideg = (int16_t)get16(j);
    p.joints[i].servoCentideg = (int16_t)get16(j + 2);
    p.joints[i].pulseUs = get16(j + 4);
  }
}

void Decoder::push(uint8_t byte)
{
  // next() keeps the buffer shorter than a frame; if the caller skipped it,
  // make room by giving up the oldest candidate.
  if (len_ == MAX_FRAME_SIZE)
    reject();
  if (len_ == 0 && byte != START)
    return; // between frames
  buf_[len_++] = byte;
}

void Decoder::drop(int count)
{
  for (int i = count; i < len_; i++)
    buf_[i - count] = buf_[i];
  len_ -= count;
}

void Decoder::reject()
{
  discarded_++;
  drop(1); // resume the search just after this candidate's start marker
}

// Checks the candidate starting at buf_[at]: Invalid, Incomplete, or a
// complete CRC-valid frame of *total bytes.
Decoder::Candidate Decoder::check(int at, int *total) const
{
  const uint8_t *b = buf_ + at;
  int len = len_ - at;
  if (b[0] != START)
    return Candidate::Invalid;
  if (len >= 2 && b[1] != VERSION)
    return Candidate::Invalid;
  int n = len >= 3 ? payloadSize(b[2]) : 0;
  if (n < 0 || (len >= 5 && b[4] != n))
    return Candidate::Invalid;
  *total = HEADER_SIZE + n + CRC_SIZE;
  if (len < HEADER_SIZE || len < *total)
    return Candidate::Incomplete;
  if (crc16(b + 1, HEADER_SIZE - 1 + n) != get16(b + HEADER_SIZE + n))
    return Candidate::Invalid;
  return Candidate::Valid;
}

void Decoder::emit(int total, Frame &out)
{
  out.type = (FrameType)buf_[2];
  out.seq = buf_[3];
  const uint8_t *payload = buf_ + HEADER_SIZE;
  switch (out.type)
  {
  case FrameType::Command:
    parseCommand(payload, out.command);
    break;
  case FrameType::Telemetry:
    parseTelemetry(payload, out.telemetry);
    break;
  case FrameType::CalibrationSet:
    parseCalibrationSet(payload, out.calibrationSet);
    break;
  case FrameType::CalibrationReport:
    parseCalibrationReport(payload, out.calibrationReport);
    break;
  case FrameType::Pose:
    parsePose(payload, out.pose);
    break;
  }
  valid_++;
  drop(total);
}

void Decoder::idle()
{
  int total = 0;
  if (len_ > 0 && check(0, &total) == Candidate::Incomplete)
    reject(); // next() rescans the bytes after its start marker
}

bool Decoder::next(Frame &out)
{
  while (len_ > 0)
  {
    int total = 0;
    Candidate c = check(0, &total);
    if (c == Candidate::Valid)
    {
      emit(total, out);
      return true;
    }
    if (c == Candidate::Invalid)
    {
      if (buf_[0] == START)
        reject();
      else
        drop(1);
      continue;
    }
    // Incomplete: keep the earliest candidate until its CRC can be checked,
    // or until idle() abandons it.
    return false;
  }
  return false;
}

} // namespace protocol
