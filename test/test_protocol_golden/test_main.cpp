// Golden frame vectors for protocol version 3: a fixed set of frames of every
// type, encoded by the C++ encoder and compared with
// test/golden/protocol_v3.txt, which other implementations (the desktop
// calibrator) test against. After a deliberate protocol change, regenerate
// the file with `SPOODER_WRITE_GOLDEN=1 mise run test` and review the diff.
#include <unity.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "protocol/protocol.h"

using namespace protocol;

void setUp() {}
void tearDown() {}

static const char *const HEADER =
    "# Spooder link protocol version 3: golden frames.\n"
    "# Generated from the C++ encoder by test/test_protocol_golden; regenerate with\n"
    "# SPOODER_WRITE_GOLDEN=1 mise run test after a deliberate protocol change.\n"
    "#\n"
    "# One frame per line, fields separated by single spaces:\n"
    "#   <name> <frame bytes as lowercase hex, no separators> <field>=<value> ...\n"
    "# Values are decimal. The header fields (version, type, seq) come first, then\n"
    "# the payload fields in payload order; report joints are joint<i>.<field>.\n"
    "# A line whose only field is \"rejected\" is a frame a decoder must discard.\n"
    "# Lines starting with # are comments.\n";

struct Text
{
  char buf[32768];
  int len;
};

static void add(Text &t, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void add(Text &t, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  t.len += vsnprintf(t.buf + t.len, sizeof(t.buf) - t.len, fmt, ap);
  va_end(ap);
  TEST_ASSERT_TRUE_MESSAGE(t.len < (int)sizeof(t.buf), "golden text buffer too small");
}

static const char *typeName(FrameType type)
{
  switch (type)
  {
  case FrameType::Command:
    return "command";
  case FrameType::Telemetry:
    return "telemetry";
  case FrameType::CalibrationSet:
    return "calibration_set";
  case FrameType::CalibrationReport:
    return "calibration_report";
  case FrameType::Pose:
    return "pose";
  case FrameType::Diagnostics:
    return "diagnostics";
  }
  return "?";
}

static void describe(Text &t, const Frame &f)
{
  add(t, "version=%d type=%s seq=%u", VERSION, typeName(f.type), f.seq);
  switch (f.type)
  {
  case FrameType::Command:
  {
    const Command &c = f.command;
    add(t,
        " mode=%u translateX=%d translateY=%d rotate=%d height=%u speed=%u gait=%u stride=%u flags=%u "
        "clearCounter=%u modeRequestCounter=%u gesture=%u gestureRequestCounter=%u",
        c.mode, c.translateX, c.translateY, c.rotate, c.height, c.speed, c.gait, c.stride, c.flags,
        c.clearCounter, c.modeRequestCounter, c.gesture, c.gestureRequestCounter);
    break;
  }
  case FrameType::Telemetry:
  {
    const Telemetry &m = f.telemetry;
    add(t,
        " bodyMode=%u gait=%u outputsOn=%d faults=%u voltageMv=%u currentMa=%u validFrames=%u "
        "discardedFrames=%u sensorsAvailable=%u tofMm=%u tofStatus=%u pitchCentideg=%d rollCentideg=%d flags=%u gesture=%u "
        "bridgeValidFrames=%u bridgeDiscardedFrames=%u",
        m.bodyMode, m.gait, m.outputsOn ? 1 : 0, m.faults, m.voltageMv, m.currentMa, m.validFrames,
        m.discardedFrames, m.sensorsAvailable, m.tofMm, m.tofStatus, m.pitchCentideg, m.rollCentideg, m.flags,
        m.gesture, m.bridgeValidFrames, m.bridgeDiscardedFrames);
    break;
  }
  case FrameType::CalibrationSet:
  {
    const CalibrationSet &c = f.calibrationSet;
    add(t, " requestId=%u joint=%u offsetCentideg=%d pulse0Us=%u pulse180Us=%u flags=%u", c.requestId, c.joint,
        c.offsetCentideg, c.pulse0Us, c.pulse180Us, c.flags);
    break;
  }
  case FrameType::CalibrationReport:
  {
    const CalibrationReport &r = f.calibrationReport;
    add(t, " lastRequestId=%u result=%u", r.lastRequestId, r.result);
    for (int i = 0; i < CALIBRATION_JOINTS; i++)
      add(t, " joint%d.offsetCentideg=%d joint%d.pulse0Us=%u joint%d.pulse180Us=%u joint%d.flags=%u", i,
          r.joints[i].offsetCentideg, i, r.joints[i].pulse0Us, i, r.joints[i].pulse180Us, i, r.joints[i].flags);
    break;
  }
  case FrameType::Pose:
  {
    const Pose &p = f.pose;
    add(t, " flags=%u", p.flags);
    for (int i = 0; i < CALIBRATION_JOINTS; i++)
      add(t, " joint%d.commandedCentideg=%d joint%d.servoCentideg=%d joint%d.pulseUs=%u", i,
          p.joints[i].commandedCentideg, i, p.joints[i].servoCentideg, i, p.joints[i].pulseUs);
    break;
  }
  case FrameType::Diagnostics:
  {
    const Diagnostics &d = f.diagnostics;
    add(t, " motionSharePermille=%u backstopTicks=%u tickMaxUs=%u tickMeanUs=%u averageCurrentMa=%u",
        d.motionSharePermille, d.backstopTicks, d.tickMaxUs, d.tickMeanUs, d.averageCurrentMa);
    add(t, " tripCurrentMa=%u sustainedCurrentMa=%u minMv=%u maxMv=%u brownoutMv=%u budgetDegPerS=%u ratedDegPerS=%u",
        d.tripCurrentMa, d.sustainedCurrentMa, d.minMv, d.maxMv, d.brownoutMv, d.budgetDegPerS, d.ratedDegPerS);
    break;
  }
  }
}

static int encode(const Frame &f, uint8_t *out)
{
  switch (f.type)
  {
  case FrameType::Command:
    return encodeCommand(f.command, f.seq, out);
  case FrameType::Telemetry:
    return encodeTelemetry(f.telemetry, f.seq, out);
  case FrameType::CalibrationSet:
    return encodeCalibrationSet(f.calibrationSet, f.seq, out);
  case FrameType::CalibrationReport:
    return encodeCalibrationReport(f.calibrationReport, f.seq, out);
  case FrameType::Pose:
    return encodePose(f.pose, f.seq, out);
  case FrameType::Diagnostics:
    return encodeDiagnostics(f.diagnostics, f.seq, out);
  }
  return 0;
}

static void addHex(Text &t, const uint8_t *bytes, int n)
{
  for (int i = 0; i < n; i++)
    add(t, "%02x", bytes[i]);
}

// Decodes bytes with a fresh decoder; returns the number of frames.
static int decodeAll(const uint8_t *bytes, int n, Frame &last)
{
  Decoder d;
  int frames = 0;
  for (int i = 0; i < n; i++)
    d.push(bytes[i]);
  d.idle();
  Frame f;
  while (d.next(f))
  {
    last = f;
    frames++;
  }
  return frames;
}

// Encodes the frame, checks that it decodes to the same fields, and adds
// its line.
static void addFrame(Text &t, const char *name, const Frame &f)
{
  uint8_t bytes[MAX_FRAME_SIZE];
  int n = encode(f, bytes);
  Frame decoded;
  TEST_ASSERT_EQUAL_MESSAGE(1, decodeAll(bytes, n, decoded), name);
  Text expected = {}, actual = {};
  describe(expected, f);
  describe(actual, decoded);
  TEST_ASSERT_EQUAL_STRING_MESSAGE(expected.buf, actual.buf, name);
  add(t, "%s ", name);
  addHex(t, bytes, n);
  add(t, " %s\n", actual.buf);
}

static void addRejected(Text &t, const char *name, const uint8_t *bytes, int n)
{
  Frame f;
  TEST_ASSERT_EQUAL_MESSAGE(0, decodeAll(bytes, n, f), name);
  add(t, "%s ", name);
  addHex(t, bytes, n);
  add(t, " rejected\n");
}

static Frame frame(FrameType type, uint8_t seq)
{
  Frame f = {};
  f.type = type;
  f.seq = seq;
  return f;
}

static void buildGolden(Text &t)
{
  add(t, "%s", HEADER);

  Frame f = frame(FrameType::Command, 0);
  f.command.mode = 2;
  f.command.translateY = 50;
  f.command.height = 50;
  f.command.speed = 44;
  f.command.flags = FLAG_CONTROLLER_CONNECTED;
  addFrame(t, "command_walk", f);

  f = frame(FrameType::Command, 255);
  f.command = {4, -128, 127, -100, 255, 100, 5, 1, 0xFF, 255, 255, 255, 255};
  addFrame(t, "command_extremes_seq_wrap", f);

  f = frame(FrameType::Command, 17);
  f.command.mode = 0;
  f.command.height = 30;
  f.command.speed = 44;
  f.command.flags = FLAG_CONTROLLER_CONNECTED;
  f.command.modeRequestCounter = 4;
  f.command.gesture = GESTURE_WIGGLE;
  f.command.gestureRequestCounter = 9;
  addFrame(t, "command_stand_gesture_wiggle", f);

  f = frame(FrameType::Command, 0);
  f.command.mode = 3;
  f.command.flags = FLAG_EMERGENCY_STOP;
  f.command.clearCounter = 1;
  f.command.modeRequestCounter = 0;
  addFrame(t, "command_estop_no_controller", f);

  f = frame(FrameType::Telemetry, 7);
  f.telemetry.bodyMode = 4;
  f.telemetry.outputsOn = true;
  f.telemetry.voltageMv = 6012;
  f.telemetry.currentMa = 1350;
  f.telemetry.validFrames = 1000;
  f.telemetry.discardedFrames = 2;
  f.telemetry.flags = TELEMETRY_USB_IN_CONTROL;
  addFrame(t, "telemetry_calibrate_usb", f);

  f = frame(FrameType::Telemetry, 128);
  f.telemetry.bodyMode = 3;
  f.telemetry.faults = 0x0020;
  f.telemetry.flags = TELEMETRY_SOURCE_CONFLICT;
  addFrame(t, "telemetry_conflict", f);

  f = frame(FrameType::Telemetry, 255);
  f.telemetry = {255, 255, true, 0xFFFF, 65535, 65535, 65535, 65535, 0xFF, 65535, 255, -32768, 32767, 0xFF, 0xFF, 65535, 65535};
  addFrame(t, "telemetry_extremes", f);

  f = frame(FrameType::Telemetry, 30);
  f.telemetry.bodyMode = 5; // Perform
  f.telemetry.outputsOn = true;
  f.telemetry.voltageMv = 6050;
  f.telemetry.currentMa = 2100;
  f.telemetry.flags = TELEMETRY_BRIDGE_IN_CONTROL;
  f.telemetry.gesture = GESTURE_SAY_HI;
  addFrame(t, "telemetry_perform_say_hi", f);

  // On USB: the bridge link up without a gamepad, with its own counts.
  f = frame(FrameType::Telemetry, 31);
  f.telemetry.bodyMode = 1;
  f.telemetry.outputsOn = true;
  f.telemetry.voltageMv = 6060;
  f.telemetry.validFrames = 420;
  f.telemetry.flags = TELEMETRY_USB_IN_CONTROL | TELEMETRY_BRIDGE_LINK_UP;
  f.telemetry.bridgeValidFrames = 3143;
  f.telemetry.bridgeDiscardedFrames = 2;
  addFrame(t, "telemetry_usb_bridge_link_up", f);

  f = frame(FrameType::CalibrationSet, 1);
  f.calibrationSet = {1, 0, 9000, 500, 2500, 0};
  addFrame(t, "calibration_set_default_joint0", f);

  f = frame(FrameType::CalibrationSet, 42);
  f.calibrationSet = {200, 17, -2450, 2400, 600, CALIBRATION_REVERSED};
  addFrame(t, "calibration_set_reversed_joint17", f);

  f = frame(FrameType::CalibrationSet, 255);
  f.calibrationSet = {255, 255, -32768, 0, 65535, 0xFF};
  addFrame(t, "calibration_set_extremes", f);

  // A report of the seed defaults before any set: a fixed wire vector, not
  // tied to core/movement/calibration_defaults.cpp.
  const int16_t defaults[CALIBRATION_JOINTS] = {9000, 7500, -1800, 9300, 7500, -1500, 9300, 7500, -1800,
                                                8700, 8000, -2600, 8500, 8900, -1600, 9300, 8500, -2400};
  f = frame(FrameType::CalibrationReport, 0);
  f.calibrationReport.lastRequestId = 0;
  f.calibrationReport.result = (uint8_t)CalibrationResult::NoneYet;
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
    f.calibrationReport.joints[i] = {defaults[i], 500, 2500, 0};
  addFrame(t, "calibration_report_defaults", f);

  // Joint 1 clamped to its default plus 30°, joint 4 reversed with its own
  // range.
  f.seq = 9;
  f.calibrationReport.lastRequestId = 17;
  f.calibrationReport.result = (uint8_t)CalibrationResult::Clamped;
  f.calibrationReport.joints[1].offsetCentideg = 10500;
  f.calibrationReport.joints[4] = {7250, 2450, 610, CALIBRATION_REVERSED};
  addFrame(t, "calibration_report_clamped", f);

  f.seq = 0;
  f.calibrationReport.lastRequestId = 18;
  f.calibrationReport.result = (uint8_t)CalibrationResult::Rejected;
  addFrame(t, "calibration_report_rejected_seq_wrap", f);

  f = frame(FrameType::CalibrationReport, 255);
  f.calibrationReport.lastRequestId = 255;
  f.calibrationReport.result = 255;
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
    f.calibrationReport.joints[i] = {(int16_t)(i % 2 ? -32768 : 32767), 0, 65535, 0xFF};
  addFrame(t, "calibration_report_extremes", f);

  // Frames a decoder must discard: version 2 (with a valid CRC and the
  // version 2 command length) and a bad CRC.
  uint8_t bytes[MAX_FRAME_SIZE];
  Frame v2 = frame(FrameType::Command, 3);
  v2.command.mode = 1;
  v2.command.flags = FLAG_CONTROLLER_CONNECTED;
  int n = encode(v2, bytes);
  const int V2_COMMAND_PAYLOAD_SIZE = 12;
  bytes[1] = 2;
  bytes[4] = V2_COMMAND_PAYLOAD_SIZE;
  n = HEADER_SIZE + V2_COMMAND_PAYLOAD_SIZE + CRC_SIZE;
  uint16_t crc = crc16(bytes + 1, n - 3);
  bytes[n - 2] = (uint8_t)(crc & 0xFF);
  bytes[n - 1] = (uint8_t)(crc >> 8);
  addRejected(t, "reject_version_2_command", bytes, n);

  Frame bad = frame(FrameType::CalibrationSet, 4);
  bad.calibrationSet = {5, 3, 100, 500, 2500, 0};
  n = encode(bad, bytes);
  bytes[n - 1] ^= 0x01;
  addRejected(t, "reject_bad_crc_calibration_set", bytes, n);

  // Pose frames (type 5, USB only), appended after the frames above.
  // The calibration pose: commanded 0/90/90 on every leg, servo angles with
  // the default offsets, pulses through the default 500-2500 us range.
  f = frame(FrameType::Pose, 3);
  f.pose.flags = POSE_OUTPUTS_ON;
  const int16_t pose[3] = {0, 9000, 9000};
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    int16_t servo = (int16_t)(pose[i % 3] + defaults[i]);
    int16_t clamped = servo < 0 ? 0 : (servo > 18000 ? 18000 : servo);
    f.pose.joints[i] = {pose[i % 3], servo, (uint16_t)(500 + (clamped * 2000 + 9000) / 18000)};
  }
  addFrame(t, "pose_calibration_outputs_on", f);

  // Output off, legs 4 and 5 never set (pulse 0), negative angles.
  f = frame(FrameType::Pose, 200);
  f.pose.flags = 0;
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
    f.pose.joints[i] = i < 12 ? PoseJoint{(int16_t)(-1234 - i), (int16_t)(-50 + 3 * i), (uint16_t)(1500 + i)}
                              : PoseJoint{0, 0, 0};
  addFrame(t, "pose_output_off_legs_unset", f);

  f = frame(FrameType::Pose, 255);
  f.pose.flags = 0xFF;
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
    f.pose.joints[i] = {(int16_t)(i % 2 ? -32768 : 32767), (int16_t)(i % 2 ? 32767 : -32768), 65535};
  addFrame(t, "pose_extremes", f);

  Frame badPose = frame(FrameType::Pose, 6);
  n = encode(badPose, bytes);
  bytes[n - 2] ^= 0x80;
  addRejected(t, "reject_bad_crc_pose", bytes, n);

  // Diagnostics frames (type 6, USB only), appended after the frames above.
  // Governed walking with the example limits and the MG996R profile.
  f = frame(FrameType::Diagnostics, 4);
  f.diagnostics = {412, 0, 6480, 5210, 2380, 9000, 6000, 5200, 6800, 4800, 257, 429};
  addFrame(t, "diagnostics_governed_walk", f);

  f = frame(FrameType::Diagnostics, 99);
  f.diagnostics = {1000, 37, 1910, 1650, 0, 9000, 6000, 5200, 6800, 4800, 257, 429};
  addFrame(t, "diagnostics_idle_backstop_count", f);

  f = frame(FrameType::Diagnostics, 255);
  f.diagnostics = {65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535};
  addFrame(t, "diagnostics_extremes", f);

  Frame badDiagnostics = frame(FrameType::Diagnostics, 7);
  n = encode(badDiagnostics, bytes);
  bytes[n - 1] ^= 0x01;
  addRejected(t, "reject_bad_crc_diagnostics", bytes, n);
}

static void goldenPath(char *out, int size)
{
  const char *file = __FILE__;
  const char *cut = strstr(file, "test/test_protocol_golden/");
  int prefix = cut ? (int)(cut - file) : 0;
  snprintf(out, size, "%.*stest/golden/protocol_v3.txt", prefix, file);
}

void test_golden_vectors()
{
  static Text t;
  t = {};
  buildGolden(t);

  char path[512];
  goldenPath(path, sizeof(path));
  const char *write = getenv("SPOODER_WRITE_GOLDEN");
  if (write && write[0] == '1')
  {
    FILE *f = fopen(path, "wb");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "cannot write test/golden/protocol_v3.txt");
    fwrite(t.buf, 1, t.len, f);
    fclose(f);
  }

  static char golden[sizeof(t.buf)];
  FILE *f = fopen(path, "rb");
  TEST_ASSERT_NOT_NULL_MESSAGE(f, "cannot read test/golden/protocol_v3.txt");
  int n = (int)fread(golden, 1, sizeof(golden) - 1, f);
  fclose(f);
  golden[n] = 0;
  TEST_ASSERT_EQUAL_STRING(golden, t.buf);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_golden_vectors);
  return UNITY_END();
}
