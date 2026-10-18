#include <unity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "protocol/protocol.h"

using namespace protocol;

static Decoder dec;

void setUp() { dec = Decoder(); }
void tearDown() {}

static Command sampleCommand()
{
  Command c = {};
  c.mode = 1;
  c.translateX = -100;
  c.translateY = 73;
  c.rotate = -5;
  c.height = 40;
  c.speed = 100;
  c.gait = 5;
  c.stride = 1;
  c.flags = FLAG_CONTROLLER_CONNECTED | FLAG_EMERGENCY_STOP;
  c.clearCounter = 200;
  c.modeRequestCounter = 255;
  return c;
}

static Telemetry sampleTelemetry()
{
  Telemetry t = {};
  t.bodyMode = 2;
  t.gait = 3;
  t.outputsOn = true;
  t.faults = 0x8011;
  t.voltageMv = 6020;
  t.currentMa = 65535;
  t.validFrames = 1234;
  t.discardedFrames = 7;
  t.sensorsAvailable = SENSOR_TOF | SENSOR_ATTITUDE;
  t.tofMm = 150;
  t.tofStatus = 11;
  t.pitchCentideg = -1000;
  t.rollCentideg = 32767;
  t.flags = TELEMETRY_SOURCE_CONFLICT | TELEMETRY_BRIDGE_IN_CONTROL;
  return t;
}

static CalibrationSet sampleSet()
{
  CalibrationSet c = {};
  c.requestId = 201;
  c.joint = 17;
  c.offsetCentideg = -2450;
  c.pulse0Us = 2600;
  c.pulse180Us = 400;
  c.flags = CALIBRATION_REVERSED;
  return c;
}

static CalibrationReport sampleReport()
{
  CalibrationReport r = {};
  r.lastRequestId = 255;
  r.result = (uint8_t)CalibrationResult::Clamped;
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    r.joints[i].offsetCentideg = (int16_t)(i * 1000 - 9000);
    r.joints[i].pulse0Us = (uint16_t)(400 + i);
    r.joints[i].pulse180Us = (uint16_t)(2600 - i);
    r.joints[i].flags = i % 2 ? CALIBRATION_REVERSED : 0;
  }
  return r;
}

static void feed(const uint8_t *bytes, int n)
{
  for (int i = 0; i < n; i++)
    dec.push(bytes[i]);
}

// Feeds bytes and counts the frames decoded, keeping the last one.
static int feedAndCount(const uint8_t *bytes, int n, Frame &last)
{
  int frames = 0;
  for (int i = 0; i < n; i++)
  {
    dec.push(bytes[i]);
    Frame f;
    while (dec.next(f))
    {
      last = f;
      frames++;
    }
  }
  return frames;
}

void test_crc_check_value()
{
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(check, 9));
}

void test_frame_sizes()
{
  uint8_t buf[MAX_FRAME_SIZE];
  TEST_ASSERT_EQUAL(19, encodeCommand(sampleCommand(), 0, buf));
  TEST_ASSERT_EQUAL(39, encodeTelemetry(sampleTelemetry(), 0, buf));
  TEST_ASSERT_EQUAL(16, encodeCalibrationSet(sampleSet(), 0, buf));
  TEST_ASSERT_EQUAL(135, encodeCalibrationReport(sampleReport(), 0, buf));
  TEST_ASSERT_EQUAL(128, CALIBRATION_REPORT_PAYLOAD_SIZE);
  TEST_ASSERT_EQUAL(135, MAX_FRAME_SIZE);
}

void test_command_round_trip()
{
  uint8_t buf[MAX_FRAME_SIZE];
  Command c = sampleCommand();
  int n = encodeCommand(c, 42, buf);
  Frame f;
  TEST_ASSERT_EQUAL(1, feedAndCount(buf, n, f));
  TEST_ASSERT_TRUE(f.type == FrameType::Command);
  TEST_ASSERT_EQUAL(42, f.seq);
  TEST_ASSERT_EQUAL(c.mode, f.command.mode);
  TEST_ASSERT_EQUAL(c.translateX, f.command.translateX);
  TEST_ASSERT_EQUAL(c.translateY, f.command.translateY);
  TEST_ASSERT_EQUAL(c.rotate, f.command.rotate);
  TEST_ASSERT_EQUAL(c.height, f.command.height);
  TEST_ASSERT_EQUAL(c.speed, f.command.speed);
  TEST_ASSERT_EQUAL(c.gait, f.command.gait);
  TEST_ASSERT_EQUAL(c.stride, f.command.stride);
  TEST_ASSERT_EQUAL(c.flags, f.command.flags);
  TEST_ASSERT_EQUAL(c.clearCounter, f.command.clearCounter);
  TEST_ASSERT_EQUAL(c.modeRequestCounter, f.command.modeRequestCounter);
  TEST_ASSERT_EQUAL(1, dec.validFrames());
  TEST_ASSERT_EQUAL(0, dec.discardedFrames());
}

void test_telemetry_round_trip()
{
  uint8_t buf[MAX_FRAME_SIZE];
  Telemetry t = sampleTelemetry();
  int n = encodeTelemetry(t, 9, buf);
  Frame f;
  TEST_ASSERT_EQUAL(1, feedAndCount(buf, n, f));
  TEST_ASSERT_TRUE(f.type == FrameType::Telemetry);
  TEST_ASSERT_EQUAL(9, f.seq);
  TEST_ASSERT_EQUAL(t.bodyMode, f.telemetry.bodyMode);
  TEST_ASSERT_EQUAL(t.gait, f.telemetry.gait);
  TEST_ASSERT_TRUE(f.telemetry.outputsOn);
  TEST_ASSERT_EQUAL_HEX16(t.faults, f.telemetry.faults);
  TEST_ASSERT_EQUAL(t.voltageMv, f.telemetry.voltageMv);
  TEST_ASSERT_EQUAL(t.currentMa, f.telemetry.currentMa);
  TEST_ASSERT_EQUAL(t.validFrames, f.telemetry.validFrames);
  TEST_ASSERT_EQUAL(t.discardedFrames, f.telemetry.discardedFrames);
  TEST_ASSERT_EQUAL(t.sensorsAvailable, f.telemetry.sensorsAvailable);
  TEST_ASSERT_EQUAL(t.tofMm, f.telemetry.tofMm);
  TEST_ASSERT_EQUAL(t.tofStatus, f.telemetry.tofStatus);
  TEST_ASSERT_EQUAL(t.pitchCentideg, f.telemetry.pitchCentideg);
  TEST_ASSERT_EQUAL(t.rollCentideg, f.telemetry.rollCentideg);
  TEST_ASSERT_EQUAL_HEX8(t.flags, f.telemetry.flags);
  // The flags are byte 21 of the payload.
  TEST_ASSERT_EQUAL_HEX8(t.flags, buf[HEADER_SIZE + 21]);
}

void test_calibration_set_round_trip()
{
  uint8_t buf[MAX_FRAME_SIZE];
  CalibrationSet c = sampleSet();
  int n = encodeCalibrationSet(c, 250, buf);
  Frame f;
  TEST_ASSERT_EQUAL(1, feedAndCount(buf, n, f));
  TEST_ASSERT_TRUE(f.type == FrameType::CalibrationSet);
  TEST_ASSERT_EQUAL(250, f.seq);
  TEST_ASSERT_EQUAL(c.requestId, f.calibrationSet.requestId);
  TEST_ASSERT_EQUAL(c.joint, f.calibrationSet.joint);
  TEST_ASSERT_EQUAL(c.offsetCentideg, f.calibrationSet.offsetCentideg);
  TEST_ASSERT_EQUAL(c.pulse0Us, f.calibrationSet.pulse0Us);
  TEST_ASSERT_EQUAL(c.pulse180Us, f.calibrationSet.pulse180Us);
  TEST_ASSERT_EQUAL(c.flags, f.calibrationSet.flags);
}

void test_calibration_report_round_trip()
{
  uint8_t buf[MAX_FRAME_SIZE];
  CalibrationReport r = sampleReport();
  int n = encodeCalibrationReport(r, 3, buf);
  Frame f;
  TEST_ASSERT_EQUAL(1, feedAndCount(buf, n, f));
  TEST_ASSERT_TRUE(f.type == FrameType::CalibrationReport);
  TEST_ASSERT_EQUAL(3, f.seq);
  TEST_ASSERT_EQUAL(r.lastRequestId, f.calibrationReport.lastRequestId);
  TEST_ASSERT_EQUAL(r.result, f.calibrationReport.result);
  for (int i = 0; i < CALIBRATION_JOINTS; i++)
  {
    TEST_ASSERT_EQUAL(r.joints[i].offsetCentideg, f.calibrationReport.joints[i].offsetCentideg);
    TEST_ASSERT_EQUAL(r.joints[i].pulse0Us, f.calibrationReport.joints[i].pulse0Us);
    TEST_ASSERT_EQUAL(r.joints[i].pulse180Us, f.calibrationReport.joints[i].pulse180Us);
    TEST_ASSERT_EQUAL(r.joints[i].flags, f.calibrationReport.joints[i].flags);
  }
}

void test_report_and_commands_back_to_back()
{
  uint8_t buf[3 * MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 1, buf);
  n += encodeCalibrationReport(sampleReport(), 2, buf + n);
  n += encodeCalibrationSet(sampleSet(), 3, buf + n);
  Frame f;
  TEST_ASSERT_EQUAL(3, feedAndCount(buf, n, f));
  TEST_ASSERT_TRUE(f.type == FrameType::CalibrationSet);
  TEST_ASSERT_EQUAL(0, dec.discardedFrames());
}

void test_every_single_byte_corruption_is_rejected()
{
  uint8_t good[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 1, good);
  for (int i = 0; i < n; i++)
  {
    for (int bit = 0; bit < 8; bit++)
    {
      setUp();
      uint8_t bad[MAX_FRAME_SIZE];
      for (int k = 0; k < n; k++)
        bad[k] = good[k];
      bad[i] ^= (uint8_t)(1u << bit);
      Frame f;
      TEST_ASSERT_EQUAL_MESSAGE(0, feedAndCount(bad, n, f), "corrupted frame accepted");
      // The next valid frame is still accepted.
      TEST_ASSERT_EQUAL(1, feedAndCount(good, n, f));
      TEST_ASSERT_EQUAL(1, f.seq);
    }
  }
}

void test_corrupted_frame_is_counted()
{
  uint8_t buf[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 1, buf);
  buf[8] ^= 0x10;
  Frame f;
  feedAndCount(buf, n, f);
  TEST_ASSERT_EQUAL(0, dec.validFrames());
  TEST_ASSERT_TRUE(dec.discardedFrames() >= 1);
}

void test_resynchronises_over_garbage()
{
  uint8_t a[MAX_FRAME_SIZE], b[MAX_FRAME_SIZE];
  int na = encodeCommand(sampleCommand(), 10, a);
  int nb = encodeTelemetry(sampleTelemetry(), 11, b);
  // Garbage including start markers and plausible headers.
  const uint8_t garbage[] = {0x00, 0xA5, 0xA5, 0x01, 0x02, 0xA5, 0x01, 0x01, 0x07, 0x0C, 0x13, 0xFF, 0xA5};

  Frame f;
  int frames = feedAndCount(a, na, f);
  frames += feedAndCount(garbage, sizeof(garbage), f);
  frames += feedAndCount(b, nb, f);
  TEST_ASSERT_EQUAL(2, frames);
  TEST_ASSERT_EQUAL(11, f.seq);
  TEST_ASSERT_TRUE(f.type == FrameType::Telemetry);
}

void test_frame_hidden_in_a_rejected_candidate_is_found()
{
  // A false telemetry header swallows the start of a real command frame; the
  // command must still be decoded once the false candidate fails its CRC.
  uint8_t cmd[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 77, cmd);
  const uint8_t falseHeader[] = {START, VERSION, (uint8_t)FrameType::Telemetry, 0x00, TELEMETRY_PAYLOAD_SIZE};
  Frame f;
  int frames = feedAndCount(falseHeader, sizeof(falseHeader), f);
  frames += feedAndCount(cmd, n, f);
  // Pad with bytes so the false candidate completes and is rejected.
  const uint8_t pad[TELEMETRY_FRAME_SIZE] = {};
  frames += feedAndCount(pad, sizeof(pad), f);
  TEST_ASSERT_EQUAL(1, frames);
  TEST_ASSERT_EQUAL(77, f.seq);
}

void test_frame_behind_incomplete_false_start_is_returned_when_idle()
{
  // A false telemetry header followed by a complete command, then silence:
  // the command comes out once the line is idle.
  uint8_t cmd[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 78, cmd);
  const uint8_t falseHeader[] = {START, VERSION, (uint8_t)FrameType::Telemetry, 0x00, TELEMETRY_PAYLOAD_SIZE};
  Frame f;
  int frames = feedAndCount(falseHeader, sizeof(falseHeader), f);
  frames += feedAndCount(cmd, n, f);
  TEST_ASSERT_EQUAL(0, frames);
  dec.idle();
  TEST_ASSERT_TRUE(dec.next(f));
  TEST_ASSERT_EQUAL(78, f.seq);
  TEST_ASSERT_EQUAL(1, dec.discardedFrames());
  TEST_ASSERT_FALSE(dec.next(f));
}

void test_command_shaped_bytes_inside_telemetry_do_not_split_it()
{
  // A telemetry payload containing a complete, CRC-valid command frame must
  // still decode as one telemetry frame.
  uint8_t inner[MAX_FRAME_SIZE];
  Command c = {};
  c.mode = 1;
  c.speed = 44;
  c.flags = FLAG_CONTROLLER_CONNECTED;
  int n = encodeCommand(c, 130, inner);
  TEST_ASSERT_EQUAL(COMMAND_FRAME_SIZE, n);

  // Build a telemetry frame by hand whose payload bytes 5..23 are that command.
  uint8_t outer[MAX_FRAME_SIZE] = {START, VERSION, (uint8_t)FrameType::Telemetry, 9, TELEMETRY_PAYLOAD_SIZE};
  for (int i = 0; i < n; i++)
    outer[HEADER_SIZE + 5 + i] = inner[i];
  uint16_t crc = crc16(outer + 1, HEADER_SIZE - 1 + TELEMETRY_PAYLOAD_SIZE);
  outer[TELEMETRY_FRAME_SIZE - 2] = (uint8_t)(crc & 0xFF);
  outer[TELEMETRY_FRAME_SIZE - 1] = (uint8_t)(crc >> 8);

  Frame f;
  TEST_ASSERT_EQUAL(1, feedAndCount(outer, TELEMETRY_FRAME_SIZE, f));
  TEST_ASSERT_TRUE(f.type == FrameType::Telemetry);
  TEST_ASSERT_EQUAL(9, f.seq);
  TEST_ASSERT_EQUAL(0, dec.discardedFrames());
}

void test_idle_keeps_a_complete_buffer_untouched()
{
  uint8_t buf[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 3, buf);
  Frame f;
  feedAndCount(buf, n, f);
  dec.idle();
  TEST_ASSERT_EQUAL(0, dec.discardedFrames());
}

// Re-encodes a command frame as another version, with a valid CRC.
static int commandWithVersion(uint8_t version, uint8_t *buf)
{
  int n = encodeCommand(sampleCommand(), 1, buf);
  buf[1] = version;
  uint16_t crc = crc16(buf + 1, HEADER_SIZE - 1 + COMMAND_PAYLOAD_SIZE);
  buf[n - 2] = (uint8_t)(crc & 0xFF);
  buf[n - 1] = (uint8_t)(crc >> 8);
  return n;
}

void test_version_1_frame_is_rejected()
{
  uint8_t buf[MAX_FRAME_SIZE];
  int n = commandWithVersion(1, buf);
  Frame f;
  TEST_ASSERT_EQUAL(0, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(1, dec.discardedFrames());
}

void test_unknown_version_is_rejected()
{
  uint8_t buf[MAX_FRAME_SIZE];
  int n = commandWithVersion(3, buf);
  Frame f;
  TEST_ASSERT_EQUAL(0, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(1, dec.discardedFrames());
}

void test_unknown_type_is_rejected()
{
  uint8_t buf[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 1, buf);
  buf[2] = 5;
  uint16_t crc = crc16(buf + 1, HEADER_SIZE - 1 + COMMAND_PAYLOAD_SIZE);
  buf[n - 2] = (uint8_t)(crc & 0xFF);
  buf[n - 1] = (uint8_t)(crc >> 8);
  Frame f;
  TEST_ASSERT_EQUAL(0, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(1, dec.discardedFrames());
}

void test_inconsistent_length_is_rejected()
{
  uint8_t buf[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 1, buf);
  buf[4] = COMMAND_PAYLOAD_SIZE - 1;
  Frame f;
  TEST_ASSERT_EQUAL(0, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(1, dec.discardedFrames());
}

void test_sequence_wraps()
{
  uint8_t buf[MAX_FRAME_SIZE];
  Frame f;
  int n = encodeCommand(sampleCommand(), 255, buf);
  TEST_ASSERT_EQUAL(1, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(255, f.seq);
  n = encodeCommand(sampleCommand(), (uint8_t)(255 + 1), buf);
  TEST_ASSERT_EQUAL(1, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(0, f.seq);
}

void test_frame_split_across_pushes_without_next()
{
  // Bytes pushed without calling next() in between still decode.
  uint8_t buf[MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 5, buf);
  feed(buf, n);
  Frame f;
  TEST_ASSERT_TRUE(dec.next(f));
  TEST_ASSERT_EQUAL(5, f.seq);
  TEST_ASSERT_FALSE(dec.next(f));
}

void test_telemetry_and_report_in_one_burst()
{
  // 174 bytes in one burst, more than the decoder holds: decoding after each
  // byte keeps both frames.
  uint8_t buf[TELEMETRY_FRAME_SIZE + CALIBRATION_REPORT_FRAME_SIZE];
  int n = encodeTelemetry(sampleTelemetry(), 1, buf);
  n += encodeCalibrationReport(sampleReport(), 2, buf + n);
  TEST_ASSERT_TRUE(n > MAX_FRAME_SIZE);
  Frame first = {}, f;
  int frames = 0;
  for (int i = 0; i < n; i++)
  {
    dec.push(buf[i]);
    while (dec.next(f))
    {
      if (frames == 0)
        first = f;
      frames++;
    }
  }
  TEST_ASSERT_EQUAL(2, frames);
  TEST_ASSERT_TRUE(first.type == FrameType::Telemetry);
  TEST_ASSERT_TRUE(f.type == FrameType::CalibrationReport);
  TEST_ASSERT_EQUAL(0, dec.discardedFrames());
}

void test_back_to_back_frames()
{
  uint8_t buf[2 * MAX_FRAME_SIZE];
  int n = encodeCommand(sampleCommand(), 1, buf);
  n += encodeCommand(sampleCommand(), 2, buf + n);
  Frame f;
  TEST_ASSERT_EQUAL(2, feedAndCount(buf, n, f));
  TEST_ASSERT_EQUAL(2, f.seq);
  TEST_ASSERT_EQUAL(0, dec.discardedFrames());
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_frame_sizes);
  RUN_TEST(test_command_round_trip);
  RUN_TEST(test_telemetry_round_trip);
  RUN_TEST(test_calibration_set_round_trip);
  RUN_TEST(test_calibration_report_round_trip);
  RUN_TEST(test_report_and_commands_back_to_back);
  RUN_TEST(test_every_single_byte_corruption_is_rejected);
  RUN_TEST(test_corrupted_frame_is_counted);
  RUN_TEST(test_resynchronises_over_garbage);
  RUN_TEST(test_frame_hidden_in_a_rejected_candidate_is_found);
  RUN_TEST(test_frame_behind_incomplete_false_start_is_returned_when_idle);
  RUN_TEST(test_command_shaped_bytes_inside_telemetry_do_not_split_it);
  RUN_TEST(test_idle_keeps_a_complete_buffer_untouched);
  RUN_TEST(test_version_1_frame_is_rejected);
  RUN_TEST(test_unknown_version_is_rejected);
  RUN_TEST(test_unknown_type_is_rejected);
  RUN_TEST(test_inconsistent_length_is_rejected);
  RUN_TEST(test_sequence_wraps);
  RUN_TEST(test_frame_split_across_pushes_without_next);
  RUN_TEST(test_telemetry_and_report_in_one_burst);
  RUN_TEST(test_back_to_back_frames);
  return UNITY_END();
}
