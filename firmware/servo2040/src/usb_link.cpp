#include "usb_link.h"

#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "tusb.h"

namespace usb_link
{

// A whole report must fit the USB transmit buffer (CFG_TUD_CDC_TX_BUFSIZE,
// raised in CMakeLists.txt), or it could never be written whole.
static_assert(CFG_TUD_CDC_TX_BUFSIZE >= protocol::MAX_FRAME_SIZE, "a whole frame fits the USB transmit buffer");

static protocol::Decoder decoder;
static link_receiver::Receiver receiver;
static uint8_t txSeq;

static void handleFrames(uint32_t nowMs)
{
  protocol::Frame f;
  while (decoder.next(f))
  {
    if (f.type == protocol::FrameType::Command)
      link_receiver::accept(receiver, f.command, nowMs);
    else if (f.type == protocol::FrameType::CalibrationSet)
      link_receiver::acceptCalibration(receiver, f.calibrationSet);
  }
}

link_receiver::Input poll(uint32_t nowMs)
{
  bool received = false;
  for (int i = 0; i < MAX_BYTES_PER_POLL; i++)
  {
    int c = stdio_getchar_timeout_us(0);
    if (c < 0)
      break; // PICO_ERROR_TIMEOUT: nothing pending
    received = true;
    decoder.push((uint8_t)c);
    handleFrames(nowMs);
  }
  // As on UART1: a tick without a byte ends any false start.
  if (!received)
  {
    decoder.idle();
    handleFrames(nowMs);
  }
  return link_receiver::take(receiver);
}

int processCalibration(joint_calibration::Table &requested, bool calibrating, bool allowOutput)
{
  return link_receiver::processCalibration(receiver, requested, calibrating, allowOutput);
}

// Writes the frame whole or not at all. stdio's own write would wait up to
// PICO_STDIO_USB_STDOUT_TIMEOUT_US for room and could then stop part-way.
static bool writeFrame(const uint8_t *frame, int n)
{
  if (!stdio_usb_connected() || tud_cdc_write_available() < (uint32_t)n)
    return false;
  // No newline, no CR translation: the bytes go out as they are.
  stdio_put_string((const char *)frame, n, false, false);
  return true;
}

bool sendTelemetry(const protocol::Telemetry &t)
{
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeTelemetry(t, txSeq, frame);
  if (!writeFrame(frame, n))
    return false;
  txSeq++;
  return true;
}

bool sendCalibrationReport(const joint_calibration::Table &table)
{
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCalibrationReport(link_receiver::makeReport(receiver, table), txSeq, frame);
  if (!writeFrame(frame, n))
    return false;
  txSeq++;
  return true;
}

bool sendPose(const protocol::Pose &p)
{
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodePose(p, txSeq, frame);
  if (!writeFrame(frame, n))
    return false;
  txSeq++;
  return true;
}

uint32_t validFrames() { return decoder.validFrames(); }
uint32_t discardedFrames() { return decoder.discardedFrames(); }

} // namespace usb_link
