#include "link.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "setup/portal_controller.h"

namespace robot_link
{

constexpr int TX_PIN = 27;
constexpr int RX_PIN = 26;
constexpr uint32_t BAUD = 115200;
constexpr int ESTOP_PIN = 35; // right-hand button, active low, board pull-up
// The robot sends whole frames back to back, so a gap this long in received
// bytes never falls inside a frame: an incomplete candidate is a false start.
constexpr uint32_t RX_IDLE_MS = 10;
// More than one command period of bytes at full line rate (about 230 bytes
// per 20 ms), with room for a late link task.
constexpr int RX_BUFFER_SIZE = 1024;

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
// Shared with loop(), guarded by lock.
static gamepad_mapping::Desired desired;
static protocol::Telemetry telemetry;
static bool haveTelemetry;
static uint32_t telemetryAtMs;
static bool emergencyStop;
static uint32_t validFrames;
static uint32_t discardedFrames;
static bool haveReport;
static uint32_t reportAtMs;
static protocol::CalibrationReport report;
// Setup mode only. portalMode is set before the task starts and never
// changes afterwards.
static bool portalMode;
static portal_controller::Controller portal;
static calibration_sync::Sync calibration;

static protocol::Decoder decoder;

static void initialDesired()
{
  desired = {};
  desired.mode = RequestedMode::Sleep;
  desired.speed = 44;
  desired.controllerConnected = false;
}

// Toggles on each press. Sampled every command period; a press is a high to
// low change after two high samples, so contact bounce cannot toggle twice.
static void sampleEmergencyStop()
{
  static uint8_t history = 0xFF;
  history = (uint8_t)((history << 1) | (digitalRead(ESTOP_PIN) ? 1 : 0));
  if ((history & 0x07) == 0x06)
  {
    portENTER_CRITICAL(&lock);
    emergencyStop = !emergencyStop;
    portEXIT_CRITICAL(&lock);
  }
}

static void sendCommand(uint8_t seq)
{
  portENTER_CRITICAL(&lock);
  // In setup mode the portal's heartbeat is judged here, every period, so a
  // stalled web server cannot keep the controller connected. The time is
  // taken inside the lock, so no poll can be stamped after it.
  if (portalMode)
    desired = portal_controller::desired(portal, millis());
  protocol::Command c = gamepad_mapping::toCommand(desired, emergencyStop);
  portEXIT_CRITICAL(&lock);
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCommand(c, seq, frame);
  // The TX buffer holds many frames, so this never waits.
  Serial1.write(frame, n);
}

// Setup mode: at most one calibration set per period, after the command.
static void sendCalibration(uint8_t seq, uint32_t now)
{
  protocol::CalibrationSet set;
  portENTER_CRITICAL(&lock);
  bool due = calibration_sync::next(calibration, now, set);
  portEXIT_CRITICAL(&lock);
  if (!due)
    return;
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCalibrationSet(set, seq, frame);
  Serial1.write(frame, n);
}

// Hands every complete frame in the decoder to loop()'s side of the lock.
static void takeFrames(uint32_t now)
{
  protocol::Frame f;
  while (decoder.next(f))
  {
    if (f.type == protocol::FrameType::Telemetry)
    {
      portENTER_CRITICAL(&lock);
      telemetry = f.telemetry;
      haveTelemetry = true;
      telemetryAtMs = now;
      portEXIT_CRITICAL(&lock);
    }
    else if (f.type == protocol::FrameType::CalibrationReport)
    {
      portENTER_CRITICAL(&lock);
      report = f.calibrationReport;
      haveReport = true;
      reportAtMs = now;
      if (portalMode)
        calibration_sync::onReport(calibration, f.calibrationReport);
      portEXIT_CRITICAL(&lock);
    }
  }
}

static void receive()
{
  static uint32_t lastByteMs = 0;
  static bool idleHandled = true;
  uint32_t now = millis();
  int avail = Serial1.available();
  if (avail > 0)
  {
    lastByteMs = now;
    idleHandled = false;
  }
  // Decode after every byte: the decoder holds one frame at most, and a poll
  // can bring a telemetry frame and a calibration report together.
  while (avail-- > 0)
  {
    decoder.push((uint8_t)Serial1.read());
    takeFrames(now);
  }
  if (!idleHandled && now - lastByteMs >= RX_IDLE_MS)
  {
    decoder.idle();
    idleHandled = true;
    takeFrames(now);
  }

  portENTER_CRITICAL(&lock);
  validFrames = decoder.validFrames();
  discardedFrames = decoder.discardedFrames();
  portEXIT_CRITICAL(&lock);
}

static void linkTask(void *)
{
  uint8_t seq = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;)
  {
    sampleEmergencyStop();
    sendCommand(seq++);
    if (portalMode)
      sendCalibration(seq++, millis());
    receive();
    // Fixed-rate: a late wake-up does not shift later frames.
    xTaskDelayUntil(&wake, pdMS_TO_TICKS(COMMAND_PERIOD_MS));
  }
}

static void startTask()
{
  initialDesired();
  pinMode(ESTOP_PIN, INPUT); // input-only pin; the board has the pull-up
  Serial1.setTxBufferSize(256);
  Serial1.setRxBufferSize(RX_BUFFER_SIZE);
  Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  // Above loop() (priority 1) so screen redraws cannot delay frames; pinned
  // to loop()'s core, leaving core 0 to the Bluetooth stack.
  xTaskCreatePinnedToCore(linkTask, "link", 4096, nullptr, 5, nullptr, ARDUINO_RUNNING_CORE);
}

void start()
{
  portalMode = false;
  startTask();
}

void startPortal()
{
  portalMode = true;
  startTask();
}

void setDesired(const gamepad_mapping::Desired &d)
{
  portENTER_CRITICAL(&lock);
  desired = d;
  portEXIT_CRITICAL(&lock);
}

Status status()
{
  Status s;
  portENTER_CRITICAL(&lock);
  uint32_t nowMs = millis();
  s.telemetryFresh = haveTelemetry && nowMs - telemetryAtMs <= TELEMETRY_TIMEOUT_MS;
  s.telemetryAgeMs = haveTelemetry ? nowMs - telemetryAtMs : 0;
  s.telemetry = telemetry;
  s.emergencyStop = emergencyStop;
  s.validFrames = validFrames;
  s.discardedFrames = discardedFrames;
  s.haveReport = haveReport;
  s.reportAtMs = reportAtMs;
  s.report = report;
  portEXIT_CRITICAL(&lock);
  return s;
}

void portalPoll()
{
  portENTER_CRITICAL(&lock);
  portal_controller::poll(portal, millis());
  portEXIT_CRITICAL(&lock);
}

bool portalRequestMode(RequestedMode mode)
{
  portENTER_CRITICAL(&lock);
  bool taken = portal_controller::requestMode(portal, mode, millis());
  portEXIT_CRITICAL(&lock);
  return taken;
}

void portalRequestClear()
{
  portENTER_CRITICAL(&lock);
  portal_controller::requestClear(portal, millis());
  portEXIT_CRITICAL(&lock);
}

bool portalChange(int joint, calibration_sync::Field field, int32_t value, bool relative)
{
  portENTER_CRITICAL(&lock);
  // Like the mode buttons, an edit comes from an open page: it counts as a
  // poll.
  portal_controller::poll(portal, millis());
  bool changed = calibration_sync::change(calibration, joint, field, value, relative);
  portEXIT_CRITICAL(&lock);
  return changed;
}

void portalView(PortalView &out)
{
  portENTER_CRITICAL(&lock);
  uint32_t nowMs = millis();
  // Judged on a copy, so a view has no effect on the frames.
  portal_controller::Controller c = portal;
  out.connected = portal_controller::connected(c, nowMs);
  out.desired = portal_controller::desired(c, nowMs);
  out.sync = calibration;
  portEXIT_CRITICAL(&lock);
}

} // namespace robot_link
