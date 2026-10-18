#include <cstdio>
#include "pico/stdlib.h"

#include "config.h"
#include "movement/body.h"
#include "protection/protection.h"
#include "protection/supervisor.h"
#include "power_monitor.h"
#include "servo_output.h"
#include "status_leds.h"
#ifdef SPOODER_SERIAL_COMMANDS
#include "serial_commands.h"
#else
#include "link.h"
#include "usb_link.h"
#endif

// Control loop period. Movement advances by elapsed time, so this is a target,
// not a hard deadline.
constexpr uint32_t LOOP_PERIOD_MS = 20;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
constexpr int TELEMETRY_EVERY_TICKS = 5; // 100 ms
// Pose frames (USB only) at the telemetry rate, two ticks after it, so the
// two never share one tick's USB transmit buffer.
constexpr int POSE_TICK = 2;

static body::BodyState bodyState;
static protection::State protectionState;
static protection::Supervisor supervisor;
// Requested: the latest accepted calibration (what reports show). Applied:
// what the IK and servo output use, following requested at a bounded rate.
// Both start from the compiled defaults; live changes last until restart.
static joint_calibration::Table requestedCalibration;
static joint_calibration::Table appliedCalibration;
#ifndef SPOODER_SERIAL_COMMANDS
static link_receiver::Arbiter arbiter;
static link_receiver::Source lastSource = link_receiver::Source::None;
static link_receiver::ReportSchedule bridgeReports;
static link_receiver::ReportSchedule usbReports;
#endif

// The LED pattern mirrors body::Mode by value.
static_assert((int)led_pattern::BodyMode::Walk == (int)body::Mode::Walk &&
                  (int)led_pattern::BodyMode::Calibrate == (int)body::Mode::Calibrate,
              "led_pattern::BodyMode must mirror body::Mode");

// One tick's operator input: from the bridge or USB link, whichever is in
// control, or (bring-up builds) the USB serial console keys.
struct TickInput
{
  MovementCommand command;
  uint32_t requestSeq;   // advances on every explicit mode request
  bool clearFaults;
  bool emergencyStop;
  bool printReadings;
  led_pattern::Input source;
  uint8_t telemetryFlags; // protocol::TELEMETRY_* source bits
};

static bool isFresh(const MovementCommand &c, uint32_t now)
{
  return c.issuedAtMs != 0 && now - c.issuedAtMs <= body::COMMAND_TIMEOUT_MS;
}

static TickInput readInput(uint32_t now)
{
  TickInput in = {};
#ifdef SPOODER_SERIAL_COMMANDS
  serial_commands::Input s = serial_commands::poll(now); // bounded
  in.command = s.command;
  in.requestSeq = s.requestSeq;
  in.clearFaults = s.clearFaults;
  in.printReadings = s.printReadings;
  in.source = led_pattern::Input::Serial;
#else
  link_receiver::Input bridge = link::poll(now); // bounded by the receive buffer
  link_receiver::Input usb = usb_link::poll(now); // bounded by MAX_BYTES_PER_POLL
  link_receiver::Arbitration a = link_receiver::arbitrate(arbiter, bridge, usb, now);
  in.command = a.input.command;
  in.requestSeq = a.input.requestSeq;
  in.clearFaults = a.input.clearRequested;
  in.emergencyStop = a.input.emergencyStop; // from either link
  in.telemetryFlags = a.telemetryFlags;
  switch (a.source)
  {
  case link_receiver::Source::Bridge:
    in.source = led_pattern::Input::LinkConnected;
    break;
  case link_receiver::Source::Usb:
    in.source = led_pattern::Input::UsbConnected;
    break;
  case link_receiver::Source::Conflict:
    in.source = led_pattern::Input::SourceConflict;
    break;
  case link_receiver::Source::None:
    in.source = link_receiver::isFresh(bridge, now) ? led_pattern::Input::LinkNoGamepad : led_pattern::Input::LinkStale;
    break;
  }
  if (a.source != lastSource)
  {
    static const char *const NAMES[] = {"none", "bridge", "USB", "conflict (bridge and USB): sleeping"};
    printf("operator source: %s\n", NAMES[(int)a.source]);
    lastSource = a.source;
  }
#endif
  return in;
}

#ifndef SPOODER_SERIAL_COMMANDS
static uint16_t clampU16(float v)
{
  if (v <= 0)
    return 0;
  if (v >= 65535.0f)
    return 65535;
  return (uint16_t)(v + 0.5f);
}

static void sendTelemetry(const protection::Reading &r, const protection::Result &p, bool outputsOn,
                          uint8_t flags)
{
  protocol::Telemetry t = {};
  t.bodyMode = (uint8_t)bodyState.mode;
  t.gait = (uint8_t)bodyState.currentGait;
  t.outputsOn = outputsOn;
  t.faults = p.faults;
  t.voltageMv = clampU16(r.voltageV * 1000.0f);
  t.currentMa = clampU16(r.currentA * 1000.0f);
  t.sensorsAvailable = 0; // filled by add-body-sensing
  t.flags = flags;
  // Each link reports its own frame counters.
  t.validFrames = (uint16_t)link::validFrames();
  t.discardedFrames = (uint16_t)link::discardedFrames();
  link::sendTelemetry(t);
  t.validFrames = (uint16_t)usb_link::validFrames();
  t.discardedFrames = (uint16_t)usb_link::discardedFrames();
  usb_link::sendTelemetry(t);
}
#endif

static led_pattern::State ledState(uint32_t now, const protection::Result &prot, bool bodyRunning,
                                   float currentA, led_pattern::Input source)
{
  led_pattern::State s;
  s.nowMs = now;
  s.faults = prot.faults;
  s.supplyAccepted = prot.allowOutput;
  s.bodyRunning = bodyRunning;
  s.mode = (led_pattern::BodyMode)bodyState.mode;
  s.gait = (int)bodyState.currentGait;
  s.currentA = currentA;
  s.tripCurrentA = config::POWER_LIMITS.tripCurrentA;
  s.sustainedCurrentA = config::POWER_LIMITS.sustainedCurrentA;
  s.input = source;
  return s;
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

static void reportFaults(uint16_t faults, const protection::Reading &r)
{
  for (uint16_t bit = 1; bit != 0; bit <<= 1)
    if (faults & bit)
      printf("fault latched: %s (%.2f A, %.2f V)\n", faultName(bit), r.currentA, r.voltageV);
}

static void reportStatus(const protection::Reading &r, const protection::Result &p, bool outputsOn,
                         bool bodyRunning)
{
  printf("supply %.2f A (avg %.2f A), %.2f V%s; output %s; body %s, mode %d; faults 0x%02x\n", r.currentA,
         p.averageCurrentA, r.voltageV, p.supplyValid ? "" : " (out of range or absent)",
         outputsOn ? "on" : "off", bodyRunning ? "running" : "stopped", (int)bodyState.mode, p.faults);
}

static void reportRejections()
{
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    if (!bodyState.newlyRejected[leg])
      continue;
    const Vector3 &t = bodyState.rejectedTarget[leg];
    printf("point impossible to reach: (%.1f, %.1f, %.1f) leg %d\n", t.x, t.y, t.z, leg);
  }
}

static uint32_t nowMs()
{
  return to_ms_since_boot(get_absolute_time());
}

int main()
{
  stdio_init_all();
  status_leds::setup();
  power_monitor::setup();
#ifndef SPOODER_SERIAL_COMMANDS
  link::setup();
#endif
  protection::init(protectionState, config::POWER_LIMITS);
  joint_calibration::loadDefaults(requestedCalibration);
  joint_calibration::loadDefaults(appliedCalibration);

  if (!servo_output::setup())
  {
    // Without the servo cluster nothing may move; report and stay here.
    while (true)
    {
      printf("servo output failed to initialise\n");
      led_pattern::State failed = {};
      failed.nowMs = nowMs();
      failed.faults = 0xFFFF; // shown as a fault: nothing may move
      status_leds::show(failed);
      sleep_ms(REPORT_PERIOD_MS);
    }
  }

#ifndef SPOODER_SERIAL_COMMANDS
  int telemetryTick = 0;
#endif
  uint32_t lastReportMs = 0;
  uint32_t lastTickMs = nowMs();
  absolute_time_t next = get_absolute_time();

  while (true)
  {
    uint32_t now = nowMs();
    uint32_t elapsedMs = now - lastTickMs;
    lastTickMs = now;
    if (elapsedMs > body::MAX_STEP_MS)
      elapsedMs = body::MAX_STEP_MS; // an overrun never makes one large calibration step

    // Input first, so an emergency stop in a frame that has arrived acts in
    // this tick. Both input sources are bounded and never block, so they
    // cannot delay the protection step that follows.
    TickInput input = readInput(now);

    protection::Reading reading = power_monitor::read();
    protection::Result prot =
        protection::step(protectionState, reading, now, input.clearFaults, input.emergencyStop);
    servo_output::setInhibited(!prot.allowOutput);
    if (prot.newlyLatched)
      reportFaults(prot.newlyLatched, reading);

    // A stale command never starts the body; the body itself sleeps on it.
    RequestedMode requested = isFresh(input.command, now) ? input.command.mode : RequestedMode::Sleep;

    // The supervisor decides whether the body runs: a fault that cuts output
    // stops it, and it restarts through the boot sequence only once output is
    // allowed and the operator has made a new request since the fault.
    protection::Decision run = protection::supervise(supervisor, prot, requested, input.requestSeq);
    if (run.stopBody)
    {
      servo_output::stop();
      printf("movement stopped: waiting for the fault to clear and a new request\n");
    }
    if (run.startBody)
    {
      body::init(bodyState);
      printf("output allowed: starting movement\n");
    }
    // Calibration changes are guarded with this tick's output state and the
    // body's mode; applied values then move towards them at the rate limit.
#ifndef SPOODER_SERIAL_COMMANDS
    bool calibrating = run.runBody && bodyState.mode == body::Mode::Calibrate;
    int processed = link::processCalibration(requestedCalibration, calibrating, prot.allowOutput);
    processed += usb_link::processCalibration(requestedCalibration, calibrating, prot.allowOutput);
    if (processed > 0)
    {
      // Every link hears about every change.
      link_receiver::requestReport(bridgeReports);
      link_receiver::requestReport(usbReports);
    }
#endif
    joint_calibration::step(appliedCalibration, requestedCalibration, elapsedMs / 1000.0f);

    if (run.runBody)
    {
      MovementCommand cmd = input.command;
      cmd.mode = run.mode; // protection only rewrites the request
      for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
        body::setOffsets(bodyState, leg, joint_calibration::legOffsets(appliedCalibration, leg));
      body::step(bodyState, cmd, now);
      reportRejections();
      servo_output::apply(bodyState, appliedCalibration);
    }

    status_leds::show(ledState(now, prot, run.runBody, reading.currentA, input.source));

#ifndef SPOODER_SERIAL_COMMANDS
    if (++telemetryTick >= TELEMETRY_EVERY_TICKS)
    {
      telemetryTick = 0;
      sendTelemetry(reading, prot, servo_output::outputsOn(), input.telemetryFlags);
    }
    if (telemetryTick == POSE_TICK)
      usb_link::sendPose(link_receiver::makePose(bodyState, servo_output::pulses(), servo_output::outputsOn()));
    // In Calibrate, front-ends follow the table through these reports.
    calibrating = run.runBody && bodyState.mode == body::Mode::Calibrate;
    if (link_receiver::reportDue(bridgeReports, calibrating, now) && link::sendCalibrationReport(requestedCalibration))
      link_receiver::reportSent(bridgeReports, now);
    if (link_receiver::reportDue(usbReports, calibrating, now) && usb_link::sendCalibrationReport(requestedCalibration))
      link_receiver::reportSent(usbReports, now);
#endif

    if (input.printReadings || now - lastReportMs >= REPORT_PERIOD_MS)
    {
      reportStatus(reading, prot, servo_output::outputsOn(), run.runBody);
      lastReportMs = now;
    }

    next = delayed_by_ms(next, LOOP_PERIOD_MS);
    if (absolute_time_diff_us(get_absolute_time(), next) > 0)
      sleep_until(next);
    else
      next = get_absolute_time(); // overran: start the next period now
  }
}
