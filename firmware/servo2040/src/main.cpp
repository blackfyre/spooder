#include <cstdio>
#include "pico/stdlib.h"

#include "config.h"
#include "movement/body.h"
#include "protection/protection.h"
#include "protection/resume_gate.h"
#include "power_monitor.h"
#include "serial_commands.h"
#include "servo_output.h"
#include "status_leds.h"

// Control loop period. Movement advances by elapsed time, so this is a target,
// not a hard deadline.
constexpr uint32_t LOOP_PERIOD_MS = 20;
constexpr uint32_t REPORT_PERIOD_MS = 1000;

static body::BodyState bodyState;
static protection::State protectionState;
static protection::ResumeGate resumeGate;

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

static void reportStatus(const protection::Reading &r, const protection::Result &p, bool outputsOn)
{
  printf("supply %.2f A (avg %.2f A), %.2f V%s; output %s; mode %d; faults 0x%02x\n", r.currentA,
         p.averageCurrentA, r.voltageV, p.supplyValid ? "" : " (out of range or absent)",
         outputsOn ? "on" : "off", (int)bodyState.mode, p.faults);
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
  protection::init(protectionState, config::POWER_LIMITS);

  if (!servo_output::setup())
  {
    // Without the servo cluster nothing may move; report and stay here.
    while (true)
    {
      printf("servo output failed to initialise\n");
      status_leds::show(nowMs(), true, 0xFFFF, body::Mode::Initialise);
      sleep_ms(REPORT_PERIOD_MS);
    }
  }

  // The body starts only once protection first allows output, so Initialise
  // runs with the servos live instead of while output is still held off.
  bool bodyStarted = false;
  bool clearRequested = false; // from the previous tick's input
  uint32_t lastReportMs = 0;
  absolute_time_t next = get_absolute_time();

  while (true)
  {
    uint32_t now = nowMs();

    // Protection first, so nothing else in the tick can delay a trip.
    protection::Reading reading = power_monitor::read();
    protection::Result prot = protection::step(protectionState, reading, now, clearRequested);
    servo_output::setInhibited(!prot.allowOutput);
    if (prot.newlyLatched)
      reportFaults(prot.newlyLatched, reading);

    serial_commands::Input input = serial_commands::poll(now); // bounded
    clearRequested = input.clearFaults;

    // Protection only rewrites the request; movement still owns transitions.
    // After a fault the body stays asleep until a new request is made. The gate
    // sees every latch, including ones before the body has started.
    MovementCommand cmd = input.command;
    cmd.mode = protection::resumeGate(resumeGate, prot.newlyLatched != 0, prot.forceSleep, cmd.mode,
                                      input.requestSeq);

    if (!bodyStarted && prot.allowOutput)
    {
      body::init(bodyState);
      bodyStarted = true;
      printf("supply accepted: starting movement\n");
    }

    if (bodyStarted)
    {
      body::step(bodyState, cmd, now);
      reportRejections();
      servo_output::apply(bodyState);
    }

    status_leds::show(now, prot.allowOutput, prot.faults, bodyState.mode);

    if (input.printReadings || now - lastReportMs >= REPORT_PERIOD_MS)
    {
      reportStatus(reading, prot, servo_output::outputsOn());
      lastReportMs = now;
    }

    next = delayed_by_ms(next, LOOP_PERIOD_MS);
    if (absolute_time_diff_us(get_absolute_time(), next) > 0)
      sleep_until(next);
    else
      next = get_absolute_time(); // overran: start the next period now
  }
}
