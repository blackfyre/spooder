#include "status_screen.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "protection/protection.h"

namespace status_screen
{

static const char *const BODY_MODES[] = {"Initialise", "Stand", "Walk", "Sleep", "Calibrate", "Perform"};

const char *faultName(uint16_t bit)
{
  switch (bit)
  {
  case protection::OVERCURRENT:
    return "overcurrent";
  case protection::SUSTAINED_CURRENT:
    return "overload";
  case protection::UNDERVOLTAGE:
    return "low V";
  case protection::BROWNOUT:
    return "brown-out";
  case protection::OVERVOLTAGE:
    return "high V";
  case protection::EMERGENCY_STOP:
    return "e-stop";
  default:
    return nullptr;
  }
}

static void set(Line &l, Tone tone, const char *text)
{
  snprintf(l.text, sizeof(l.text), "%s", text);
  l.tone = tone;
}

static const char *name(const char *const *names, int count, int i)
{
  return i >= 0 && i < count ? names[i] : "?";
}

// Centidegrees as degrees with one decimal, keeping the sign of -0.x.
static int formatTenths(char *buf, int size, int16_t centideg)
{
  int v = centideg < 0 ? -(int)centideg : centideg;
  return snprintf(buf, size, "%s%d.%d", centideg < 0 ? "-" : "", v / 100, v % 100 / 10);
}

static const char *const BODY_MODE_LABELS[] = {"INITIALISE", "STAND", "WALK", "SLEEP", "CALIBRATE", "PERFORM"};
static const char *const GESTURE_LABELS[] = {"NONE", "SAY HI", "WIGGLE", "DANCE", "ATTACK", "BALL"};
static const char *const GAIT_LABELS[] = {"TRI", "WAVE", "RIPPLE", "BI", "QUAD", "HOP"};
// The middle dot, in UTF-8: the screen's fonts carry it.
#define DOT "\xc2\xb7"

// A fault's name on the status strip.
static const char *stripName(uint16_t bit)
{
  switch (bit)
  {
  case protection::OVERCURRENT:
    return "OVERCURRENT";
  case protection::SUSTAINED_CURRENT:
    return "OVERLOAD";
  case protection::UNDERVOLTAGE:
    return "LOW V";
  case protection::BROWNOUT:
    return "BROWN-OUT";
  case protection::OVERVOLTAGE:
    return "HIGH V";
  case protection::EMERGENCY_STOP:
    return "E-STOP";
  default:
    return nullptr;
  }
}

int batteryPercent(uint8_t battery)
{
  if (battery == 0)
    return -1;
  return (battery - 1) * 100 / 254;
}

static void text(Text &t, Tone tone, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void text(Text &t, Tone tone, const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(t.text, sizeof(t.text), fmt, args);
  va_end(args);
  t.tone = tone;
}

// An absent value: "--" after the label.
static void none(Text &t, const char *label)
{
  text(t, Tone::Muted, "%s%s--", label, *label ? " " : "");
}

static float fraction(float value, float low, float high)
{
  float f = (value - low) / (high - low);
  return f < 0 ? 0 : (f > 1 ? 1 : f);
}

// Volts or amps with two decimals, from milli-units.
static void milli(Text &t, Tone tone, const char *prefix, uint16_t value, const char *unit)
{
  text(t, tone, "%s%u.%02u %s", prefix, value / 1000, (value % 1000) / 10, unit);
}

// Appends @p word to the strip, on the first line while it fits, then on the
// second; a word that fits neither is cut on the last line. Words are two
// spaces apart: every fault, the e-stop and the source state fit two lines.
static void stripAdd(Strip &s, int &line, const char *word)
{
  for (; line < STRIP_LINES; line++)
  {
    char *l = s.lines[line];
    int used = (int)strlen(l);
    int need = used ? used + 2 + (int)strlen(word) : (int)strlen(word);
    if (need <= STRIP_LINE_CHARS || line == STRIP_LINES - 1)
    {
      snprintf(l + used, STRIP_LINE_CHARS + 1 - used, "%s%s", used ? "  " : "", word);
      return;
    }
  }
}

static void composeStrip(const Inputs &in, Strip &s)
{
  int line = 0;
  bool bad = false, warn = false;
  const protocol::Telemetry &t = in.telemetry;
  if (in.emergencyStop || (in.telemetryFresh && (t.faults & protection::EMERGENCY_STOP)))
  {
    stripAdd(s, line, "E-STOP");
    bad = true;
  }
  if (in.telemetryFresh)
  {
    for (uint16_t bit = 1; bit != 0; bit <<= 1)
    {
      if (!(t.faults & bit) || bit == protection::EMERGENCY_STOP)
        continue;
      char unknown[8];
      const char *n = stripName(bit);
      if (!n)
      {
        snprintf(unknown, sizeof(unknown), "0x%04X", bit);
        n = unknown;
      }
      stripAdd(s, line, n);
      bad = true;
    }
    if (t.flags & protocol::TELEMETRY_SOURCE_CONFLICT)
    {
      stripAdd(s, line, "SOURCE CONFLICT");
      bad = true;
    }
    else if (t.flags & protocol::TELEMETRY_USB_IN_CONTROL)
    {
      stripAdd(s, line, "USB IN CONTROL");
      warn = true;
    }
  }
  else
  {
    stripAdd(s, line, "LINK LOST");
    bad = true;
  }
  if (!bad && !warn)
    stripAdd(s, line, "NO FAULTS");
  s.tone = bad ? Tone::Bad : (warn ? Tone::Warn : Tone::Good);
}

static void composeHeader(const Inputs &in, Header &h)
{
  const protocol::Telemetry &t = in.telemetry;
  if (!in.telemetryFresh)
    none(h.mode, "");
  else if (t.gesture != 0)
    text(h.mode, Tone::Normal, "PERFORM " DOT " %s", name(GESTURE_LABELS, 6, t.gesture));
  else
    text(h.mode, Tone::Normal, "%s " DOT " %s", name(BODY_MODE_LABELS, 6, t.bodyMode), name(GAIT_LABELS, 6, t.gait));

  int battery = batteryPercent(in.gamepadBattery);
  if (!in.gamepadConnected)
    text(h.gamepad, Tone::Warn, "NO PAD");
  else if (battery < 0)
    text(h.gamepad, Tone::Good, "PAD");
  else
    text(h.gamepad, battery < 20 ? Tone::Warn : Tone::Good, "PAD %d%%", battery);

  if (!in.telemetryFresh)
    text(h.link, Tone::Bad, "LOST");
  else if (t.flags & protocol::TELEMETRY_SOURCE_CONFLICT)
    text(h.link, Tone::Bad, "CONFLICT");
  else if (t.flags & protocol::TELEMETRY_USB_IN_CONTROL)
    text(h.link, Tone::Warn, "USB");
  else
    text(h.link, Tone::Good, "LINK");
}

static void composeDrive(const Inputs &in, DriveBody &d)
{
  const gamepad_mapping::Desired &want = in.desired;
  text(d.height, Tone::Normal, "H %u", want.height);
  d.heightBar = {true, fraction(want.height, 0, 100)};
  text(d.speed, Tone::Normal, "SPD %u", want.speed);
  d.speedBar = {true, fraction(want.speed, 0, 100)};
  text(d.modes, Tone::Muted, "%s " DOT " %s", want.stride == StrideMode::Fixed ? "FIXED" : "DYNAMIC",
       want.stickMode == gamepad_mapping::StickMode::Crab ? "CRAB" : "STEER");

  if (!in.telemetryFresh)
  {
    none(d.voltage, "");
    none(d.current, "");
    none(d.output, "OUT");
    return; // bars absent
  }
  const protocol::Telemetry &t = in.telemetry;
  text(d.output, t.outputsOn ? Tone::Good : Tone::Muted, "%s", t.outputsOn ? "OUT ON" : "OUT OFF");
  // Amber while setting a new low or high beyond the held one.
  milli(d.voltage, in.voltageLow.present && in.voltageLow.warning ? Tone::Warn : Tone::Normal, "", t.voltageMv, "V");
  milli(d.current, in.currentPeak.present && in.currentPeak.warning ? Tone::Warn : Tone::Normal, "", t.currentMa, "A");
  d.voltageBar = {true, fraction(t.voltageMv, BAR_MIN_MV, BAR_MAX_MV), in.voltageLow.present,
                  fraction(in.voltageLow.value, BAR_MIN_MV, BAR_MAX_MV)};
  d.currentBar = {true, fraction(t.currentMa, 0, BAR_MAX_MA), in.currentPeak.present,
                  fraction(in.currentPeak.value, 0, BAR_MAX_MA)};

  if (t.sensorsAvailable & (protocol::SENSOR_TOF | protocol::SENSOR_ATTITUDE))
  {
    // Compact, so the widest values still fit: "TOF 65535MM P-327.6 R-327.6".
    char buf[40];
    int used = 0;
    if (t.sensorsAvailable & protocol::SENSOR_TOF)
      used += snprintf(buf + used, sizeof(buf) - used, "TOF %uMM ", t.tofMm);
    if (t.sensorsAvailable & protocol::SENSOR_ATTITUDE)
    {
      used += snprintf(buf + used, sizeof(buf) - used, "P");
      used += formatTenths(buf + used, sizeof(buf) - used, t.pitchCentideg);
      used += snprintf(buf + used, sizeof(buf) - used, " R");
      formatTenths(buf + used, sizeof(buf) - used, t.rollCentideg);
    }
    text(d.sensors, Tone::Normal, "%s", buf);
  }
}

static void composePower(const Inputs &in, PowerBody &p)
{
  text(p.note, Tone::Muted, "10 HZ SAMPLES " DOT " 60 S");
  none(p.voltage, "");
  none(p.current, "");
  none(p.minVoltage, "MIN");
  none(p.maxCurrent, "MAX");
  p.traceLowMv = BAR_MIN_MV;
  p.traceHighMv = BAR_MAX_MV;
  p.traceHighMa = TRACE_MIN_MA;
  if (!in.history)
    return;
  // The newest values only while the link is up: the history records the
  // loss at its next sample.
  power_history::Sample newest;
  if (in.telemetryFresh && power_history::newest(*in.history, newest))
  {
    milli(p.voltage, Tone::Normal, "", newest.voltageMv, "V");
    milli(p.current, Tone::Normal, "", newest.currentMa, "A");
  }
  uint16_t minMv, maxMv, maxMa;
  if (!power_history::extremes(*in.history, minMv, maxMv, maxMa))
    return;
  milli(p.minVoltage, Tone::Normal, "MIN ", minMv, "V");
  milli(p.maxCurrent, Tone::Normal, "MAX ", maxMa, "A");
  int low = minMv, high = maxMv;
  if (high - low < TRACE_MIN_SPAN_MV)
  {
    // Widen about the middle, kept within the 16-bit range.
    int mid = (low + high) / 2;
    low = mid - TRACE_MIN_SPAN_MV / 2;
    if (low < 0)
      low = 0;
    if (low > UINT16_MAX - TRACE_MIN_SPAN_MV)
      low = UINT16_MAX - TRACE_MIN_SPAN_MV;
    high = low + TRACE_MIN_SPAN_MV;
  }
  p.traceLowMv = (uint16_t)low;
  p.traceHighMv = (uint16_t)high;
  p.traceHighMa = maxMa > TRACE_MIN_MA ? maxMa : TRACE_MIN_MA;
}

static void composeLink(const Inputs &in, LinkBody &l)
{
  const Rates &r = in.rates;
  if (in.telemetryFresh && r.robotValid)
    text(l.robot, r.robotRejected ? Tone::Warn : Tone::Normal, "ROBOT %u/S REJ %u/S", r.robotAccepted, r.robotRejected);
  else
    none(l.robot, "ROBOT");
  if (r.bridgeValid)
    text(l.bridge, r.bridgeRejected ? Tone::Warn : Tone::Normal, "BRIDGE %u/S REJ %u/S", r.bridgeAccepted,
         r.bridgeRejected);
  else
    none(l.bridge, "BRIDGE");

  const protocol::Telemetry &t = in.telemetry;
  if (!in.telemetryFresh)
  {
    none(l.age, "AGE");
    none(l.source, "SOURCE");
  }
  else
  {
    text(l.age, Tone::Normal, "AGE %lu MS", (unsigned long)in.telemetryAgeMs);
    if (t.flags & protocol::TELEMETRY_SOURCE_CONFLICT)
      text(l.source, Tone::Bad, "SOURCE CONFLICT");
    else if (t.flags & protocol::TELEMETRY_USB_IN_CONTROL)
      text(l.source, Tone::Warn, "SOURCE USB");
    else
      text(l.source, Tone::Good, "SOURCE BRIDGE");
  }

  int battery = batteryPercent(in.gamepadBattery);
  if (!in.gamepadConnected)
    text(l.gamepad, Tone::Warn, "PAD NONE");
  else if (battery < 0)
    text(l.gamepad, Tone::Good, "PAD CONNECTED");
  else
    text(l.gamepad, battery < 20 ? Tone::Warn : Tone::Good, "PAD CONNECTED %d%%", battery);
}

void composePage(const Inputs &in, page_button::Page page, View &out)
{
  out = View();
  out.page = page;
  composeHeader(in, out.header);
  composeStrip(in, out.strip);
  switch (page)
  {
  case page_button::Page::Drive:
    composeDrive(in, out.drive);
    break;
  case page_button::Page::Power:
    composePower(in, out.power);
    break;
  case page_button::Page::Link:
    composeLink(in, out.link);
    break;
  }
}

void updateRates(RateMeter &m, uint32_t nowMs, bool telemetryFresh, const protocol::Telemetry &t,
                 uint32_t bridgeAccepted, uint32_t bridgeRejected)
{
  if (m.started)
  {
    uint32_t elapsed = nowMs - m.atMs;
    if (elapsed < RATE_PERIOD_MS)
      return;
    m.rates.bridgeValid = true;
    m.rates.bridgeAccepted = (uint16_t)((bridgeAccepted - m.bridgeAccepted) * 1000u / elapsed);
    m.rates.bridgeRejected = (uint16_t)((bridgeRejected - m.bridgeRejected) * 1000u / elapsed);
    // The robot's counters are 16-bit and wrap.
    m.rates.robotValid = telemetryFresh && m.robotKnown;
    if (m.rates.robotValid)
    {
      m.rates.robotAccepted = (uint16_t)((uint16_t)(t.validFrames - m.robotAccepted) * 1000u / elapsed);
      m.rates.robotRejected = (uint16_t)((uint16_t)(t.discardedFrames - m.robotRejected) * 1000u / elapsed);
    }
  }
  m.started = true;
  m.atMs = nowMs;
  m.bridgeAccepted = bridgeAccepted;
  m.bridgeRejected = bridgeRejected;
  m.robotKnown = telemetryFresh;
  m.robotAccepted = t.validFrames;
  m.robotRejected = t.discardedFrames;
}

// Like set(), but cut to SETUP_LINE_CHARS.
static void setShort(Line &l, Tone tone, const char *text)
{
  snprintf(l.text, sizeof(l.text), "%.*s", SETUP_LINE_CHARS, text ? text : "");
  l.tone = tone;
}

void composeSetup(const SetupInputs &in, Line out[LINE_COUNT])
{
  for (int i = 0; i < LINE_COUNT; i++)
    set(out[i], Tone::Normal, "");

  setShort(out[0], Tone::Warn, "SETUP MODE");
  setShort(out[1], Tone::Normal, in.ssid);
  setShort(out[2], Tone::Normal, in.password);
  setShort(out[3], Tone::Normal, in.address);
  setShort(out[4], in.portalConnected ? Tone::Good : Tone::Warn, in.portalConnected ? "Portal: yes" : "Portal: no");

  if (!in.telemetryFresh)
    setShort(out[5], Tone::Bad, "Link: LOST");
  else if (in.telemetry.flags & protocol::TELEMETRY_SOURCE_CONFLICT)
    setShort(out[5], Tone::Bad, "Link: CONFLICT");
  else if (in.telemetry.flags & protocol::TELEMETRY_USB_IN_CONTROL)
    setShort(out[5], Tone::Warn, "Link: USB");
  else
    setShort(out[5], Tone::Good, "Link: ok");

  setShort(out[6], in.emergencyStop ? Tone::Bad : Tone::Normal, in.emergencyStop ? "E-STOP" : "E-stop: off");

  // The body mode, red while any fault is latched; nothing while the link is
  // down, as on the normal screen.
  if (in.telemetryFresh)
    setShort(out[7], in.telemetry.faults ? Tone::Bad : Tone::Normal, name(BODY_MODES, 6, in.telemetry.bodyMode));
}

} // namespace status_screen
