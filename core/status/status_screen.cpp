#include "status_screen.h"

#include <stdio.h>
#include "protection/protection.h"

namespace status_screen
{

static const char *const BODY_MODES[] = {"Initialise", "Stand", "Walk", "Sleep", "Calibrate", "Perform"};
static const char *const GESTURES[] = {"none", "say hi", "wiggle", "dance", "attack", "ball"};
static const char *const GAITS[] = {"Tri", "Wave", "Ripple", "Bi", "Quad", "Hop"};

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

// Writes the latched faults' names from line `first`, wrapping at
// LINE_CHARS. The short names keep all six within two lines, so they never
// reach the sensor line.
static void writeFaults(uint16_t faults, Line out[LINE_COUNT], int first)
{
  int line = first;
  int used = 0;
  set(out[line], Tone::Bad, "");
  for (uint16_t bit = 1; bit != 0; bit <<= 1)
  {
    if (!(faults & bit))
      continue;
    char word[16];
    const char *n = faultName(bit);
    int len = n ? snprintf(word, sizeof(word), "%s", n) : snprintf(word, sizeof(word), "0x%04x", bit);
    int need = used ? len + 1 : len;
    if (used + need > LINE_CHARS)
    {
      if (line + 1 >= first + 2)
        return;
      set(out[++line], Tone::Bad, "");
      used = 0;
    }
    used += snprintf(out[line].text + used, sizeof(out[line].text) - used, "%s%s", used ? " " : "", word);
  }
}

// Centidegrees as degrees with one decimal, keeping the sign of -0.x.
static int formatTenths(char *buf, int size, int16_t centideg)
{
  int v = centideg < 0 ? -(int)centideg : centideg;
  return snprintf(buf, size, "%s%d.%d", centideg < 0 ? "-" : "", v / 100, v % 100 / 10);
}

void compose(const Inputs &in, Line out[LINE_COUNT])
{
  for (int i = 0; i < LINE_COUNT; i++)
    set(out[i], Tone::Normal, "");

  char buf[64];
  const char *pad = in.gamepadConnected ? "Pad: connected" : "Pad: none";
  if (in.emergencyStop)
  {
    snprintf(buf, sizeof(buf), "E-STOP  %s", pad);
    set(out[0], Tone::Bad, buf);
  }
  else
  {
    set(out[0], in.gamepadConnected ? Tone::Good : Tone::Warn, pad);
  }

  const gamepad_mapping::Desired &d = in.desired;
  // Longest: "H 100 Spd 100 dynamic steer", 27 of LINE_CHARS.
  snprintf(buf, sizeof(buf), "H %u Spd %u %s %s", d.height, d.speed,
           d.stride == StrideMode::Fixed ? "fixed" : "dynamic", d.stickMode == gamepad_mapping::StickMode::Crab ? "crab" : "steer");
  set(out[4], Tone::Normal, buf);

  if (!in.telemetryFresh)
  {
    // Robot values are not current: show none of them.
    set(out[1], Tone::Bad, "Link: LOST");
    set(out[2], Tone::Normal, "Body --");
    set(out[3], Tone::Normal, "Out --  -- V  -- A");
    set(out[5], Tone::Normal, "Faults: --");
    return;
  }

  const protocol::Telemetry &t = in.telemetry;
  // Line 1: the link, and which operator source the robot follows.
  if (t.flags & protocol::TELEMETRY_SOURCE_CONFLICT)
    set(out[1], Tone::Bad, "Link: ok  CONFLICT with USB");
  else if (t.flags & protocol::TELEMETRY_USB_IN_CONTROL)
    set(out[1], Tone::Warn, "Link: ok  USB in control");
  else
    set(out[1], Tone::Good, "Link: ok");
  // Longest: "Body Initialise  Gait Ripple", exactly LINE_CHARS. While a
  // gesture plays, it replaces the gait: "Body Perform: say hi".
  if (t.gesture != 0)
    snprintf(buf, sizeof(buf), "Body Perform: %s", name(GESTURES, 6, t.gesture));
  else
    snprintf(buf, sizeof(buf), "Body %s  Gait %s", name(BODY_MODES, 6, t.bodyMode), name(GAITS, 6, t.gait));
  set(out[2], Tone::Normal, buf);
  snprintf(buf, sizeof(buf), "Out %s  %u.%02u V  %u.%02u A", t.outputsOn ? "on" : "off", t.voltageMv / 1000,
           (t.voltageMv % 1000) / 10, t.currentMa / 1000, (t.currentMa % 1000) / 10);
  set(out[3], t.faults ? Tone::Bad : (t.outputsOn ? Tone::Good : Tone::Normal), buf);

  // Lines 5-6: fault names (all six fit in two lines); line 7: sensors.
  if (t.faults == 0)
    set(out[5], Tone::Good, "Faults: none");
  else
    writeFaults(t.faults, out, 5);

  if (t.sensorsAvailable & (protocol::SENSOR_TOF | protocol::SENSOR_ATTITUDE))
  {
    // Compact, so the widest values still fit: "ToF 65535mm P-327.6 R-327.6".
    int used = 0;
    if (t.sensorsAvailable & protocol::SENSOR_TOF)
      used += snprintf(buf + used, sizeof(buf) - used, "ToF %umm ", t.tofMm);
    if (t.sensorsAvailable & protocol::SENSOR_ATTITUDE)
    {
      used += snprintf(buf + used, sizeof(buf) - used, "P");
      used += formatTenths(buf + used, sizeof(buf) - used, t.pitchCentideg);
      used += snprintf(buf + used, sizeof(buf) - used, " R");
      formatTenths(buf + used, sizeof(buf) - used, t.rollCentideg);
    }
    set(out[7], Tone::Normal, buf);
  }
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
