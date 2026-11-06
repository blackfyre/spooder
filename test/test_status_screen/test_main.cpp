#include <unity.h>
#include <initializer_list>
#include <stdio.h>
#include <string.h>
#include "status/status_screen.h"
#include "protection/protection.h"

using namespace status_screen;
using page_button::Page;

static Line out[LINE_COUNT];
static View view;

static Inputs healthy()
{
  Inputs in = {};
  in.gamepadConnected = true;
  in.desired.height = 40;
  in.desired.speed = 44;
  in.desired.stride = StrideMode::Dynamic;
  in.telemetryFresh = true;
  in.telemetryAgeMs = 40;
  in.telemetry.bodyMode = 2;
  in.telemetry.gait = 1;
  in.telemetry.outputsOn = true;
  in.telemetry.voltageMv = 6020;
  in.telemetry.currentMa = 1234;
  return in;
}

static bool stripHas(const char *word)
{
  for (const auto &l : view.strip.lines)
    if (strstr(l, word))
      return true;
  return false;
}

static void assertStripFits()
{
  for (const auto &l : view.strip.lines)
    TEST_ASSERT_TRUE_MESSAGE(strlen(l) <= (size_t)STRIP_LINE_CHARS, l);
}

void setUp() {}
void tearDown() {}

// --- header and status strip, on every page ---------------------------------------

void test_healthy_header_and_strip()
{
  composePage(healthy(), Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("WALK \xc2\xb7 WAVE", view.header.mode.text);
  TEST_ASSERT_EQUAL_STRING("OUT ON", view.drive.output.text);
  TEST_ASSERT_TRUE(view.drive.output.tone == Tone::Good);
  TEST_ASSERT_EQUAL_STRING("PAD", view.header.gamepad.text);
  TEST_ASSERT_EQUAL_STRING("LINK", view.header.link.text);
  TEST_ASSERT_TRUE(view.header.link.tone == Tone::Good);
  TEST_ASSERT_EQUAL_STRING("NO FAULTS", view.strip.lines[0]);
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Good);
}

void test_playing_gesture_replaces_the_gait()
{
  Inputs in = healthy();
  in.telemetry.bodyMode = 5;
  in.telemetry.gesture = protocol::GESTURE_SAY_HI;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("PERFORM \xc2\xb7 SAY HI", view.header.mode.text);
}

void test_every_mode_and_gait_fits_the_header()
{
  for (int m = 0; m < 6; m++)
    for (int g = 0; g < 6; g++)
    {
      Inputs in = healthy();
      in.telemetry.bodyMode = (uint8_t)m;
      in.telemetry.gait = (uint8_t)g;
      composePage(in, Page::Drive, view);
      TEST_ASSERT_TRUE(strlen(view.header.mode.text) < (size_t)TEXT_CHARS);
      TEST_ASSERT_TRUE(strstr(view.header.mode.text, "?") == nullptr);
    }
  Inputs in = healthy();
  in.telemetry.bodyMode = 200;
  in.telemetry.gait = 99;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("? \xc2\xb7 ?", view.header.mode.text);
}

// The spec's battery scenarios.
void test_gamepad_battery()
{
  Inputs in = healthy();
  in.gamepadBattery = 255;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("PAD 100%", view.header.gamepad.text);
  in.gamepadBattery = 1;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("PAD 0%", view.header.gamepad.text);
  TEST_ASSERT_TRUE(view.header.gamepad.tone == Tone::Warn);
  in.gamepadBattery = 0; // not reported
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("PAD", view.header.gamepad.text);
  TEST_ASSERT_TRUE(view.header.gamepad.tone == Tone::Good);
  in.gamepadConnected = false;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("NO PAD", view.header.gamepad.text);
  TEST_ASSERT_TRUE(view.header.gamepad.tone == Tone::Warn);
}

// The spec's fault scenario: the strip names it on every page.
void test_undervoltage_is_named_on_every_page()
{
  Inputs in = healthy();
  in.telemetry.faults = protection::UNDERVOLTAGE;
  in.telemetry.voltageMv = 4950;
  for (Page p : {Page::Drive, Page::Power, Page::Link})
  {
    composePage(in, p, view);
    TEST_ASSERT_TRUE(stripHas("LOW V"));
    TEST_ASSERT_TRUE(view.strip.tone == Tone::Bad);
  }
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("4.95 V", view.drive.voltage.text);
}

// The spec's e-stop scenario.
void test_emergency_stop_on_another_page()
{
  Inputs in = healthy();
  in.emergencyStop = true;
  composePage(in, Page::Power, view);
  TEST_ASSERT_TRUE(view.page == Page::Power);
  TEST_ASSERT_EQUAL_STRING("E-STOP", view.strip.lines[0]);
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Bad);
}

void test_robot_e_stop_fault_is_named_once()
{
  Inputs in = healthy();
  in.emergencyStop = true;
  in.telemetry.faults = protection::EMERGENCY_STOP;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("E-STOP", view.strip.lines[0]);
  TEST_ASSERT_EQUAL_STRING("", view.strip.lines[1]);
}

void test_every_fault_and_source_combination_is_named_and_fits()
{
  const uint16_t bits[] = {protection::OVERCURRENT, protection::SUSTAINED_CURRENT, protection::UNDERVOLTAGE,
                           protection::BROWNOUT,    protection::OVERVOLTAGE,       protection::EMERGENCY_STOP};
  const char *names[] = {"OVERCURRENT", "OVERLOAD", "LOW V", "BROWN-OUT", "HIGH V", "E-STOP"};
  const uint8_t flags[] = {0, protocol::TELEMETRY_SOURCE_CONFLICT, protocol::TELEMETRY_USB_IN_CONTROL};
  for (uint8_t f : flags)
    for (uint16_t combo = 0; combo < 64; combo++)
    {
      Inputs in = healthy();
      in.telemetry.flags = f;
      for (int i = 0; i < 6; i++)
        if (combo & (1u << i))
          in.telemetry.faults |= bits[i];
      composePage(in, Page::Drive, view);
      assertStripFits();
      for (int i = 0; i < 6; i++)
        if (combo & (1u << i))
          TEST_ASSERT_TRUE_MESSAGE(stripHas(names[i]), names[i]);
      if (f == protocol::TELEMETRY_SOURCE_CONFLICT)
        TEST_ASSERT_TRUE(stripHas("SOURCE CONFLICT"));
      if (f == protocol::TELEMETRY_USB_IN_CONTROL)
        TEST_ASSERT_TRUE(stripHas("USB IN CONTROL"));
      TEST_ASSERT_EQUAL(combo == 0 && f == 0, stripHas("NO FAULTS"));
    }
}

void test_usb_in_control_warns_and_conflict_is_bad()
{
  Inputs in = healthy();
  in.telemetry.flags = protocol::TELEMETRY_USB_IN_CONTROL;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("USB IN CONTROL", view.strip.lines[0]);
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Warn);
  TEST_ASSERT_EQUAL_STRING("USB", view.header.link.text);
  in.telemetry.flags = protocol::TELEMETRY_SOURCE_CONFLICT;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Bad);
  TEST_ASSERT_EQUAL_STRING("CONFLICT", view.header.link.text);
  in.telemetry.flags = protocol::TELEMETRY_BRIDGE_IN_CONTROL;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("NO FAULTS", view.strip.lines[0]);
}

// The spec's link-loss scenario: lost on every page, no robot values.
void test_link_lost_on_every_page_hides_robot_values()
{
  Inputs in = healthy();
  in.telemetryFresh = false;
  in.telemetry.faults = protection::OVERCURRENT;
  for (Page p : {Page::Drive, Page::Power, Page::Link})
  {
    composePage(in, p, view);
    TEST_ASSERT_TRUE(stripHas("LINK LOST"));
    TEST_ASSERT_FALSE(stripHas("OVERCURRENT")); // stale
    TEST_ASSERT_EQUAL_STRING("LOST", view.header.link.text);
    TEST_ASSERT_EQUAL_STRING("--", view.header.mode.text);
  }
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("--", view.drive.voltage.text);
  TEST_ASSERT_FALSE(view.drive.voltageBar.present);
  TEST_ASSERT_FALSE(view.drive.currentBar.present);
  TEST_ASSERT_EQUAL_STRING("OUT --", view.drive.output.text);
  TEST_ASSERT_EQUAL_STRING("SPD 44", view.drive.speed.text); // the bridge's own settings stay
  composePage(in, Page::Link, view);
  TEST_ASSERT_EQUAL_STRING("AGE --", view.link.age.text);
  TEST_ASSERT_EQUAL_STRING("SOURCE --", view.link.source.text);
}

// --- drive page -----------------------------------------------------------------

void test_drive_page_values_and_bars()
{
  Inputs in = healthy();
  in.telemetry.voltageMv = 6000;
  in.telemetry.currentMa = 7500;
  in.desired.stickMode = gamepad_mapping::StickMode::Crab;
  in.desired.stride = StrideMode::Fixed;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("6.00 V", view.drive.voltage.text);
  TEST_ASSERT_EQUAL_STRING("7.50 A", view.drive.current.text);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, view.drive.voltageBar.fraction); // 4.5..7.5 V
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, view.drive.currentBar.fraction); // 0..15 A
  TEST_ASSERT_EQUAL_STRING("H 40", view.drive.height.text);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, view.drive.heightBar.fraction);
  TEST_ASSERT_EQUAL_STRING("SPD 44", view.drive.speed.text);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.44f, view.drive.speedBar.fraction);
  TEST_ASSERT_EQUAL_STRING("FIXED \xc2\xb7 CRAB", view.drive.modes.text);
}

void test_held_extremes_mark_the_bars()
{
  Inputs in = healthy();
  composePage(in, Page::Drive, view);
  TEST_ASSERT_FALSE(view.drive.currentBar.held);
  TEST_ASSERT_FALSE(view.drive.voltageBar.held);
  in.currentPeak.present = true;
  in.currentPeak.value = 4500;
  in.voltageLow.present = true;
  in.voltageLow.value = 5250;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_TRUE(view.drive.currentBar.held);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, view.drive.currentBar.heldFraction); // 4.5 of 15 A
  TEST_ASSERT_TRUE(view.drive.voltageBar.held);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, view.drive.voltageBar.heldFraction); // 5.25 V on 4.5..7.5
  TEST_ASSERT_TRUE(view.drive.current.tone == Tone::Normal);
  in.currentPeak.warning = true; // a new high beyond the held peak
  in.voltageLow.warning = true;  // a deeper sag
  composePage(in, Page::Drive, view);
  TEST_ASSERT_TRUE(view.drive.current.tone == Tone::Warn);
  TEST_ASSERT_TRUE(view.drive.voltage.tone == Tone::Warn);
  in.telemetryFresh = false; // lost: no bars, no marks
  composePage(in, Page::Drive, view);
  TEST_ASSERT_FALSE(view.drive.currentBar.held);
}

void test_bars_clamp_to_their_scale()
{
  Inputs in = healthy();
  in.telemetry.voltageMv = 3000;
  in.telemetry.currentMa = 65535;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_FLOAT(0, view.drive.voltageBar.fraction);
  TEST_ASSERT_EQUAL_FLOAT(1, view.drive.currentBar.fraction);
  TEST_ASSERT_EQUAL_STRING("65.53 A", view.drive.current.text);
}

void test_sensors_shown_only_when_available()
{
  Inputs in = healthy();
  in.telemetry.tofMm = 123;
  in.telemetry.pitchCentideg = -50;
  in.telemetry.rollCentideg = 1234;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("", view.drive.sensors.text);
  in.telemetry.sensorsAvailable = protocol::SENSOR_TOF | protocol::SENSOR_ATTITUDE;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("TOF 123MM P-0.5 R12.3", view.drive.sensors.text);
  in.telemetry.tofMm = 65535;
  in.telemetry.pitchCentideg = -32768;
  in.telemetry.rollCentideg = -32768;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("TOF 65535MM P-327.6 R-327.6", view.drive.sensors.text);
}

// --- power page -----------------------------------------------------------------

// The spec's supply-sag scenario, through the page.
void test_power_page_shows_the_windows_extremes()
{
  power_history::History h;
  uint32_t t = 0;
  for (; t < 10000; t += 20)
    power_history::record(h, t, true, 6050, 400);
  for (uint32_t end = t + 200; t < end; t += 20)
    power_history::record(h, t, true, 5650, 4300);
  for (uint32_t end = t + 20000; t < end; t += 20)
    power_history::record(h, t, true, 6050, 400);
  Inputs in = healthy();
  in.history = &h;
  composePage(in, Page::Power, view);
  TEST_ASSERT_EQUAL_STRING("6.05 V", view.power.voltage.text);
  TEST_ASSERT_EQUAL_STRING("0.40 A", view.power.current.text);
  TEST_ASSERT_EQUAL_STRING("MIN 5.65 V", view.power.minVoltage.text);
  TEST_ASSERT_EQUAL_STRING("MAX 4.30 A", view.power.maxCurrent.text);
  // 5.65-6.05 V is under the minimum span: widened about its middle.
  TEST_ASSERT_EQUAL_UINT16(5600, view.power.traceLowMv);
  TEST_ASSERT_EQUAL_UINT16(6100, view.power.traceHighMv);
  TEST_ASSERT_EQUAL_UINT16(4300, view.power.traceHighMa);
  TEST_ASSERT_TRUE(strstr(view.power.note.text, "10 HZ") != nullptr);
}

void test_power_trace_scales_have_a_minimum()
{
  power_history::History h;
  for (uint32_t t = 0; t < 2000; t += 20)
    power_history::record(h, t, true, 6000, 300);
  Inputs in = healthy();
  in.history = &h;
  composePage(in, Page::Power, view);
  TEST_ASSERT_EQUAL_UINT16(5750, view.power.traceLowMv);
  TEST_ASSERT_EQUAL_UINT16(6250, view.power.traceHighMv);
  TEST_ASSERT_EQUAL_UINT16(TRACE_MIN_MA, view.power.traceHighMa);
}

void test_power_trace_span_stays_within_range()
{
  for (uint16_t mv : {(uint16_t)0, (uint16_t)100, (uint16_t)65535, (uint16_t)65400})
  {
    power_history::History h;
    power_history::record(h, 0, true, mv, 0);
    Inputs in = healthy();
    in.history = &h;
    composePage(in, Page::Power, view);
    TEST_ASSERT_TRUE(view.power.traceLowMv < view.power.traceHighMv);
    TEST_ASSERT_EQUAL_INT(TRACE_MIN_SPAN_MV, view.power.traceHighMv - view.power.traceLowMv);
    TEST_ASSERT_TRUE(view.power.traceLowMv <= mv && mv <= view.power.traceHighMv);
  }
}

// Lost, but the history has not recorded the gap yet: no current values,
// the window's extremes stay.
void test_power_page_hides_newest_values_once_the_link_is_lost()
{
  power_history::History h;
  for (uint32_t t = 0; t < 1000; t += 20)
    power_history::record(h, t, true, 6000, 500);
  Inputs in = healthy();
  in.history = &h;
  in.telemetryFresh = false;
  composePage(in, Page::Power, view);
  TEST_ASSERT_EQUAL_STRING("--", view.power.voltage.text);
  TEST_ASSERT_EQUAL_STRING("--", view.power.current.text);
  TEST_ASSERT_EQUAL_STRING("MIN 6.00 V", view.power.minVoltage.text);
}

void test_power_page_without_history()
{
  Inputs in = healthy();
  composePage(in, Page::Power, view);
  TEST_ASSERT_EQUAL_STRING("--", view.power.voltage.text);
  TEST_ASSERT_EQUAL_STRING("MIN --", view.power.minVoltage.text);
  TEST_ASSERT_EQUAL_STRING("MAX --", view.power.maxCurrent.text);
}

// --- link page and rates --------------------------------------------------------

// The spec's healthy-link scenario: counters fed as the link would advance
// them, one call per 20 ms.
void test_healthy_link_rates()
{
  RateMeter m;
  protocol::Telemetry t = {};
  t.validFrames = 65500; // wraps during the run
  uint32_t bridgeOk = 0;
  for (uint32_t now = 0; now <= 3000; now += 20)
  {
    t.validFrames++; // 50 command frames per second
    // Telemetry, pose and diagnostics at 10 Hz each, on ticks 0, 2 and 4 of
    // every five: 30 frames per second.
    if (now % 100 == 0 || now % 100 == 40 || now % 100 == 80)
      bridgeOk++;
    updateRates(m, now, true, t, bridgeOk, 0);
  }
  TEST_ASSERT_TRUE(m.rates.robotValid);
  TEST_ASSERT_UINT16_WITHIN(1, 50, m.rates.robotAccepted);
  TEST_ASSERT_EQUAL_UINT16(0, m.rates.robotRejected);
  TEST_ASSERT_TRUE(m.rates.bridgeValid);
  TEST_ASSERT_UINT16_WITHIN(1, 30, m.rates.bridgeAccepted);
  TEST_ASSERT_EQUAL_UINT16(0, m.rates.bridgeRejected);

  Inputs in = healthy();
  in.rates = m.rates;
  in.gamepadBattery = 205;
  composePage(in, Page::Link, view);
  char expected[32];
  snprintf(expected, sizeof(expected), "ROBOT %u/S REJ 0/S", m.rates.robotAccepted);
  TEST_ASSERT_EQUAL_STRING(expected, view.link.robot.text);
  TEST_ASSERT_TRUE(view.link.robot.tone == Tone::Normal);
  snprintf(expected, sizeof(expected), "BRIDGE %u/S REJ 0/S", m.rates.bridgeAccepted);
  TEST_ASSERT_EQUAL_STRING(expected, view.link.bridge.text);
  TEST_ASSERT_TRUE(view.link.bridge.tone == Tone::Normal);
  TEST_ASSERT_EQUAL_STRING("AGE 40 MS", view.link.age.text);
  TEST_ASSERT_EQUAL_STRING("SOURCE BRIDGE", view.link.source.text);
  TEST_ASSERT_EQUAL_STRING("PAD CONNECTED 80%", view.link.gamepad.text);
}

void test_rejected_frames_warn()
{
  Inputs in = healthy();
  in.rates = {true, 45, 5, true, 10, 1};
  composePage(in, Page::Link, view);
  TEST_ASSERT_EQUAL_STRING("ROBOT 45/S REJ 5/S", view.link.robot.text);
  TEST_ASSERT_TRUE(view.link.robot.tone == Tone::Warn);
  TEST_ASSERT_EQUAL_STRING("BRIDGE 10/S REJ 1/S", view.link.bridge.text);
  TEST_ASSERT_TRUE(view.link.bridge.tone == Tone::Warn);
}

void test_robot_rates_need_fresh_telemetry_at_both_ends()
{
  RateMeter m;
  protocol::Telemetry t = {};
  updateRates(m, 0, false, t, 0, 0);
  t.validFrames = 50;
  updateRates(m, 1000, true, t, 10, 0);
  TEST_ASSERT_FALSE(m.rates.robotValid);
  TEST_ASSERT_TRUE(m.rates.bridgeValid);
  t.validFrames = 100;
  updateRates(m, 1500, true, t, 10, 0); // not a period yet
  TEST_ASSERT_FALSE(m.rates.robotValid);
  updateRates(m, 2000, true, t, 20, 0);
  TEST_ASSERT_TRUE(m.rates.robotValid);
  TEST_ASSERT_EQUAL_UINT16(50, m.rates.robotAccepted);
}

void test_battery_percent()
{
  TEST_ASSERT_EQUAL_INT(-1, batteryPercent(0));
  TEST_ASSERT_EQUAL_INT(0, batteryPercent(1));
  TEST_ASSERT_EQUAL_INT(100, batteryPercent(255));
}

// --- setup screen --------------------------------------------------------------

// Glyph widths of the screen font (LovyanGFX fonts::Font2, widtbl_f16) for
// characters 32..127.
static const uint8_t FONT2_WIDTHS[96] = {
    6, 3, 4, 9, 8, 9, 9, 3, 7, 7, 8, 6, 3, 6,  5, 7, 8, 8, 8, 8, 8, 8, 8, 8,  8, 8, 3, 3, 6, 6, 6, 8,
    9, 8, 8, 8, 8, 8, 8, 8, 8, 4, 8, 8, 7, 10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 10, 8, 8, 8, 4, 7, 4, 7, 9,
    4, 7, 7, 7, 7, 7, 6, 7, 7, 4, 5, 6, 4, 8,  7, 8, 7, 8, 6, 6, 5, 7, 8, 8,  6, 7, 7, 5, 3, 5, 8, 6,
};

static int textWidthPx(const char *text)
{
  int w = 0;
  for (const char *c = text; *c; c++)
  {
    const unsigned char ch = static_cast<unsigned char>(*c);
    w += (ch >= 32 && ch < 128) ? FONT2_WIDTHS[ch - 32] : 10;
  }
  return w;
}

static SetupInputs setupHealthy()
{
  SetupInputs in = {};
  in.ssid = "spooder-ab0f";
  in.password = "abcdefgh23";
  in.address = "192.168.4.1";
  in.portalConnected = true;
  in.telemetryFresh = true;
  in.telemetry.bodyMode = 4;
  in.telemetry.outputsOn = true;
  in.telemetry.voltageMv = 6020;
  return in;
}

static void assertSetupLinesFit()
{
  for (int l = 0; l < LINE_COUNT; l++)
  {
    TEST_ASSERT_TRUE_MESSAGE(strlen(out[l].text) <= (size_t)SETUP_LINE_CHARS, out[l].text);
    TEST_ASSERT_TRUE_MESSAGE(textWidthPx(out[l].text) <= SETUP_TEXT_WIDTH_PX, out[l].text);
  }
}

void test_setup_screen_shows_every_item()
{
  composeSetup(setupHealthy(), out);
  TEST_ASSERT_EQUAL_STRING("SETUP MODE", out[0].text);
  TEST_ASSERT_EQUAL_STRING("spooder-ab0f", out[1].text);
  TEST_ASSERT_EQUAL_STRING("abcdefgh23", out[2].text);
  TEST_ASSERT_EQUAL_STRING("192.168.4.1", out[3].text);
  TEST_ASSERT_EQUAL_STRING("Portal: yes", out[4].text);
  TEST_ASSERT_TRUE(out[4].tone == Tone::Good);
  TEST_ASSERT_EQUAL_STRING("Link: ok", out[5].text);
  TEST_ASSERT_TRUE(out[5].tone == Tone::Good);
  TEST_ASSERT_EQUAL_STRING("E-stop: off", out[6].text);
  TEST_ASSERT_EQUAL_STRING("Calibrate", out[7].text);
  assertSetupLinesFit();
}

void test_setup_screen_states()
{
  SetupInputs in = setupHealthy();
  in.portalConnected = false;
  in.emergencyStop = true;
  composeSetup(in, out);
  TEST_ASSERT_EQUAL_STRING("Portal: no", out[4].text);
  TEST_ASSERT_TRUE(out[4].tone == Tone::Warn);
  TEST_ASSERT_EQUAL_STRING("E-STOP", out[6].text);
  TEST_ASSERT_TRUE(out[6].tone == Tone::Bad);

  in.telemetry.flags = protocol::TELEMETRY_SOURCE_CONFLICT;
  composeSetup(in, out);
  TEST_ASSERT_EQUAL_STRING("Link: CONFLICT", out[5].text);
  TEST_ASSERT_TRUE(out[5].tone == Tone::Bad);
  in.telemetry.flags = protocol::TELEMETRY_USB_IN_CONTROL;
  composeSetup(in, out);
  TEST_ASSERT_EQUAL_STRING("Link: USB", out[5].text);
  TEST_ASSERT_TRUE(out[5].tone == Tone::Warn);

  in.telemetry.faults = protection::OVERCURRENT;
  composeSetup(in, out);
  TEST_ASSERT_TRUE(out[7].tone == Tone::Bad);

  in.telemetryFresh = false;
  composeSetup(in, out);
  TEST_ASSERT_EQUAL_STRING("Link: LOST", out[5].text);
  TEST_ASSERT_TRUE(out[5].tone == Tone::Bad);
  TEST_ASSERT_EQUAL_STRING("", out[7].text); // no robot values while lost
  // The credentials stay on screen whatever the link does.
  TEST_ASSERT_EQUAL_STRING("spooder-ab0f", out[1].text);
  TEST_ASSERT_EQUAL_STRING("abcdefgh23", out[2].text);
  assertSetupLinesFit();
}

void test_every_setup_line_fits_at_the_widest()
{
  // The widest password and network name the bridge can produce, every link
  // state and every body mode.
  SetupInputs in = setupHealthy();
  in.ssid = "spooder-ffff";
  in.password = "wwwwwwwwww";
  const uint8_t flags[] = {0, protocol::TELEMETRY_SOURCE_CONFLICT, protocol::TELEMETRY_USB_IN_CONTROL};
  for (int f = 0; f < 3; f++)
    for (int m = 0; m < 6; m++)
      for (int e = 0; e < 2; e++)
      {
        in.telemetry.flags = flags[f];
        in.telemetry.bodyMode = (uint8_t)(m == 5 ? 200 : m);
        in.emergencyStop = e != 0;
        in.portalConnected = e == 0;
        composeSetup(in, out);
        assertSetupLinesFit();
      }
}

void test_setup_screen_cuts_overlong_text()
{
  SetupInputs in = setupHealthy();
  in.ssid = "a-network-name-far-too-long";
  in.password = nullptr;
  composeSetup(in, out);
  TEST_ASSERT_EQUAL_INT(SETUP_LINE_CHARS, (int)strlen(out[1].text));
  TEST_ASSERT_EQUAL_STRING("", out[2].text);
}

// --- WiFi drive and the drive mode switch notice --------------------------------

using handheld_controller::State;

static Inputs wifi(State handheld)
{
  Inputs in = healthy();
  in.gamepadConnected = false;
  in.driveMode = drive_mode::Mode::Wifi;
  in.handheld = handheld;
  in.network = "spooder-drive-ab0f";
  in.password = "abcdefgh23";
  in.desired.stride = StrideMode::Fixed;
  in.desired.stickMode = gamepad_mapping::StickMode::Crab; // never shown in WiFi drive
  return in;
}

// The spec's handheld scenarios, and the three states.
void test_wifi_drive_header_names_the_handheld_state()
{
  const State states[] = {State::None, State::Idle, State::InControl};
  const char *header[] = {"WF NONE", "WF IDLE", "WF CTRL"};
  const char *link[] = {"HANDHELD NONE", "HANDHELD IDLE", "HANDHELD IN CONTROL"};
  const Tone tones[] = {Tone::Warn, Tone::Warn, Tone::Good};
  for (int i = 0; i < 3; i++)
  {
    Inputs in = wifi(states[i]);
    in.gamepadBattery = 255; // a stale gamepad reading is never shown
    composePage(in, Page::Drive, view);
    TEST_ASSERT_EQUAL_STRING(header[i], view.header.gamepad.text);
    TEST_ASSERT_TRUE(view.header.gamepad.tone == tones[i]);
    TEST_ASSERT_LESS_OR_EQUAL_INT(HEADER_CONTROLLER_CHARS, (int)strlen(view.header.gamepad.text));
    TEST_ASSERT_NULL(strchr(view.header.gamepad.text, '%'));
    composePage(in, Page::Link, view);
    TEST_ASSERT_EQUAL_STRING(link[i], view.link.handheld.text);
    TEST_ASSERT_TRUE(view.link.handheld.tone == tones[i]);
  }
}

void test_bluetooth_header_fits_the_controller_field()
{
  Inputs in = healthy();
  in.gamepadBattery = 255;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_INT(HEADER_CONTROLLER_CHARS, (int)strlen(view.header.gamepad.text)); // "PAD 100%"
}

void test_wifi_drive_shows_no_stick_mode()
{
  composePage(wifi(State::InControl), Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("FIXED", view.drive.modes.text);
  Inputs in = wifi(State::InControl);
  in.desired.stride = StrideMode::Dynamic;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("DYNAMIC", view.drive.modes.text);
}

// The spec's joining scenario, with the link up and lost.
void test_wifi_drive_link_page_shows_the_network_to_join()
{
  for (int fresh = 0; fresh < 2; fresh++)
  {
    Inputs in = wifi(State::None);
    in.telemetryFresh = fresh != 0;
    composePage(in, Page::Link, view);
    TEST_ASSERT_EQUAL_STRING("NET spooder-drive-ab0f", view.link.network.text);
    TEST_ASSERT_EQUAL_STRING("PASS abcdefgh23", view.link.password.text);
    TEST_ASSERT_EQUAL_STRING("HANDHELD NONE", view.link.handheld.text);
    TEST_ASSERT_EQUAL_STRING("", view.link.gamepad.text); // in place of the gamepad line
  }
  Inputs in = wifi(State::None);
  in.network = nullptr;
  in.password = nullptr;
  composePage(in, Page::Link, view);
  TEST_ASSERT_EQUAL_STRING("NET --", view.link.network.text);
  TEST_ASSERT_EQUAL_STRING("PASS --", view.link.password.text);
}

void test_every_wifi_drive_line_fits_its_field()
{
  // The widest network name and password the bridge makes, and every state.
  const State states[] = {State::None, State::Idle, State::InControl};
  for (State st : states)
  {
    Inputs in = wifi(st);
    in.network = "spooder-drive-ffff";
    in.password = "wwwwwwwwww";
    in.switchNotice = true;
    for (Page p : {Page::Drive, Page::Power, Page::Link})
    {
      composePage(in, p, view);
      TEST_ASSERT_LESS_OR_EQUAL_INT(HEADER_CONTROLLER_CHARS, (int)strlen(view.header.gamepad.text));
      assertStripFits();
    }
    composePage(in, Page::Link, view);
    // Whole, not cut by the field's buffer.
    TEST_ASSERT_EQUAL_STRING("NET spooder-drive-ffff", view.link.network.text);
    TEST_ASSERT_EQUAL_STRING("PASS wwwwwwwwww", view.link.password.text);
    for (const Text *t : {&view.link.network, &view.link.password, &view.link.handheld})
      TEST_ASSERT_LESS_THAN_INT(TEXT_CHARS, (int)strlen(t->text));
  }
}

void test_bluetooth_drive_pages_are_unchanged()
{
  // Zero drive mode fields: Bluetooth drive, as before WiFi drive existed.
  Inputs in = healthy();
  in.desired.stickMode = gamepad_mapping::StickMode::Crab;
  in.handheld = State::InControl; // ignored outside WiFi drive
  in.network = "spooder-drive-ab0f";
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("PAD", view.header.gamepad.text);
  TEST_ASSERT_EQUAL_STRING("DYNAMIC \xc2\xb7 CRAB", view.drive.modes.text);
  TEST_ASSERT_EQUAL_STRING("NO FAULTS", view.strip.lines[0]);
  composePage(in, Page::Link, view);
  TEST_ASSERT_EQUAL_STRING("PAD CONNECTED", view.link.gamepad.text);
  TEST_ASSERT_EQUAL_STRING("", view.link.network.text);
  TEST_ASSERT_EQUAL_STRING("", view.link.password.text);
  TEST_ASSERT_EQUAL_STRING("", view.link.handheld.text);
}

// The spec's switch notice scenarios.
void test_switch_notice_alone_warns()
{
  for (drive_mode::Mode m : {drive_mode::Mode::Bluetooth, drive_mode::Mode::Wifi})
  {
    Inputs in = m == drive_mode::Mode::Wifi ? wifi(State::InControl) : healthy();
    in.switchNotice = true;
    for (Page p : {Page::Drive, Page::Power, Page::Link})
    {
      composePage(in, p, view);
      TEST_ASSERT_EQUAL_STRING(SWITCH_NOTICE, view.strip.lines[0]);
      TEST_ASSERT_FALSE(stripHas("NO FAULTS"));
      TEST_ASSERT_TRUE(view.strip.tone == Tone::Warn);
    }
    in.switchNotice = false; // moved back
    composePage(in, Page::Drive, view);
    TEST_ASSERT_FALSE(stripHas(SWITCH_NOTICE));
    TEST_ASSERT_EQUAL_STRING("NO FAULTS", view.strip.lines[0]);
  }
}

void test_switch_notice_beside_a_fault_in_the_faults_tone()
{
  Inputs in = healthy();
  in.switchNotice = true;
  in.telemetry.faults = protection::UNDERVOLTAGE;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_TRUE(stripHas("LOW V"));
  TEST_ASSERT_TRUE(stripHas(SWITCH_NOTICE));
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Bad);
  // After the faults and the source state.
  TEST_ASSERT_TRUE(strstr(view.strip.lines[0], "LOW V") == view.strip.lines[0]);
  in.telemetry.faults = 0;
  in.telemetry.flags = protocol::TELEMETRY_USB_IN_CONTROL;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("USB IN CONTROL  SWITCH MOVED: RESTART", view.strip.lines[0]);
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Warn);
  in.telemetryFresh = false;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_TRUE(stripHas("LINK LOST"));
  TEST_ASSERT_TRUE(stripHas(SWITCH_NOTICE));
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Bad);
}

void test_switch_notice_never_cuts_a_fault()
{
  const uint16_t bits[] = {protection::OVERCURRENT, protection::SUSTAINED_CURRENT, protection::UNDERVOLTAGE,
                           protection::BROWNOUT,    protection::OVERVOLTAGE,       protection::EMERGENCY_STOP};
  const char *names[] = {"OVERCURRENT", "OVERLOAD", "LOW V", "BROWN-OUT", "HIGH V", "E-STOP"};
  const uint8_t flags[] = {0, protocol::TELEMETRY_SOURCE_CONFLICT, protocol::TELEMETRY_USB_IN_CONTROL};
  bool leftOut = false;
  for (uint8_t f : flags)
    for (uint16_t combo = 0; combo < 64; combo++)
    {
      Inputs in = healthy();
      in.switchNotice = true;
      in.telemetry.flags = f;
      for (int i = 0; i < 6; i++)
        if (combo & (1u << i))
          in.telemetry.faults |= bits[i];
      composePage(in, Page::Drive, view);
      assertStripFits();
      for (int i = 0; i < 6; i++)
        if (combo & (1u << i))
          TEST_ASSERT_TRUE_MESSAGE(stripHas(names[i]), names[i]);
      if (f == protocol::TELEMETRY_SOURCE_CONFLICT)
        TEST_ASSERT_TRUE(stripHas("SOURCE CONFLICT"));
      if (f == protocol::TELEMETRY_USB_IN_CONTROL)
        TEST_ASSERT_TRUE(stripHas("USB IN CONTROL"));
      // The notice is whole or absent, never cut.
      bool whole = stripHas(SWITCH_NOTICE);
      bool part = stripHas("SWITCH");
      TEST_ASSERT_EQUAL(whole, part);
      leftOut |= !whole;
      TEST_ASSERT_FALSE(stripHas("NO FAULTS"));
    }
  TEST_ASSERT_TRUE(leftOut); // the full strip happens
  // The spec's full strip: every named fault and the source conflict.
  Inputs in = healthy();
  in.switchNotice = true;
  in.telemetry.flags = protocol::TELEMETRY_SOURCE_CONFLICT;
  for (uint16_t b : bits)
    in.telemetry.faults |= b;
  composePage(in, Page::Drive, view);
  for (const char *n : names)
    TEST_ASSERT_TRUE_MESSAGE(stripHas(n), n);
  TEST_ASSERT_TRUE(stripHas("SOURCE CONFLICT"));
  TEST_ASSERT_FALSE(stripHas("SWITCH"));
}

// --- bench simulation ------------------------------------------------------------

static Inputs simulating()
{
  Inputs in = healthy();
  in.telemetry.flags = protocol::TELEMETRY_SIMULATED;
  return in;
}

// True if @p word stands whole in the strip: strip words are two spaces apart
// and may hold single spaces, so "SIM" is not found in "SIM SUPPLY".
static bool stripHasWord(const char *word)
{
  size_t n = strlen(word);
  for (const auto &l : view.strip.lines)
    for (const char *at = l; *at;)
    {
      const char *end = strstr(at, "  ");
      size_t len = end ? (size_t)(end - at) : strlen(at);
      if (len == n && strncmp(at, word, n) == 0)
        return true;
      if (!end)
        break;
      at = end + 2;
    }
  return false;
}

// The delta's simulation scenario: SIM on every page, in the warning tone;
// the drive page's output as simulated.
void test_simulation_is_named_on_every_page()
{
  Inputs in = simulating();
  for (Page p : {Page::Drive, Page::Power, Page::Link})
  {
    composePage(in, p, view);
    TEST_ASSERT_EQUAL_STRING("SIM", view.strip.lines[0]);
    TEST_ASSERT_EQUAL_STRING("", view.strip.lines[1]);
    TEST_ASSERT_TRUE(view.strip.tone == Tone::Warn);
    TEST_ASSERT_FALSE(stripHas("NO FAULTS"));
  }
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("OUT SIM", view.drive.output.text);
  TEST_ASSERT_TRUE(view.drive.output.tone == Tone::Warn);
  in.telemetry.outputsOn = false;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("OUT OFF", view.drive.output.text);
  TEST_ASSERT_TRUE(view.drive.output.tone == Tone::Muted);
}

void test_simulation_needs_fresh_telemetry()
{
  Inputs in = simulating();
  in.telemetryFresh = false;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_FALSE(stripHasWord("SIM"));
  TEST_ASSERT_EQUAL_STRING("OUT --", view.drive.output.text);
}

// The delta's supply scenario: the fault named in the fault tone, beside SIM.
void test_simulation_supply_fault_is_named_beside_sim()
{
  Inputs in = simulating();
  in.telemetry.faults = protection::SIMULATION_SUPPLY;
  composePage(in, Page::Drive, view);
  TEST_ASSERT_EQUAL_STRING("SIM SUPPLY  SIM", view.strip.lines[0]);
  TEST_ASSERT_TRUE(view.strip.tone == Tone::Bad);
  TEST_ASSERT_EQUAL_STRING("sim supply", faultName(protection::SIMULATION_SUPPLY));
  TEST_ASSERT_NULL(faultName(1u << 7));
}

// The strip's names of fault bits 0 to 6.
static const char *const FAULT_STRIP_NAMES[] = {"OVERCURRENT", "OVERLOAD", "LOW V",     "BROWN-OUT",
                                                "HIGH V",      "E-STOP",   "SIM SUPPLY"};

// SIM beside each named fault, and beside the source states, the reachable
// combination in simulation (an e-stop and the supply fault with a source
// conflict) included: every word whole and every line within its field.
void test_sim_beside_each_fault_and_source_state_fits()
{
  const uint16_t bits[] = {protection::OVERCURRENT,       protection::SUSTAINED_CURRENT,
                           protection::UNDERVOLTAGE,      protection::BROWNOUT,
                           protection::OVERVOLTAGE,       protection::EMERGENCY_STOP,
                           protection::SIMULATION_SUPPLY, protection::EMERGENCY_STOP | protection::SIMULATION_SUPPLY};
  const uint8_t sources[] = {0, protocol::TELEMETRY_SOURCE_CONFLICT, protocol::TELEMETRY_USB_IN_CONTROL};
  for (uint8_t source : sources)
    for (uint16_t faults : bits)
      for (Page p : {Page::Drive, Page::Power, Page::Link})
      {
        Inputs in = simulating();
        in.telemetry.flags |= source;
        in.telemetry.faults = faults;
        composePage(in, p, view);
        assertStripFits();
        TEST_ASSERT_TRUE(stripHasWord("SIM"));
        for (uint16_t bit = 1; bit < (1u << 7); bit <<= 1)
          if (faults & bit)
          {
            TEST_ASSERT_NOT_NULL(faultName(bit));
            const char *n = FAULT_STRIP_NAMES[__builtin_ctz(bit)];
            TEST_ASSERT_TRUE_MESSAGE(stripHasWord(n), n);
          }
        if (source == protocol::TELEMETRY_SOURCE_CONFLICT)
          TEST_ASSERT_TRUE(stripHasWord("SOURCE CONFLICT"));
        if (source == protocol::TELEMETRY_USB_IN_CONTROL)
          TEST_ASSERT_TRUE(stripHasWord("USB IN CONTROL"));
        TEST_ASSERT_TRUE(view.strip.tone ==
                         (faults || source == protocol::TELEMETRY_SOURCE_CONFLICT ? Tone::Bad : Tone::Warn));
        if (p == Page::Drive)
          TEST_ASSERT_TRUE(view.drive.output.text[0] != '\0' && strlen(view.drive.output.text) <= (size_t)TEXT_CHARS);
      }
}

// Without the flag every page is as before: composing with the flag cleared
// gives the same view as the other bits alone, and differs from simulation
// only in the strip and the drive page's output.
void test_pages_without_the_simulation_flag_are_unchanged()
{
  for (Page p : {Page::Drive, Page::Power, Page::Link})
  {
    Inputs in = healthy();
    in.telemetry.flags = protocol::TELEMETRY_BRIDGE_IN_CONTROL | protocol::TELEMETRY_BRIDGE_LINK_UP;
    composePage(in, p, view);
    View plain = view;
    TEST_ASSERT_EQUAL_STRING("NO FAULTS", plain.strip.lines[0]);
    TEST_ASSERT_FALSE(stripHasWord("SIM"));
    if (p == Page::Drive)
      TEST_ASSERT_EQUAL_STRING("OUT ON", plain.drive.output.text);

    in.telemetry.flags |= protocol::TELEMETRY_SIMULATED;
    composePage(in, p, view);
    TEST_ASSERT_EQUAL_MEMORY(&plain.header, &view.header, sizeof(Header));
    TEST_ASSERT_EQUAL_MEMORY(&plain.power, &view.power, sizeof(PowerBody));
    TEST_ASSERT_EQUAL_MEMORY(&plain.link, &view.link, sizeof(LinkBody));
    // Field by field: Bar has padding.
    for (const Text DriveBody::*t : {&DriveBody::voltage, &DriveBody::current, &DriveBody::height, &DriveBody::speed,
                                     &DriveBody::modes, &DriveBody::sensors})
    {
      TEST_ASSERT_EQUAL_STRING((plain.drive.*t).text, (view.drive.*t).text);
      TEST_ASSERT_TRUE((plain.drive.*t).tone == (view.drive.*t).tone);
    }
    for (const Bar DriveBody::*b :
         {&DriveBody::voltageBar, &DriveBody::currentBar, &DriveBody::heightBar, &DriveBody::speedBar})
    {
      TEST_ASSERT_EQUAL((plain.drive.*b).present, (view.drive.*b).present);
      TEST_ASSERT_EQUAL_FLOAT((plain.drive.*b).fraction, (view.drive.*b).fraction);
      TEST_ASSERT_EQUAL((plain.drive.*b).held, (view.drive.*b).held);
      TEST_ASSERT_EQUAL_FLOAT((plain.drive.*b).heldFraction, (view.drive.*b).heldFraction);
    }
  }
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_healthy_header_and_strip);
  RUN_TEST(test_playing_gesture_replaces_the_gait);
  RUN_TEST(test_every_mode_and_gait_fits_the_header);
  RUN_TEST(test_gamepad_battery);
  RUN_TEST(test_undervoltage_is_named_on_every_page);
  RUN_TEST(test_emergency_stop_on_another_page);
  RUN_TEST(test_robot_e_stop_fault_is_named_once);
  RUN_TEST(test_every_fault_and_source_combination_is_named_and_fits);
  RUN_TEST(test_usb_in_control_warns_and_conflict_is_bad);
  RUN_TEST(test_link_lost_on_every_page_hides_robot_values);
  RUN_TEST(test_drive_page_values_and_bars);
  RUN_TEST(test_held_extremes_mark_the_bars);
  RUN_TEST(test_bars_clamp_to_their_scale);
  RUN_TEST(test_sensors_shown_only_when_available);
  RUN_TEST(test_power_page_shows_the_windows_extremes);
  RUN_TEST(test_power_trace_scales_have_a_minimum);
  RUN_TEST(test_power_trace_span_stays_within_range);
  RUN_TEST(test_power_page_hides_newest_values_once_the_link_is_lost);
  RUN_TEST(test_power_page_without_history);
  RUN_TEST(test_healthy_link_rates);
  RUN_TEST(test_rejected_frames_warn);
  RUN_TEST(test_robot_rates_need_fresh_telemetry_at_both_ends);
  RUN_TEST(test_battery_percent);
  RUN_TEST(test_setup_screen_shows_every_item);
  RUN_TEST(test_setup_screen_states);
  RUN_TEST(test_every_setup_line_fits_at_the_widest);
  RUN_TEST(test_setup_screen_cuts_overlong_text);
  RUN_TEST(test_wifi_drive_header_names_the_handheld_state);
  RUN_TEST(test_bluetooth_header_fits_the_controller_field);
  RUN_TEST(test_wifi_drive_shows_no_stick_mode);
  RUN_TEST(test_wifi_drive_link_page_shows_the_network_to_join);
  RUN_TEST(test_every_wifi_drive_line_fits_its_field);
  RUN_TEST(test_bluetooth_drive_pages_are_unchanged);
  RUN_TEST(test_switch_notice_alone_warns);
  RUN_TEST(test_switch_notice_beside_a_fault_in_the_faults_tone);
  RUN_TEST(test_switch_notice_never_cuts_a_fault);
  RUN_TEST(test_simulation_is_named_on_every_page);
  RUN_TEST(test_simulation_needs_fresh_telemetry);
  RUN_TEST(test_simulation_supply_fault_is_named_beside_sim);
  RUN_TEST(test_sim_beside_each_fault_and_source_state_fits);
  RUN_TEST(test_pages_without_the_simulation_flag_are_unchanged);
  return UNITY_END();
}
