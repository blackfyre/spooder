#include <unity.h>
#include <stdio.h>
#include <string.h>
#include "status/status_screen.h"
#include "protection/protection.h"

using namespace status_screen;

static Line out[LINE_COUNT];

static Inputs healthy()
{
  Inputs in = {};
  in.gamepadConnected = true;
  in.desired.height = 40;
  in.desired.speed = 44;
  in.desired.stride = StrideMode::Dynamic;
  in.telemetryFresh = true;
  in.telemetry.bodyMode = 2;
  in.telemetry.gait = 1;
  in.telemetry.outputsOn = true;
  in.telemetry.voltageMv = 6020;
  in.telemetry.currentMa = 1234;
  return in;
}

static bool screenContains(const char *text)
{
  for (int i = 0; i < LINE_COUNT; i++)
    if (strstr(out[i].text, text))
      return true;
  return false;
}

void setUp() {}
void tearDown() {}

void test_healthy_screen_shows_every_item()
{
  compose(healthy(), out);
  TEST_ASSERT_TRUE(screenContains("Pad: connected"));
  TEST_ASSERT_TRUE(screenContains("Link: ok"));
  TEST_ASSERT_TRUE(screenContains("Walk"));
  TEST_ASSERT_TRUE(screenContains("Wave"));
  TEST_ASSERT_TRUE(screenContains("Out on"));
  TEST_ASSERT_TRUE(screenContains("6.02 V"));
  TEST_ASSERT_TRUE(screenContains("1.23 A"));
  TEST_ASSERT_TRUE(screenContains("H 40"));
  TEST_ASSERT_TRUE(screenContains("Speed 44"));
  TEST_ASSERT_TRUE(screenContains("dynamic"));
  TEST_ASSERT_TRUE(screenContains("Faults: none"));
  TEST_ASSERT_FALSE(screenContains("E-STOP"));
}

void test_no_gamepad_is_shown()
{
  Inputs in = healthy();
  in.gamepadConnected = false;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("Pad: none"));
}

void test_emergency_stop_is_shown()
{
  Inputs in = healthy();
  in.emergencyStop = true;
  compose(in, out);
  TEST_ASSERT_TRUE(strstr(out[0].text, "E-STOP") != nullptr);
  TEST_ASSERT_TRUE(out[0].tone == Tone::Bad);
}

void test_undervoltage_fault_shows_name_and_voltage()
{
  Inputs in = healthy();
  in.telemetry.faults = protection::UNDERVOLTAGE;
  in.telemetry.voltageMv = 4950;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("low V"));
  TEST_ASSERT_TRUE(screenContains("4.95 V"));
}

void test_every_fault_combination_shows_every_name()
{
  const uint16_t bits[] = {protection::OVERCURRENT, protection::SUSTAINED_CURRENT, protection::UNDERVOLTAGE,
                           protection::BROWNOUT, protection::OVERVOLTAGE, protection::EMERGENCY_STOP};
  for (uint16_t combo = 1; combo < 64; combo++)
  {
    Inputs in = healthy();
    for (int i = 0; i < 6; i++)
      if (combo & (1u << i))
        in.telemetry.faults |= bits[i];
    compose(in, out);
    for (int i = 0; i < 6; i++)
      if (combo & (1u << i))
        TEST_ASSERT_TRUE_MESSAGE(screenContains(faultName(bits[i])), "fault name missing");
    for (int l = 0; l < LINE_COUNT; l++)
      TEST_ASSERT_TRUE(strlen(out[l].text) <= (size_t)LINE_CHARS);
  }
}

void test_source_conflict_is_shown()
{
  Inputs in = healthy();
  in.telemetry.flags = protocol::TELEMETRY_SOURCE_CONFLICT;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("CONFLICT"));
  TEST_ASSERT_TRUE(out[1].tone == Tone::Bad);
  TEST_ASSERT_TRUE(strlen(out[1].text) <= (size_t)LINE_CHARS);
}

void test_usb_in_control_is_shown()
{
  Inputs in = healthy();
  in.telemetry.flags = protocol::TELEMETRY_USB_IN_CONTROL;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("USB in control"));
  TEST_ASSERT_TRUE(out[1].tone == Tone::Warn);
  in.telemetry.flags = protocol::TELEMETRY_BRIDGE_IN_CONTROL;
  compose(in, out);
  TEST_ASSERT_FALSE(screenContains("USB"));
  TEST_ASSERT_FALSE(screenContains("CONFLICT"));
  TEST_ASSERT_TRUE(out[1].tone == Tone::Good);
}

void test_link_lost_hides_robot_values()
{
  Inputs in = healthy();
  in.telemetryFresh = false;
  in.telemetry.faults = protection::OVERCURRENT;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("Link: LOST"));
  TEST_ASSERT_FALSE(screenContains("6.02"));
  TEST_ASSERT_FALSE(screenContains("Walk"));
  TEST_ASSERT_FALSE(screenContains("overcurrent"));
  TEST_ASSERT_FALSE(screenContains("Out on"));
  // Bridge-side settings are still shown.
  TEST_ASSERT_TRUE(screenContains("Speed 44"));
}

void test_sensors_shown_only_when_available()
{
  Inputs in = healthy();
  in.telemetry.tofMm = 123;
  in.telemetry.pitchCentideg = -50;
  in.telemetry.rollCentideg = 1234;
  compose(in, out);
  TEST_ASSERT_FALSE(screenContains("ToF"));
  in.telemetry.sensorsAvailable = protocol::SENSOR_TOF | protocol::SENSOR_ATTITUDE;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("ToF 123mm"));
  TEST_ASSERT_TRUE(screenContains("P-0.5"));
  TEST_ASSERT_TRUE(screenContains("R12.3"));
}

void test_sensors_and_all_faults_fit_together_at_extremes()
{
  Inputs in = healthy();
  in.telemetry.faults = 0x3F; // all six
  in.telemetry.sensorsAvailable = protocol::SENSOR_TOF | protocol::SENSOR_ATTITUDE;
  in.telemetry.tofMm = 65535;
  in.telemetry.pitchCentideg = -32768;
  in.telemetry.rollCentideg = -32768;
  in.telemetry.voltageMv = 65535;
  in.telemetry.currentMa = 65535;
  in.emergencyStop = true;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("ToF 65535mm"));
  TEST_ASSERT_TRUE(screenContains("R-327.6"));
  TEST_ASSERT_TRUE(screenContains("e-stop"));
  for (int l = 0; l < LINE_COUNT; l++)
    TEST_ASSERT_TRUE_MESSAGE(strlen(out[l].text) <= (size_t)LINE_CHARS, out[l].text);
}

void test_every_mode_and_gait_name_is_shown_in_full()
{
  const char *modes[] = {"Initialise", "Stand", "Walk", "Sleep", "Calibrate"};
  const char *gaits[] = {"Tri", "Wave", "Ripple", "Bi", "Quad", "Hop"};
  for (int m = 0; m < 5; m++)
  {
    for (int g = 0; g < 6; g++)
    {
      Inputs in = healthy();
      in.telemetry.bodyMode = (uint8_t)m;
      in.telemetry.gait = (uint8_t)g;
      compose(in, out);
      char expected[64];
      snprintf(expected, sizeof(expected), "Body %s  Gait %s", modes[m], gaits[g]);
      TEST_ASSERT_EQUAL_STRING(expected, out[2].text);
    }
  }
}

void test_unknown_mode_and_gait_do_not_crash()
{
  Inputs in = healthy();
  in.telemetry.bodyMode = 200;
  in.telemetry.gait = 99;
  compose(in, out);
  TEST_ASSERT_TRUE(screenContains("Body ?"));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_healthy_screen_shows_every_item);
  RUN_TEST(test_no_gamepad_is_shown);
  RUN_TEST(test_emergency_stop_is_shown);
  RUN_TEST(test_undervoltage_fault_shows_name_and_voltage);
  RUN_TEST(test_every_fault_combination_shows_every_name);
  RUN_TEST(test_source_conflict_is_shown);
  RUN_TEST(test_usb_in_control_is_shown);
  RUN_TEST(test_link_lost_hides_robot_values);
  RUN_TEST(test_sensors_shown_only_when_available);
  RUN_TEST(test_sensors_and_all_faults_fit_together_at_extremes);
  RUN_TEST(test_every_mode_and_gait_name_is_shown_in_full);
  RUN_TEST(test_unknown_mode_and_gait_do_not_crash);
  return UNITY_END();
}
