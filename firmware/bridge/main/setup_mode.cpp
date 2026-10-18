#include "setup_mode.h"

#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <bootloader_random.h>
#include <esp_attr.h>
#include <esp_bt.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_system.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "link.h"
#include "portal_page.h"
#include "setup/access_point.h"
#include "setup/calibration_export.h"
#include "setup/setup_entry.h"

// Survives esp_restart(), not a power-up; its power-up contents are
// arbitrary, which is why the reset reason is checked as well.
static RTC_NOINIT_ATTR uint32_t rtcMagic;

bool setup_mode_requested(void)
{
  bool setup = setup_entry::startInSetupMode(rtcMagic, esp_reset_reason() == ESP_RST_SW);
  rtcMagic = 0;
  return setup;
}

namespace setup_mode
{

void restartIntoSetup()
{
  rtcMagic = setup_entry::MAGIC;
  esp_restart();
}

// The portal's address; the access point's default network.
static const char ADDRESS[] = "192.168.4.1";
static const char PORTAL_URL[] = "http://192.168.4.1/";
constexpr int MAX_STATIONS = 1;
constexpr int WIFI_CHANNEL = 1;
constexpr uint32_t EXIT_DELAY_MS = 300; // lets the reply to Exit go out first
constexpr uint32_t TASK_PERIOD_MS = 2;

static char ssid[access_point::SSID_SIZE];
static char password[access_point::PASSWORD_LENGTH + 1];
static char joinCode[access_point::JOIN_CODE_SIZE];

// Created only in setup mode, so normal operation constructs no network
// objects at all.
static WebServer *server;
static DNSServer *dns;

static bool exitRequested;
static uint32_t exitAtMs;

// The password: made once from the hardware random source, then kept in NVS.
static void loadPassword()
{
  Preferences prefs;
  prefs.begin("spooder", false);
  String stored = prefs.getString("ap_pass", "");
  if (access_point::validPassword(stored.c_str()))
  {
    snprintf(password, sizeof(password), "%s", stored.c_str());
  }
  else
  {
    // WiFi is not running yet, so the RNG needs this entropy source to be
    // truly random.
    bootloader_random_enable();
    access_point::makePassword(esp_random, password);
    bootloader_random_disable();
    prefs.putString("ap_pass", password);
    printf("setup mode: new access point password stored\n");
  }
  prefs.end();
}

// --- HTTP -----------------------------------------------------------------------

static void noCache()
{
  server->sendHeader("Cache-Control", "no-store");
}

static void redirectToPortal()
{
  server->sendHeader("Location", PORTAL_URL, true);
  noCache();
  server->send(302, "text/plain", "");
}

static void handleRoot()
{
  // A captive-portal browser may ask for "/" under another host name; send
  // it to the portal's own address so the page's requests go there too.
  if (server->hostHeader() != ADDRESS)
  {
    redirectToPortal();
    return;
  }
  noCache();
  server->send_P(200, "text/html", PORTAL_PAGE, sizeof(PORTAL_PAGE) - 1);
}

// A small JSON writer over a fixed buffer; output past the end is dropped
// and reported by ok().
struct Json
{
  char buf[3072];
  int len = 0;
  bool overflow = false;

  void add(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
  {
    if (overflow)
      return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + len, sizeof(buf) - len, fmt, ap);
    va_end(ap);
    if (n < 0 || n >= (int)sizeof(buf) - len)
      overflow = true;
    else
      len += n;
  }
  bool ok() const { return !overflow; }
};

static const char *modeName(RequestedMode m)
{
  switch (m)
  {
  case RequestedMode::Stand:
    return "Stand";
  case RequestedMode::Walk:
    return "Walk";
  case RequestedMode::Sleep:
    return "Sleep";
  case RequestedMode::Calibrate:
    return "Calibrate";
  default:
    return "?";
  }
}

// GET /api/state: the portal heartbeat, and everything the page shows.
static void handleState()
{
  robot_link::portalPoll();
  static robot_link::PortalView view;
  robot_link::portalView(view);
  robot_link::Status link = robot_link::status();

  static Json j;
  j.len = 0;
  j.overflow = false;
  j.add("{\"portal\":%d,\"req\":\"%s\",\"estop\":%d,\"link\":%d", view.connected ? 1 : 0,
        modeName(view.desired.mode), link.emergencyStop ? 1 : 0, link.telemetryFresh ? 1 : 0);
  const protocol::Telemetry &t = link.telemetry;
  j.add(",\"tel\":{\"mode\":%u,\"out\":%d,\"mv\":%u,\"ma\":%u,\"faults\":%u,\"flags\":%u}", t.bodyMode,
        t.outputsOn ? 1 : 0, t.voltageMv, t.currentMa, t.faults, t.flags);
  const calibration_sync::Sync &s = view.sync;
  if (!s.haveReport)
  {
    j.add(",\"cal\":null}");
  }
  else
  {
    j.add(",\"cal\":{\"id\":%u,\"res\":%u,\"j\":[", s.report.lastRequestId, s.report.result);
    for (int i = 0; i < protocol::CALIBRATION_JOINTS; i++)
    {
      const protocol::CalibrationJoint &c = s.report.joints[i];
      j.add("%s[%d,%u,%u,%d,%d]", i ? "," : "", c.offsetCentideg, c.pulse0Us, c.pulse180Us,
            (c.flags & protocol::CALIBRATION_REVERSED) ? 1 : 0, calibration_sync::pending(s, i) ? 1 : 0);
    }
    j.add("]}}");
  }
  noCache();
  if (!j.ok())
  {
    server->send(500, "text/plain", "state too large");
    return;
  }
  server->send(200, "application/json", j.buf);
}

// POST /api/mode, m = stand | calibrate | sleep | clear
static void handleMode()
{
  String m = server->arg("m");
  bool taken = true;
  if (m == "stand")
    taken = robot_link::portalRequestMode(RequestedMode::Stand);
  else if (m == "calibrate")
    taken = robot_link::portalRequestMode(RequestedMode::Calibrate);
  else if (m == "sleep")
    taken = robot_link::portalRequestMode(RequestedMode::Sleep);
  else if (m == "clear")
    robot_link::portalRequestClear();
  else
    taken = false;
  noCache();
  server->send(taken ? 200 : 400, "text/plain", taken ? "ok" : "unknown mode");
}

// POST /api/joint, j = 0..17, f = offset | p0 | p180 | rev, v = integer
// (centidegrees, microseconds, or 0/1), rel = 1 for a nudge by v.
static void handleJoint()
{
  String f = server->arg("f");
  calibration_sync::Field field;
  if (f == "offset")
    field = calibration_sync::Field::Offset;
  else if (f == "p0")
    field = calibration_sync::Field::Pulse0;
  else if (f == "p180")
    field = calibration_sync::Field::Pulse180;
  else if (f == "rev")
    field = calibration_sync::Field::Reversed;
  else
  {
    server->send(400, "text/plain", "unknown field");
    return;
  }
  long joint = server->arg("j").toInt();
  long value = server->arg("v").toInt();
  if (!server->hasArg("j") || !server->hasArg("v") || joint < 0 || joint >= protocol::CALIBRATION_JOINTS ||
      value < -100000 || value > 100000)
  {
    server->send(400, "text/plain", "bad joint or value");
    return;
  }
  bool relative = server->arg("rel") == "1";
  noCache();
  if (!robot_link::portalChange((int)joint, field, (int32_t)value, relative))
    server->send(409, "text/plain", "no calibration report yet: press Calibrate");
  else
    server->send(200, "text/plain", "ok");
}

// GET /api/export: the reported table as calibration_defaults.cpp.
static void handleExport()
{
  static robot_link::PortalView view;
  robot_link::portalView(view);
  noCache();
  if (!view.sync.haveReport)
  {
    server->send(409, "text/plain", "no calibration report yet: press Calibrate");
    return;
  }
  static char text[calibration_export::MAX_TEXT_SIZE];
  if (calibration_export::write(view.sync.report.joints, text, sizeof(text)) < 0)
  {
    server->send(500, "text/plain", "export too large");
    return;
  }
  server->send(200, "text/plain; charset=utf-8", text);
}

// POST /api/exit: restart into normal operation (the RTC magic is already
// clear).
static void handleExit()
{
  noCache();
  server->send(200, "text/plain", "restarting");
  exitRequested = true;
  exitAtMs = millis() + EXIT_DELAY_MS;
}

static void startServer()
{
  server = new WebServer(80);
  server->on("/", HTTP_GET, handleRoot);
  server->on("/api/state", HTTP_GET, handleState);
  server->on("/api/mode", HTTP_POST, handleMode);
  server->on("/api/joint", HTTP_POST, handleJoint);
  server->on("/api/export", HTTP_GET, handleExport);
  server->on("/api/exit", HTTP_POST, handleExit);
  // Captive-portal checks (Android, Apple, Windows) and every other path go
  // to the portal, so a joining phone opens it by itself.
  static const char *const PROBES[] = {"/generate_204", "/gen_204",       "/hotspot-detect.html",
                                       "/connecttest.txt", "/ncsi.txt",   "/redirect",
                                       "/canonical.html", "/success.txt", "/library/test/success.html"};
  for (const char *p : PROBES)
    server->on(p, redirectToPortal);
  server->onNotFound(redirectToPortal);
  server->begin();
}

static void setupTask(void *)
{
  uint32_t lastScreenMs = 0;
  bool screenDrawn = false;
  for (;;)
  {
    server->handleClient();
    uint32_t now = millis();
    if (!screenDrawn || now - lastScreenMs >= display::REFRESH_MS)
    {
      static robot_link::PortalView view;
      robot_link::portalView(view);
      display::setupUpdate(now, ssid, password, view.connected, robot_link::status());
      lastScreenMs = now;
      screenDrawn = true;
    }
    if (exitRequested && (int32_t)(now - exitAtMs) >= 0)
    {
      printf("setup mode: exit, restarting into normal operation\n");
      esp_restart(); // the RTC magic is clear: normal operation
    }
    vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

} // namespace setup_mode

void setup_mode_start(void)
{
  using namespace setup_mode;
  // Bluetooth is never started in setup mode; its controller memory goes
  // back to the heap for WiFi.
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  initArduino();
  printf("spooder bridge: setup mode (Bluetooth off)\n");

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  access_point::ssid(mac, ssid);
  loadPassword();
  access_point::joinCode(ssid, password, joinCode, sizeof(joinCode));

  display::setupStart(joinCode);
  robot_link::startPortal();

  WiFi.persistent(false); // nothing of the WiFi configuration goes to flash
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_STATIONS))
    printf("setup mode: access point failed to start\n");
  dns = new DNSServer();
  dns->start(); // captive mode: every name resolves to the access point
  startServer();
  printf("setup mode: access point %s, portal at %s\n", ssid, PORTAL_URL);

  // Below the link task (priority 5) on the same core, as loop() is in
  // normal operation, so serving the portal never delays command frames.
  xTaskCreatePinnedToCore(setupTask, "setup", 8192, nullptr, 1, nullptr, ARDUINO_RUNNING_CORE);
}
