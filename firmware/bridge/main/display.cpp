#include "display.h"

#include <stdio.h>
#include <string.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "logo.h"
#include "screen_draw.h"
#include "status/power_history.h"
#include "status/status_screen.h"

namespace display
{

// TTGO T-Display v1.1: ST7789V, 135×240, on SPI with the backlight on GPIO4.
class Panel : public lgfx::LGFX_Device
{
public:
  Panel()
  {
    {
      auto cfg = bus_.config();
      cfg.spi_host = SPI3_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.pin_sclk = 18;
      cfg.pin_mosi = 19;
      cfg.pin_miso = -1;
      cfg.pin_dc = 16;
      bus_.config(cfg);
      panel_.setBus(&bus_);
    }
    {
      auto cfg = panel_.config();
      cfg.pin_cs = 5;
      cfg.pin_rst = 23;
      cfg.pin_busy = -1;
      cfg.panel_width = 135;
      cfg.panel_height = 240;
      cfg.offset_x = 52; // the 135×240 glass sits inside the 240×320 controller RAM
      cfg.offset_y = 40;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.bus_shared = false;
      panel_.config(cfg);
    }
    {
      auto cfg = light_.config();
      cfg.pin_bl = 4;
      cfg.invert = false;
      cfg.freq = 12000;
      cfg.pwm_channel = 7;
      light_.config(cfg);
      panel_.setLight(&light_);
    }
    setPanel(&panel_);
  }

private:
  lgfx::Bus_SPI bus_;
  lgfx::Panel_ST7789 panel_;
  lgfx::Light_PWM light_;
};

static Panel lcd;

using namespace screen_draw;

// One region at a time is drawn into this sprite, then pushed: no flicker,
// and the fonts blend against the right ground. Without it (allocation
// failed), regions are drawn straight to the panel.
static LGFX_Sprite canvas(&lcd);
static bool haveCanvas;

enum class Phase
{
  Logo,
  BootLog,
  Status,
};
static Phase phase = Phase::Logo;
static uint32_t phaseSinceMs;
static bool logoHeld;

constexpr int BOOT_LINES = 8;
static status_screen::Line bootLines[BOOT_LINES];
static int bootLineCount;

static status_screen::Line shown[status_screen::LINE_COUNT]; // setup mode
static bool firstDraw = true;
static uint32_t lastDrawMs;

constexpr int LINE_HEIGHT = 16;
constexpr int MARGIN_X = 4;
constexpr int MARGIN_Y = 3;

static uint32_t colour(status_screen::Tone t)
{
  switch (t)
  {
  case status_screen::Tone::Good:
    return TFT_GREEN;
  case status_screen::Tone::Warn:
    return TFT_ORANGE;
  case status_screen::Tone::Bad:
    return TFT_RED;
  default:
    return TFT_WHITE;
  }
}

static uint32_t rgb(uint32_t c)
{
  return lcd.color888(c >> 16, (c >> 8) & 0xff, c & 0xff);
}

// A start-up log line, in the status screen's palette and label font.
static void drawLine(int i, const status_screen::Line &l)
{
  int y = MARGIN_Y + i * LINE_HEIGHT;
  lcd.fillRect(0, y, lcd.width(), LINE_HEIGHT, rgb(bf::INK));
  useFont(lcd, mono12);
  lcd.setTextDatum(lgfx::top_left);
  lcd.setTextColor(rgb(toneColour(l.tone)), rgb(bf::INK));
  lcd.drawString(l.text, MARGIN_X, y);
}

// The logo's drawn area, at 2x, centred: it fills the 135 px height.
static void drawLogo()
{
  int minX = logo::WIDTH, maxX = -1, minY = logo::HEIGHT, maxY = -1;
  for (int y = 0; y < logo::HEIGHT; y++)
    for (int x = 0; x < logo::WIDTH; x++)
      if (logo::pixel(x, y))
      {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
  lcd.fillScreen(TFT_BLACK);
  if (maxX < 0)
    return;
  constexpr int SCALE = 2;
  int ox = (lcd.width() - (maxX - minX + 1) * SCALE) / 2;
  int oy = (lcd.height() - (maxY - minY + 1) * SCALE) / 2;
  lcd.startWrite();
  for (int y = minY; y <= maxY; y++)
    for (int x = minX; x <= maxX; x++)
      if (logo::pixel(x, y))
        lcd.fillRect(ox + (x - minX) * SCALE, oy + (y - minY) * SCALE, SCALE, SCALE, TFT_WHITE);
  lcd.endWrite();
}

static void drawBootLog()
{
  lcd.fillScreen(rgb(bf::INK));
  for (int i = 0; i < bootLineCount; i++)
    drawLine(i, bootLines[i]);
}

static void initPanel()
{
  lcd.init();
  lcd.setRotation(1); // landscape, USB connector on the left
  lcd.setBrightness(160);
  lcd.setFont(&fonts::Font2);
  lcd.setTextWrap(false);
}

void setup()
{
  initPanel();
  loadFonts();
  // Before Bluepad32 starts and takes its share of the heap.
  canvas.setColorDepth(16);
  haveCanvas = canvas.createSprite(WIDTH, BODY.h) != nullptr;
  if (!haveCanvas)
    printf("display: no memory for the region sprite; drawing straight to the panel\n");
  drawLogo();
  phase = Phase::Logo;
  phaseSinceMs = millis();
  bootMessage("Boot");
}

void bootMessage(const char *text, bool ok)
{
  if (bootLineCount >= BOOT_LINES)
    return;
  status_screen::Line &l = bootLines[bootLineCount++];
  snprintf(l.text, sizeof(l.text), "%s", text);
  l.tone = ok ? status_screen::Tone::Normal : status_screen::Tone::Bad;
}

// Advances the start-up sequence; returns true once the status screen is due.
static bool startupDone(uint32_t nowMs, bool emergencyStop)
{
  if (phase != Phase::Status && emergencyStop)
  {
    phase = Phase::Status; // never hide an emergency stop behind the logo
    return true;
  }
  if (phase == Phase::Logo && nowMs - phaseSinceMs >= LOGO_MS && !logoHeld)
  {
    phase = Phase::BootLog;
    phaseSinceMs = nowMs;
    drawBootLog();
  }
  else if (phase == Phase::BootLog && nowMs - phaseSinceMs >= BOOT_LOG_MS)
  {
    phase = Phase::Status;
  }
  return phase == Phase::Status;
}

// The surface to draw region @p r on (screen_draw fills its ground).
static lgfx::LovyanGFX &begin(const Region &r)
{
  if (haveCanvas)
    return canvas;
  lcd.setClipRect(0, r.y, WIDTH, r.h);
  return lcd;
}

static void end(const Region &r)
{
  if (haveCanvas) // the region's rows are the sprite's first ones
    lcd.pushImage(0, r.y, WIDTH, r.h, (const lgfx::swap565_t *)canvas.getBuffer());
  else
    lcd.clearClipRect();
}

// The y offset of a region's own coordinates on the drawing surface.
static int top(const Region &r)
{
  return haveCanvas ? 0 : r.y;
}

// Draws one region through the sprite (or straight to the panel).
template <typename Draw> static void drawRegion(const Region &r, Draw draw)
{
  draw(begin(r));
  end(r);
}

// --- what changed --------------------------------------------------------------

static bool same(const status_screen::Text &a, const status_screen::Text &b)
{
  return a.tone == b.tone && strcmp(a.text, b.text) == 0;
}

static bool same(const status_screen::Bar &a, const status_screen::Bar &b)
{
  return a.present == b.present && (!a.present || (int)(a.fraction * 1000) == (int)(b.fraction * 1000)) &&
         a.held == b.held && (!a.held || (int)(a.heldFraction * 1000) == (int)(b.heldFraction * 1000));
}

static bool same(const status_screen::Header &a, const status_screen::Header &b)
{
  return same(a.mode, b.mode) && same(a.gamepad, b.gamepad) && same(a.link, b.link);
}

static bool same(const status_screen::Strip &a, const status_screen::Strip &b)
{
  return a.tone == b.tone && strcmp(a.lines[0], b.lines[0]) == 0 && strcmp(a.lines[1], b.lines[1]) == 0;
}

static bool same(const status_screen::DriveBody &a, const status_screen::DriveBody &b)
{
  return same(a.voltage, b.voltage) && same(a.current, b.current) && same(a.voltageBar, b.voltageBar) &&
         same(a.currentBar, b.currentBar) && same(a.height, b.height) && same(a.speed, b.speed) &&
         same(a.heightBar, b.heightBar) && same(a.speedBar, b.speedBar) && same(a.modes, b.modes) &&
         same(a.output, b.output) && same(a.sensors, b.sensors);
}

static bool same(const status_screen::LinkBody &a, const status_screen::LinkBody &b)
{
  return same(a.robot, b.robot) && same(a.bridge, b.bridge) && same(a.age, b.age) && same(a.source, b.source) &&
         same(a.gamepad, b.gamepad);
}

static power_history::History history;
static status_screen::RateMeter rates;
static peak_hold::Hold currentPeak, voltageLow;
static status_screen::View shownView;
static uint32_t historyMarkShown; // count + next: changes with every sample

void update(uint32_t nowMs, bool gamepadConnected, uint8_t gamepadBattery, page_button::Page page,
            const gamepad_mapping::Desired &desired, const robot_link::Status &linkStatus)
{
  // The history and rates run whatever is shown.
  power_history::record(history, nowMs, linkStatus.telemetryFresh, linkStatus.telemetry.voltageMv,
                        linkStatus.telemetry.currentMa);
  status_screen::updateRates(rates, nowMs, linkStatus.telemetryFresh, linkStatus.telemetry, linkStatus.validFrames,
                             linkStatus.discardedFrames);
  peak_hold::update(currentPeak, peak_hold::Extreme::Max, peak_hold::CURRENT_FALL_MA_PER_S,
                    peak_hold::CURRENT_WARN_MARGIN_MA, nowMs,
                    linkStatus.telemetryFresh, linkStatus.telemetry.currentMa);
  peak_hold::update(voltageLow, peak_hold::Extreme::Min, peak_hold::VOLTAGE_RISE_MV_PER_S,
                    peak_hold::VOLTAGE_WARN_MARGIN_MV, nowMs,
                    linkStatus.telemetryFresh, linkStatus.telemetry.voltageMv);

  if (!startupDone(nowMs, linkStatus.emergencyStop))
    return;
  if (!firstDraw && nowMs - lastDrawMs < REFRESH_MS)
    return;
  lastDrawMs = nowMs;

  status_screen::Inputs in = {};
  in.gamepadConnected = gamepadConnected;
  in.gamepadBattery = gamepadBattery;
  in.emergencyStop = linkStatus.emergencyStop;
  in.desired = desired;
  in.telemetryFresh = linkStatus.telemetryFresh;
  in.telemetry = linkStatus.telemetry;
  in.telemetryAgeMs = linkStatus.telemetryAgeMs;
  in.rates = rates.rates;
  in.history = &history;
  in.currentPeak = currentPeak;
  in.voltageLow = voltageLow;
  static status_screen::View view; // too large for loop()'s stack
  status_screen::composePage(in, page, view);

  bool pageChanged = firstDraw || view.page != shownView.page;
  if (firstDraw)
    lcd.fillScreen(lcd.color888(bf::INK >> 16, (bf::INK >> 8) & 0xff, bf::INK & 0xff));
  if (pageChanged || !same(view.header, shownView.header))
    drawRegion(HEADER, [&](lgfx::LovyanGFX &g) { drawHeader(g, top(HEADER), view.header); });
  if (pageChanged || !same(view.strip, shownView.strip))
    drawRegion(STRIP, [&](lgfx::LovyanGFX &g) { drawStrip(g, top(STRIP), view.strip); });
  if (pageChanged)
    drawRegion(MARKER, [&](lgfx::LovyanGFX &g) { drawMarker(g, top(MARKER), view.page); });
  switch (view.page)
  {
  case page_button::Page::Drive:
    if (pageChanged || !same(view.drive, shownView.drive))
      drawRegion(BODY, [&](lgfx::LovyanGFX &g) { drawDrive(g, top(BODY), view.drive); });
    break;
  case page_button::Page::Power:
    // The traces move with every new sample.
    if (pageChanged || (uint32_t)history.count + (uint32_t)history.next != historyMarkShown ||
        !same(view.power.voltage, shownView.power.voltage) || !same(view.power.current, shownView.power.current))
      drawRegion(BODY, [&](lgfx::LovyanGFX &g) { drawPower(g, top(BODY), view.power, &history); });
    historyMarkShown = (uint32_t)history.count + (uint32_t)history.next;
    break;
  case page_button::Page::Link:
    if (pageChanged || !same(view.link, shownView.link))
      drawRegion(BODY, [&](lgfx::LovyanGFX &g) { drawLink(g, top(BODY), view.link); });
    break;
  }
  shownView = view;
  firstDraw = false;
}

bool statusShown()
{
  return phase == Phase::Status;
}

bool showingLogo()
{
  return phase == Phase::Logo;
}

void holdLogo(bool hold)
{
  logoHeld = hold;
}

// --- setup mode ---------------------------------------------------------------

// The QR code fills the screen height on the right; the text keeps to the
// left of it (status_screen::SETUP_TEXT_WIDTH_PX).
constexpr int QR_SIZE = 135;
// The join code is 40 bytes: version 3 (29 modules) at 4 px per module,
// leaving a white quiet zone of 9 px.
constexpr uint8_t QR_VERSION = 3;

static void drawSetupLine(int i, const status_screen::Line &l)
{
  int y = MARGIN_Y + i * LINE_HEIGHT;
  lcd.fillRect(0, y, lcd.width() - QR_SIZE, LINE_HEIGHT, TFT_BLACK);
  lcd.setTextColor(colour(l.tone), TFT_BLACK);
  lcd.drawString(l.text, MARGIN_X, y);
}

void setupStart(const char *joinCode)
{
  initPanel();
  lcd.fillScreen(TFT_BLACK);
  lcd.qrcode(joinCode, lcd.width() - QR_SIZE, (lcd.height() - QR_SIZE) / 2, QR_SIZE, QR_VERSION, false);
  phase = Phase::Status;
  firstDraw = true;
}

void setupUpdate(uint32_t nowMs, const char *ssid, const char *password, bool portalConnected,
                 const robot_link::Status &linkStatus)
{
  if (!firstDraw && nowMs - lastDrawMs < REFRESH_MS)
    return;
  lastDrawMs = nowMs;

  status_screen::SetupInputs in = {};
  in.ssid = ssid;
  in.password = password;
  in.address = "192.168.4.1";
  in.portalConnected = portalConnected;
  in.emergencyStop = linkStatus.emergencyStop;
  in.telemetryFresh = linkStatus.telemetryFresh;
  in.telemetry = linkStatus.telemetry;
  status_screen::Line lines[status_screen::LINE_COUNT];
  status_screen::composeSetup(in, lines);

  for (int i = 0; i < status_screen::LINE_COUNT; i++)
  {
    if (firstDraw || strcmp(lines[i].text, shown[i].text) != 0 || lines[i].tone != shown[i].tone)
    {
      drawSetupLine(i, lines[i]);
      shown[i] = lines[i];
    }
  }
  firstDraw = false;
}

} // namespace display
