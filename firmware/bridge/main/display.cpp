#include "display.h"

#include <string.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "logo.h"
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

static status_screen::Line shown[status_screen::LINE_COUNT];
static bool shownFresh;
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

static void drawLine(int i, const status_screen::Line &l)
{
  int y = MARGIN_Y + i * LINE_HEIGHT;
  lcd.fillRect(0, y, lcd.width(), LINE_HEIGHT, TFT_BLACK);
  lcd.setTextColor(colour(l.tone), TFT_BLACK);
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
  lcd.fillScreen(TFT_BLACK);
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

void update(uint32_t nowMs, bool gamepadConnected, const gamepad_mapping::Desired &desired,
            const robot_link::Status &linkStatus)
{
  if (!startupDone(nowMs, linkStatus.emergencyStop))
    return;
  if (!firstDraw && nowMs - lastDrawMs < REFRESH_MS)
    return;
  lastDrawMs = nowMs;

  status_screen::Inputs in = {};
  in.gamepadConnected = gamepadConnected;
  in.emergencyStop = linkStatus.emergencyStop;
  in.desired = desired;
  in.telemetryFresh = linkStatus.telemetryFresh;
  in.telemetry = linkStatus.telemetry;
  status_screen::Line lines[status_screen::LINE_COUNT];
  status_screen::compose(in, lines);

  // A change of link state redraws everything; otherwise only changed lines.
  bool full = firstDraw || linkStatus.telemetryFresh != shownFresh;
  if (full)
    lcd.fillScreen(TFT_BLACK);
  for (int i = 0; i < status_screen::LINE_COUNT; i++)
  {
    if (full || strcmp(lines[i].text, shown[i].text) != 0 || lines[i].tone != shown[i].tone)
    {
      drawLine(i, lines[i]);
      shown[i] = lines[i];
    }
  }
  shownFresh = linkStatus.telemetryFresh;
  firstDraw = false;
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
