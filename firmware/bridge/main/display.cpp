#include "display.h"

#include <string.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

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

void setup()
{
  lcd.init();
  lcd.setRotation(1); // landscape, USB connector on the left
  lcd.setBrightness(160);
  lcd.setFont(&fonts::Font2);
  lcd.setTextWrap(false);
  lcd.fillScreen(TFT_BLACK);
}

void update(uint32_t nowMs, bool gamepadConnected, const gamepad_mapping::Desired &desired,
            const robot_link::Status &linkStatus)
{
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

} // namespace display
