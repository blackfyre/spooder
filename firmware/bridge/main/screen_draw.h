#ifndef BRIDGE_SCREEN_DRAW_H
#define BRIDGE_SCREEN_DRAW_H

// Draws the status screen's regions (core/status/status_screen's View) in the
// Blackfyre dark palette onto any LovyanGFX surface: the bridge's region
// sprite, or a whole-screen sprite in the host screenshot tool
// (tools/bridge-screenshots). Each function fills its region's ground at
// y0 and draws within it.
#include <stdint.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "status/page_button.h"
#include "status/power_history.h"
#include "status/status_screen.h"

namespace screen_draw
{

// The Blackfyre design system's dark app palette (RGB888).
namespace bf
{
constexpr uint32_t INK = 0x1b1b19;
constexpr uint32_t INK_RAISED = 0x2c2c29;
constexpr uint32_t INK_CARD_HOVER = 0x383834;
constexpr uint32_t INK_CODE = 0x121211;
constexpr uint32_t SURFACE = 0xf3f2ee;
constexpr uint32_t INK_SUBTLE = 0xa19e96;
constexpr uint32_t ACCENT = 0xe2551d;
constexpr uint32_t ACCENT_ON_DARK = 0xff8a4c;
constexpr uint32_t OK = 0x7fd88a;
constexpr uint32_t WARN = 0xe0af68;
constexpr uint32_t SERIES_1_DARK = 0x3987e5; // voltage
constexpr uint32_t SERIES_2 = 0xd55181;      // current
} // namespace bf

// The screen's regions, top to bottom.
struct Region
{
  int y, h;
};
constexpr Region HEADER = {0, 22};
constexpr Region BODY = {22, 82};
constexpr Region STRIP = {104, 24};
constexpr Region MARKER = {128, 7};
constexpr int WIDTH = 240;
constexpr int PAD = 6;
static_assert(MARKER.y + MARKER.h == 135, "regions fill the screen");
constexpr int HEIGHT = 135;

// The smooth fonts, loaded once and shared by the panel and the sprite.
struct Font
{
  lgfx::PointerWrapper data;
  lgfx::VLWfont font;
  bool loaded = false;

  void load(const uint8_t *array, size_t size)
  {
    data.set(array, size);
    loaded = font.loadFont(&data);
  }
};
extern Font mono10, mono12, sans14, sans22;

/**
 * @brief Loads the four fonts from their generated data.
 */
void loadFonts();

void useFont(lgfx::LovyanGFX &g, Font &f);
uint32_t toneColour(status_screen::Tone t);

void drawHeader(lgfx::LovyanGFX &g, int y0, const status_screen::Header &hd);
void drawStrip(lgfx::LovyanGFX &g, int y0, const status_screen::Strip &s);
void drawMarker(lgfx::LovyanGFX &g, int y0, page_button::Page page);
void drawDrive(lgfx::LovyanGFX &g, int y0, const status_screen::DriveBody &d);
void drawPower(lgfx::LovyanGFX &g, int y0, const status_screen::PowerBody &p, const power_history::History *h);
void drawLink(lgfx::LovyanGFX &g, int y0, const status_screen::LinkBody &l);

} // namespace screen_draw

#endif // BRIDGE_SCREEN_DRAW_H
