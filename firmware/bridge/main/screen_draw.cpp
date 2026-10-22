#include "screen_draw.h"

#include "fonts/mono_10.h"
#include "fonts/mono_12.h"
#include "fonts/sans_14.h"
#include "fonts/sans_22.h"

namespace screen_draw
{

// Text in a tone, on the dark grounds.
uint32_t toneColour(status_screen::Tone t)
{
  switch (t)
  {
  case status_screen::Tone::Good:
    return bf::OK;
  case status_screen::Tone::Warn:
    return bf::WARN;
  case status_screen::Tone::Bad:
    return bf::ACCENT_ON_DARK;
  case status_screen::Tone::Muted:
    return bf::INK_SUBTLE;
  default:
    return bf::SURFACE;
  }
}

Font mono10, mono12, sans14, sans22;

void useFont(lgfx::LovyanGFX &g, Font &f)
{
  if (f.loaded)
    g.setFont(&f.font);
  else
    g.setFont(&fonts::Font2); // still legible if a font failed to load
}


void loadFonts()
{
  mono10.load(fonts::MONO_10, sizeof(fonts::MONO_10));
  mono12.load(fonts::MONO_12, sizeof(fonts::MONO_12));
  sans14.load(fonts::SANS_14, sizeof(fonts::SANS_14));
  sans22.load(fonts::SANS_22, sizeof(fonts::SANS_22));
}

// A region's ground.
static void ground(lgfx::LovyanGFX &g, const Region &r, int y0)
{
  g.fillRect(0, y0, WIDTH, r.h, g.color888(bf::INK >> 16, (bf::INK >> 8) & 0xff, bf::INK & 0xff));
}

static uint32_t c565(lgfx::LovyanGFX &g, uint32_t rgb)
{
  return g.color888(rgb >> 16, (rgb >> 8) & 0xff, rgb & 0xff);
}

static void text(lgfx::LovyanGFX &g, Font &f, const char *s, uint32_t colour, uint32_t ground, int x, int y,
                 lgfx::textdatum_t datum = lgfx::top_left)
{
  useFont(g, f);
  g.setTextDatum(datum);
  g.setTextColor(c565(g, colour), c565(g, ground));
  g.drawString(s, x, y);
}

static void bar(lgfx::LovyanGFX &g, const status_screen::Bar &b, uint32_t fill, int x, int y, int w, int h)
{
  g.fillRoundRect(x, y, w, h, h / 2, c565(g, bf::INK_RAISED));
  if (b.present && b.fraction > 0)
  {
    int fw = (int)(b.fraction * w + 0.5f);
    if (fw < h)
      fw = h; // keep the rounded end visible
    g.fillRoundRect(x, y, fw, h, h / 2, c565(g, fill));
  }
  if (b.present && b.held)
  {
    // The held extreme: a tick across the bar.
    int tx = x + (int)(b.heldFraction * (w - 2) + 0.5f);
    g.fillRect(tx, y - 2, 2, h + 4, c565(g, bf::SURFACE));
  }
}

void drawHeader(lgfx::LovyanGFX &g, int y0, const status_screen::Header &hd)
{
  ground(g, HEADER, y0);
  // Right: the gamepad and the link, right-aligned.
  useFont(g, mono10);
  int linkW = g.textWidth(hd.link.text);
  int padW = g.textWidth(hd.gamepad.text);
  // The widest case (INITIALISE · RIPPLE, PAD 100%, CONFLICT) fits with the
  // chip in the small font.
  int right = WIDTH - 4;
  text(g, mono10, hd.link.text, toneColour(hd.link.tone), bf::INK, right, y0 + 11, lgfx::middle_right);
  right -= linkW + 6;
  text(g, mono10, hd.gamepad.text, toneColour(hd.gamepad.tone), bf::INK, right, y0 + 11, lgfx::middle_right);
  right -= padW + 4;
  // Left: the mode chip, in the larger font when it fits.
  Font *f = &mono12;
  useFont(g, *f);
  if (4 + g.textWidth(hd.mode.text) + 10 > right)
    f = &mono10;
  useFont(g, *f);
  int chipW = g.textWidth(hd.mode.text) + 10;
  g.fillRoundRect(4, y0 + 3, chipW, 16, 4, c565(g, bf::INK_RAISED));
  text(g, *f, hd.mode.text, toneColour(hd.mode.tone), bf::INK_RAISED, 9, y0 + 11, lgfx::middle_left);
}

void drawStrip(lgfx::LovyanGFX &g, int y0, const status_screen::Strip &s)
{
  ground(g, STRIP, y0);
  uint32_t fill = bf::INK_RAISED, ink = toneColour(s.tone);
  if (s.tone == status_screen::Tone::Bad)
  {
    fill = bf::ACCENT;
    ink = bf::INK;
  }
  else if (s.tone == status_screen::Tone::Warn)
  {
    fill = bf::WARN;
    ink = bf::INK;
  }
  g.fillRoundRect(4, y0 + 1, WIDTH - 8, STRIP.h - 2, 4, c565(g, fill));
  if (s.lines[1][0] == '\0')
  {
    text(g, mono10, s.lines[0], ink, fill, 9, y0 + STRIP.h / 2, lgfx::middle_left);
  }
  else
  {
    text(g, mono10, s.lines[0], ink, fill, 9, y0 + 7, lgfx::middle_left);
    text(g, mono10, s.lines[1], ink, fill, 9, y0 + 17, lgfx::middle_left);
  }
}

void drawMarker(lgfx::LovyanGFX &g, int y0, page_button::Page page)
{
  ground(g, MARKER, y0);
  int y = y0 + 3;
  for (int i = 0; i < page_button::PAGE_COUNT; i++)
  {
    int x = WIDTH / 2 + (i - 1) * 10;
    bool shown = i == (int)page;
    g.fillCircle(x, y, 2, c565(g, shown ? bf::ACCENT_ON_DARK : bf::INK_CARD_HOVER));
  }
}

void drawDrive(lgfx::LovyanGFX &g, int y0, const status_screen::DriveBody &d)
{
  ground(g, BODY, y0);
  constexpr int COL = 120;     // second column
  constexpr int BAR_W = 106;
  if (d.sensors.text[0])
    text(g, mono10, d.sensors.text, toneColour(d.sensors.tone), bf::INK, PAD, y0 + 0);
  text(g, sans22, d.voltage.text, toneColour(d.voltage.tone), bf::INK, PAD, y0 + 12);
  text(g, sans22, d.current.text, toneColour(d.current.tone), bf::INK, COL + PAD, y0 + 12);
  bar(g, d.voltageBar, bf::SERIES_1_DARK, PAD, y0 + 38, BAR_W, 5);
  bar(g, d.currentBar, bf::SERIES_2, COL + PAD, y0 + 38, BAR_W, 5);
  text(g, sans14, d.height.text, toneColour(d.height.tone), bf::INK, PAD, y0 + 47);
  text(g, sans14, d.speed.text, toneColour(d.speed.tone), bf::INK, COL + PAD, y0 + 47);
  bar(g, d.heightBar, bf::INK_SUBTLE, PAD + 50, y0 + 53, BAR_W - 50, 4);
  bar(g, d.speedBar, bf::INK_SUBTLE, COL + PAD + 60, y0 + 53, BAR_W - 60, 4);
  text(g, mono10, d.modes.text, toneColour(d.modes.tone), bf::INK, PAD, y0 + 68);
  text(g, mono10, d.output.text, toneColour(d.output.tone), bf::INK, WIDTH - PAD, y0 + 68, lgfx::top_right);
}

// Draws one trace of the power history into a well.
static void trace(lgfx::LovyanGFX &g, const power_history::History *h, bool current, uint16_t low, uint16_t high,
                  uint32_t colour, int x, int y, int w, int ht)
{
  g.fillRoundRect(x, y, w, ht, 4, c565(g, bf::INK_CODE));
  if (!h || high <= low)
    return;
  constexpr int MAX_W = 200;
  static power_history::Column cols[MAX_W];
  int cw = w - 4 < MAX_W ? w - 4 : MAX_W;
  power_history::columns(*h, cw, cols);
  int inner = ht - 4;
  // A value's row, clamped to the well; higher values higher up.
  auto rowOf = [&](uint16_t v) {
    int r = v <= low ? 0 : (int)((long)(v - low) * (inner - 1) / (high - low));
    if (r > inner - 1)
      r = inner - 1;
    return y + 2 + (inner - 1 - r);
  };
  uint32_t c = c565(g, colour);
  for (int i = 0; i < cw; i++)
  {
    const power_history::Column &col = cols[i];
    if (!col.present)
      continue;
    uint16_t lo = current ? col.minCurrentMa : col.minVoltageMv;
    uint16_t hi = current ? col.maxCurrentMa : col.maxVoltageMv;
    int top = rowOf(hi), bottom = rowOf(lo);
    g.drawFastVLine(x + 2 + i, top, bottom - top + 1, c);
  }
}

void drawPower(lgfx::LovyanGFX &g, int y0, const status_screen::PowerBody &p, const power_history::History *h)
{
  ground(g, BODY, y0);
  constexpr int TEXT_W = 74;
  constexpr int WELL_X = PAD + TEXT_W;
  constexpr int WELL_W = WIDTH - PAD - WELL_X;
  text(g, sans14, p.voltage.text, toneColour(p.voltage.tone), bf::INK, PAD, y0 + 0);
  text(g, mono10, p.minVoltage.text, toneColour(p.minVoltage.tone), bf::INK, PAD, y0 + 19);
  trace(g, h, false, p.traceLowMv, p.traceHighMv, bf::SERIES_1_DARK, WELL_X, y0 + 0, WELL_W, 32);
  text(g, sans14, p.current.text, toneColour(p.current.tone), bf::INK, PAD, y0 + 35);
  text(g, mono10, p.maxCurrent.text, toneColour(p.maxCurrent.tone), bf::INK, PAD, y0 + 54);
  trace(g, h, true, 0, p.traceHighMa, bf::SERIES_2, WELL_X, y0 + 35, WELL_W, 32);
  text(g, mono10, p.note.text, toneColour(p.note.tone), bf::INK, WIDTH - PAD, y0 + 70, lgfx::top_right);
}

void drawLink(lgfx::LovyanGFX &g, int y0, const status_screen::LinkBody &l)
{
  ground(g, BODY, y0);
  const status_screen::Text *rows[] = {&l.robot, &l.bridge, &l.age, &l.source, &l.gamepad};
  for (int i = 0; i < 5; i++)
    text(g, mono12, rows[i]->text, toneColour(rows[i]->tone), bf::INK, PAD, y0 + 1 + i * 16);
}

} // namespace screen_draw
