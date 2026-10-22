// Renders the bridge's status screen pages, in chosen states, to PNGs:
// the bridge's drawing code (screen_draw) and page content (status_screen)
// into a 240×135 sprite, scaled up without smoothing so the pixels stay
// those of the panel.
//
//   bridge-screenshots <output directory> [scale]
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "protection/protection.h"
#include "screen_draw.h"

using namespace screen_draw;
using page_button::Page;

struct Shot
{
  const char *name;
  Page page;
  status_screen::Inputs in;
};

static status_screen::Inputs healthy()
{
  status_screen::Inputs in = {};
  in.gamepadConnected = true;
  in.gamepadBattery = 205; // 80 %
  in.desired.height = 60;
  in.desired.speed = 44;
  in.desired.stride = StrideMode::Dynamic;
  in.desired.stickMode = gamepad_mapping::StickMode::Steer;
  in.telemetryFresh = true;
  in.telemetryAgeMs = 40;
  in.telemetry.bodyMode = 2; // Walk
  in.telemetry.gait = 0;     // Tri
  in.telemetry.outputsOn = true;
  in.telemetry.voltageMv = 6080;
  in.telemetry.currentMa = 1840;
  in.rates = {true, 50, 0, true, 10, 0};
  // A stand a moment ago: the surge and the sag still held.
  in.currentPeak = {true, 4300, 0, 0};
  in.voltageLow = {true, 5540, 0, 0};
  return in;
}

// A minute on the bench: idle, a stand from Sleep (the surge and its sag),
// standing, then walking; the supply sags 0.125 Ω with the current.
static void benchMinute(power_history::History &h)
{
  uint32_t t = 0;
  auto at = [&](uint32_t untilMs, auto currentMa) {
    for (; t < untilMs; t += 20)
    {
      uint16_t ma = (uint16_t)currentMa(t);
      uint16_t mv = (uint16_t)(6080 - ma / 8); // 0.125 Ω
      power_history::record(h, t, true, mv, ma);
    }
  };
  at(10000, [](uint32_t) { return 60; });
  at(10300, [](uint32_t) { return 4300; });
  at(11000, [](uint32_t ms) { return 4300 - (ms - 10300) * 5; });
  at(25000, [](uint32_t ms) { return 450 + (ms / 20 % 7) * 10; });
  at(60000, [](uint32_t ms) { return 1150 + (int)(900 * ((ms / 20) % 25) / 25) + (ms / 20 % 3) * 120; });
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    fprintf(stderr, "usage: %s <output directory> [scale]\n", argv[0]);
    return 2;
  }
  std::string dir = argv[1];
  int scale = argc > 2 ? atoi(argv[2]) : 3;
  if (scale < 1)
    scale = 1;

  loadFonts();
  static power_history::History history;
  benchMinute(history);

  Shot shots[7];
  shots[0] = {"bridge-drive", Page::Drive, healthy()};
  shots[1] = {"bridge-power", Page::Power, healthy()};
  shots[1].in.history = &history;
  shots[1].in.telemetry.currentMa = 1840;
  shots[2] = {"bridge-link", Page::Link, healthy()};
  shots[3] = {"bridge-fault", Page::Drive, healthy()};
  shots[3].in.telemetry.bodyMode = 3; // Sleep
  shots[3].in.telemetry.outputsOn = false;
  shots[3].in.telemetry.faults = protection::UNDERVOLTAGE | protection::BROWNOUT;
  shots[3].in.telemetry.voltageMv = 4620;
  shots[3].in.telemetry.currentMa = 30;
  shots[3].in.voltageLow.value = 4480; // the dip that browned out
  shots[4] = {"bridge-estop", Page::Power, healthy()};
  shots[4].in.history = &history;
  shots[4].in.emergencyStop = true;
  shots[4].in.telemetry.bodyMode = 3; // Sleep
  shots[4].in.telemetry.faults = protection::EMERGENCY_STOP;
  shots[4].in.telemetry.outputsOn = false;
  shots[5] = {"bridge-lost", Page::Drive, healthy()};
  shots[5].in.telemetryFresh = false;
  // The moment of a stand surge: a new high current and a deeper sag, both
  // beyond their held extremes, in the warning colour.
  shots[6] = {"bridge-surge", Page::Drive, healthy()};
  shots[6].in.telemetry.bodyMode = 1; // Stand
  shots[6].in.telemetry.currentMa = 4300;
  shots[6].in.telemetry.voltageMv = 5540;
  shots[6].in.currentPeak = {true, 4300, 0, 0, true, 0};
  shots[6].in.voltageLow = {true, 5540, 0, 0, true, 0};

  LGFX_Sprite screen;
  screen.setColorDepth(16);
  LGFX_Sprite scaled;
  scaled.setColorDepth(24);
  if (!screen.createSprite(WIDTH, HEIGHT) || !scaled.createSprite(WIDTH * scale, HEIGHT * scale))
  {
    fprintf(stderr, "no memory for the sprites\n");
    return 1;
  }

  static status_screen::View view;
  for (const Shot &shot : shots)
  {
    status_screen::composePage(shot.in, shot.page, view);
    drawHeader(screen, HEADER.y, view.header);
    switch (view.page)
    {
    case Page::Drive:
      drawDrive(screen, BODY.y, view.drive);
      break;
    case Page::Power:
      drawPower(screen, BODY.y, view.power, shot.in.history);
      break;
    case Page::Link:
      drawLink(screen, BODY.y, view.link);
      break;
    }
    drawStrip(screen, STRIP.y, view.strip);
    drawMarker(screen, MARKER.y, view.page);

    for (int y = 0; y < HEIGHT; y++)
      for (int x = 0; x < WIDTH; x++)
        scaled.fillRect(x * scale, y * scale, scale, scale, screen.readPixel(x, y));
    size_t length = 0;
    void *png = scaled.createPng(&length, 0, 0, scaled.width(), scaled.height());
    if (!png)
      fprintf(stderr, "createPng failed (%dx%d)\n", (int)scaled.width(), (int)scaled.height());
    std::string path = dir + "/" + shot.name + ".png";
    FILE *f = png ? fopen(path.c_str(), "wb") : nullptr;
    if (!f || fwrite(png, 1, length, f) != length)
    {
      fprintf(stderr, "could not write %s\n", path.c_str());
      return 1;
    }
    fclose(f);
    free(png);
    printf("%s\n", path.c_str());
  }
  return 0;
}
