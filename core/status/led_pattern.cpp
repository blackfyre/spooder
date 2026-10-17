#include "led_pattern.h"

#include <math.h>
#include "protection/protection.h"

namespace led_pattern
{

// Full-scale colours.
static const Rgb RED = {255, 0, 0};
static const Rgb AMBER = {255, 128, 0};
static const Rgb GREEN = {0, 255, 0};
static const Rgb BLUE = {0, 64, 255};
static const Rgb WHITE = {255, 255, 255};
static const Rgb CYAN = {0, 255, 255};
static const Rgb MAGENTA = {255, 0, 255};
static const Rgb OFF = {0, 0, 0};

struct FaultIndication
{
  uint16_t fault;
  Rgb colour;
};
// Lowest bit wins when several faults latch.
static const FaultIndication FAULT_COLOURS[] = {
    {protection::OVERCURRENT, WHITE},
    {protection::BROWNOUT, MAGENTA},
    {protection::OVERVOLTAGE, BLUE},
    {protection::SUSTAINED_CURRENT, {255, 96, 0}}, // orange
    {protection::UNDERVOLTAGE, {255, 255, 0}},     // yellow
};

// One hue per gait: Tri, Wave, Ripple, Bi, Quad, Hop.
static const Rgb GAIT_COLOURS[] = {
    {0, 255, 0},     // Tri: green
    {0, 160, 255},   // Wave: sky blue
    {160, 0, 255},   // Ripple: violet
    {255, 220, 0},   // Bi: yellow
    {0, 255, 160},   // Quad: mint
    {255, 0, 160},   // Hop: pink
};
constexpr int GAIT_COLOUR_COUNT = sizeof(GAIT_COLOURS) / sizeof(GAIT_COLOURS[0]);

constexpr uint32_t FAULT_FLASH_MS = 500;
constexpr uint32_t SUPPLY_PULSE_MS = 1500;
constexpr uint32_t STOPPED_BREATH_MS = 3000;
constexpr uint32_t NO_GAMEPAD_BLINK_MS = 1000;
constexpr uint32_t STALE_BLINK_MS = 250;
constexpr float INPUT_SERIAL_LEVEL = 0.4f; // dim white

static Rgb scaled(Rgb c, float level)
{
  if (level < 0)
    level = 0;
  if (level > 1)
    level = 1;
  return Rgb{(uint8_t)(c.r * level), (uint8_t)(c.g * level), (uint8_t)(c.b * level)};
}

// Triangle wave from lo to 1.0 and back over the period.
static float triangle(uint32_t nowMs, uint32_t periodMs, float lo)
{
  float t = (float)(nowMs % periodMs) / (float)periodMs;
  return lo + (1.0f - lo) * (t < 0.5f ? 2 * t : 2 * (1 - t));
}

static bool blinkOn(uint32_t nowMs, uint32_t periodMs)
{
  return (nowMs % periodMs) < periodMs / 2;
}

Rgb gaitColour(int gait)
{
  if (gait < 0 || gait >= GAIT_COLOUR_COUNT)
    return GREEN;
  return GAIT_COLOURS[gait];
}

int loadSegments(float currentA, float tripCurrentA)
{
  if (!(tripCurrentA > 0) || !(currentA > 0))
    return 1;
  int n = (int)ceilf(currentA / tripCurrentA * MAIN_LEDS);
  if (n < 1)
    n = 1;
  if (n > MAIN_LEDS)
    n = MAIN_LEDS;
  return n;
}

static Rgb inputIndicator(Input input, uint32_t nowMs)
{
  switch (input)
  {
  case Input::Serial:
    return scaled(WHITE, INPUT_SERIAL_LEVEL);
  case Input::LinkConnected:
    return CYAN;
  case Input::LinkNoGamepad:
    return blinkOn(nowMs, NO_GAMEPAD_BLINK_MS) ? MAGENTA : OFF;
  case Input::LinkStale:
    return blinkOn(nowMs, STALE_BLINK_MS) ? AMBER : OFF;
  }
  return OFF;
}

static void fill(Rgb out[LED_COUNT], int from, int to, Rgb c)
{
  for (int i = from; i < to; i++)
    out[i] = c;
}

static void applyBrightness(Rgb out[LED_COUNT], float brightness)
{
  for (int i = 0; i < LED_COUNT; i++)
    out[i] = scaled(out[i], brightness);
}

void compute(const State &s, Rgb out[LED_COUNT])
{
  fill(out, 0, LED_COUNT, OFF);

  if (s.faults != 0)
  {
    Rgb c = RED;
    for (const FaultIndication &f : FAULT_COLOURS)
    {
      if (s.faults & f.fault)
      {
        c = f.colour;
        break;
      }
    }
    bool phase = (s.nowMs / FAULT_FLASH_MS) % 2 == 0;
    fill(out, 0, LED_COUNT, phase ? c : RED);
    applyBrightness(out, FAULT_BRIGHTNESS);
    return;
  }

  if (!s.supplyAccepted)
  {
    fill(out, 0, LED_COUNT, scaled(AMBER, triangle(s.nowMs, SUPPLY_PULSE_MS, 0.2f)));
    applyBrightness(out, NORMAL_BRIGHTNESS);
    return;
  }

  if (!s.bodyRunning)
  {
    fill(out, 0, MAIN_LEDS, scaled(BLUE, triangle(s.nowMs, STOPPED_BREATH_MS, 0.1f)));
  }
  else if (s.mode == BodyMode::Walk)
  {
    int lit = loadSegments(s.currentA, s.tripCurrentA);
    // Each segment covers trip/5 of current; one whose midpoint is above the
    // sustained limit is red, so the bar turns red where Sleep would trip.
    float perSegment = s.tripCurrentA / MAIN_LEDS;
    Rgb hue = gaitColour(s.gait);
    for (int i = 0; i < lit; i++)
    {
      float midpoint = perSegment * ((float)i + 0.5f);
      out[i] = midpoint > s.sustainedCurrentA ? RED : hue;
    }
  }
  else
  {
    fill(out, 0, (int)s.mode + 1, GREEN);
  }

  out[INPUT_LED] = inputIndicator(s.input, s.nowMs);
  applyBrightness(out, NORMAL_BRIGHTNESS);
}

} // namespace led_pattern
