#include "status_leds.h"

#include "servo2040.hpp"

using namespace plasma;
using namespace servo;

namespace status_leds
{

constexpr uint UPDATE_FPS = 10;
static_assert(led_pattern::LED_COUNT == servo2040::NUM_LEDS, "one pattern entry per LED");

// Six LEDs on PIO 1, state machine 0 (PIO 0 drives the servos).
static WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

void setup()
{
  leds.start(UPDATE_FPS);
}

void show(const led_pattern::State &state)
{
  led_pattern::Rgb colours[led_pattern::LED_COUNT];
  led_pattern::compute(state, colours);
  // Brightness is already applied by the pattern, so no gamma here.
  for (uint i = 0; i < servo2040::NUM_LEDS; i++)
    leds.set_rgb(i, colours[i].r, colours[i].g, colours[i].b, 0, false);
}

} // namespace status_leds
