#include <cstdio>
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "movement/body.h"

using namespace plasma;
using namespace servo;

// Six on-board RGB LEDs on PIO 1, state machine 0 (PIO 0 is reserved for the servos).
static WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

int main()
{
  stdio_init_all();
  leds.start();

  // Exercise the movement core so it is linked and measured on the target.
  static body::BodyState state;
  body::init(state);
  MovementCommand cmd;

  bool on = false;
  while (true)
  {
    on = !on;
    for (uint i = 0; i < servo2040::NUM_LEDS; i++)
      leds.set_rgb(i, 0, on ? 32 : 0, 0);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    cmd.issuedAtMs = now;
    uint64_t t0 = time_us_64();
    body::step(state, cmd, now);
    uint32_t stepUs = (uint32_t)(time_us_64() - t0);
    printf("spooder: alive, mode %d, step %lu us\n", (int)state.mode, (unsigned long)stepUs);
    sleep_ms(500);
  }
}
