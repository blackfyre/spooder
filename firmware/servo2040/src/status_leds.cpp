#include "status_leds.h"

#include "servo2040.hpp"
#include "protection/protection.h"

using namespace plasma;
using namespace servo;

namespace status_leds
{

struct Colour
{
  uint8_t r, g, b;
};

// One colour per fault, alternated with red. Lowest bit wins when several latch.
struct FaultIndication
{
  uint16_t fault;
  Colour colour;
};
static const FaultIndication FAULT_COLOURS[] = {
    {protection::OVERCURRENT, {255, 255, 255}},      // white
    {protection::BROWNOUT, {255, 0, 255}},           // magenta
    {protection::OVERVOLTAGE, {0, 0, 255}},          // blue
    {protection::SUSTAINED_CURRENT, {255, 96, 0}},   // orange
    {protection::UNDERVOLTAGE, {255, 255, 0}},       // yellow
};
static const Colour FAULT_ALTERNATE = {255, 0, 0}; // red
static const Colour WAITING = {255, 128, 0};        // amber, pulsing
static const Colour HEALTHY = {0, 255, 0};          // green; lit LEDs show the mode
static const Colour UNKNOWN_FAULT = {255, 0, 0};

constexpr float BRIGHTNESS = 0.25f;
constexpr uint32_t FLASH_PERIOD_MS = 500;
constexpr uint32_t PULSE_PERIOD_MS = 1500;
constexpr uint UPDATE_FPS = 10;

// Six LEDs on PIO 1, state machine 0 (PIO 0 drives the servos).
static WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

static void fill(Colour c, float level, uint count)
{
  for (uint i = 0; i < servo2040::NUM_LEDS; i++)
  {
    float k = i < count ? level * BRIGHTNESS : 0.0f;
    leds.set_rgb(i, (uint8_t)(c.r * k), (uint8_t)(c.g * k), (uint8_t)(c.b * k));
  }
}

void setup()
{
  leds.start(UPDATE_FPS);
  fill(WAITING, 1.0f, servo2040::NUM_LEDS);
}

void show(uint32_t nowMs, bool supplyAccepted, uint16_t faults, body::Mode mode)
{
  if (faults != 0)
  {
    Colour c = UNKNOWN_FAULT;
    for (const FaultIndication &f : FAULT_COLOURS)
    {
      if (faults & f.fault)
      {
        c = f.colour;
        break;
      }
    }
    bool phase = (nowMs / FLASH_PERIOD_MS) % 2 == 0;
    fill(phase ? c : FAULT_ALTERNATE, 1.0f, servo2040::NUM_LEDS);
    return;
  }

  if (!supplyAccepted)
  {
    // Triangle wave 0.2..1.0 over the pulse period.
    float t = (float)(nowMs % PULSE_PERIOD_MS) / (float)PULSE_PERIOD_MS;
    float level = 0.2f + 0.8f * (t < 0.5f ? 2 * t : 2 * (1 - t));
    fill(WAITING, level, servo2040::NUM_LEDS);
    return;
  }

  // Healthy: one more lit LED per mode (Initialise=1 ... Calibrate=5).
  fill(HEALTHY, 1.0f, (uint)mode + 1);
}

} // namespace status_leds
