#include "power_monitor.h"

#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"

using namespace pimoroni;
using namespace servo;

namespace power_monitor
{

constexpr int SAMPLES = 8;

// Both quantities are read through the shared ADC behind the multiplexer.
static AnalogMux mux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
                     PIN_UNUSED, servo2040::SHARED_ADC);
static Analog currentAdc(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                         servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);
static Analog voltageAdc(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);

void setup()
{
  // Analog constructors initialise the ADC; select a known channel to start.
  mux.select(servo2040::VOLTAGE_SENSE_ADDR);
}

protection::Reading read()
{
  protection::Reading r = {0, 0};

  mux.select(servo2040::CURRENT_SENSE_ADDR); // includes a 10 us settle
  for (int i = 0; i < SAMPLES; i++)
    r.currentA += currentAdc.read_current();
  r.currentA /= SAMPLES;

  mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  for (int i = 0; i < SAMPLES; i++)
    r.voltageV += voltageAdc.read_voltage();
  r.voltageV /= SAMPLES;

  return r;
}

} // namespace power_monitor
