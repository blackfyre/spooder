#include "bench_sim.h"

namespace bench_sim
{

using robot_tick::BenchEvent;

static led_pattern::Bench indication(const Entry &e)
{
  switch (e.phase)
  {
  case Phase::Window:
    return led_pattern::Bench::Available;
  case Phase::Refused:
    return e.refusal == BenchEvent::RefusedSupply ? led_pattern::Bench::RefusedSupply : led_pattern::Bench::RefusedUsb;
  case Phase::Simulating:
    return led_pattern::Bench::Simulating;
  case Phase::Live:
    break;
  }
  return led_pattern::Bench::None;
}

Result step(Entry &e, const Sample &sample)
{
  if (!e.started)
  {
    e.started = true;
    e.startMs = sample.nowMs;
  }
  // Every tick counts, the press tick's included.
  if (!(sample.measuredSupplyV < ABSENT_SUPPLY_V))
    e.supplyAbsent = false;

  BenchEvent event = BenchEvent::None;
  switch (e.phase)
  {
  case Phase::Window:
    if (sample.nowMs - e.startMs >= WINDOW_MS || sample.realOutputOn)
    {
      e.phase = Phase::Live;
    }
    else if (sample.buttonPressed && e.seenReleased)
    {
      // The first press edge decides, once per boot.
      e.pressMs = sample.nowMs;
      if (!e.supplyAbsent)
        event = BenchEvent::RefusedSupply;
      else if (!sample.usbHost)
        event = BenchEvent::RefusedUsb;
      else
        event = BenchEvent::Entered;
      if (event == BenchEvent::Entered)
      {
        e.phase = Phase::Simulating;
      }
      else
      {
        e.phase = Phase::Refused;
        e.refusal = event;
      }
    }
    else if (!sample.buttonPressed)
    {
      e.seenReleased = true;
    }
    break;
  case Phase::Refused:
    if (sample.nowMs - e.pressMs >= REFUSED_SHOW_MS)
      e.phase = Phase::Live;
    break;
  case Phase::Simulating:
    if (sample.measuredSupplyV > sample.brownoutV)
      e.supplyFault = true;
    break;
  case Phase::Live:
    break;
  }

  Result r = {};
  r.phase = e.phase;
  r.bench.simulated = e.phase == Phase::Simulating;
  r.bench.supplyFault = e.supplyFault;
  r.bench.indication = indication(e);
  r.bench.event = event;
  return r;
}

void BenchBoard::setInhibited(bool inhibited)
{
  if (simulated_)
  {
    realSetInhibited(true);
    model_.inhibited = inhibited;
    return;
  }
  realSetInhibited(inhibited);
}

void BenchBoard::stopServos()
{
  realStop();
  if (simulated_)
    model_.stop();
}

void BenchBoard::applyServos(const body::BodyState &body, const joint_calibration::Table &applied, int legsOn)
{
  if (simulated_)
  {
    model_.apply(body, applied, legsOn);
    return;
  }
  realApply(body, applied, legsOn);
}

bool BenchBoard::servosOn()
{
  return simulated_ ? model_.on : realOn();
}

const uint16_t *BenchBoard::servoPulses()
{
  return simulated_ ? model_.desiredUs : realPulses();
}

void BenchBoard::enterSimulation()
{
  if (simulated_)
    return;
  simulated_ = true; // first, so no later call reaches the real port's output
  realForceOff();
}

} // namespace bench_sim
