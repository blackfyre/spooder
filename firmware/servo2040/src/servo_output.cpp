#include "servo_output.h"

#include "hardware/gpio.h"
#include "pico/time.h"
#include "servo2040.hpp"

using namespace servo;

namespace servo_output
{

constexpr int OUTPUT_COUNT = kinematics::LEG_COUNT * kinematics::JOINTS_PER_LEG;
static_assert(OUTPUT_COUNT == servo2040::NUM_SERVOS, "one output per joint");
static_assert(OUTPUT_COUNT == joint_calibration::JOINT_COUNT, "one calibration per output");
// ServoCluster's hard pulse limits (Calibration::LOWER_HARD_LIMIT and
// UPPER_HARD_LIMIT, private in pimoroni-pico v1.29).
constexpr float CLUSTER_MIN_PULSE_US = 400;
constexpr float CLUSTER_MAX_PULSE_US = 2600;
static_assert(joint_calibration::MIN_PULSE_US >= CLUSTER_MIN_PULSE_US &&
                  joint_calibration::MAX_PULSE_US <= CLUSTER_MAX_PULSE_US,
              "every calibrated pulse is one ServoCluster can produce");
// Longest pulse ServoCluster can produce (its hard limit) plus margin: within
// this time every pulse in progress has finished.
constexpr uint64_t MAX_PULSE_WAIT_US = 2700;
// After switch-off, the zero-pulse sequence is guaranteed to be active after one
// 20 ms frame; only then may the pin override be released.
constexpr uint32_t RELEASE_AFTER_MS = 25;

static ServoCluster cluster(pio0, 0, servo2040::SERVO_1, OUTPUT_COUNT);

// Last pulse the body asked for, per output; 0 = never set. Kept while output
// is off, so switching on restores the commanded pose instead of ServoCluster's
// default mid-position.
static uint16_t desiredUs[OUTPUT_COUNT];
// Legs whose angles the body has solved at least once since power-up.
static bool legSet[kinematics::LEG_COUNT];
// Legs switched on since output last switched on (switch-on is in groups).
static bool legOn[kinematics::LEG_COUNT];
static bool on = false;
static bool inhibited = true;
// Pins are forced low through the GPIO output override while output is off.
static bool pinsForcedLow = false;
static absolute_time_t switchedOffAt;

bool setup()
{
  if (!cluster.init())
    return false;
  // Pulses come from joint_calibration, already within 400-2600 µs. Each
  // channel's default calibration would clamp them to 500-2500 µs; keep only
  // ServoCluster's hard limits.
  for (int i = 0; i < OUTPUT_COUNT; i++)
    cluster.calibration(i).limit_to_calibration(false, false);
  cluster.disable_all(); // no pulses until the body and protection allow them
  on = false;
  return true;
}

void setInhibited(bool value)
{
  inhibited = value;
}

// Stops every output immediately, independent of ServoCluster's frame timing
// (a disable only takes effect at the next 20 ms sequence boundary). Each pin
// is forced low only once it is low, so no pulse is cut short: a truncated
// pulse would itself be a valid-looking position command.
static void forcePinsLow()
{
  // Sweep: force each pin as soon as it is low. A forced pin can't start a new
  // pulse, and every pulse in progress ends within MAX_PULSE_WAIT_US, so the
  // whole cut-off is bounded by one pulse length, not one per pin.
  bool forced[OUTPUT_COUNT] = {};
  int remaining = OUTPUT_COUNT;
  absolute_time_t deadline = make_timeout_time_us(MAX_PULSE_WAIT_US);
  while (remaining > 0)
  {
    bool late = time_reached(deadline);
    for (int i = 0; i < OUTPUT_COUNT; i++)
    {
      uint pin = servo2040::SERVO_1 + i;
      if (forced[i] || (gpio_get(pin) && !late))
        continue;
      gpio_set_outover(pin, GPIO_OVERRIDE_LOW);
      forced[i] = true;
      remaining--;
    }
  }
  pinsForcedLow = true;
}

// Called only after the cluster has been loaded with zero pulses at least one
// frame earlier, so the PIO is already driving the pins low.
static void releasePins()
{
  for (uint pin = servo2040::SERVO_1; pin < servo2040::SERVO_1 + OUTPUT_COUNT; pin++)
    gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
  pinsForcedLow = false;
}

static void switchOff()
{
  forcePinsLow();
  cluster.disable_all(false);
  cluster.load();
  on = false;
  for (bool &l : legOn)
    l = false;
  switchedOffAt = get_absolute_time();
}

void apply(const body::BodyState &body, const joint_calibration::Table &calibration, int legsOn)
{
  bool changed[OUTPUT_COUNT] = {};
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    if (body.legUpdated[leg])
      legSet[leg] = true;
    if (!legSet[leg])
      continue;
    const kinematics::JointAngles &a = body.angles[leg];
    const float angles[kinematics::JOINTS_PER_LEG] = {a.coxa, a.femur, a.tibia};
    for (int j = 0; j < kinematics::JOINTS_PER_LEG; j++)
    {
      int i = kinematics::servoOutputIndex(leg, j);
      uint16_t us = joint_calibration::toPulse(calibration.joints[i], angles[j]);
      if (us != desiredUs[i])
      {
        desiredUs[i] = us;
        changed[i] = true;
      }
    }
  }

  bool wantOn = !inhibited && body.outputsEnabled;
  if (!wantOn)
  {
    if (on)
      switchOff();
    return;
  }

  if (pinsForcedLow)
  {
    if (absolute_time_diff_us(switchedOffAt, get_absolute_time()) < (int64_t)RELEASE_AFTER_MS * 1000)
      return; // stay off until the zero-pulse sequence is certainly active
    releasePins();
  }

  // pulse() also enables a disabled output, so this both switches on and
  // updates. Outputs the body has never set, and legs not yet released,
  // stay disabled rather than going to ServoCluster's mid-position.
  bool any = false;
  for (int i = 0; i < OUTPUT_COUNT; i++)
  {
    int leg = i / kinematics::JOINTS_PER_LEG;
    if (desiredUs[i] == 0 || leg >= legsOn || (legOn[leg] && !changed[i]))
      continue;
    cluster.pulse(i, desiredUs[i], false);
    any = true;
  }
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
    legOn[leg] = leg < legsOn;
  on = true;
  if (any)
    cluster.load();
}

void stop()
{
  if (on)
    switchOff();
}

bool outputsOn()
{
  return on;
}

const uint16_t *pulses()
{
  return desiredUs;
}

} // namespace servo_output
