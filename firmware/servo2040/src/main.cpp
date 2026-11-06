#include <cstdio>
#include "hardware/gpio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "tusb.h"

#include "config.h"
#include "identity/build_identity.h"
#include "power_monitor.h"
#include "robot/bench_model.h"
#include "robot/bench_sim.h"
#include "robot/robot_tick.h"
#include "servo_output.h"
#include "status_leds.h"
#ifdef SPOODER_SERIAL_COMMANDS
#include "serial_commands.h"
#else
#include "link.h"
#include "usb_link.h"
#endif

// The control loop's logic is core/robot/robot_tick, shared with the virtual
// robot (tools/virtual-robot). This file reads the hardware into its inputs,
// drives the hardware from it, and keeps the 20 ms period.

static robot_tick::RobotState robot;
// Bench simulation's entry decision: in RAM, zeroed at every start-up.
static bench_sim::Entry benchEntry;
// The modelled supply in simulation: a steady 6 V at load 1.
static const bench_model::Supply benchSupply;

// The tick's view of the board. The servo calls are BenchBoard's (final):
// they reach servo_output through the hooks below until enterSimulation(),
// and the modelled servos from then on.
class Board final : public bench_sim::BenchBoard
{
public:
  void showLeds(const led_pattern::State &state) override { status_leds::show(state); }
  bool sendFrame(robot_tick::Link link, const uint8_t *frame, int length) override
  {
#ifdef SPOODER_SERIAL_COMMANDS
    (void)link;
    (void)frame;
    (void)length;
    return false; // no links in bring-up builds (the tick sends nothing)
#else
    return link == robot_tick::Link::Bridge ? link::sendFrame(frame, length) : usb_link::sendFrame(frame, length);
#endif
  }
  void log(const char *line) override { printf("%s", line); }

protected:
  void realSetInhibited(bool inhibited) override { servo_output::setInhibited(inhibited); }
  void realStop() override { servo_output::stop(); }
  void realApply(const body::BodyState &body, const joint_calibration::Table &applied, int legsOn) override
  {
    servo_output::apply(body, applied, legsOn);
  }
  bool realOn() override { return servo_output::outputsOn(); }
  const uint16_t *realPulses() override { return servo_output::pulses(); }
  void realForceOff() override { servo_output::forceOff(); }
};

static Board board;

static uint32_t nowMs()
{
  return to_ms_since_boot(get_absolute_time());
}

// The USER button (GPIO23, active low). It is also the BOOT switch: held
// through reset it enters BOOTSEL, so the firmware only sees later presses.
static void setupUserButton()
{
  gpio_init(servo::servo2040::USER_SW);
  gpio_set_dir(servo::servo2040::USER_SW, GPIO_IN);
  gpio_pull_up(servo::servo2040::USER_SW);
}

int main()
{
  stdio_init_all();
  setupUserButton();
  status_leds::setup();
  power_monitor::setup();
#ifdef SPOODER_SERIAL_COMMANDS
  constexpr robot_tick::InputMode INPUT_MODE = robot_tick::InputMode::Serial;
#else
  link::setup();
  constexpr robot_tick::InputMode INPUT_MODE = robot_tick::InputMode::Links;
#endif

  if (!servo_output::setup())
  {
    // Without the servo cluster nothing may move; report and stay here.
    while (true)
    {
      printf("servo output failed to initialise\n");
      led_pattern::State failed = {};
      failed.nowMs = nowMs();
      failed.faults = protection::EMERGENCY_STOP; // steady red, as a fault: nothing may move
      status_leds::show(failed);
      sleep_ms(robot_tick::STATUS_PERIOD_MS);
    }
  }

  robot_tick::init(robot, INPUT_MODE, config::POWER_LIMITS, nowMs(), build_identity::ID);
  absolute_time_t next = get_absolute_time();
  uint32_t previousTickUs = 0;

  while (true)
  {
    uint32_t now = nowMs();
    robot_tick::Inputs in = {};
    // Sampled first: the build banner on each connect, which the serial-keys
    // build's command echo must not precede.
    in.usbConnected = stdio_usb_connected();

    // Input first, so an emergency stop in a frame that has arrived acts in
    // this tick. Both input sources are bounded and never block, so they
    // cannot delay the protection step that follows.
#ifdef SPOODER_SERIAL_COMMANDS
    // Bounded; on a tick that logs the build banner the keys wait one tick,
    // so their echo follows the banner.
    serial_commands::Input s = serial_commands::poll(now, !robot_tick::bannerDue(robot, in.usbConnected));
    in.serial.command = s.command;
    in.serial.requestSeq = s.requestSeq;
    in.serial.clearFaults = s.clearFaults;
    in.serial.printReadings = s.printReadings;
#else
    static uint8_t bridgeBytes[link::RX_BUFFER_SIZE];
    static uint8_t usbBytes[usb_link::MAX_BYTES_PER_POLL];
    in.byteCount[(int)robot_tick::Link::Bridge] = link::read(bridgeBytes, link::RX_BUFFER_SIZE);
    in.bytes[(int)robot_tick::Link::Bridge] = bridgeBytes;
    in.byteCount[(int)robot_tick::Link::Usb] = usb_link::read(usbBytes, usb_link::MAX_BYTES_PER_POLL);
    in.bytes[(int)robot_tick::Link::Usb] = usbBytes;
#endif
    protection::Reading measured = power_monitor::read();
    in.previousTickUs = previousTickUs;

    // The working time covers the bench simulation step and the model's
    // reading, as well as the tick.
    uint32_t tickStart = time_us_32();
    bench_sim::Sample sample = {};
    sample.nowMs = now;
    sample.buttonPressed = !gpio_get(servo::servo2040::USER_SW);
    sample.measuredSupplyV = measured.voltageV;
    sample.realOutputOn = servo_output::outputsOn();
    // Enumerated by a host and not suspended: a charger or a bare cable is
    // not a host (VBUS sense is forced present on this board).
    sample.usbHost = tud_mounted() && !tud_suspended();
    sample.brownoutV = config::POWER_LIMITS.brownoutV;
    bench_sim::Result bench = bench_sim::step(benchEntry, sample);
    if (bench.phase == bench_sim::Phase::Simulating)
      board.enterSimulation(); // once: later calls do nothing
    in.bench = bench.bench;
    // In simulation the tick sees the modelled supply; the measured one only
    // feeds bench_sim (the supply fault).
    in.power = board.simulated() ? benchSupply.read(board.model()) : measured;

    robot_tick::tick(robot, in, now, board);
    previousTickUs = time_us_32() - tickStart;

    next = delayed_by_ms(next, robot_tick::LOOP_PERIOD_MS);
    if (absolute_time_diff_us(get_absolute_time(), next) > 0)
      sleep_until(next);
    else
      next = get_absolute_time(); // overran: start the next period now
  }
}
