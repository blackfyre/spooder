#include <cstdio>
#include "pico/stdlib.h"

#include "config.h"
#include "power_monitor.h"
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

// The tick's view of the board.
class Board final : public robot_tick::Hardware
{
public:
  void setInhibited(bool inhibited) override { servo_output::setInhibited(inhibited); }
  void stopServos() override { servo_output::stop(); }
  void applyServos(const body::BodyState &body, const joint_calibration::Table &applied) override
  {
    servo_output::apply(body, applied);
  }
  bool servosOn() override { return servo_output::outputsOn(); }
  const uint16_t *servoPulses() override { return servo_output::pulses(); }
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
};

static Board board;

static uint32_t nowMs()
{
  return to_ms_since_boot(get_absolute_time());
}

int main()
{
  stdio_init_all();
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
      failed.faults = 0xFFFF; // shown as a fault: nothing may move
      status_leds::show(failed);
      sleep_ms(robot_tick::STATUS_PERIOD_MS);
    }
  }

  robot_tick::init(robot, INPUT_MODE, config::POWER_LIMITS, nowMs());
  absolute_time_t next = get_absolute_time();

  while (true)
  {
    uint32_t now = nowMs();
    robot_tick::Inputs in = {};

    // Input first, so an emergency stop in a frame that has arrived acts in
    // this tick. Both input sources are bounded and never block, so they
    // cannot delay the protection step that follows.
#ifdef SPOODER_SERIAL_COMMANDS
    serial_commands::Input s = serial_commands::poll(now); // bounded
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
    in.power = power_monitor::read();

    robot_tick::tick(robot, in, now, board);

    next = delayed_by_ms(next, robot_tick::LOOP_PERIOD_MS);
    if (absolute_time_diff_us(get_absolute_time(), next) > 0)
      sleep_until(next);
    else
      next = get_absolute_time(); // overran: start the next period now
  }
}
