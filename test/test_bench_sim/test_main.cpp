// Bench simulation (core/robot/bench_sim): the entry decision per tick, and
// the board boundary that keeps every real servo call off once simulating.
#include <unity.h>
#include <string>
#include <vector>
#include "robot/bench_sim.h"

using namespace bench_sim;
using robot_tick::BenchEvent;

static const float BROWNOUT_V = 4.8f;

// --- entry -------------------------------------------------------------------------

static Entry entry;
static Sample sample;
static Result result;

// Boot at t = 0 on the first step; each step is one 20 ms tick.
static void resetEntry()
{
  entry = Entry();
  sample = {};
  sample.brownoutV = BROWNOUT_V;
  sample.usbHost = true;
  sample.measuredSupplyV = 0.05f;
}

static Result stepAt(uint32_t t)
{
  sample.nowMs = t;
  result = step(entry, sample);
  return result;
}

// Released ticks from @p from up to (not including) @p to.
static void releasedUntil(uint32_t from, uint32_t to)
{
  sample.buttonPressed = false;
  for (uint32_t t = from; t < to; t += 20)
    stepAt(t);
}

// Released from boot until @p pressMs, then a pressed tick there.
static Result pressAt(uint32_t pressMs)
{
  releasedUntil(0, pressMs);
  sample.buttonPressed = true;
  return stepAt(pressMs);
}

void setUp()
{
  resetEntry();
}
void tearDown() {}

void test_window_is_offered_from_the_first_tick_in_live_mode()
{
  Result r = stepAt(0);
  TEST_ASSERT_EQUAL((int)Phase::Window, (int)r.phase);
  TEST_ASSERT_FALSE(r.bench.simulated);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::Available, (int)r.bench.indication);
  releasedUntil(20, 3000);
  TEST_ASSERT_EQUAL((int)Phase::Window, (int)result.phase);
  stepAt(3000);
  TEST_ASSERT_EQUAL((int)Phase::Live, (int)result.phase);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::None, (int)result.bench.indication);
  TEST_ASSERT_FALSE(result.bench.simulated);
}

void test_window_is_measured_from_the_first_tick_not_zero()
{
  releasedUntil(5000, 7980);
  sample.buttonPressed = true;
  stepAt(7980); // 2,980 ms after boot
  TEST_ASSERT_EQUAL((int)Phase::Simulating, (int)result.phase);
}

void test_press_at_2980_ms_enters()
{
  Result r = pressAt(2980);
  TEST_ASSERT_EQUAL((int)Phase::Simulating, (int)r.phase);
  TEST_ASSERT_TRUE(r.bench.simulated);
  TEST_ASSERT_EQUAL((int)BenchEvent::Entered, (int)r.bench.event);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::Simulating, (int)r.bench.indication);
  sample.buttonPressed = false;
  Result next = stepAt(3000);
  TEST_ASSERT_EQUAL((int)BenchEvent::None, (int)next.bench.event); // reported once
  TEST_ASSERT_TRUE(next.bench.simulated);
}

void test_press_at_3000_ms_changes_nothing()
{
  Result r = pressAt(3000);
  TEST_ASSERT_EQUAL((int)Phase::Live, (int)r.phase);
  TEST_ASSERT_EQUAL((int)BenchEvent::None, (int)r.bench.event);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::None, (int)r.bench.indication);
}

void test_button_held_from_the_first_tick_never_enters()
{
  sample.buttonPressed = true;
  for (uint32_t t = 0; t <= 3500; t += 20)
  {
    stepAt(t);
    TEST_ASSERT_FALSE(result.bench.simulated);
    TEST_ASSERT_EQUAL((int)BenchEvent::None, (int)result.bench.event);
  }
  TEST_ASSERT_EQUAL((int)Phase::Live, (int)result.phase);
}

void test_button_held_then_released_then_pressed_enters()
{
  sample.buttonPressed = true;
  for (uint32_t t = 0; t < 1000; t += 20)
    stepAt(t);
  TEST_ASSERT_EQUAL((int)Phase::Window, (int)result.phase);
  releasedUntil(1000, 1500);
  sample.buttonPressed = true;
  stepAt(1500);
  TEST_ASSERT_EQUAL((int)Phase::Simulating, (int)result.phase);
}

void test_supply_of_0_99_v_on_one_tick_before_the_press_enters()
{
  releasedUntil(0, 500);
  sample.measuredSupplyV = 0.99f;
  stepAt(500);
  sample.measuredSupplyV = 0.05f;
  releasedUntil(520, 1000);
  sample.buttonPressed = true;
  stepAt(1000);
  TEST_ASSERT_EQUAL((int)Phase::Simulating, (int)result.phase);
}

void test_supply_of_1_0_v_on_one_tick_before_the_press_refuses()
{
  releasedUntil(0, 500);
  sample.measuredSupplyV = 1.0f;
  stepAt(500);
  sample.measuredSupplyV = 0.05f;
  releasedUntil(520, 1000);
  sample.buttonPressed = true;
  stepAt(1000);
  TEST_ASSERT_EQUAL((int)Phase::Refused, (int)result.phase);
  TEST_ASSERT_EQUAL((int)BenchEvent::RefusedSupply, (int)result.bench.event);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::RefusedSupply, (int)result.bench.indication);
  TEST_ASSERT_FALSE(result.bench.simulated);
}

void test_supply_on_the_press_tick_refuses()
{
  releasedUntil(0, 1000);
  sample.measuredSupplyV = 5.0f; // the servo rail fed from USB
  sample.buttonPressed = true;
  stepAt(1000);
  TEST_ASSERT_EQUAL((int)BenchEvent::RefusedSupply, (int)result.bench.event);
}

void test_output_on_at_1_s_closes_the_window()
{
  releasedUntil(0, 1000);
  sample.realOutputOn = true;
  stepAt(1000);
  TEST_ASSERT_EQUAL((int)Phase::Live, (int)result.phase);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::None, (int)result.bench.indication);
  sample.realOutputOn = false;
  releasedUntil(1020, 1500);
  sample.buttonPressed = true;
  stepAt(1500);
  TEST_ASSERT_EQUAL((int)Phase::Live, (int)result.phase);
  TEST_ASSERT_EQUAL((int)BenchEvent::None, (int)result.bench.event);
}

void test_no_usb_host_refuses()
{
  sample.usbHost = false; // not mounted, or mounted but suspended
  pressAt(1000);
  TEST_ASSERT_EQUAL((int)Phase::Refused, (int)result.phase);
  TEST_ASSERT_EQUAL((int)BenchEvent::RefusedUsb, (int)result.bench.event);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::RefusedUsb, (int)result.bench.indication);
}

void test_usb_host_is_read_only_at_the_press()
{
  sample.usbHost = false;
  releasedUntil(0, 980); // no host until the press
  sample.usbHost = true;
  sample.buttonPressed = true;
  stepAt(980);
  TEST_ASSERT_EQUAL((int)Phase::Simulating, (int)result.phase);
}

void test_both_gates_failing_shows_the_supply_reason()
{
  sample.usbHost = false;
  sample.measuredSupplyV = 6.0f;
  pressAt(1000);
  TEST_ASSERT_EQUAL((int)BenchEvent::RefusedSupply, (int)result.bench.event);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::RefusedSupply, (int)result.bench.indication);
}

void test_second_press_after_a_refusal_does_nothing()
{
  sample.usbHost = false;
  pressAt(1000);
  sample.usbHost = true; // the host has now enumerated
  releasedUntil(1020, 1500);
  sample.buttonPressed = true;
  stepAt(1500);
  TEST_ASSERT_EQUAL((int)Phase::Refused, (int)result.phase);
  TEST_ASSERT_EQUAL((int)BenchEvent::None, (int)result.bench.event);
  TEST_ASSERT_FALSE(result.bench.simulated);
}

// The refusal is shown until exactly 3,000 ms after the press, then nothing.
static void checkRefusalShownFor3s(uint32_t pressMs)
{
  resetEntry();
  sample.usbHost = false;
  pressAt(pressMs);
  sample.buttonPressed = false;
  for (uint32_t t = pressMs + 20; t < pressMs + 3000; t += 20)
  {
    stepAt(t);
    TEST_ASSERT_EQUAL((int)led_pattern::Bench::RefusedUsb, (int)result.bench.indication);
  }
  stepAt(pressMs + 3000);
  TEST_ASSERT_EQUAL((int)Phase::Live, (int)result.phase);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::None, (int)result.bench.indication);
  stepAt(pressMs + 5000);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::None, (int)result.bench.indication);
}

void test_refusal_shown_for_3_s_after_a_press_at_1_s()
{
  checkRefusalShownFor3s(1000);
}

void test_refusal_shown_for_3_s_after_a_press_at_2980_ms_outlasting_the_window()
{
  checkRefusalShownFor3s(2980);
}

void test_simulation_lasts_and_ignores_a_lost_usb_host()
{
  pressAt(1000);
  sample.buttonPressed = false;
  sample.usbHost = false;
  for (uint32_t t = 1020; t < 60000; t += 20)
    stepAt(t);
  TEST_ASSERT_TRUE(result.bench.simulated);
  TEST_ASSERT_EQUAL((int)Phase::Simulating, (int)result.phase);
  TEST_ASSERT_FALSE(result.bench.supplyFault);
}

void test_supply_fault_latches_above_the_brownout_voltage()
{
  pressAt(1000);
  sample.buttonPressed = false;
  sample.measuredSupplyV = 4.8f; // at the limit: not above it
  stepAt(1020);
  TEST_ASSERT_FALSE(result.bench.supplyFault);
  sample.measuredSupplyV = 4.9f;
  stepAt(1040);
  TEST_ASSERT_TRUE(result.bench.supplyFault);
  sample.measuredSupplyV = 0;
  stepAt(1060); // until restart
  TEST_ASSERT_TRUE(result.bench.supplyFault);
  TEST_ASSERT_TRUE(result.bench.simulated); // still in simulation
}

void test_supply_fault_never_latches_in_live_mode()
{
  releasedUntil(0, 3000);
  sample.measuredSupplyV = 6.0f;
  stepAt(3000);
  stepAt(3020);
  TEST_ASSERT_FALSE(result.bench.supplyFault);
}

void test_restart_after_simulation_starts_in_the_window_in_live_mode()
{
  pressAt(1000);
  TEST_ASSERT_TRUE(result.bench.simulated);
  entry = Entry(); // the restart: a fresh Entry
  sample.buttonPressed = false;
  stepAt(0);
  TEST_ASSERT_EQUAL((int)Phase::Window, (int)result.phase);
  TEST_ASSERT_FALSE(result.bench.simulated);
  TEST_ASSERT_EQUAL((int)led_pattern::Bench::Available, (int)result.bench.indication);
}

// --- the board boundary ------------------------------------------------------------

static const protection::Limits LIMITS = {9.0f, 60, 13.0f, 6.0f, 2000, 5.2f, 200, 4.8f, 6.8f};

// Records every hook call; after entry, a real apply or allow fails the test.
class HookBoard final : public BenchBoard
{
public:
  std::vector<std::string> hooks;
  int forceOffs = 0;
  bool realInhibited = true;
  bool realIsOn = false;
  uint16_t pulses[joint_calibration::JOINT_COUNT] = {};
  std::vector<protocol::Frame> usbFrames;
  protocol::Decoder decoder;

  void showLeds(const led_pattern::State &) override {}
  bool sendFrame(robot_tick::Link link, const uint8_t *frame, int n) override
  {
    if (link != robot_tick::Link::Usb)
      return true;
    for (int i = 0; i < n; i++)
    {
      decoder.push(frame[i]);
      protocol::Frame f;
      while (decoder.next(f))
        usbFrames.push_back(f);
    }
    return true;
  }
  void log(const char *) override {}

protected:
  void realSetInhibited(bool inhibited) override
  {
    hooks.push_back(inhibited ? "inhibit" : "allow");
    if (simulated() && !inhibited)
      TEST_FAIL_MESSAGE("realSetInhibited(false) after entry");
    realInhibited = inhibited;
    if (inhibited)
      realIsOn = false;
  }
  void realStop() override
  {
    hooks.push_back("stop");
    realIsOn = false;
  }
  void realApply(const body::BodyState &body, const joint_calibration::Table &, int) override
  {
    hooks.push_back("apply");
    if (simulated())
      TEST_FAIL_MESSAGE("realApply after entry");
    realIsOn = !realInhibited && body.outputsEnabled;
    pulses[0] = 1500;
  }
  bool realOn() override
  {
    hooks.push_back("on");
    return realIsOn;
  }
  const uint16_t *realPulses() override
  {
    hooks.push_back("pulses");
    return pulses;
  }
  void realForceOff() override
  {
    hooks.push_back("forceOff");
    forceOffs++;
    // The flag is set first: nothing after this reaches real output.
    TEST_ASSERT_TRUE(simulated());
    realIsOn = false;
  }
};

static robot_tick::RobotState robot;
static HookBoard hookBoard;
static uint32_t tickMs;
static uint8_t usbSeq;
static protocol::Command cmd;
static std::vector<uint8_t> extra;
static BenchEvent pendingEvent; // reported on the next tick, as bench_sim does

static void resetRobot()
{
  robot = robot_tick::RobotState();
  hookBoard = HookBoard();
  tickMs = 1000;
  usbSeq = 0;
  cmd = {};
  cmd.flags = protocol::FLAG_CONTROLLER_CONNECTED;
  cmd.speed = 44;
  extra.clear();
  pendingEvent = BenchEvent::None;
  robot_tick::init(robot, robot_tick::InputMode::Links, LIMITS, tickMs);
}

// One tick with the USB link's command; @p power is the measured supply, or,
// once simulated, replaced by the model's reading (as the firmware does).
static void robotTick(protection::Reading power, bool simulated)
{
  std::vector<uint8_t> bytes;
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCommand(cmd, usbSeq++, frame);
  bytes.insert(bytes.end(), frame, frame + n);
  bytes.insert(bytes.end(), extra.begin(), extra.end());
  extra.clear();
  robot_tick::Inputs in = {};
  in.bytes[(int)robot_tick::Link::Usb] = bytes.data();
  in.byteCount[(int)robot_tick::Link::Usb] = (int)bytes.size();
  in.power = simulated ? bench_model::Supply().read(hookBoard.model()) : power;
  in.bench.simulated = simulated;
  in.bench.event = pendingEvent;
  pendingEvent = BenchEvent::None;
  tickMs += robot_tick::LOOP_PERIOD_MS;
  robot_tick::tick(robot, in, tickMs, hookBoard);
}

static void simTicks(int count)
{
  for (int i = 0; i < count; i++)
    robotTick({0, 0}, true);
}

static void requestMode(RequestedMode m)
{
  cmd.mode = (uint8_t)m;
  cmd.modeRequestCounter++;
}

static int count(const std::vector<std::string> &v, const char *what)
{
  int n = 0;
  for (const std::string &s : v)
    n += s == what;
  return n;
}

void test_before_entry_every_call_reaches_the_hooks()
{
  resetRobot();
  robotTick({0.5f, 6.0f}, false); // baselines
  requestMode(RequestedMode::Stand);
  for (int i = 0; i < 100; i++)
    robotTick({0.5f, 6.0f}, false);
  TEST_ASSERT_TRUE(robot.supervisor.bodyRunning);
  TEST_ASSERT_TRUE(hookBoard.realIsOn);
  TEST_ASSERT_TRUE(hookBoard.servosOn());
  TEST_ASSERT_EQUAL_PTR(hookBoard.pulses, hookBoard.servoPulses());
  TEST_ASSERT_TRUE(count(hookBoard.hooks, "apply") > 0);
  TEST_ASSERT_TRUE(count(hookBoard.hooks, "allow") > 0);
  TEST_ASSERT_EQUAL(0, hookBoard.forceOffs);
  TEST_ASSERT_EQUAL(0, count(hookBoard.hooks, "forceOff"));
  hookBoard.stopServos();
  TEST_ASSERT_EQUAL_STRING("stop", hookBoard.hooks.back().c_str());
  TEST_ASSERT_FALSE(hookBoard.realIsOn);
}

void test_enter_simulation_forces_output_off_once_first()
{
  resetRobot();
  for (int i = 0; i < 20; i++)
    robotTick({0, 0}, false);
  hookBoard.hooks.clear();
  hookBoard.enterSimulation();
  TEST_ASSERT_TRUE(hookBoard.simulated());
  TEST_ASSERT_EQUAL(1, hookBoard.forceOffs);
  TEST_ASSERT_EQUAL(1, (int)hookBoard.hooks.size());
  TEST_ASSERT_EQUAL_STRING("forceOff", hookBoard.hooks[0].c_str());
  hookBoard.enterSimulation(); // again: nothing more
  TEST_ASSERT_EQUAL(1, hookBoard.forceOffs);
}

static void queueCalibration(uint8_t joint, int16_t offsetCentideg)
{
  const joint_calibration::Joint &d = joint_calibration::DEFAULTS[joint];
  protocol::CalibrationSet c = {};
  c.requestId = 7;
  c.joint = joint;
  c.offsetCentideg = offsetCentideg;
  c.pulse0Us = (uint16_t)d.pulse0Us;
  c.pulse180Us = (uint16_t)d.pulse180Us;
  c.flags = d.reversed ? protocol::CALIBRATION_REVERSED : 0;
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCalibrationSet(c, usbSeq++, frame);
  extra.insert(extra.end(), frame, frame + n);
}

void test_no_real_output_in_simulation_through_every_mode()
{
  resetRobot();
  robotTick({0, 0}, false); // baselines
  requestMode(RequestedMode::Stand);
  for (int i = 0; i < 20; i++)
    robotTick({0, 0}, false); // USB only: nothing starts
  TEST_ASSERT_FALSE(robot.supervisor.bodyRunning);
  hookBoard.enterSimulation();
  pendingEvent = BenchEvent::Entered;
  hookBoard.hooks.clear();

  simTicks(30); // the modelled supply settles (500 ms); the old request starts nothing
  TEST_ASSERT_FALSE(robot.supervisor.bodyRunning);
  requestMode(RequestedMode::Stand);
  // Legs switch on two per tick in the model.
  int seen[4] = {};
  for (int i = 0; i < 100; i++)
  {
    simTicks(1);
    int legs = hookBoard.model().legsOn;
    if (legs >= 0 && legs <= 6 && legs % 2 == 0)
      seen[legs / 2] = seen[legs / 2] ? seen[legs / 2] : i + 1;
  }
  TEST_ASSERT_TRUE(robot.supervisor.bodyRunning);
  TEST_ASSERT_EQUAL((int)body::Mode::Stand, (int)robot.body.mode);
  TEST_ASSERT_TRUE(hookBoard.servosOn());
  TEST_ASSERT_TRUE(seen[1] > 0 && seen[2] == seen[1] + 1 && seen[3] == seen[2] + 1);
  TEST_ASSERT_EQUAL_PTR(hookBoard.model().desiredUs, hookBoard.servoPulses());
  TEST_ASSERT_TRUE(hookBoard.servoPulses()[0] != 0);
  TEST_ASSERT_FALSE(hookBoard.realIsOn);

  requestMode(RequestedMode::Walk);
  cmd.translateY = 100;
  simTicks(100);
  TEST_ASSERT_EQUAL((int)body::Mode::Walk, (int)robot.body.mode);
  cmd.translateY = 0;

  requestMode(RequestedMode::Calibrate);
  for (int i = 0; i < 300 && robot.body.mode != body::Mode::Calibrate; i++)
    simTicks(1); // the walk ends its cycle first
  simTicks(10);
  TEST_ASSERT_EQUAL((int)body::Mode::Calibrate, (int)robot.body.mode);
  uint16_t before = hookBoard.servoPulses()[0];
  queueCalibration(0, 1000); // +10 deg on output 0
  simTicks(30);
  TEST_ASSERT_TRUE(hookBoard.servoPulses()[0] != before); // the model follows it

  cmd.flags |= protocol::FLAG_EMERGENCY_STOP;
  simTicks(5);
  TEST_ASSERT_TRUE(robot.protection.faults & protection::EMERGENCY_STOP);
  TEST_ASSERT_FALSE(hookBoard.servosOn());
  cmd.flags &= (uint8_t)~protocol::FLAG_EMERGENCY_STOP;
  simTicks(5);
  cmd.clearCounter++;
  simTicks(5);
  TEST_ASSERT_EQUAL(0, robot.protection.faults);
  simTicks(30);
  requestMode(RequestedMode::Stand);
  simTicks(100);
  TEST_ASSERT_TRUE(robot.supervisor.bodyRunning);
  TEST_ASSERT_TRUE(hookBoard.servosOn());

  TEST_ASSERT_EQUAL(0, count(hookBoard.hooks, "apply"));
  TEST_ASSERT_EQUAL(0, count(hookBoard.hooks, "allow"));
  TEST_ASSERT_EQUAL(0, count(hookBoard.hooks, "forceOff"));
  TEST_ASSERT_TRUE(count(hookBoard.hooks, "inhibit") > 0);
  TEST_ASSERT_TRUE(count(hookBoard.hooks, "stop") > 0); // the e-stop
  TEST_ASSERT_EQUAL(1, hookBoard.forceOffs);
  TEST_ASSERT_FALSE(hookBoard.realIsOn);

  // Pose frames carry the model's pulses and output on.
  const protocol::Frame *pose = nullptr;
  for (const protocol::Frame &f : hookBoard.usbFrames)
    if (f.type == protocol::FrameType::Pose)
      pose = &f;
  TEST_ASSERT_NOT_NULL(pose);
  TEST_ASSERT_TRUE(pose->pose.flags & protocol::POSE_OUTPUTS_ON);
  TEST_ASSERT_TRUE(pose->pose.joints[0].pulseUs != 0);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_window_is_offered_from_the_first_tick_in_live_mode);
  RUN_TEST(test_window_is_measured_from_the_first_tick_not_zero);
  RUN_TEST(test_press_at_2980_ms_enters);
  RUN_TEST(test_press_at_3000_ms_changes_nothing);
  RUN_TEST(test_button_held_from_the_first_tick_never_enters);
  RUN_TEST(test_button_held_then_released_then_pressed_enters);
  RUN_TEST(test_supply_of_0_99_v_on_one_tick_before_the_press_enters);
  RUN_TEST(test_supply_of_1_0_v_on_one_tick_before_the_press_refuses);
  RUN_TEST(test_supply_on_the_press_tick_refuses);
  RUN_TEST(test_output_on_at_1_s_closes_the_window);
  RUN_TEST(test_no_usb_host_refuses);
  RUN_TEST(test_usb_host_is_read_only_at_the_press);
  RUN_TEST(test_both_gates_failing_shows_the_supply_reason);
  RUN_TEST(test_second_press_after_a_refusal_does_nothing);
  RUN_TEST(test_refusal_shown_for_3_s_after_a_press_at_1_s);
  RUN_TEST(test_refusal_shown_for_3_s_after_a_press_at_2980_ms_outlasting_the_window);
  RUN_TEST(test_simulation_lasts_and_ignores_a_lost_usb_host);
  RUN_TEST(test_supply_fault_latches_above_the_brownout_voltage);
  RUN_TEST(test_supply_fault_never_latches_in_live_mode);
  RUN_TEST(test_restart_after_simulation_starts_in_the_window_in_live_mode);
  RUN_TEST(test_before_entry_every_call_reaches_the_hooks);
  RUN_TEST(test_enter_simulation_forces_output_off_once_first);
  RUN_TEST(test_no_real_output_in_simulation_through_every_mode);
  return UNITY_END();
}
