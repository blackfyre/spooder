// The shared control tick (core/robot/robot_tick): the loop rules that were
// checked only on the board while they lived in the firmware's main.cpp.
#include <unity.h>
#include <string>
#include <vector>
#include "robot/robot_tick.h"
#include <math.h>
#include <stdio.h>

using namespace robot_tick;

static const protection::Limits LIMITS = {9.0f, 6.0f, 2000, 5.2f, 200, 4.8f, 6.8f};

// --- a recording board -------------------------------------------------------------

struct SentFrame
{
  int tick;
  protocol::Frame frame;
};

class FakeBoard final : public Hardware
{
public:
  int tickNo = 0;
  bool inhibited = true;
  bool on = false;
  bool refuse[LINK_COUNT] = {};
  int applies = 0;
  std::vector<std::string> events; // this tick's calls, in order
  std::vector<std::string> logs;
  std::vector<SentFrame> frames[LINK_COUNT];
  protocol::Decoder decoders[LINK_COUNT];
  led_pattern::State leds = {};

  void setInhibited(bool v) override
  {
    inhibited = v;
    events.push_back(v ? "inhibit" : "allow");
  }
  void stopServos() override
  {
    on = false;
    events.push_back("stop");
  }
  kinematics::JointAngles out[kinematics::LEG_COUNT] = {}; // the angles last handed over
  void applyServos(const body::BodyState &b, const joint_calibration::Table &) override
  {
    for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
      if (b.legUpdated[leg])
        out[leg] = b.angles[leg];
    on = !inhibited && b.outputsEnabled;
    applies++;
    events.push_back("apply");
  }
  bool servosOn() override { return on; }
  uint16_t pulses[joint_calibration::JOINT_COUNT] = {};
  const uint16_t *servoPulses() override { return pulses; }
  void showLeds(const led_pattern::State &s) override
  {
    leds = s;
    events.push_back("leds");
  }
  bool sendFrame(Link link, const uint8_t *frame, int n) override
  {
    if (refuse[(int)link])
      return false;
    protocol::Decoder &d = decoders[(int)link];
    for (int i = 0; i < n; i++)
    {
      d.push(frame[i]);
      protocol::Frame f;
      while (d.next(f))
        frames[(int)link].push_back({tickNo, f});
    }
    const char *kind = frame[2] == (uint8_t)protocol::FrameType::Telemetry     ? "telemetry"
                       : frame[2] == (uint8_t)protocol::FrameType::Pose        ? "pose"
                       : frame[2] == (uint8_t)protocol::FrameType::Diagnostics ? "diagnostics"
                                                                               : "report";
    events.push_back(std::string(link == Link::Usb ? "usb " : "bridge ") + kind);
    return true;
  }
  void log(const char *line) override
  {
    logs.push_back(line);
    events.push_back(std::string("log ") + line);
  }
};

// A front-end on one link: sends its desired state every tick while active.
struct Frontend
{
  bool active = false;
  bool connected = false;
  bool estop = false;
  RequestedMode mode = RequestedMode::Sleep;
  int8_t translateY = 0;
  int8_t rotate = 0;
  uint8_t height = 0;
  uint8_t speed = 44;
  uint8_t gait = 0;
  uint8_t clearCounter = 0;
  uint8_t modeCounter = 0;
  uint8_t gesture = 0;
  uint8_t gestureCounter = 0;
  uint8_t seq = 0;
  std::vector<uint8_t> extra; // further frames for the next tick
};

static RobotState state;
static FakeBoard board;
static Frontend fe[LINK_COUNT];
static uint32_t now;
static protection::Reading supply;
static uint32_t tickUs; // the previous tick's working time each tick reports

void setUp()
{
  state = RobotState();
  board = FakeBoard();
  for (Frontend &f : fe)
    f = Frontend();
  now = 1000;
  supply = {0.5f, 6.0f};
  tickUs = 0;
  init(state, InputMode::Links, LIMITS, now);
}
void tearDown() {}

static Frontend &usb() { return fe[(int)Link::Usb]; }
static Frontend &bridge() { return fe[(int)Link::Bridge]; }

static void tick()
{
  std::vector<uint8_t> bytes[LINK_COUNT];
  for (int l = 0; l < LINK_COUNT; l++)
  {
    Frontend &f = fe[l];
    if (f.active)
    {
      protocol::Command c = {};
      c.mode = (uint8_t)f.mode;
      c.translateY = f.translateY;
      c.rotate = f.rotate;
      c.height = f.height;
      c.speed = f.speed;
      c.gait = f.gait;
      c.flags = (f.connected ? protocol::FLAG_CONTROLLER_CONNECTED : 0) | (f.estop ? protocol::FLAG_EMERGENCY_STOP : 0);
      c.clearCounter = f.clearCounter;
      c.modeRequestCounter = f.modeCounter;
      c.gesture = f.gesture;
      c.gestureRequestCounter = f.gestureCounter;
      uint8_t frame[protocol::MAX_FRAME_SIZE];
      int n = protocol::encodeCommand(c, f.seq++, frame);
      bytes[l].insert(bytes[l].end(), frame, frame + n);
    }
    bytes[l].insert(bytes[l].end(), f.extra.begin(), f.extra.end());
    f.extra.clear();
  }
  Inputs in = {};
  for (int l = 0; l < LINK_COUNT; l++)
  {
    in.bytes[l] = bytes[l].data();
    in.byteCount[l] = (int)bytes[l].size();
  }
  in.power = supply;
  in.previousTickUs = tickUs;
  board.events.clear();
  board.tickNo++;
  now += LOOP_PERIOD_MS;
  robot_tick::tick(state, in, now, board);
}

static void ticks(int n)
{
  for (int i = 0; i < n; i++)
    tick();
}

static void request(Frontend &f, RequestedMode m)
{
  f.mode = m;
  f.modeCounter++;
}

static bool has(const std::vector<std::string> &v, const std::string &prefix)
{
  for (const std::string &s : v)
    if (s.compare(0, prefix.size(), prefix) == 0)
      return true;
  return false;
}

static int indexOf(const std::vector<std::string> &v, const std::string &prefix)
{
  for (size_t i = 0; i < v.size(); i++)
    if (v[i].compare(0, prefix.size(), prefix) == 0)
      return (int)i;
  return -1;
}

static const protocol::Frame *lastOf(Link link, protocol::FrameType type)
{
  const std::vector<SentFrame> &v = board.frames[(int)link];
  for (size_t i = v.size(); i-- > 0;)
    if (v[i].frame.type == type)
      return &v[i].frame;
  return nullptr;
}

static int countOf(Link link, protocol::FrameType type)
{
  int n = 0;
  for (const SentFrame &s : board.frames[(int)link])
    n += s.frame.type == type;
  return n;
}

// USB takes control and requests Stand; the supply settles; the body boots.
static void standOnUsb()
{
  usb().active = true;
  usb().connected = true;
  tick(); // the first frame after a gap only sets the counter baselines
  request(usb(), RequestedMode::Stand);
  ticks(100); // 2 s: settle, Initialise, Stand
  TEST_ASSERT_TRUE(state.supervisor.bodyRunning);
  TEST_ASSERT_TRUE(board.on);
}

static void calibrateOnUsb()
{
  standOnUsb();
  request(usb(), RequestedMode::Calibrate);
  ticks(10);
  TEST_ASSERT_EQUAL((int)body::Mode::Calibrate, (int)state.body.mode);
}

static void queueSet(Frontend &f, uint8_t id, uint8_t joint, int16_t offsetCentideg)
{
  const joint_calibration::Joint &d = joint_calibration::DEFAULTS[joint];
  protocol::CalibrationSet c = {};
  c.requestId = id;
  c.joint = joint;
  c.offsetCentideg = offsetCentideg;
  c.pulse0Us = (uint16_t)d.pulse0Us;
  c.pulse180Us = (uint16_t)d.pulse180Us;
  c.flags = d.reversed ? protocol::CALIBRATION_REVERSED : 0;
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCalibrationSet(c, f.seq++, frame);
  f.extra.insert(f.extra.end(), frame, frame + n);
}

// --- start and stale ---------------------------------------------------------------

void test_first_tick_names_the_servo_profile()
{
  tick();
  TEST_ASSERT_TRUE(board.logs.size() >= 1);
  TEST_ASSERT_EQUAL_STRING("servo profile MG996R: joint speed budget 257 deg/s\n", board.logs[0].c_str());
  size_t first = board.logs.size();
  ticks(10);
  for (size_t i = first; i < board.logs.size(); i++)
    TEST_ASSERT_TRUE(board.logs[i].find("servo profile") == std::string::npos);
}

// --- Output rate backstop ----------------------------------------------------

static float largestOutputChange(const kinematics::JointAngles *a, const kinematics::JointAngles *b)
{
  float m = 0;
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    m = fmaxf(m, fmaxf(fabsf(a[i].coxa - b[i].coxa), fmaxf(fabsf(a[i].femur - b[i].femur), fabsf(a[i].tibia - b[i].tibia))));
  return m;
}

// Shifts a joint's calibration offset at once (both tables, so the applied
// table does not ramp to it): its commanded angle jumps.
static void jumpOffset(int joint, float deg)
{
  state.requested.joints[joint].offsetDeg += deg;
  state.applied.joints[joint].offsetDeg += deg;
}

void test_backstop_holds_a_fast_command_back()
{
  standOnUsb();
  state.backstopDegPerS = 325; // 6.5 deg per 20 ms tick
  ticks(5);
  TEST_ASSERT_EQUAL_UINT32(0, state.backstopTicks);
  const int femur = kinematics::servoOutputIndex(0, 1);
  jumpOffset(femur, 25);
  float before = board.out[0].femur;
  tick();
  float target = state.body.angles[0].femur;
  TEST_ASSERT_TRUE(fabsf(target - before) > 20);           // the command jumped
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.5f, fabsf(board.out[0].femur - before)); // 325 deg/s x 20 ms
  TEST_ASSERT_EQUAL_UINT32(1, state.backstopTicks);
  for (int i = 0; i < 4; i++)
  {
    float last = board.out[0].femur;
    tick();
    TEST_ASSERT_TRUE(fabsf(board.out[0].femur - last) <= 6.5f + 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, state.body.angles[0].femur, board.out[0].femur); // caught up
  uint32_t held = state.backstopTicks;
  ticks(5);
  TEST_ASSERT_EQUAL_UINT32(held, state.backstopTicks);
}

void test_backstop_is_reported_on_its_own_console_line()
{
  standOnUsb();
  state.backstopDegPerS = 325; // 6.5 deg per 20 ms tick
  ticks(5);
  jumpOffset(kinematics::servoOutputIndex(2, 2), 25);
  board.logs.clear();
  ticks(STATUS_PERIOD_MS / LOOP_PERIOD_MS + 1);
  std::string line;
  int statusLines = 0;
  for (const std::string &l : board.logs)
  {
    if (l.find("output rate backstop:") == 0)
      line = l;
    if (l.find("supply ") == 0)
      statusLines++;
  }
  TEST_ASSERT_EQUAL_STRING("output rate backstop: 3 ticks held back; most: leg 2 tibia, 18.5 deg behind\n", line.c_str());
  TEST_ASSERT_TRUE(statusLines >= 1);
  for (char ch : line)
    TEST_ASSERT_TRUE(ch == '\n' || (ch >= 0x20 && ch < 0x7F));
  // Not repeated while nothing more is held back.
  board.logs.clear();
  ticks(STATUS_PERIOD_MS / LOOP_PERIOD_MS + 1);
  TEST_ASSERT_FALSE(has(board.logs, "output rate backstop:"));
}

void test_first_pulse_after_output_switches_on_is_not_limited()
{
  standOnUsb();
  state.backstopDegPerS = 325; // 6.5 deg per 20 ms tick
  request(usb(), RequestedMode::Sleep);
  for (int i = 0; i < 200 && board.on; i++)
    tick();
  TEST_ASSERT_FALSE(board.on);
  // While the servos are off they may be anywhere: the first pulses go
  // straight to the command, however far it is from the last ones.
  jumpOffset(kinematics::servoOutputIndex(1, 1), 25);
  request(usb(), RequestedMode::Stand);
  uint32_t held = state.backstopTicks;
  for (int i = 0; i < 50 && !board.on; i++)
    tick();
  TEST_ASSERT_TRUE(board.on);
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
    TEST_ASSERT_FLOAT_WITHIN(0.001f, state.body.angles[leg].femur, board.out[leg].femur);
  TEST_ASSERT_EQUAL_UINT32(held, state.backstopTicks);
}

void test_walking_up_to_the_default_dial_never_engages_the_backstop()
{
  const uint8_t heights[] = {0, 50, 100};
  const uint8_t speeds[] = {0, 44};
  for (int g = 0; g < gait::GAIT_COUNT; g++)
    for (uint8_t height : heights)
      for (uint8_t speed : speeds)
      {
        setUp();
        usb().height = height;
        usb().gait = (uint8_t)g;
        usb().speed = speed;
        standOnUsb();
        ticks(75); // settle the height
        usb().translateY = 100;
        usb().rotate = 50;
        request(usb(), RequestedMode::Walk);
        ticks(150);
        usb().translateY = 0;
        usb().rotate = 0;
        request(usb(), RequestedMode::Stand);
        ticks(100);
        request(usb(), RequestedMode::Sleep);
        ticks(100);
        char msg[48];
        snprintf(msg, sizeof msg, "gait %d height %d speed %d", g, height, speed);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, state.backstopTicks, msg);
      }
}

// --- diagnostics -----------------------------------------------------------------

// Spec "Diagnostics frame on USB only": once per telemetry period, on its own
// tick, never on the bridge link.
void test_diagnostics_on_usb_only_on_their_own_tick()
{
  standOnUsb();
  bridge().active = true;
  board.frames[(int)Link::Usb].clear();
  ticks(50);
  int diagnostics = 0;
  for (const SentFrame &f : board.frames[(int)Link::Usb])
  {
    if (f.frame.type != protocol::FrameType::Diagnostics)
      continue;
    diagnostics++;
    for (const SentFrame &g : board.frames[(int)Link::Usb])
      if (g.tick == f.tick && g.frame.type != protocol::FrameType::Diagnostics)
        TEST_FAIL_MESSAGE("diagnostics share a tick with another frame");
  }
  TEST_ASSERT_EQUAL(10, diagnostics);
  TEST_ASSERT_EQUAL(0, countOf(Link::Bridge, protocol::FrameType::Diagnostics));
}

// Spec "Diagnostics frame contents": the window's tick times, the motion
// share, the averaged current and the configured limits.
void test_diagnostics_contents()
{
  standOnUsb();
  ticks(5); // a whole window from here on
  tickUs = 1500;
  ticks(4);
  tickUs = 3000;
  ticks(1);
  tickUs = 1500;
  ticks(5);
  const protocol::Frame *f = lastOf(Link::Usb, protocol::FrameType::Diagnostics);
  TEST_ASSERT_NOT_NULL(f);
  const protocol::Diagnostics &d = f->diagnostics;
  TEST_ASSERT_EQUAL(3000, d.tickMaxUs);
  TEST_ASSERT_EQUAL(1800, d.tickMeanUs); // four of 1500 and one of 3000
  TEST_ASSERT_EQUAL(1000, d.motionSharePermille); // standing: not slowed
  TEST_ASSERT_EQUAL(9000, d.tripCurrentMa);
  TEST_ASSERT_EQUAL(6000, d.sustainedCurrentMa);
  TEST_ASSERT_EQUAL(5200, d.minMv);
  TEST_ASSERT_EQUAL(6800, d.maxMv);
  TEST_ASSERT_EQUAL(4800, d.brownoutMv);
  TEST_ASSERT_EQUAL(257, d.budgetDegPerS);
  TEST_ASSERT_EQUAL(429, d.ratedDegPerS);
  TEST_ASSERT_EQUAL(500, d.averageCurrentMa); // the 0.5 A supply reading, once the window filled
}

// A window holds the working times of the five ticks before the one that
// sends it (each is known one tick late), the same ticks as its motion
// shares.
void test_diagnostics_window_holds_the_five_previous_ticks()
{
  standOnUsb();
  while (board.frames[(int)Link::Usb].empty() || board.frames[(int)Link::Usb].back().frame.type != protocol::FrameType::Diagnostics)
    tick(); // the window starts with the tick after a diagnostics frame
  const uint32_t times[5] = {1000, 2000, 3000, 4000, 5000};
  for (uint32_t us : times)
  {
    tickUs = us; // reported by the next tick
    tick();
  }
  tickUs = 9000; // the sending tick's own time belongs to the next window
  tick();
  const protocol::Frame *f = lastOf(Link::Usb, protocol::FrameType::Diagnostics);
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_EQUAL(5000, f->diagnostics.tickMaxUs);
  TEST_ASSERT_EQUAL(3000, f->diagnostics.tickMeanUs);
}

// Spec "Governed walking": walking fast reports a motion share below full.
void test_diagnostics_report_the_budget_slowing_walking()
{
  usb().speed = 100;
  standOnUsb();
  usb().translateY = 100;
  request(usb(), RequestedMode::Walk);
  uint16_t smallest = 1000;
  for (int i = 0; i < 100; i++)
  {
    tick();
    const protocol::Frame *f = lastOf(Link::Usb, protocol::FrameType::Diagnostics);
    if (f && f->diagnostics.motionSharePermille < smallest)
      smallest = f->diagnostics.motionSharePermille;
  }
  TEST_ASSERT_TRUE(smallest < 1000);
}

void test_boots_into_stand_on_a_fresh_request()
{
  standOnUsb();
  TEST_ASSERT_TRUE(has(board.logs, "operator source: USB"));
  TEST_ASSERT_TRUE(has(board.logs, "output allowed: starting movement"));
  TEST_ASSERT_EQUAL((int)body::Mode::Stand, (int)state.body.mode);
}

void test_stale_command_never_starts_the_body()
{
  // A Stand request arrives while the supply is absent, then the link goes
  // silent. When the supply settles, the command is stale: nothing moves.
  supply = {0, 0};
  usb().active = true;
  usb().connected = true;
  tick();
  request(usb(), RequestedMode::Stand);
  tick();
  usb().active = false;
  ticks(60); // 1.2 s: the command is now older than COMMAND_TIMEOUT_MS
  supply = {0.5f, 6.0f};
  ticks(100);
  TEST_ASSERT_FALSE(state.supervisor.bodyRunning);
  TEST_ASSERT_EQUAL(0, board.applies);
  TEST_ASSERT_FALSE(has(board.logs, "output allowed"));
}

void test_command_going_stale_puts_the_body_to_sleep()
{
  standOnUsb();
  usb().active = false;
  ticks(60);
  TEST_ASSERT_EQUAL((int)body::Mode::Sleep, (int)state.body.mode);
}

// --- e-stop ------------------------------------------------------------------------

void test_emergency_stop_acts_in_the_tick_its_frame_arrives()
{
  standOnUsb();
  // The bridge, without a gamepad (not in control), signals the e-stop.
  bridge().active = true;
  bridge().estop = true;
  tick();
  // Input is read before protection: the same tick inhibits and stops.
  int inhibit = indexOf(board.events, "inhibit");
  int stop = indexOf(board.events, "stop");
  TEST_ASSERT_TRUE(inhibit >= 0);
  TEST_ASSERT_TRUE(stop > inhibit);
  TEST_ASSERT_EQUAL(-1, indexOf(board.events, "apply"));
  TEST_ASSERT_TRUE(has(board.logs, "fault latched: emergency stop"));
  TEST_ASSERT_TRUE(has(board.logs, "movement stopped"));
  TEST_ASSERT_FALSE(board.on);
  ticks(5);
  const protocol::Frame *t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(t);
  TEST_ASSERT_EQUAL_HEX16(protection::EMERGENCY_STOP, t->telemetry.faults & protection::EMERGENCY_STOP);
  TEST_ASSERT_FALSE(t->telemetry.outputsOn);
}

void test_clear_does_nothing_while_the_stop_is_signalled_and_restart_needs_a_request()
{
  standOnUsb();
  usb().estop = true;
  tick();
  TEST_ASSERT_TRUE(state.protection.faults & protection::EMERGENCY_STOP);
  usb().clearCounter++;
  ticks(5);
  TEST_ASSERT_TRUE(state.protection.faults & protection::EMERGENCY_STOP);
  usb().estop = false;
  usb().clearCounter++;
  ticks(30);
  TEST_ASSERT_EQUAL_HEX16(0, state.protection.faults);
  TEST_ASSERT_FALSE(state.supervisor.bodyRunning); // the same Stand request again is not a new one
  request(usb(), RequestedMode::Stand);
  ticks(2);
  TEST_ASSERT_TRUE(state.supervisor.bodyRunning);
}

// --- arbitration -------------------------------------------------------------------

void test_conflict_between_links_sleeps_and_is_reported()
{
  standOnUsb();
  bridge().active = true;
  bridge().connected = true;
  bridge().mode = RequestedMode::Walk;
  tick();
  TEST_ASSERT_TRUE(has(board.logs, "operator source: conflict (bridge and USB): sleeping"));
  TEST_ASSERT_TRUE(board.leds.input == led_pattern::Input::SourceConflict);
  ticks(50);
  TEST_ASSERT_EQUAL((int)body::Mode::Sleep, (int)state.body.mode);
  const protocol::Frame *t = lastOf(Link::Bridge, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(t);
  // The source bits (the bridge link bits are set too: it is up, with a gamepad).
  TEST_ASSERT_EQUAL_HEX8(protocol::TELEMETRY_SOURCE_CONFLICT, t->telemetry.flags & 0x07);
}

// --- calibration -------------------------------------------------------------------

void test_calibration_set_rejected_outside_calibrate()
{
  standOnUsb();
  joint_calibration::Table before = state.requested;
  queueSet(usb(), 7, 4, (int16_t)(joint_calibration::DEFAULTS[4].offsetDeg * 100 + 300));
  tick();
  TEST_ASSERT_EQUAL_FLOAT(before.joints[4].offsetDeg, state.requested.joints[4].offsetDeg);
  // A report follows on both links, carrying the rejection on the sender's.
  ticks(1);
  const protocol::Frame *r = lastOf(Link::Usb, protocol::FrameType::CalibrationReport);
  TEST_ASSERT_NOT_NULL(r);
  TEST_ASSERT_EQUAL(7, r->calibrationReport.lastRequestId);
  TEST_ASSERT_EQUAL((int)protocol::CalibrationResult::Rejected, r->calibrationReport.result);
  const protocol::Frame *b = lastOf(Link::Bridge, protocol::FrameType::CalibrationReport);
  TEST_ASSERT_NOT_NULL(b);
  TEST_ASSERT_EQUAL((int)protocol::CalibrationResult::NoneYet, b->calibrationReport.result);
}

void test_calibration_set_accepted_in_calibrate_and_applied_at_the_rate_limit()
{
  calibrateOnUsb();
  const float start = state.applied.joints[4].offsetDeg;
  const float target = start + 3.0f;
  queueSet(usb(), 9, 4, (int16_t)(target * 100));
  tick();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target, state.requested.joints[4].offsetDeg);
  // 60°/s: at most 1.2° per 20 ms tick.
  float last = state.applied.joints[4].offsetDeg;
  TEST_ASSERT_TRUE(last - start <= 1.2f + 1e-3f);
  for (int i = 0; i < 10; i++)
  {
    tick();
    float v = state.applied.joints[4].offsetDeg;
    TEST_ASSERT_TRUE(v - last <= 1.2f + 1e-3f);
    last = v;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target, last);
  const protocol::Frame *r = lastOf(Link::Usb, protocol::FrameType::CalibrationReport);
  TEST_ASSERT_NOT_NULL(r);
  TEST_ASSERT_EQUAL(9, r->calibrationReport.lastRequestId);
  TEST_ASSERT_EQUAL((int)protocol::CalibrationResult::Accepted, r->calibrationReport.result);
  TEST_ASSERT_EQUAL((int16_t)(target * 100), r->calibrationReport.joints[4].offsetCentideg);
}

// --- frames and their rates --------------------------------------------------------

void test_telemetry_every_fifth_tick_on_both_links()
{
  ticks(50);
  TEST_ASSERT_EQUAL(10, countOf(Link::Usb, protocol::FrameType::Telemetry));
  TEST_ASSERT_EQUAL(10, countOf(Link::Bridge, protocol::FrameType::Telemetry));
  for (int l = 0; l < LINK_COUNT; l++)
  {
    int i = 0;
    for (const SentFrame &f : board.frames[l])
      if (f.frame.type == protocol::FrameType::Telemetry)
        TEST_ASSERT_EQUAL(5 * ++i, f.tick);
  }
}

void test_pose_on_usb_two_ticks_after_telemetry()
{
  standOnUsb();
  board.pulses[0] = 1500;
  int before = countOf(Link::Usb, protocol::FrameType::Pose);
  ticks(50);
  TEST_ASSERT_EQUAL(10, countOf(Link::Usb, protocol::FrameType::Pose) - before);
  TEST_ASSERT_EQUAL(0, countOf(Link::Bridge, protocol::FrameType::Pose));
  int lastTelemetry = -1;
  for (const SentFrame &f : board.frames[(int)Link::Usb])
  {
    if (f.frame.type == protocol::FrameType::Telemetry)
      lastTelemetry = f.tick;
    else if (f.frame.type == protocol::FrameType::Pose && lastTelemetry >= 0)
      TEST_ASSERT_EQUAL(lastTelemetry + POSE_TICK, f.tick);
  }
  const protocol::Frame *p = lastOf(Link::Usb, protocol::FrameType::Pose);
  TEST_ASSERT_EQUAL(1500, p->pose.joints[0].pulseUs);
  TEST_ASSERT_EQUAL_HEX8(protocol::POSE_OUTPUTS_ON, p->pose.flags & protocol::POSE_OUTPUTS_ON);
}

void test_telemetry_reports_each_links_own_frame_counts()
{
  usb().active = true;
  ticks(5);
  const protocol::Frame *u = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  const protocol::Frame *b = lastOf(Link::Bridge, protocol::FrameType::Telemetry);
  TEST_ASSERT_EQUAL(5, u->telemetry.validFrames);
  TEST_ASSERT_EQUAL(0, b->telemetry.validFrames);
  TEST_ASSERT_EQUAL(6000, u->telemetry.voltageMv);
  TEST_ASSERT_EQUAL(500, u->telemetry.currentMa);
}

void test_reports_in_calibrate_at_one_hertz_and_none_otherwise()
{
  standOnUsb();
  TEST_ASSERT_EQUAL(0, countOf(Link::Usb, protocol::FrameType::CalibrationReport));
  request(usb(), RequestedMode::Calibrate);
  ticks(1);
  int first = countOf(Link::Usb, protocol::FrameType::CalibrationReport);
  ticks(149); // 3 s in Calibrate
  int n = countOf(Link::Usb, protocol::FrameType::CalibrationReport);
  TEST_ASSERT_TRUE(n - first >= 2 && n - first <= 3);
  TEST_ASSERT_EQUAL(n, countOf(Link::Bridge, protocol::FrameType::CalibrationReport));
}

void test_reports_after_sets_no_more_than_five_per_second()
{
  calibrateOnUsb();
  int before = countOf(Link::Usb, protocol::FrameType::CalibrationReport);
  for (int i = 0; i < 50; i++) // a set every tick for 1 s
  {
    queueSet(usb(), (uint8_t)(20 + i), 4, (int16_t)(joint_calibration::DEFAULTS[4].offsetDeg * 100 + (i % 2) * 50));
    tick();
  }
  int n = countOf(Link::Usb, protocol::FrameType::CalibrationReport) - before;
  TEST_ASSERT_TRUE(n >= 4 && n <= 6);
}

void test_sequence_numbers_advance_only_for_sent_frames()
{
  board.refuse[(int)Link::Usb] = true;
  ticks(10); // two telemetry, two pose and two diagnostics frames refused
  board.refuse[(int)Link::Usb] = false;
  ticks(5); // a pose (tick 12), diagnostics (tick 14), then telemetry (tick 15)
  TEST_ASSERT_EQUAL(3, (int)board.frames[(int)Link::Usb].size());
  TEST_ASSERT_EQUAL(0, board.frames[(int)Link::Usb][0].frame.seq);
  const protocol::Frame *t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(t);
  TEST_ASSERT_EQUAL(2, t->seq);
  TEST_ASSERT_EQUAL(3, lastOf(Link::Bridge, protocol::FrameType::Telemetry)->seq + 1);
}

void test_refused_report_is_retried()
{
  calibrateOnUsb();
  board.refuse[(int)Link::Usb] = true;
  queueSet(usb(), 5, 4, (int16_t)(joint_calibration::DEFAULTS[4].offsetDeg * 100 + 100));
  ticks(3);
  board.refuse[(int)Link::Usb] = false;
  tick();
  const protocol::Frame *r = lastOf(Link::Usb, protocol::FrameType::CalibrationReport);
  TEST_ASSERT_NOT_NULL(r);
  TEST_ASSERT_EQUAL(5, r->calibrationReport.lastRequestId);
  TEST_ASSERT_EQUAL(board.tickNo, board.frames[(int)Link::Usb].back().tick);
}

// --- text --------------------------------------------------------------------------

void test_status_line_once_a_second()
{
  ticks(150);
  int n = 0;
  for (const std::string &s : board.logs)
    n += s.compare(0, 7, "supply ") == 0;
  TEST_ASSERT_EQUAL(3, n);
  TEST_ASSERT_TRUE(has(board.logs, "supply 0.50 A (avg "));
  TEST_ASSERT_TRUE(has(board.logs, "operator source") == false); // still none
}

void test_text_and_frames_in_loop_order()
{
  // Within one tick: fault text, then the LEDs, then frames, then the status line.
  ticks(49);
  supply = {0.5f, 7.5f};
  state.lastStatusMs = now + LOOP_PERIOD_MS - STATUS_PERIOD_MS; // the status line is due too
  tick();
  int fault = indexOf(board.events, "log fault latched: overvoltage");
  int leds = indexOf(board.events, "leds");
  int telemetry = indexOf(board.events, "usb telemetry");
  int status = indexOf(board.events, "log supply ");
  TEST_ASSERT_TRUE(fault >= 0 && leds > fault && telemetry > leds && status > telemetry);
  TEST_ASSERT_TRUE(indexOf(board.events, "bridge telemetry") < telemetry);
}

void test_serial_input_mode()
{
  state = RobotState();
  init(state, InputMode::Serial, LIMITS, now);
  Inputs in = {};
  in.power = supply;
  in.serial.command.mode = RequestedMode::Stand;
  for (int i = 0; i < 100; i++)
  {
    now += LOOP_PERIOD_MS;
    in.serial.command.issuedAtMs = now;
    in.serial.requestSeq = 1;
    in.serial.printReadings = i == 10;
    robot_tick::tick(state, in, now, board);
  }
  TEST_ASSERT_TRUE(state.supervisor.bodyRunning);
  TEST_ASSERT_TRUE(board.leds.input == led_pattern::Input::Serial);
  TEST_ASSERT_EQUAL(0, (int)board.frames[0].size() + (int)board.frames[1].size());
  int n = 0;
  for (const std::string &s : board.logs)
    n += s.compare(0, 7, "supply ") == 0;
  TEST_ASSERT_EQUAL(3, n); // at 1 s, 2 s and on request
}

// --- Gestures ---

static void requestGesture(Frontend &f, uint8_t g)
{
  f.gesture = g;
  f.gestureCounter++;
}

void test_gesture_plays_over_usb_and_telemetry_reports_it()
{
  standOnUsb();
  requestGesture(usb(), protocol::GESTURE_WIGGLE);
  ticks(10);
  TEST_ASSERT_TRUE(state.body.mode == body::Mode::Perform);
  const protocol::Frame *t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(t);
  TEST_ASSERT_EQUAL(protocol::GESTURE_WIGGLE, t->telemetry.gesture);
  TEST_ASSERT_EQUAL((uint8_t)body::Mode::Perform, t->telemetry.bodyMode);
  ticks(250); // 5 s: the whole wiggle
  TEST_ASSERT_TRUE(state.body.mode == body::Mode::Stand);
  t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_EQUAL(0, t->telemetry.gesture);
}

void test_explicit_stand_request_ends_a_gesture()
{
  standOnUsb();
  requestGesture(usb(), protocol::GESTURE_SAY_HI);
  ticks(10);
  TEST_ASSERT_TRUE(state.body.mode == body::Mode::Perform);
  request(usb(), RequestedMode::Stand); // Stand again: a new explicit request
  ticks(60); // the first keyframe ends by 600 ms, the return takes 400 ms more
  TEST_ASSERT_TRUE(state.body.mode == body::Mode::Stand);
}

void test_fault_mid_gesture_stops_reporting_the_gesture()
{
  standOnUsb();
  requestGesture(usb(), protocol::GESTURE_SAY_HI);
  ticks(10);
  TEST_ASSERT_TRUE(state.body.mode == body::Mode::Perform);
  bridge().active = true;
  bridge().estop = true; // the e-stop stops the body mid-gesture
  ticks(10);
  TEST_ASSERT_FALSE(state.supervisor.bodyRunning);
  const protocol::Frame *t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(t);
  TEST_ASSERT_EQUAL(0, t->telemetry.gesture);
}

void test_gesture_from_the_idle_bridge_is_ignored()
{
  standOnUsb();
  bridge().active = true; // no gamepad: not in control
  tick();
  requestGesture(bridge(), protocol::GESTURE_WIGGLE);
  ticks(10);
  TEST_ASSERT_TRUE(state.body.mode == body::Mode::Stand);
}

// --- Bridge link state in telemetry ---

void test_usb_telemetry_reports_the_bridge_link()
{
  standOnUsb();
  // Not yet running: down, no counts.
  const protocol::Frame *t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(t);
  TEST_ASSERT_EQUAL_HEX8(0, t->telemetry.flags & (protocol::TELEMETRY_BRIDGE_LINK_UP | protocol::TELEMETRY_BRIDGE_GAMEPAD));
  // A bridge without a gamepad: up, no gamepad, rising counts.
  bridge().active = true;
  ticks(10);
  t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_TRUE(t->telemetry.flags & protocol::TELEMETRY_BRIDGE_LINK_UP);
  TEST_ASSERT_FALSE(t->telemetry.flags & protocol::TELEMETRY_BRIDGE_GAMEPAD);
  uint16_t before = t->telemetry.bridgeValidFrames;
  TEST_ASSERT_TRUE(before >= 5);
  ticks(10);
  t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_TRUE(t->telemetry.bridgeValidFrames > before);
  // The same counts reach the bridge as its own.
  const protocol::Frame *b = lastOf(Link::Bridge, protocol::FrameType::Telemetry);
  TEST_ASSERT_NOT_NULL(b);
  TEST_ASSERT_EQUAL_UINT16(b->telemetry.validFrames, b->telemetry.bridgeValidFrames);
  // A gamepad on the bridge (a conflict with USB, but the state is reported).
  bridge().connected = true;
  ticks(5);
  t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_TRUE(t->telemetry.flags & protocol::TELEMETRY_BRIDGE_GAMEPAD);
  // Unplugged: down within the link-gap time.
  bridge().active = false;
  ticks((link_receiver::LINK_GAP_MS / LOOP_PERIOD_MS) + 6);
  t = lastOf(Link::Usb, protocol::FrameType::Telemetry);
  TEST_ASSERT_EQUAL_HEX8(0, t->telemetry.flags & (protocol::TELEMETRY_BRIDGE_LINK_UP | protocol::TELEMETRY_BRIDGE_GAMEPAD));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_first_tick_names_the_servo_profile);
  RUN_TEST(test_diagnostics_on_usb_only_on_their_own_tick);
  RUN_TEST(test_diagnostics_contents);
  RUN_TEST(test_diagnostics_window_holds_the_five_previous_ticks);
  RUN_TEST(test_diagnostics_report_the_budget_slowing_walking);
  RUN_TEST(test_backstop_holds_a_fast_command_back);
  RUN_TEST(test_backstop_is_reported_on_its_own_console_line);
  RUN_TEST(test_first_pulse_after_output_switches_on_is_not_limited);
  RUN_TEST(test_walking_up_to_the_default_dial_never_engages_the_backstop);
  RUN_TEST(test_boots_into_stand_on_a_fresh_request);
  RUN_TEST(test_stale_command_never_starts_the_body);
  RUN_TEST(test_command_going_stale_puts_the_body_to_sleep);
  RUN_TEST(test_emergency_stop_acts_in_the_tick_its_frame_arrives);
  RUN_TEST(test_clear_does_nothing_while_the_stop_is_signalled_and_restart_needs_a_request);
  RUN_TEST(test_conflict_between_links_sleeps_and_is_reported);
  RUN_TEST(test_calibration_set_rejected_outside_calibrate);
  RUN_TEST(test_calibration_set_accepted_in_calibrate_and_applied_at_the_rate_limit);
  RUN_TEST(test_telemetry_every_fifth_tick_on_both_links);
  RUN_TEST(test_pose_on_usb_two_ticks_after_telemetry);
  RUN_TEST(test_telemetry_reports_each_links_own_frame_counts);
  RUN_TEST(test_reports_in_calibrate_at_one_hertz_and_none_otherwise);
  RUN_TEST(test_reports_after_sets_no_more_than_five_per_second);
  RUN_TEST(test_sequence_numbers_advance_only_for_sent_frames);
  RUN_TEST(test_refused_report_is_retried);
  RUN_TEST(test_status_line_once_a_second);
  RUN_TEST(test_text_and_frames_in_loop_order);
  RUN_TEST(test_serial_input_mode);
  RUN_TEST(test_gesture_plays_over_usb_and_telemetry_reports_it);
  RUN_TEST(test_explicit_stand_request_ends_a_gesture);
  RUN_TEST(test_fault_mid_gesture_stops_reporting_the_gesture);
  RUN_TEST(test_gesture_from_the_idle_bridge_is_ignored);
  RUN_TEST(test_usb_telemetry_reports_the_bridge_link);
  return UNITY_END();
}
