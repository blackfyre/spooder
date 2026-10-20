#ifndef ROBOT_TICK_H
#define ROBOT_TICK_H

// One tick of the robot's 20 ms control loop, shared by the Servo 2040
// firmware and the virtual robot (tools/virtual-robot). Hardware-free:
// compiled for the board and for host tests.
//
// Per tick, in this order:
//   decode the bytes each link received, arbitrate (or take the serial keys)
//   protection step with the power reading; inhibit output unless allowed
//   the stale rule, then the supervisor: stop or (re)start the body
//   calibration sets from both links (only in Calibrate with output allowed)
//   applied calibration follows the requested table at the rate limit
//   body step with the applied offsets; servo output
//   status LEDs
//   every fifth tick: telemetry on both links; two ticks later the pose on
//   USB; calibration reports when due
//   once a second (or on request in --serial builds): the status line
//
// The adapter (firmware main.cpp, the virtual robot) reads the link bytes and
// the power reading, calls tick(), and implements Hardware: the tick calls it
// where the loop drives servo output, the LEDs, the links and debug text.
#include <stdint.h>
#include "movement/body.h"
#include "movement/joint_calibration.h"
#include "movement/movement_command.h"
#include "protection/protection.h"
#include "protection/supervisor.h"
#include "protocol/link_receiver.h"
#include "protocol/protocol.h"
#include "status/led_pattern.h"

namespace robot_tick
{

// Control loop period. Movement advances by elapsed time, so this is a target,
// not a hard deadline.
constexpr uint32_t LOOP_PERIOD_MS = 20;
// The status line, at most this often (unless asked for).
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr int TELEMETRY_EVERY_TICKS = 5; // 100 ms
// Pose frames (USB only) at the telemetry rate, two ticks after it, so the
// two never share one tick's USB transmit buffer.
constexpr int POSE_TICK = 2;
// Diagnostics frames (USB only) likewise, four ticks after telemetry.
constexpr int DIAGNOSTICS_TICK = 4;
// Longest debug text line, including the newline.
constexpr int MAX_LOG_LINE = 160;

enum class Link : uint8_t
{
  Bridge, // UART1, the gamepad bridge
  Usb,    // a computer on the USB port, sharing it with the debug text
};
constexpr int LINK_COUNT = 2;

// Where the operator's input comes from: the links (default builds), or the
// USB serial console keys (--serial bring-up builds; no frames at all).
enum class InputMode : uint8_t
{
  Links,
  Serial,
};

// The serial keys' input for one tick (see serial_commands).
struct SerialInput
{
  MovementCommand command; // stamped with the time it was read
  uint32_t requestSeq;     // advances on every mode key
  bool clearFaults;
  bool printReadings;
};

struct Inputs
{
  // InputMode::Links: the bytes each link received since the previous tick,
  // in arrival order. None means the line was quiet for a whole tick.
  const uint8_t *bytes[LINK_COUNT];
  int byteCount[LINK_COUNT];
  // InputMode::Serial.
  SerialInput serial;
  protection::Reading power;
  // The working time of the previous tick() call, in microseconds, as the
  // caller measured it (0 when not measured).
  uint32_t previousTickUs;
};

// What the tick drives. Every call happens from inside tick().
class Hardware
{
public:
  // While inhibited (protection), no output may produce a pulse.
  virtual void setInhibited(bool inhibited) = 0;
  // Output off immediately: the body has stopped.
  virtual void stopServos() = 0;
  // One movement step: every joint the body has set, through the applied table.
  virtual void applyServos(const body::BodyState &body, const joint_calibration::Table &applied) = 0;
  virtual bool servosOn() = 0;
  // The last pulse set per output, in µs (servo output order; 0 = never set).
  virtual const uint16_t *servoPulses() = 0;
  virtual void showLeds(const led_pattern::State &state) = 0;
  // Sends a frame whole or not at all, without waiting.
  // @return whether it was sent (the sequence number and report schedule
  // advance only then)
  virtual bool sendFrame(Link link, const uint8_t *frame, int length) = 0;
  // One line of debug text, printable ASCII ending in '\n'.
  virtual void log(const char *line) = 0;

protected:
  ~Hardware() = default;
};

// One link's protocol endpoint: decoder, receiver and transmit sequence.
struct LinkEndpoint
{
  protocol::Decoder decoder;
  link_receiver::Receiver receiver;
  uint8_t txSeq = 0;
  link_receiver::ReportSchedule reports;
};

struct RobotState
{
  InputMode inputMode = InputMode::Links;
  float tripCurrentA = 0;      // for the LED load bar
  float sustainedCurrentA = 0;

  body::BodyState body = {};
  protection::State protection = {};
  protection::Supervisor supervisor;
  // Requested: the latest accepted calibration (what reports show). Applied:
  // what the IK and servo output use, following requested at a bounded rate.
  // Both start from the compiled defaults; live changes last until restart.
  joint_calibration::Table requested = {};
  joint_calibration::Table applied = {};

  LinkEndpoint links[LINK_COUNT];
  link_receiver::Arbiter arbiter;
  link_receiver::Source lastSource = link_receiver::Source::None;
  // The bridge link's state for telemetry (TELEMETRY_BRIDGE_* flags).
  uint8_t bridgeFlags = 0;

  // The servo profile's joint speed budget, which movement keeps to, and its
  // rated no-load speed, which the output rate backstop never exceeds.
  // init() sets both from the build's profile.
  float jointBudgetDegPerS = 0;
  float backstopDegPerS = 0;
  bool profileAnnounced = false; // the start-up line naming the profile

  // Output rate backstop: the angles last handed to the servos, per leg
  // whether that is known since output switched on, and how often (ticks)
  // a joint was held back, with the joint furthest behind since the last
  // report.
  kinematics::JointAngles outputAngles[kinematics::LEG_COUNT] = {};
  bool outputKnown[kinematics::LEG_COUNT] = {};
  uint32_t backstopTicks = 0;
  uint32_t backstopReported = 0;
  float backstopWorstDeg = 0;
  int backstopWorstJoint = -1; // servo output order

  // Diagnostics over the current telemetry window: the smallest motion
  // share, and the tick working times. A tick's working time is known only
  // in the next tick, so each tick adds the previous tick's time and motion
  // share together, and a window covers the same five ticks for both.
  float previousMotionShare = 1;
  float windowMotionShare = 1;
  uint32_t windowTickMaxUs = 0;
  uint32_t windowTickSumUs = 0;
  uint32_t windowTicks = 0;

  int telemetryTick = 0;
  uint32_t lastStatusMs = 0;
  uint32_t lastTickMs = 0;
};

/**
 * @brief Prepares a default-constructed @p s (call once): protection with
 * @p limits, both calibration tables from the compiled defaults, the joint
 * speed budget and backstop speed from the build's servo profile, and @p nowMs
 * as the previous tick's time.
 */
void init(RobotState &s, InputMode mode, const protection::Limits &limits, uint32_t nowMs);

/**
 * @brief Runs one tick at @p nowMs (see the header comment).
 */
void tick(RobotState &s, const Inputs &in, uint32_t nowMs, Hardware &hw);

} // namespace robot_tick

#endif // ROBOT_TICK_H
