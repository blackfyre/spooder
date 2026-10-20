#ifndef BODY_H
#define BODY_H

// Hardware-free body state machine: modes, transitions, smoothing and gaits.
// Produces per-leg joint angles; the hardware layer writes them.
// Compiled for the board and for host tests.
#include <stdint.h>
#include "vectors.h"
#include "kinematics.h"
#include "joint_calibration.h"
#include "gait.h"
#include "gesture.h"
#include "movement_command.h"

namespace body
{

using kinematics::JointAngles;
using kinematics::LEG_COUNT;

enum class Mode : uint8_t
{
  Initialise,
  Stand,
  Walk,
  Sleep,
  Calibrate,
  Perform, // playing a gesture; entered from a settled Stand, returns to Stand
};

// A command older than this is stale and puts the body to sleep.
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;
// A command stamped up to this far after nowMs (another thread read millis()
// later) still counts as fresh.
constexpr uint32_t MAX_COMMAND_LEAD_MS = 100;
// Longest time a single step may advance.
constexpr uint32_t MAX_STEP_MS = 100;

// Rest pose for Sleep and calibration pose for Calibrate, in the leg frame.
const Vector3 REST_POSE(130, 0, -46);
const Vector3 CALIBRATION_POSE(224, 0, 116);

struct StandTransition
{
  bool active;
  bool moveAllAtOnce;
  int currentLegs[3];     // -1 when unused
  bool legDone[LEG_COUNT]; // reached the standing point; tracks height from then on
  float progress;         // progress units, 0..CYCLE_POINTS
  Vector3 controlPoints[LEG_COUNT][3];
};

struct BodyState
{
  Mode mode;
  uint32_t lastStepMs;
  bool started;

  // Recorded (accepted) foot positions and the matching joint angles.
  Vector3 feet[LEG_COUNT];
  JointAngles angles[LEG_COUNT];
  // Set for a leg whose angles changed in the last step.
  bool legUpdated[LEG_COUNT];
  // Feet recorded during this step's sub-steps, solved once at the end.
  bool pendingSolve[LEG_COUNT];
  // Rejection tracking: newlyRejected is set only on the step a leg goes from
  // accepted to rejected, so reports are not repeated every step.
  bool rejected[LEG_COUNT];
  bool newlyRejected[LEG_COUNT];
  Vector3 rejectedTarget[LEG_COUNT];

  // Calibration offsets (coxa, femur, tibia) the IK adds, per leg. A change
  // re-solves the leg in the next step even if its foot did not move.
  Vector3 offsets[LEG_COUNT];
  bool offsetsChanged[LEG_COUNT];

  // Joint speed budget in degrees per second (0: none). init() loads the
  // build's servo profile. Each step advances motion by the share of the
  // elapsed time that keeps every joint within it (motionScale: the
  // smallest share in the last step, 1 = all).
  float jointBudgetDegPerS;
  float motionScale;
  uint32_t motionPendingUs; // elapsed time not yet advanced (under a budget)
  uint8_t pendingGesture;   // a gesture request waiting for the next piece

  // false once Sleep has folded the legs: servo output should be off.
  bool outputsEnabled;

  // Once a command goes stale, that issuedAtMs stays stale; only a command
  // with a different stamp can wake the body. Avoids millis() wrap making an
  // old command look fresh again.
  bool staleLatched;
  uint32_t staleStamp;

  // Smoothed command values.
  Vector2 translate;
  float rotate;
  float height;
  float speed;       // speed dial, 0..100
  float fixedStride; // share of the fixed stride mode, 0..1

  gait::Gait currentGait;
  float cycleProgress[LEG_COUNT];
  gait::LegCycle legCycles[LEG_COUNT];

  int initialiseStep;
  uint32_t initialiseElapsedMs; // time spent in the current Initialise pose
  uint32_t initialiseCarryUs;   // time below a whole millisecond, not yet counted
  StandTransition stand;
  bool sleepFolded;

  // Gestures: the player, the mode-request sequence it started under, and
  // the last gesture request sequence seen.
  gesture::Player perform;
  uint32_t performCarryUs; // time below a whole millisecond, not yet played
  uint32_t performRequestSeq;
  bool gestureSeqKnown;
  uint32_t lastGestureSeq;
};

/**
 * @brief Puts the body into Initialise. Call once before the first step.
 */
void init(BodyState &s);

/**
 * @brief Sets a leg's calibration offsets (the applied table's), used from
 * the next step on. init() loads the compiled defaults.
 */
void setOffsets(BodyState &s, int leg, Vector3 offsets);

/**
 * @brief Advances the body by the time since the previous step (clamped to
 * MAX_STEP_MS) and returns. Outside Initialise, motion advances by less when
 * the full time would turn a joint faster than the joint speed budget; the
 * stale-command timeout still counts elapsed time.
 *
 * The whole state machine runs in fixed internal sub-steps, so transitions,
 * holds and smoothing land on the same time grid whatever the tick length.
 * Joint angles are solved once per step for the legs that moved.
 */
void step(BodyState &s, const MovementCommand &cmd, uint32_t nowMs);

/**
 * @brief Clamps a command to its documented ranges. Unknown gaits are kept
 * as-is; step() ignores them.
 */
MovementCommand clampCommand(const MovementCommand &cmd);

/**
 * @brief True while a transition (Initialise, Stand, Sleep fold) is running.
 */
bool transitionActive(const BodyState &s);

/**
 * @brief The gesture::Gesture id being played, 0 when not in Perform.
 */
uint8_t playingGesture(const BodyState &s);

/**
 * @brief Foot height relative to the coxa pivot for a height dial value.
 */
float groundForHeight(float height);

} // namespace body

#endif // BODY_H
