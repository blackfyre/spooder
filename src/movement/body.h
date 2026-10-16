#ifndef BODY_H
#define BODY_H

// Hardware-free body state machine: modes, transitions, smoothing and gaits.
// Produces per-leg joint angles; the hardware layer writes them.
// Compiled for the board and for host tests.
#include <stdint.h>
#include "vectors.h"
#include "kinematics.h"
#include "gait.h"
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
};

// A command older than this is stale and puts the body to sleep.
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;
// Longest time a single step may advance.
constexpr uint32_t MAX_STEP_MS = 100;

// Rest pose for Sleep and calibration pose for Calibrate, in the leg frame.
const Vector3 REST_POSE(130, 0, -46);
const Vector3 CALIBRATION_POSE(224, 0, 116);

struct StandTransition
{
  bool active;
  bool moveAllAtOnce;
  int legsDone;           // legs moved so far when moving three at a time
  int currentLegs[3];     // -1 when unused
  float progress;         // progress units, 0..CYCLE_POINTS
  Vector3 controlPoints[LEG_COUNT][3];
};

struct BodyState
{
  Mode mode;
  uint32_t lastStepMs;
  bool started;
  uint32_t modeEnteredMs;

  // Recorded (accepted) foot positions and the matching joint angles.
  Vector3 feet[LEG_COUNT];
  JointAngles angles[LEG_COUNT];
  // Set for a leg whose angles changed in the last step.
  bool legUpdated[LEG_COUNT];
  // Rejection tracking: newlyRejected is set only on the step a leg goes from
  // accepted to rejected, so reports are not repeated every step.
  bool rejected[LEG_COUNT];
  bool newlyRejected[LEG_COUNT];
  Vector3 rejectedTarget[LEG_COUNT];

  // false once Sleep has folded the legs: servo output should be off.
  bool outputsEnabled;

  // Smoothed command values.
  Vector2 translate;
  float rotate;
  float height;

  gait::Gait currentGait;
  float cycleProgress[LEG_COUNT];
  gait::LegCycle legCycles[LEG_COUNT];

  int initialiseStep;
  StandTransition stand;
  bool sleepFolded;
};

/**
 * @brief Puts the body into Initialise. Call once before the first step.
 */
void init(BodyState &s);

/**
 * @brief Advances the body by the time since the previous step (clamped to
 * MAX_STEP_MS) and returns. Never loops internally.
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
 * @brief Foot height relative to the coxa pivot for a height dial value.
 */
float groundForHeight(float height);

} // namespace body

#endif // BODY_H
