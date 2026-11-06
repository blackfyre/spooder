#ifndef GESTURE_H
#define GESTURE_H

// Scripted gestures for the Perform body mode: constant keyframe tables and
// the player that turns them into foot targets in each leg's frame.
// Hardware-free: compiled for the board and for host tests.
//
// A keyframe is a body pose (a translation of the body over its planted
// feet and a rotation, add-gestures design D1) plus foot targets of their own
// for any of the legs, reached with an eased move over the keyframe's duration
// from where the previous keyframe ended. A planted foot stays where it is on
// the ground: moving the body moves it the other way in its leg frame.
#include <stdint.h>
#include "kinematics.h"
#include "vectors.h"

namespace gesture
{

using kinematics::LEG_COUNT;

// Values match the link's GESTURE_* ids.
enum class Gesture : uint8_t
{
  None = 0,
  SayHi = 1,
  Wiggle = 2,
  Dance = 3,
  Attack = 4,
  Ball = 5,
};
constexpr int GESTURE_ID_COUNT = 6; // ids 0..5

// Every leg may take a foot target of its own (ball folds all six).
constexpr int MAX_LIFTED_LEGS = LEG_COUNT;

struct Lift
{
  int8_t leg = -1;                 // -1 when unused
  Vector3 foot;                    // target in the leg frame
  bool relativeToStanding = false; // foot is an offset from standing (height-independent taps)
  bool relativeToPlanted = false;  // foot is an offset from where the keyframe's pose plants it (ball's fold)
};

// The body over its planted feet, in the body frame (x right, y forward,
// z up, millimetres): a translation and a rotation about the body's centre
// by yaw (about z, + anticlockwise seen from above), pitch (about x, + nose
// up) and roll (about y, + right side down). The body-to-world transform is
// translation + Rz(yaw) Rx(pitch) Ry(roll) * point: roll first, then pitch,
// then yaw about fixed axes. Planted feet use its inverse.
struct Pose
{
  Vector3 translation;
  float yawDeg = 0;
  float pitchDeg = 0;
  float rollDeg = 0;
};

struct Keyframe
{
  uint16_t durationMs;
  Pose pose;
  Lift lifts[MAX_LIFTED_LEGS]; // legs with a target of their own; the rest stay planted
  float maxFootSpeedMmS =
      0; // 0: MAX_FOOT_SPEED_MM_S; a higher value for this keyframe only, still under the joint budget (D2)
  // When set, pose.translation.z is not a shift but the height of the body's
  // underside (body_geometry::UNDERSIDE_Z) above the ground, resolved from
  // the standing height by keyframePose: ball lowers onto its frame at every
  // body height.
  bool zAboveGround = false;
};

struct Script
{
  const Keyframe *frames;
  int count;
  // A holding script stays at its last keyframe, in Perform, until another
  // request; that request first plays the unfold script, which ends at the
  // stand pose (D1).
  bool holds = false;
  const Keyframe *unfold = nullptr;
  int unfoldCount = 0;
};

/**
 * @brief The script for @p g, or an empty one (count 0) for a gesture that
 * is not available yet or not a gesture.
 */
Script script(Gesture g);

/**
 * @brief True when @p id names a gesture with a script.
 */
bool available(uint8_t id);

/**
 * @brief Where a planted foot of @p leg is, in its leg frame, with the body
 * at @p pose over a foot standing at @p standing.
 */
Vector3 plantedFoot(int leg, Vector3 standing, const Pose &pose);

/**
 * @brief Keyframe @p k's body pose, standing at @p standing: its pose, with
 * an underside height above the ground (zAboveGround) turned into a shift.
 */
Pose keyframePose(const Keyframe &k, Vector3 standing);

/**
 * @brief Foot target of @p leg at keyframe @p k's end, standing at @p standing.
 */
Vector3 keyframeFoot(const Keyframe &k, int leg, Vector3 standing);

// Fastest a foot may move during a gesture, at the peak of an eased move
// (1.5 times its average speed): a keyframe takes longer than scripted when
// its largest foot move needs it.
constexpr float MAX_FOOT_SPEED_MM_S = 300;
constexpr uint32_t RETURN_MS = 400; // each step of a stopped gesture's return

// What the player is moving towards.
enum class Stage : uint8_t
{
  Keyframes, // the script, keyframe `index`
  Holding,   // last keyframe, waiting for a stop request
  Unfold,    // holding script: safe return through its unfold keyframes
  Plant,     // stopped: lifted feet down onto their planted spots, body as it is
  Straighten // stopped: the body back over its standing feet
};

// The player: one eased pose segment at a time, with foot endpoints for
// legs that have explicit targets.
struct Player
{
  Gesture gesture = Gesture::None;
  Script sequence = {nullptr, 0}; // what it plays: script(gesture), or a test's own
  Stage stage = Stage::Keyframes;
  int index = 0;                 // keyframe in progress
  uint32_t elapsedMs = 0;        // within the segment
  uint32_t durationMs = 0;       // of the segment, rate limit included
  Vector3 from[LEG_COUNT];       // feet at the segment's start
  Vector3 to[LEG_COUNT];         // and end
  bool straight[LEG_COUNT] = {}; // a target in the segment's keyframe, or not starting where the pose plants it
  Pose pose;                     // the body pose reached by the last keyframe
  bool stopRequested = false;
  bool done = true;
};

/**
 * @brief Starts @p g from the feet at @p feet, standing at @p standing.
 */
void start(Player &p, Gesture g, const Vector3 *feet, Vector3 standing);

/**
 * @brief Starts @p sequence as gesture @p g: for host tests that play a
 * script of their own.
 */
void start(Player &p, Gesture g, const Script &sequence, const Vector3 *feet, Vector3 standing);

/**
 * @brief Advances by @p dtMs, carrying time over segment boundaries, and
 * writes every leg's foot target to @p out. After a stop request, the
 * keyframe in progress completes, lifted feet come down, then the body
 * straightens over its standing feet. A holding script instead plays its
 * unfold from the completed keyframe, including stops before the hold.
 * Sets done only after the return; the hold itself never finishes.
 */
void advance(Player &p, uint32_t dtMs, Vector3 standing, Vector3 *out);

/**
 * @brief The shortest duration in which an eased move of @p distanceMm keeps
 * to @p speedMmS.
 */
uint32_t minimumDurationMs(float distanceMm, float speedMmS = MAX_FOOT_SPEED_MM_S);

/**
 * @brief Smoothstep easing, 0..1 to 0..1.
 */
float ease(float t);

} // namespace gesture

#endif // GESTURE_H
