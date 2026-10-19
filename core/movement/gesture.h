#ifndef GESTURE_H
#define GESTURE_H

// Scripted gestures for the Perform body mode: constant keyframe tables and
// the player that turns them into foot targets in each leg's frame.
// Hardware-free: compiled for the board and for host tests.
//
// A keyframe is a body pose (a shift of the body over its planted feet, a
// body height change and a twist) plus up to two lifted legs with their own
// foot targets, reached with an eased move over the keyframe's duration from
// where the previous keyframe ended. A planted foot stays where it is on the
// ground: shifting the body moves it the other way in its leg frame.
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

constexpr int MAX_LIFTED_LEGS = 2;

struct Lift
{
  int8_t leg;   // -1 when unused
  Vector3 foot; // target in the leg frame
};

struct Keyframe
{
  uint16_t durationMs;
  Vector3 shift; // body over its planted feet, body frame: x right, y forward, z up (mm)
  float twistMm; // planted feet moved along each leg frame's y: + turns the body anticlockwise
  Lift lifts[MAX_LIFTED_LEGS];
};

struct Script
{
  const Keyframe *frames;
  int count;
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
 * at @p shift and @p twistMm over a foot standing at @p standing.
 */
Vector3 plantedFoot(int leg, Vector3 standing, Vector3 shift, float twistMm);

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
  Plant,     // stopped: lifted feet down onto their planted spots, body as it is
  Straighten // stopped: the body back over its standing feet
};

// The player: one eased move (a segment) at a time, from `from` to `to`.
struct Player
{
  Gesture gesture = Gesture::None;
  Stage stage = Stage::Keyframes;
  int index = 0;             // keyframe in progress
  uint32_t elapsedMs = 0;    // within the segment
  uint32_t durationMs = 0;   // of the segment, rate limit included
  Vector3 from[LEG_COUNT];   // feet at the segment's start
  Vector3 to[LEG_COUNT];     // and end
  Vector3 shift;             // the body shift reached by the last keyframe
  float twistMm = 0;
  bool stopRequested = false;
  bool done = true;
};

/**
 * @brief Starts @p g from the feet at @p feet, standing at @p standing.
 */
void start(Player &p, Gesture g, const Vector3 *feet, Vector3 standing);

/**
 * @brief Advances by @p dtMs, carrying time over segment boundaries, and
 * writes every leg's foot target to @p out. After a stop request, the
 * keyframe in progress completes, lifted feet come down, then the body
 * straightens over its standing feet. Sets done at the end.
 */
void advance(Player &p, uint32_t dtMs, Vector3 standing, Vector3 *out);

/**
 * @brief The shortest duration in which an eased move of @p distanceMm keeps
 * to MAX_FOOT_SPEED_MM_S.
 */
uint32_t minimumDurationMs(float distanceMm);

/**
 * @brief Smoothstep easing, 0..1 to 0..1.
 */
float ease(float t);

} // namespace gesture

#endif // GESTURE_H
