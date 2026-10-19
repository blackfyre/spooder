#include "gesture.h"

#include <math.h>
#include "gait.h"

namespace gesture
{

static const Lift NO_LIFT = {-1, Vector3()};

// --- Scripts ---------------------------------------------------------------

// Say hi: lean back, to the right and a little down, away from the front
// left leg (leg 1, index 0), raise it, wave it twice, put it down and stand
// straight. Lowering the body keeps the outer planted feet within the
// tibia's travel when the body is high; the wave stays below 70 mm over the
// coxa, as higher the femur passes its travel (test_gesture checks both
// with the default calibration).
static const Vector3 HI_SHIFT(30, -35, -25);
static const Keyframe SAY_HI[] = {
    {600, HI_SHIFT, 0, {NO_LIFT, NO_LIFT}},
    {400, HI_SHIFT, 0, {{0, Vector3(200, 0, 40)}, NO_LIFT}},
    {350, HI_SHIFT, 0, {{0, Vector3(190, 60, 70)}, NO_LIFT}},
    {350, HI_SHIFT, 0, {{0, Vector3(190, -60, 70)}, NO_LIFT}},
    {350, HI_SHIFT, 0, {{0, Vector3(190, 60, 70)}, NO_LIFT}},
    {350, HI_SHIFT, 0, {{0, Vector3(190, -60, 70)}, NO_LIFT}},
    {400, HI_SHIFT, 0, {{0, Vector3(200, 0, 40)}, NO_LIFT}},
    {400, HI_SHIFT, 0, {NO_LIFT, NO_LIFT}},
    {600, Vector3(), 0, {NO_LIFT, NO_LIFT}},
};

// Wiggle: sway side to side with a small bob, all six feet planted, then
// stand straight. The twist waits for the measured body geometry (part two):
// a uniform sideways move of every foot is not a rigid rotation.
static const Keyframe WIGGLE[] = {
    {400, Vector3(25, 0, 8), 0, {NO_LIFT, NO_LIFT}},
    {300, Vector3(-25, 0, -8), 0, {NO_LIFT, NO_LIFT}},
    {300, Vector3(25, 0, 8), 0, {NO_LIFT, NO_LIFT}},
    {300, Vector3(-25, 0, -8), 0, {NO_LIFT, NO_LIFT}},
    {300, Vector3(25, 0, 8), 0, {NO_LIFT, NO_LIFT}},
    {300, Vector3(-25, 0, -8), 0, {NO_LIFT, NO_LIFT}},
    {400, Vector3(), 0, {NO_LIFT, NO_LIFT}},
};

Script script(Gesture g)
{
  switch (g)
  {
  case Gesture::SayHi:
    return {SAY_HI, (int)(sizeof(SAY_HI) / sizeof(SAY_HI[0]))};
  case Gesture::Wiggle:
    return {WIGGLE, (int)(sizeof(WIGGLE) / sizeof(WIGGLE[0]))};
  default:
    // Dance, attack and ball need the measured body geometry
    // (add-gestures part two).
    return {nullptr, 0};
  }
}

bool available(uint8_t id)
{
  return id > 0 && id < GESTURE_ID_COUNT && script((Gesture)id).count > 0;
}

// --- Poses -----------------------------------------------------------------

Vector3 plantedFoot(int leg, Vector3 standing, Vector3 shift, float twistMm)
{
  // The body moves by shift over feet that stay put, so each foot moves by
  // -shift relative to the body: projected onto the leg frame's axes.
  float a = gait::mountAngleDeg(leg) * (float)M_PI / 180.0f;
  float ex = cosf(a), ey = sinf(a);         // leg x in the body frame
  float yx = sinf(a), yy = -cosf(a);        // leg y: x turned clockwise
  float dx = -shift.x, dy = -shift.y;
  return Vector3(standing.x + dx * ex + dy * ey, standing.y + dx * yx + dy * yy + twistMm, standing.z - shift.z);
}

Vector3 keyframeFoot(const Keyframe &k, int leg, Vector3 standing)
{
  for (int j = 0; j < MAX_LIFTED_LEGS; j++)
    if (k.lifts[j].leg == leg)
      return k.lifts[j].foot;
  return plantedFoot(leg, standing, k.shift, k.twistMm);
}

float ease(float t)
{
  if (t <= 0)
    return 0;
  if (t >= 1)
    return 1;
  return t * t * (3 - 2 * t);
}

// --- Player ----------------------------------------------------------------

uint32_t minimumDurationMs(float distanceMm)
{
  // Smoothstep's peak speed is 1.5 times the average.
  return (uint32_t)ceilf(1.5f * distanceMm / MAX_FOOT_SPEED_MM_S * 1000.0f);
}

// Sets the segment's targets and duration for the current stage.
static void beginSegment(Player &p, Vector3 standing)
{
  uint32_t base = RETURN_MS;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    switch (p.stage)
    {
    case Stage::Keyframes:
      p.to[i] = keyframeFoot(script(p.gesture).frames[p.index], i, standing);
      break;
    case Stage::Plant:
      p.to[i] = plantedFoot(i, standing, p.shift, p.twistMm);
      break;
    case Stage::Straighten:
      p.to[i] = standing;
      break;
    }
  }
  float longest = 0;
  for (int i = 0; i < LEG_COUNT; i++)
    longest = fmaxf(longest, p.from[i].distanceTo(p.to[i]));
  if (p.stage == Stage::Keyframes)
    base = script(p.gesture).frames[p.index].durationMs;
  else if (longest < 0.5f)
    base = 0; // a return step with nothing to move: skip it
  uint32_t minimum = minimumDurationMs(longest);
  p.durationMs = base > minimum ? base : minimum;
  if (p.durationMs == 0)
    p.durationMs = 1;
  p.elapsedMs = 0;
}

// Moves on from a completed segment.
static void nextSegment(Player &p, Vector3 standing)
{
  Script s = script(p.gesture);
  switch (p.stage)
  {
  case Stage::Keyframes:
    p.shift = s.frames[p.index].shift;
    p.twistMm = s.frames[p.index].twistMm;
    p.index++;
    if (p.index >= s.count)
    {
      p.done = true; // the last keyframe stands straight
      return;
    }
    if (p.stopRequested)
      p.stage = Stage::Plant;
    break;
  case Stage::Plant:
    p.stage = Stage::Straighten;
    break;
  case Stage::Straighten:
    p.done = true;
    return;
  }
  beginSegment(p, standing);
}

void start(Player &p, Gesture g, const Vector3 *feet, Vector3 standing)
{
  p = Player();
  p.gesture = g;
  for (int i = 0; i < LEG_COUNT; i++)
    p.from[i] = p.to[i] = feet[i];
  p.done = script(g).count == 0;
  if (!p.done)
    beginSegment(p, standing);
}

void advance(Player &p, uint32_t dtMs, Vector3 standing, Vector3 *out)
{
  uint32_t remaining = dtMs;
  while (!p.done)
  {
    uint32_t left = p.durationMs - p.elapsedMs;
    if (remaining < left)
    {
      p.elapsedMs += remaining;
      float t = ease((float)p.elapsedMs / (float)p.durationMs);
      for (int i = 0; i < LEG_COUNT; i++)
        out[i] = interpolate(p.from[i], p.to[i], t);
      return;
    }
    // The segment completes within this step; the rest carries over.
    remaining -= left;
    for (int i = 0; i < LEG_COUNT; i++)
      p.from[i] = p.to[i];
    nextSegment(p, standing);
  }
  for (int i = 0; i < LEG_COUNT; i++)
    out[i] = p.from[i];
}

} // namespace gesture
