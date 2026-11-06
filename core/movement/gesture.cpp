#include "gesture.h"

#include <math.h>
#include "gait.h"
#include "body_geometry.h"

namespace gesture
{


// --- Scripts ---------------------------------------------------------------

// Say hi: lean back, to the right and a little down, away from the front
// left leg (leg 1, index 0), raise it, wave it twice, put it down and stand
// straight. Lowering the body keeps the outer planted feet within the
// tibia's travel when the body is high; the wave stays below 70 mm over the
// coxa, as higher the femur passes its travel (test_gesture checks both
// with the default calibration).
static const Vector3 HI_SHIFT(30, -35, -25);
static const Keyframe SAY_HI[] = {
    {600, {HI_SHIFT}, {}},
    {400, {HI_SHIFT}, {{0, Vector3(200, 0, 40)}}},
    {350, {HI_SHIFT}, {{0, Vector3(190, 60, 70)}}},
    {350, {HI_SHIFT}, {{0, Vector3(190, -60, 70)}}},
    {350, {HI_SHIFT}, {{0, Vector3(190, 60, 70)}}},
    {350, {HI_SHIFT}, {{0, Vector3(190, -60, 70)}}},
    {400, {HI_SHIFT}, {{0, Vector3(200, 0, 40)}}},
    {400, {HI_SHIFT}, {}},
    {600, {Vector3()}, {}},
};

// Wiggle: sway side to side with a small bob, all six feet planted, then
// stand straight. The twist waits for the measured body geometry (part two):
// a uniform sideways move of every foot is not a rigid rotation.
static const Keyframe WIGGLE[] = {
    {400, {Vector3(25, 0, 8)}, {}},   {300, {Vector3(-25, 0, -8)}, {}}, {300, {Vector3(25, 0, 8)}, {}},
    {300, {Vector3(-25, 0, -8)}, {}}, {300, {Vector3(25, 0, 8)}, {}},   {300, {Vector3(-25, 0, -8)}, {}},
    {400, {Vector3()}, {}},
};

// Attack: brace on the middle and rear four, raise the front pair near
// the right femur's travel, strike forward and down twice, then plant.
// The strike's foot limit is raised; body::step still governs joint speed.
static const Pose ATTACK_BRACE = {Vector3(0, -40, -15), 0, 4, 0};
static const Keyframe ATTACK[] = {
    {450, ATTACK_BRACE, {}},
    {450, ATTACK_BRACE, {{0, Vector3(210, 0, 75)}, {5, Vector3(210, 0, 75)}}},
    {180, ATTACK_BRACE, {{0, Vector3(240, 35, -30)}, {5, Vector3(240, -35, -30)}}, 900},
    {400, ATTACK_BRACE, {{0, Vector3(210, 0, 75)}, {5, Vector3(210, 0, 75)}}},
    {180, ATTACK_BRACE, {{0, Vector3(240, 35, -30)}, {5, Vector3(240, -35, -30)}}, 900},
    {450, ATTACK_BRACE, {}},
    {450, {}, {}},
};

// Provisional lowering only: task 2.5 replaces this with the underside-derived
// ground clearance. All rotations stay zero while task 2.2 runs separately.
constexpr float BALL_PROVISIONAL_LOWERING_MM = 10;
static const Pose BALL_LOW = {Vector3(0, 0, -BALL_PROVISIONAL_LOWERING_MM)};
static const Keyframe BALL[] = {
    {600, BALL_LOW, {}},
    {800,
     BALL_LOW,
     {{0, Vector3(160, 0, -80)},
      {1, Vector3(160, 0, -80)},
      {2, Vector3(160, 0, -80)},
      {3, Vector3(160, 0, -80)},
      {4, Vector3(160, 0, -80)},
      {5, Vector3(160, 0, -80)}}},
};
static const Keyframe BALL_UNFOLD[] = {{800, BALL_LOW, {}}, {600, {Vector3()}, {}}};

Script script(Gesture g)
{
  switch (g)
  {
  case Gesture::SayHi:
    return {SAY_HI, (int)(sizeof(SAY_HI) / sizeof(SAY_HI[0]))};
  case Gesture::Wiggle:
    return {WIGGLE, (int)(sizeof(WIGGLE) / sizeof(WIGGLE[0]))};
  case Gesture::Attack:
    return {ATTACK, (int)(sizeof(ATTACK) / sizeof(ATTACK[0]))};
  case Gesture::Ball:
    return {BALL, 2, true, BALL_UNFOLD, 2};
  default:
    return {nullptr, 0};
  }
}

bool available(uint8_t id)
{
  return id > 0 && id < GESTURE_ID_COUNT && script((Gesture)id).count > 0;
}

// --- Poses -----------------------------------------------------------------

Vector3 plantedFoot(int leg, Vector3 standing, const Pose &pose)
{
  float a = gait::mountAngleDeg(leg) * (float)M_PI / 180.0f;
  float ex = cosf(a), ey = sinf(a);
  float yx = ey, yy = -ex; // leg y: x turned clockwise
  Vector3 shift = pose.translation;
  if (pose.yawDeg == 0 && pose.pitchDeg == 0 && pose.rollDeg == 0)
    return Vector3(standing.x - shift.x * ex - shift.y * ey, standing.y - shift.x * yx - shift.y * yy,
                   standing.z - shift.z);
  const Vector3 &pivot = body_geometry::PIVOTS[leg];
  Vector3 v(pivot.x + standing.x * ex + standing.y * yx - shift.x,
            pivot.y + standing.x * ey + standing.y * yy - shift.y, pivot.z + standing.z - shift.z);
  // Undo Rz(yaw) Rx(pitch) Ry(roll), in reverse order.
  float yaw = pose.yawDeg * (float)M_PI / 180.0f;
  float pitch = pose.pitchDeg * (float)M_PI / 180.0f;
  float roll = pose.rollDeg * (float)M_PI / 180.0f;
  v = Vector3(cosf(yaw) * v.x + sinf(yaw) * v.y, -sinf(yaw) * v.x + cosf(yaw) * v.y, v.z);
  v = Vector3(v.x, cosf(pitch) * v.y + sinf(pitch) * v.z, -sinf(pitch) * v.y + cosf(pitch) * v.z);
  v = Vector3(cosf(roll) * v.x - sinf(roll) * v.z, v.y, sinf(roll) * v.x + cosf(roll) * v.z);
  v = v - pivot;
  return Vector3(v.x * ex + v.y * ey, v.x * yx + v.y * yy, v.z);
}

Vector3 keyframeFoot(const Keyframe &k, int leg, Vector3 standing)
{
  for (int j = 0; j < MAX_LIFTED_LEGS; j++)
    if (k.lifts[j].leg == leg)
      return k.lifts[j].foot;
  return plantedFoot(leg, standing, k.pose);
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

uint32_t minimumDurationMs(float distanceMm, float speedMmS)
{
  // Smoothstep's peak speed is 1.5 times the average.
  return (uint32_t)ceilf(1.5f * distanceMm / speedMmS * 1000.0f);
}

// The keyframe the current segment moves to: in the script or its unfold;
// none while planting, straightening or holding.
static const Keyframe *segmentFrame(const Player &p)
{
  if (p.stage == Stage::Keyframes)
    return &p.sequence.frames[p.index];
  if (p.stage == Stage::Unfold)
    return &p.sequence.unfold[p.index];
  return nullptr;
}

static Pose segmentPose(const Player &p, float t)
{
  // Planting and holding keep the pose; straightening ends at none.
  const Keyframe *k = segmentFrame(p);
  Pose end;
  if (k)
    end = k->pose;
  else if (p.stage == Stage::Plant || p.stage == Stage::Holding)
    end = p.pose;
  return {interpolate(p.pose.translation, end.translation, t), p.pose.yawDeg + (end.yawDeg - p.pose.yawDeg) * t,
          p.pose.pitchDeg + (end.pitchDeg - p.pose.pitchDeg) * t, p.pose.rollDeg + (end.rollDeg - p.pose.rollDeg) * t};
}

static bool hasTarget(const Keyframe &k, int leg)
{
  for (const Lift &lift : k.lifts)
    if (lift.leg == leg)
      return true;
  return false;
}

static Vector3 segmentFoot(const Player &p, int leg, Vector3 standing, const Pose &pose, float t)
{
  return p.straight[leg] ? interpolate(p.from[leg], p.to[leg], t) : plantedFoot(leg, standing, pose);
}

// Sets the segment's targets and duration for the current stage.
static void beginSegment(Player &p, Vector3 standing)
{
  uint32_t base = RETURN_MS;
  const Keyframe *k = segmentFrame(p);
  for (int i = 0; i < LEG_COUNT; i++)
  {
    // A foot moves straight when the segment gives it a target of its own,
    // or when it does not start where the pose plants it (a target of the
    // previous keyframe, or a foot away from its spot when the gesture
    // starts); otherwise it stays planted through the pose.
    p.straight[i] = (k && hasTarget(*k, i)) || p.from[i].distanceTo(plantedFoot(i, standing, p.pose)) > 0.01f;
    switch (p.stage)
    {
    case Stage::Holding:
      p.to[i] = p.from[i];
      break;
    case Stage::Unfold:
      p.to[i] = keyframeFoot(p.sequence.unfold[p.index], i, standing);
      break;
    case Stage::Keyframes:
      p.to[i] = keyframeFoot(p.sequence.frames[p.index], i, standing);
      break;
    case Stage::Plant:
      p.to[i] = plantedFoot(i, standing, p.pose);
      break;
    case Stage::Straighten:
      p.to[i] = standing;
      break;
    }
  }
  float longest = 0;
  float lengths[LEG_COUNT] = {};
  Vector3 previous[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
    previous[i] = p.from[i];
  // Eight pieces measure the arc rather than its endpoint chord.
  for (int sample = 1; sample <= 8; sample++)
  {
    float t = (float)sample / 8;
    Pose pose = segmentPose(p, t);
    for (int i = 0; i < LEG_COUNT; i++)
    {
      Vector3 foot = segmentFoot(p, i, standing, pose, t);
      lengths[i] += previous[i].distanceTo(foot);
      previous[i] = foot;
      longest = fmaxf(longest, lengths[i]);
    }
  }
  if (k)
    base = k->durationMs;
  else if (longest < 0.5f)
    base = 0; // a return step with nothing to move: skip it
  float speed = k ? k->maxFootSpeedMmS : 0;
  uint32_t minimum = minimumDurationMs(longest, speed != 0 ? speed : MAX_FOOT_SPEED_MM_S);
  p.durationMs = base > minimum ? base : minimum;
  if (p.durationMs == 0)
    p.durationMs = 1;
  p.elapsedMs = 0;
}

// Moves on from a completed segment.
static void nextSegment(Player &p, Vector3 standing)
{
  const Script &s = p.sequence;
  switch (p.stage)
  {
  case Stage::Keyframes:
    p.pose = s.frames[p.index].pose;
    p.index++;
    if (s.holds && p.stopRequested)
    {
      // Even before the hold, finish this keyframe then take the unfold.
      // Its first target spreads the feet at the lowered height before rising.
      p.stage = Stage::Unfold;
      p.index = 0;
      break;
    }
    if (p.index >= s.count)
    {
      if (s.holds)
      {
        p.stage = Stage::Holding;
        return;
      }
      p.done = true; // the last keyframe stands straight
      return;
    }
    if (p.stopRequested)
      p.stage = Stage::Plant;
    break;
  case Stage::Holding:
    p.stage = Stage::Unfold;
    p.index = 0;
    break;
  case Stage::Unfold:
    p.pose = s.unfold[p.index].pose;
    if (++p.index >= s.unfoldCount)
    {
      p.done = true;
      return;
    }
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
  start(p, g, script(g), feet, standing);
}

void start(Player &p, Gesture g, const Script &sequence, const Vector3 *feet, Vector3 standing)
{
  p = Player();
  p.gesture = g;
  p.sequence = sequence;
  for (int i = 0; i < LEG_COUNT; i++)
    p.from[i] = p.to[i] = feet[i];
  p.done = sequence.count == 0;
  if (!p.done)
    beginSegment(p, standing);
}

void advance(Player &p, uint32_t dtMs, Vector3 standing, Vector3 *out)
{
  uint32_t remaining = dtMs;
  while (!p.done)
  {
    if (p.stage == Stage::Holding)
    {
      if (!p.stopRequested)
        break;
      nextSegment(p, standing);
    }
    uint32_t left = p.durationMs - p.elapsedMs;
    if (remaining < left)
    {
      p.elapsedMs += remaining;
      float t = ease((float)p.elapsedMs / (float)p.durationMs);
      Pose pose = segmentPose(p, t);
      for (int i = 0; i < LEG_COUNT; i++)
        out[i] = segmentFoot(p, i, standing, pose, t);
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
