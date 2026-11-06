#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "movement/body_geometry.h"
#include "movement/centre_of_gravity.h"
#include "movement/gait.h"
#include "movement/gesture.h"
#include "movement/joint_calibration.h"
#include "movement/kinematics.h"

using namespace gesture;

void setUp() {}
void tearDown() {}

static const gesture::Gesture AVAILABLE[] = {Gesture::SayHi, Gesture::Wiggle, Gesture::Dance, Gesture::Attack};
// Feet at body height 0 and 100 (body::groundForHeight).
static const float GROUNDS[] = {-60, -230};

// Say hi's body shift (gesture.cpp), for the planted height after a stop.
static const Vector3 HI_SHIFT_FOR_TEST(30, -35, -25);

static Vector3 standingAt(float ground)
{
  return Vector3(gait::STANCE_DISTANCE, 0, ground);
}

// The leg frame's axes in the body frame (x right, y forward).
static Vector2 legX(int leg)
{
  float a = gait::mountAngleDeg(leg) * (float)M_PI / 180.0f;
  return Vector2(cosf(a), sinf(a));
}
static Vector2 legY(int leg)
{
  float a = gait::mountAngleDeg(leg) * (float)M_PI / 180.0f;
  return Vector2(sinf(a), -cosf(a));
}

static Vector2 toBody(int leg, Vector3 foot)
{
  Vector2 x = legX(leg), y = legY(leg);
  const Vector3 &pivot = body_geometry::PIVOTS[leg];
  return Vector2(pivot.x + foot.x * x.x + foot.y * y.x, pivot.y + foot.x * x.y + foot.y * y.y);
}

// The servo angle (commanded plus offset) of every joint is within 0-180.
static bool withinServoRange(int leg, Vector3 foot)
{
  joint_calibration::Table t;
  joint_calibration::loadDefaults(t);
  kinematics::JointAngles a;
  if (!kinematics::solveLeg(foot, joint_calibration::legOffsets(t, leg), a))
    return false;
  const float angles[3] = {a.coxa, a.femur, a.tibia};
  for (float v : angles)
    if (!(v >= 0 && v <= 180))
      return false;
  return true;
}

// Signed distance of p inside the convex polygon (counter-clockwise), the
// smallest distance to any edge; negative when outside.
static float insideMargin(const Vector2 *poly, int n, Vector2 p)
{
  float margin = 1e9f;
  for (int i = 0; i < n; i++)
  {
    Vector2 a = poly[i], b = poly[(i + 1) % n];
    float ex = b.x - a.x, ey = b.y - a.y;
    float len = sqrtf(ex * ex + ey * ey);
    float d = (ex * (p.y - a.y) - ey * (p.x - a.x)) / len;
    margin = fminf(margin, d);
  }
  return margin;
}

void test_only_scripted_gestures_are_available()
{
  TEST_ASSERT_FALSE(available(0));
  TEST_ASSERT_TRUE(available((uint8_t)Gesture::SayHi));
  TEST_ASSERT_TRUE(available((uint8_t)Gesture::Wiggle));
  TEST_ASSERT_TRUE(available((uint8_t)Gesture::Dance));
  TEST_ASSERT_TRUE(available((uint8_t)Gesture::Attack));
  TEST_ASSERT_TRUE(available((uint8_t)Gesture::Ball));
  TEST_ASSERT_FALSE(available(GESTURE_ID_COUNT));
  TEST_ASSERT_FALSE(available(255));
}

void test_shifting_the_body_moves_planted_feet_the_other_way()
{
  Vector3 standing = standingAt(-100);
  Vector3 shift(20, 30, 10);
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    Vector2 before = toBody(leg, standing);
    Vector3 foot = plantedFoot(leg, standing, Pose{shift});
    Vector2 after = toBody(leg, foot);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -shift.x, after.x - before.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -shift.y, after.y - before.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, standing.z - shift.z, foot.z); // body up: feet further down
  }
}

void test_every_pose_is_reachable_and_within_servo_range()
{
  for (Gesture g : AVAILABLE)
    for (float ground : GROUNDS)
    {
      Vector3 standing = standingAt(ground);
      Vector3 feet[kinematics::LEG_COUNT];
      for (int i = 0; i < kinematics::LEG_COUNT; i++)
        feet[i] = standing;
      Player p;
      start(p, g, feet, standing);
      // Every 5 ms through the whole gesture, eased moves included.
      for (int n = 0; n < 4000 && !p.done; n++)
      {
        advance(p, 5, standing, feet);
        for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
        {
          TEST_ASSERT_TRUE_MESSAGE(kinematics::isValidTarget(feet[leg]), "unreachable gesture target");
          char detail[160];
          snprintf(detail, sizeof(detail), "gesture %d ground %.0f frame %d leg %d foot %.2f %.2f %.2f outside travel",
                   (int)g, (double)ground, p.index, leg, (double)feet[leg].x, (double)feet[leg].y, (double)feet[leg].z);
          TEST_ASSERT_TRUE_MESSAGE(withinServoRange(leg, feet[leg]), detail);
        }
      }
      TEST_ASSERT_TRUE(p.done);
    }
}

// Whether the body centre (the body frame's origin) is inside the polygon of
// the feet on the ground, with @p marginMm, whenever a foot is off it. A
// foot more than 10 mm above the lowest counts as lifted.
static Vector3 worldFoot(int leg, Vector3 foot, const Pose &pose);

static Vector3 worldPoint(Vector3 v, const Pose &pose)
{
  float r = pose.rollDeg * (float)M_PI / 180;
  float p = pose.pitchDeg * (float)M_PI / 180;
  float y = pose.yawDeg * (float)M_PI / 180;
  v = Vector3(cosf(r) * v.x + sinf(r) * v.z, v.y, -sinf(r) * v.x + cosf(r) * v.z);
  v = Vector3(v.x, cosf(p) * v.y - sinf(p) * v.z, sinf(p) * v.y + cosf(p) * v.z);
  v = Vector3(cosf(y) * v.x - sinf(y) * v.y, sinf(y) * v.x + cosf(y) * v.y, v.z);
  return v + pose.translation;
}

static Pose playerPose(const Player &p)
{
  if (p.done)
    return {};
  Pose end = p.stage == Stage::Keyframes ? p.sequence.frames[p.index].pose : p.stage == Stage::Plant ? p.pose : Pose{};
  float t = ease((float)p.elapsedMs / p.durationMs);
  return {interpolate(p.pose.translation, end.translation, t), p.pose.yawDeg + (end.yawDeg - p.pose.yawDeg) * t,
          p.pose.pitchDeg + (end.pitchDeg - p.pose.pitchDeg) * t, p.pose.rollDeg + (end.rollDeg - p.pose.rollDeg) * t};
}

static bool supported(const Vector3 *feet, float marginMm, const Pose &pose)
{
  float lowest = 1e9f;
  Vector3 world[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
  {
    world[i] = worldFoot(i, feet[i], pose);
    lowest = fminf(lowest, world[i].z);
  }
  // Anticlockwise: front right, front left, middle left, rear left,
  // rear right, middle right. Height above the floor is in the world frame.
  const int ORDER[] = {5, 0, 1, 2, 3, 4};
  Vector2 poly[LEG_COUNT];
  int n = 0;
  for (int leg : ORDER)
    if (world[leg].z <= lowest + 10)
      poly[n++] = Vector2(world[leg].x, world[leg].y);
  return n == LEG_COUNT || insideMargin(poly, n, Vector2(pose.translation.x, pose.translation.y)) >= marginMm;
}

// The smallest support margin of the estimated centre of gravity, with and
// without the battery.
static float gravityMargin(const Vector3 *feet, const Pose &pose)
{
  kinematics::JointAngles legs[kinematics::LEG_COUNT];
  Vector3 body[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
  {
    TEST_ASSERT_TRUE(kinematics::solveLeg(feet[i], Vector3(0, 0, 0), legs[i]));
    body[i] = worldFoot(i, feet[i], pose);
  }
  float worst = 1e9f;
  for (int battery = 0; battery < 2; battery++)
  {
    float margin;
    Vector3 centre = worldPoint(centre_of_gravity::estimate(legs, battery).centre, pose);
    TEST_ASSERT_TRUE(centre_of_gravity::supportMargin(body, centre, margin));
    worst = fminf(worst, margin);
  }
  return worst;
}

void test_lifted_legs_keep_the_body_over_its_support()
{
  // Sampled through the whole gesture, eased moves and interrupted returns
  // included. The pivots are measured from the frame's STLs (body_geometry)
  // but the masses estimated until the robot is weighed, so this guards the
  // scripts' intent, not the physical margin.
  const float MARGIN_MM = 20;
  const uint32_t STOPS[] = {0, 1200, 1900, 2600, 3575, 3725};
  for (Gesture g : AVAILABLE)
    for (float ground : GROUNDS)
      for (uint32_t stopAt : STOPS)
      {
        if (g == Gesture::Attack)
          continue; // pitched support is checked in world coordinates below
        Vector3 standing = standingAt(ground);
        Vector3 feet[kinematics::LEG_COUNT];
        for (int i = 0; i < kinematics::LEG_COUNT; i++)
          feet[i] = standing;
        Player p;
        start(p, g, feet, standing);
        for (uint32_t t = 0; !p.done && t < 60000; t += 5)
        {
          if (stopAt && t >= stopAt)
            p.stopRequested = true;
          advance(p, 5, standing, feet);
          Pose pose = playerPose(p);
          TEST_ASSERT_TRUE_MESSAGE(supported(feet, MARGIN_MM, pose), "body outside its support");
          TEST_ASSERT_TRUE_MESSAGE(gravityMargin(feet, pose) >= MARGIN_MM, "centre of gravity outside its support");
        }
      }
}

// Plays @p g to the end from standing, in steps of @p stepMs, optionally
// stopping after @p stopAfterMs; checks the foot speed limit on the way and
// returns the time taken.
static uint32_t playChecked(Gesture g, float ground, uint32_t stepMs, uint32_t stopAfterMs = 0)
{
  Vector3 standing = standingAt(ground);
  Vector3 feet[kinematics::LEG_COUNT], before[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    feet[i] = standing;
  Player p;
  start(p, g, feet, standing);
  uint32_t t = 0;
  while (!p.done && t < 120000)
  {
    if (stopAfterMs && t >= stopAfterMs)
      p.stopRequested = true;
    for (int i = 0; i < kinematics::LEG_COUNT; i++)
      before[i] = feet[i];
    float limit = MAX_FOOT_SPEED_MM_S;
    if (p.stage == Stage::Keyframes)
      limit = fmaxf(limit, p.sequence.frames[p.index].maxFootSpeedMmS);
    advance(p, stepMs, standing, feet);
    if (!p.done && p.stage == Stage::Keyframes)
      limit = fmaxf(limit, p.sequence.frames[p.index].maxFootSpeedMmS);
    t += stepMs;
    for (int i = 0; i < kinematics::LEG_COUNT; i++)
    {
      float speed = feet[i].distanceTo(before[i]) / (stepMs / 1000.0f);
      // A step that crosses a segment end sees a little more than the peak.
      TEST_ASSERT_TRUE_MESSAGE(speed <= limit * 1.1f, "foot faster than the gesture limit");
    }
  }
  TEST_ASSERT_TRUE(p.done);
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, feet[i].distanceTo(standing));
  return t;
}

void test_gesture_keeps_the_speed_limit_and_ends_standing()
{
  for (Gesture g : AVAILABLE)
    for (float ground : GROUNDS)
    {
      Script s = script(g);
      uint32_t scripted = 0;
      for (int k = 0; k < s.count; k++)
        scripted += s.frames[k].durationMs;
      // At least the scripted time; longer where a move needs it.
      TEST_ASSERT_TRUE(playChecked(g, ground, 5) >= scripted);
    }
}

void test_time_carries_over_keyframe_boundaries()
{
  // The total does not depend on how the time is cut, beyond rounding up to
  // the last step.
  for (Gesture g : AVAILABLE)
  {
    uint32_t fine = playChecked(g, -100, 1);
    uint32_t coarse = playChecked(g, -100, 23);
    TEST_ASSERT_TRUE(coarse >= fine && coarse < fine + 23);
  }
}

void test_stop_finishes_the_keyframe_puts_feet_down_then_straightens()
{
  Vector3 standing = standingAt(-100);
  Vector3 feet[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    feet[i] = standing;
  Player p;
  start(p, Gesture::SayHi, feet, standing);
  // Into the wave (the leg is up), then stop.
  while (!(p.stage == Stage::Keyframes && p.index == 3))
    advance(p, 5, standing, feet);
  p.stopRequested = true;
  bool planted = false;
  while (!p.done)
  {
    advance(p, 5, standing, feet);
    TEST_ASSERT_TRUE(p.stage != Stage::Keyframes || p.index == 3);
    if (p.stage == Stage::Straighten && !planted)
    {
      // Every foot is on the ground before the body straightens.
      planted = true;
      for (int i = 0; i < kinematics::LEG_COUNT; i++)
        TEST_ASSERT_FLOAT_WITHIN(0.5f, standing.z - HI_SHIFT_FOR_TEST.z, p.from[i].z);
    }
  }
  TEST_ASSERT_TRUE(planted);
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, feet[i].distanceTo(standing));
  // The interrupted return keeps the speed limit and the poses reachable.
  playChecked(Gesture::SayHi, -230, 5, 1500);
  playChecked(Gesture::SayHi, -60, 5, 1500);
}

void test_ball_holds_and_every_interrupted_unfold_is_safe()
{
  Script ball = script(Gesture::Ball);
  for (float ground : GROUNDS)
    for (int stopFrame = 0; stopFrame <= ball.count; stopFrame++)
    {
      Vector3 standing = standingAt(ground), feet[LEG_COUNT];
      for (auto &foot : feet)
        foot = standing;
      Player p;
      start(p, Gesture::Ball, feet, standing);
      bool stopped = false, unfolded = false;
      int holdSteps = 0;
      for (int n = 0; n < 10000 && !p.done; n++)
      {
        if (stopFrame < ball.count && p.stage == Stage::Keyframes && p.index == stopFrame)
          p.stopRequested = stopped = true;
        if (p.stage == Stage::Holding)
        {
          TEST_ASSERT_FALSE(p.done);
          for (int leg = 0; leg < LEG_COUNT; leg++)
            TEST_ASSERT_FLOAT_WITHIN(0.001f, 0,
                                     feet[leg].distanceTo(keyframeFoot(ball.frames[ball.count - 1], leg, standing)));
          if (++holdSteps == 200)
            p.stopRequested = stopped = true;
        }
        advance(p, 5, standing, feet);
        if (stopped && p.stage == Stage::Keyframes)
          TEST_ASSERT_EQUAL_INT(stopFrame, p.index);
        if (!unfolded && p.stage == Stage::Unfold && stopFrame < ball.count)
          for (int leg = 0; leg < LEG_COUNT; leg++)
            TEST_ASSERT_FLOAT_WITHIN(0.001f, 0,
                                     p.from[leg].distanceTo(keyframeFoot(ball.frames[stopFrame], leg, standing)));
        unfolded |= p.stage == Stage::Unfold;
        TEST_ASSERT_TRUE(p.stage != Stage::Plant && p.stage != Stage::Straighten);
        for (int leg = 0; leg < LEG_COUNT; leg++)
        {
          TEST_ASSERT_TRUE(kinematics::isValidTarget(feet[leg]));
          TEST_ASSERT_TRUE(withinServoRange(leg, feet[leg]));
        }
      }
      TEST_ASSERT_TRUE(p.done);
      TEST_ASSERT_TRUE(unfolded);
      TEST_ASSERT_TRUE(stopped);
      for (auto foot : feet)
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0, foot.distanceTo(standing));
    }
}

void test_unavailable_gesture_is_done_at_once()
{
  Vector3 feet[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    feet[i] = standingAt(-100);
  Player p;
  start(p, (Gesture)GESTURE_ID_COUNT, feet, standingAt(-100)); // past the last gesture
  TEST_ASSERT_TRUE(p.done);
}

// Independently put a leg target back into the body-centre world frame.
static Vector3 worldFoot(int leg, Vector3 foot, const Pose &pose)
{
  Vector2 xy = toBody(leg, foot);
  Vector3 v(xy.x, xy.y, body_geometry::PIVOTS[leg].z + foot.z);
  return worldPoint(v, pose);
}

// Check support in the world frame: pitching makes planted leg-frame z
// coordinates differ, so selecting contacts there would discard planted feet.
void test_attack_keeps_four_feet_planted_and_supports_body_and_gravity()
{
  Script attack = script(Gesture::Attack);
  for (float ground : GROUNDS)
    for (int stopFrame = -1; stopFrame < attack.count; stopFrame++)
    {
      Vector3 standing = standingAt(ground), feet[LEG_COUNT];
      for (auto &foot : feet)
        foot = standing;
      Player p;
      start(p, Gesture::Attack, feet, standing);
      bool fourContacts = false;
      for (int n = 0; n < 12000 && !p.done; n++)
      {
        if (p.stage == Stage::Keyframes && p.index == stopFrame)
          p.stopRequested = true;
        advance(p, 5, standing, feet);
        Pose end = p.done                        ? Pose{}
                   : p.stage == Stage::Keyframes ? attack.frames[p.index].pose
                   : p.stage == Stage::Plant     ? p.pose
                                                 : Pose{};
        float t = p.done ? 1 : ease((float)p.elapsedMs / p.durationMs);
        Pose pose = {interpolate(p.pose.translation, end.translation, t), 0,
                     p.pose.pitchDeg + (end.pitchDeg - p.pose.pitchDeg) * t, 0};
        Vector3 world[LEG_COUNT];
        kinematics::JointAngles angles[LEG_COUNT];
        int contacts = 0;
        for (int leg = 0; leg < LEG_COUNT; leg++)
        {
          TEST_ASSERT_TRUE(kinematics::isValidTarget(feet[leg]));
          TEST_ASSERT_TRUE(withinServoRange(leg, feet[leg]));
          world[leg] = worldFoot(leg, feet[leg], pose);
          contacts += world[leg].z <= ground + centre_of_gravity::CONTACT_MM;
          if (leg != 0 && leg != 5)
            TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, world[leg].distanceTo(worldFoot(leg, standing, {})));
          TEST_ASSERT_TRUE(kinematics::solveLeg(feet[leg], {}, angles[leg]));
        }
        fourContacts |= contacts == 4;
        // Transform a body-frame point with the same independent forward
        // transform as worldFoot (undo its leg-to-body part first).
        auto inWorld = [&](Vector3 point)
        {
          Vector2 x = legX(0), y = legY(0);
          Vector3 v = point - body_geometry::PIVOTS[0];
          return worldFoot(0, Vector3(v.x * x.x + v.y * x.y, v.x * y.x + v.y * y.y, v.z), pose);
        };
        float margin;
        TEST_ASSERT_TRUE(centre_of_gravity::supportMargin(world, inWorld({}), margin));
        TEST_ASSERT_TRUE_MESSAGE(margin >= 20, "attack body support margin below 20 mm");
        for (int battery = 0; battery < 2; battery++)
        {
          TEST_ASSERT_TRUE(centre_of_gravity::supportMargin(
              world, inWorld(centre_of_gravity::estimate(angles, battery).centre), margin));
          TEST_ASSERT_TRUE_MESSAGE(margin >= 20, "attack gravity support margin below 20 mm");
        }
      }
      TEST_ASSERT_TRUE(p.done);
      if (stopFrame == -1)
        TEST_ASSERT_TRUE(fourContacts);
    }
}

void test_pose_keeps_planted_feet_fixed_through_rotation_and_stop()
{
  const Keyframe frames[] = {{1000, {Vector3(12, -9, 5), 25, 15, -18}, {}}, {1000, {}, {}}};
  for (int stop = 0; stop < 2; stop++)
  {
    Vector3 standing = standingAt(-100), feet[LEG_COUNT];
    for (Vector3 &foot : feet)
      foot = standing;
    Player player;
    start(player, Gesture::Dance, {frames, 2}, feet, standing);
    player.stopRequested = stop;
    while (!player.done)
    {
      advance(player, 5, standing, feet);
      Pose pose;
      if (!player.done)
      {
        float t = ease((float)player.elapsedMs / player.durationMs);
        Pose end = player.stage == Stage::Keyframes ? frames[player.index].pose
                   : player.stage == Stage::Plant   ? player.pose
                                                    : Pose{};
        pose = {interpolate(player.pose.translation, end.translation, t),
                player.pose.yawDeg + (end.yawDeg - player.pose.yawDeg) * t,
                player.pose.pitchDeg + (end.pitchDeg - player.pose.pitchDeg) * t,
                player.pose.rollDeg + (end.rollDeg - player.pose.rollDeg) * t};
      }
      for (int leg = 0; leg < LEG_COUNT; leg++)
        TEST_ASSERT_FLOAT_WITHIN(0.1f, 0, worldFoot(leg, feet[leg], pose).distanceTo(worldFoot(leg, standing, {})));
    }
  }
}

// The scenario's moves, played: within the first seven beats the body rolls
// both ways, nods, twists and bounces below its resting height.
void test_dance_rolls_nods_twists_and_bounces()
{
  Vector3 standing = standingAt(-100), feet[LEG_COUNT];
  for (Vector3 &foot : feet)
    foot = standing;
  Player p;
  start(p, Gesture::Dance, feet, standing);
  float rollMin = 0, rollMax = 0, pitch = 0, yaw = 0, zMin = 0;
  float rest = script(Gesture::Dance).frames[0].pose.translation.z;
  for (uint32_t t = 0; t < 3500 && !p.done; t += 5)
  {
    advance(p, 5, standing, feet);
    rollMin = fminf(rollMin, p.pose.rollDeg);
    rollMax = fmaxf(rollMax, p.pose.rollDeg);
    pitch = fmaxf(pitch, fabsf(p.pose.pitchDeg));
    yaw = fmaxf(yaw, fabsf(p.pose.yawDeg));
    zMin = fminf(zMin, p.pose.translation.z);
  }
  TEST_ASSERT_TRUE_MESSAGE(rollMin <= -1 && rollMax >= 1, "rolls left and right");
  TEST_ASSERT_TRUE_MESSAGE(pitch >= 1, "nods");
  TEST_ASSERT_TRUE_MESSAGE(yaw >= 1, "twists");
  TEST_ASSERT_TRUE_MESSAGE(zMin <= rest - 4, "bounces below its resting height");
}

void test_dance_keeps_the_beat_and_only_taps_on_the_last_beat()
{
  Script dance = script(Gesture::Dance);
  for (float ground : GROUNDS)
  {
    Vector3 standing = standingAt(ground), feet[LEG_COUNT];
    for (auto &foot : feet)
      foot = standing;
    Player p;
    start(p, Gesture::Dance, feet, standing);
    uint32_t elapsed = 0, scripted = 0;
    bool tapped = false, planted = false;
    for (int frame = 0; frame < dance.count; frame++)
    {
      TEST_ASSERT_EQUAL_INT(frame, p.index);
      TEST_ASSERT_EQUAL_UINT32(dance.frames[frame].durationMs, p.durationMs);
      scripted += dance.frames[frame].durationMs;
      while (!p.done && p.index == frame)
      {
        advance(p, 5, standing, feet);
        elapsed += 5;
        Pose end = dance.frames[frame].pose;
        float t = ease((float)(elapsed - (scripted - dance.frames[frame].durationMs)) / dance.frames[frame].durationMs);
        Pose from = frame ? dance.frames[frame - 1].pose : Pose{};
        Pose pose = {interpolate(from.translation, end.translation, t), from.yawDeg + (end.yawDeg - from.yawDeg) * t,
                     from.pitchDeg + (end.pitchDeg - from.pitchDeg) * t,
                     from.rollDeg + (end.rollDeg - from.rollDeg) * t};
        for (int leg = 0; leg < LEG_COUNT; leg++)
        {
          Vector3 world = worldFoot(leg, feet[leg], pose);
          Vector3 original = worldFoot(leg, standing, {});
          if (elapsed <= 3500 || leg != 0 || elapsed >= 3800)
            TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, world.distanceTo(original));
          if (leg == 0 && world.z > original.z + 10)
            tapped = true;
          if (tapped && world.distanceTo(original) < 0.01f && leg == 0)
            planted = true;
          if (elapsed > 3800)
            TEST_ASSERT_TRUE(planted);
        }
      }
      TEST_ASSERT_EQUAL_UINT32(scripted, elapsed);
      // The seven planted beats and the complete final tap/straighten beat.
      if (frame < 7 || frame == dance.count - 1)
        TEST_ASSERT_EQUAL_UINT32(0, elapsed % 500);
    }
    TEST_ASSERT_TRUE(tapped);
    TEST_ASSERT_TRUE(planted);
    TEST_ASSERT_TRUE(p.done);
    TEST_ASSERT_EQUAL_UINT32(4000, elapsed);
  }
}

// A holding script's unfold moves the body like any keyframe: planted feet
// stay put through its rotation (the unfold's own pose, not none).
void test_unfold_keeps_planted_feet_fixed_through_its_pose()
{
  const Keyframe frames[] = {{800, {Vector3(0, 0, -15), 10, -8, 12}, {}}};
  const Keyframe unfold[] = {{800, {Vector3(6, 4, -5), -12, 9, -6}, {}}, {800, {}, {}}};
  Script sequence = {frames, 1, true, unfold, 2};
  Vector3 standing = standingAt(-100), feet[LEG_COUNT];
  for (Vector3 &foot : feet)
    foot = standing;
  Player player;
  start(player, Gesture::Ball, sequence, feet, standing);
  bool unfolded = false;
  for (int step = 0; step < 2000 && !player.done; step++)
  {
    if (player.stage == Stage::Holding)
      player.stopRequested = true;
    advance(player, 5, standing, feet);
    if (player.done || player.stage == Stage::Holding)
      continue;
    float t = ease((float)player.elapsedMs / player.durationMs);
    const Keyframe *k = player.stage == Stage::Keyframes ? &frames[player.index]
                        : player.stage == Stage::Unfold  ? &unfold[player.index]
                                                         : nullptr;
    unfolded |= player.stage == Stage::Unfold;
    Pose end = k ? k->pose : Pose{};
    Pose pose = {interpolate(player.pose.translation, end.translation, t),
                 player.pose.yawDeg + (end.yawDeg - player.pose.yawDeg) * t,
                 player.pose.pitchDeg + (end.pitchDeg - player.pose.pitchDeg) * t,
                 player.pose.rollDeg + (end.rollDeg - player.pose.rollDeg) * t};
    for (int leg = 0; leg < LEG_COUNT; leg++)
      TEST_ASSERT_FLOAT_WITHIN(0.1f, 0, worldFoot(leg, feet[leg], pose).distanceTo(worldFoot(leg, standing, {})));
  }
  TEST_ASSERT_TRUE(unfolded);
  TEST_ASSERT_TRUE(player.done);
  for (int leg = 0; leg < LEG_COUNT; leg++)
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0, feet[leg].distanceTo(standing));
}

void test_explicit_foot_targets_follow_straight_segments()
{
  const Keyframe frames[] = {{1000, {Vector3(), 25, 10, -12}, {{0, Vector3(180, 20, -20)}}}, {1000, {}, {}}};
  Vector3 standing = standingAt(-100), feet[LEG_COUNT];
  for (Vector3 &foot : feet)
    foot = standing;
  Player player;
  start(player, Gesture::Dance, {frames, 2}, feet, standing);
  while (!player.done)
  {
    advance(player, 5, standing, feet);
    if (!player.done)
    {
      float t = ease((float)player.elapsedMs / player.durationMs);
      TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, feet[0].distanceTo(interpolate(player.from[0], player.to[0], t)));
    }
  }
}

void test_zero_rotation_matches_translation_for_every_leg()
{
  Vector3 standing(190, 17, -130), shift(23, -31, 11);
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    Vector2 x = legX(leg), y = legY(leg);
    Vector3 expected(standing.x - shift.x * x.x - shift.y * x.y, standing.y - shift.x * y.x - shift.y * y.y,
                     standing.z - shift.z);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, expected.distanceTo(plantedFoot(leg, standing, {shift})));
  }
}

void test_rotation_rate_limit_uses_the_path_and_keyframe_speed()
{
  Keyframe frame = {1, {Vector3(), 100, 0, 0}, {}};
  Vector3 standing = standingAt(-100), feet[LEG_COUNT];
  for (Vector3 &foot : feet)
    foot = standing;
  Player player;
  start(player, Gesture::Dance, {&frame, 1}, feet, standing);
  uint32_t duration = player.durationMs;
  float chord = 0;
  for (int leg = 0; leg < LEG_COUNT; leg++)
    chord = fmaxf(chord, standing.distanceTo(player.to[leg]));
  TEST_ASSERT_TRUE(duration > minimumDurationMs(chord));
  float longest = 0;
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    Vector3 world = worldFoot(leg, standing, {});
    longest = fmaxf(longest, sqrtf(world.x * world.x + world.y * world.y) * 100 * (float)M_PI / 180);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.003f * duration, minimumDurationMs(longest), duration);
  while (!player.done)
  {
    Vector3 before[LEG_COUNT];
    for (int leg = 0; leg < LEG_COUNT; leg++)
      before[leg] = feet[leg];
    advance(player, 5, standing, feet);
    for (int leg = 0; leg < LEG_COUNT; leg++)
      TEST_ASSERT_TRUE(feet[leg].distanceTo(before[leg]) / 0.005f <= MAX_FOOT_SPEED_MM_S * 1.003f);
  }
  frame.maxFootSpeedMmS = 600;
  for (Vector3 &foot : feet)
    foot = standing;
  start(player, Gesture::Dance, {&frame, 1}, feet, standing);
  TEST_ASSERT_TRUE(player.durationMs < duration);
  TEST_ASSERT_UINT32_WITHIN(1, duration / 2, player.durationMs);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_attack_keeps_four_feet_planted_and_supports_body_and_gravity);
  RUN_TEST(test_dance_keeps_the_beat_and_only_taps_on_the_last_beat);
  RUN_TEST(test_dance_rolls_nods_twists_and_bounces);
  RUN_TEST(test_pose_keeps_planted_feet_fixed_through_rotation_and_stop);
  RUN_TEST(test_explicit_foot_targets_follow_straight_segments);
  RUN_TEST(test_zero_rotation_matches_translation_for_every_leg);
  RUN_TEST(test_rotation_rate_limit_uses_the_path_and_keyframe_speed);
  RUN_TEST(test_only_scripted_gestures_are_available);
  RUN_TEST(test_shifting_the_body_moves_planted_feet_the_other_way);
  RUN_TEST(test_every_pose_is_reachable_and_within_servo_range);
  RUN_TEST(test_lifted_legs_keep_the_body_over_its_support);
  RUN_TEST(test_gesture_keeps_the_speed_limit_and_ends_standing);
  RUN_TEST(test_time_carries_over_keyframe_boundaries);
  RUN_TEST(test_stop_finishes_the_keyframe_puts_feet_down_then_straightens);
  RUN_TEST(test_ball_holds_and_every_interrupted_unfold_is_safe);
  RUN_TEST(test_unavailable_gesture_is_done_at_once);
  RUN_TEST(test_unfold_keeps_planted_feet_fixed_through_its_pose);
  return UNITY_END();
}
