#include <unity.h>
#include <math.h>
#include "movement/body_geometry.h"
#include "movement/centre_of_gravity.h"
#include "movement/gait.h"
#include "movement/gesture.h"
#include "movement/joint_calibration.h"
#include "movement/kinematics.h"

using namespace gesture;

void setUp() {}
void tearDown() {}

static const gesture::Gesture AVAILABLE[] = {Gesture::SayHi, Gesture::Wiggle};
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
  // Part two: they need the measured body geometry.
  TEST_ASSERT_FALSE(available((uint8_t)Gesture::Dance));
  TEST_ASSERT_FALSE(available((uint8_t)Gesture::Attack));
  TEST_ASSERT_FALSE(available((uint8_t)Gesture::Ball));
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
          TEST_ASSERT_TRUE_MESSAGE(withinServoRange(leg, feet[leg]), "joint outside its servo range");
        }
      }
      TEST_ASSERT_TRUE(p.done);
    }
}

// Whether the body rests on its underside (centre_of_gravity::undersideGrounded),
// the feet given in their leg frames.
static bool groundedOnFrame(const Vector3 *feet)
{
  Vector3 body[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    body[i] = centre_of_gravity::toBody(i, feet[i]);
  return centre_of_gravity::undersideGrounded(body);
}

// How far @p p lies inside the frame's support polygon: where the underside
// touches the ground, under the coxa pivots.
static float frameMargin(Vector2 p)
{
  // Anticlockwise around the body, as in supported().
  const int ORDER[] = {5, 0, 1, 2, 3, 4};
  Vector2 poly[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    poly[i] = Vector2(body_geometry::PIVOTS[ORDER[i]].x, body_geometry::PIVOTS[ORDER[i]].y);
  return insideMargin(poly, kinematics::LEG_COUNT, p);
}

// Whether the body centre (the body frame's origin) is inside the support
// polygon, with @p marginMm, whenever a foot is off the ground: the frame's
// when the underside rests on the ground, else the feet's. A foot more than
// 10 mm above the lowest counts as lifted.
static bool supported(const Vector3 *feet, float marginMm)
{
  if (groundedOnFrame(feet))
    return frameMargin(Vector2()) >= marginMm;
  float lowest = 1e9f;
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    lowest = fminf(lowest, feet[i].z);
  // Anticlockwise around the body: front right, front left, middle left,
  // rear left, rear right, middle right.
  const int ORDER[] = {5, 0, 1, 2, 3, 4};
  Vector2 poly[kinematics::LEG_COUNT];
  int n = 0;
  for (int leg : ORDER)
    if (feet[leg].z <= lowest + 10)
      poly[n++] = toBody(leg, feet[leg]);
  return n == kinematics::LEG_COUNT || insideMargin(poly, n, Vector2()) >= marginMm;
}

// The smallest support margin of the estimated centre of gravity, with and
// without the battery: in the frame's polygon when the underside rests on the
// ground, else the feet's.
static float gravityMargin(const Vector3 *feet)
{
  kinematics::JointAngles legs[kinematics::LEG_COUNT];
  Vector3 body[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
  {
    TEST_ASSERT_TRUE(kinematics::solveLeg(feet[i], Vector3(0, 0, 0), legs[i]));
    body[i] = centre_of_gravity::toBody(i, feet[i]);
  }
  bool onFrame = centre_of_gravity::undersideGrounded(body);
  float worst = 1e9f;
  for (int battery = 0; battery < 2; battery++)
  {
    Vector3 centre = centre_of_gravity::estimate(legs, battery).centre;
    float margin;
    if (onFrame)
      margin = frameMargin(Vector2(centre.x, centre.y));
    else
      TEST_ASSERT_TRUE(centre_of_gravity::supportMargin(body, centre, margin));
    worst = fminf(worst, margin);
  }
  return worst;
}

// A body resting on its underside is supported by the frame, whatever its
// feet do: here the left feet hang just above the ground and the right ones
// are folded, so the feet alone would leave the centre outside their polygon.
void test_a_body_on_its_underside_is_supported_by_the_frame()
{
  const float U = body_geometry::UNDERSIDE_Z;
  Vector3 feet[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    feet[i] = Vector3(120, 0, i < 3 ? U + 5 : U + 60);
  TEST_ASSERT_TRUE(groundedOnFrame(feet));
  TEST_ASSERT_TRUE(supported(feet, 20));
  TEST_ASSERT_TRUE(gravityMargin(feet) >= 20);
  // The same feet with the body 10 mm lower: the left feet carry it alone.
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    feet[i].z -= 10;
  TEST_ASSERT_FALSE(groundedOnFrame(feet));
  TEST_ASSERT_FALSE(supported(feet, 20));
}

void test_lifted_legs_keep_the_body_over_its_support()
{
  // Sampled through the whole gesture, eased moves and interrupted returns
  // included. The pivots are measured from the frame's STLs (body_geometry)
  // but the masses estimated until the robot is weighed, so this guards the
  // scripts' intent, not the physical margin.
  const float MARGIN_MM = 20;
  const uint32_t STOPS[] = {0, 1200, 1900, 2600};
  for (Gesture g : AVAILABLE)
    for (float ground : GROUNDS)
      for (uint32_t stopAt : STOPS)
      {
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
          TEST_ASSERT_TRUE_MESSAGE(supported(feet, MARGIN_MM), "body outside its support");
          TEST_ASSERT_TRUE_MESSAGE(gravityMargin(feet) >= MARGIN_MM, "centre of gravity outside its support");
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
    advance(p, stepMs, standing, feet);
    t += stepMs;
    for (int i = 0; i < kinematics::LEG_COUNT; i++)
    {
      float speed = feet[i].distanceTo(before[i]) / (stepMs / 1000.0f);
      // A step that crosses a segment end sees a little more than the peak.
      TEST_ASSERT_TRUE_MESSAGE(speed <= MAX_FOOT_SPEED_MM_S * 1.1f, "foot faster than the gesture limit");
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

void test_unavailable_gesture_is_done_at_once()
{
  Vector3 feet[kinematics::LEG_COUNT];
  for (int i = 0; i < kinematics::LEG_COUNT; i++)
    feet[i] = standingAt(-100);
  Player p;
  start(p, Gesture::Dance, feet, standingAt(-100));
  TEST_ASSERT_TRUE(p.done);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_only_scripted_gestures_are_available);
  RUN_TEST(test_shifting_the_body_moves_planted_feet_the_other_way);
  RUN_TEST(test_every_pose_is_reachable_and_within_servo_range);
  RUN_TEST(test_a_body_on_its_underside_is_supported_by_the_frame);
  RUN_TEST(test_lifted_legs_keep_the_body_over_its_support);
  RUN_TEST(test_gesture_keeps_the_speed_limit_and_ends_standing);
  RUN_TEST(test_time_carries_over_keyframe_boundaries);
  RUN_TEST(test_stop_finishes_the_keyframe_puts_feet_down_then_straightens);
  RUN_TEST(test_unavailable_gesture_is_done_at_once);
  return UNITY_END();
}
