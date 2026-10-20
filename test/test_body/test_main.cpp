#include <unity.h>
#include "movement/body.h"
#include "movement/servo_profile.h"
#include <math.h>
#include <stdio.h>
#include <vector>

using namespace body;

static BodyState s;
static uint32_t now;

static MovementCommand command(RequestedMode mode)
{
  MovementCommand c;
  c.mode = mode;
  c.issuedAtMs = now;
  return c;
}

// Steps with a fresh copy of @p cmd every @p stepMs for @p durationMs.
static void run(MovementCommand cmd, uint32_t durationMs, uint32_t stepMs = 20)
{
  for (uint32_t t = 0; t < durationMs; t += stepMs)
  {
    now += stepMs;
    cmd.issuedAtMs = now;
    step(s, cmd, now);
  }
}

static void bootToStand()
{
  init(s);
  now = 1000;
  step(s, command(RequestedMode::Stand), now);
  run(command(RequestedMode::Stand), 2000);
}

static bool allFinite()
{
  for (int i = 0; i < LEG_COUNT; i++)
    if (!s.feet[i].isFinite())
      return false;
  return true;
}

void setUp() {}
void tearDown() {}

// --- 2.1 command interface -------------------------------------------------

void test_clamp_command_ranges()
{
  MovementCommand c;
  c.translate = Vector2(300, 400);
  c.rotate = -250;
  c.height = 140;
  c.speed = -5;
  MovementCommand k = clampCommand(c);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, k.translate.magnitude());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 60, k.translate.x);
  TEST_ASSERT_EQUAL_FLOAT(-100, k.rotate);
  TEST_ASSERT_EQUAL_FLOAT(100, k.height);
  TEST_ASSERT_EQUAL_FLOAT(0, k.speed);
}

void test_clamp_command_non_finite_values()
{
  MovementCommand c;
  c.translate = Vector2(NAN, 1);
  c.rotate = NAN;
  MovementCommand k = clampCommand(c);
  TEST_ASSERT_TRUE(k.translate.magnitude() == 0);
  TEST_ASSERT_EQUAL_FLOAT(0, k.rotate);
}

void test_boot_initialises_then_stands()
{
  init(s);
  now = 1000;
  step(s, command(RequestedMode::Stand), now);
  TEST_ASSERT_TRUE(s.mode == Mode::Initialise);
  run(command(RequestedMode::Stand), 2000);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_FALSE(transitionActive(s));
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.01, groundForHeight(0), s.feet[i].z);
}

void test_unknown_gait_is_ignored()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.gait = 42;
  run(c, 100);
  TEST_ASSERT_TRUE(s.currentGait == gait::Gait::Tri);
}

void test_walk_request_waits_for_stand_transition()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 50);
  run(walk, 500);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);

  // Stand from Walk, then immediately ask for Walk again.
  run(command(RequestedMode::Stand), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_TRUE(transitionActive(s));
  run(walk, 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);

  // Walk is entered only once the transition has finished.
  int steps = 0;
  while (transitionActive(s) && steps++ < 500)
    run(walk, 20);
  run(walk, 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
}

// --- 2.2 stand transition --------------------------------------------------

void test_stand_from_walk_moves_at_most_three_legs()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 80);
  run(walk, 1300);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);

  MovementCommand stand = command(RequestedMode::Stand);
  int steps = 0;
  do
  {
    Vector3 before[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++)
      before[i] = s.feet[i];
    run(stand, 20);
    int moved = 0;
    for (int i = 0; i < LEG_COUNT; i++)
      if (s.feet[i].distanceTo(before[i]) > 0.01f)
        moved++;
    if (s.mode == Mode::Stand && transitionActive(s))
      TEST_ASSERT_LESS_OR_EQUAL(3, moved);
  } while (transitionActive(s) && ++steps < 500);

  TEST_ASSERT_FALSE(transitionActive(s));
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.01, gait::STANCE_DISTANCE, s.feet[i].x);
}

void test_stand_transition_spans_several_steps()
{
  init(s);
  now = 1000;
  run(command(RequestedMode::Stand), 600); // Initialise holds
  int steps = 0;
  while (s.mode == Mode::Initialise && steps++ < 100)
    run(command(RequestedMode::Stand), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_TRUE(transitionActive(s));
  run(command(RequestedMode::Stand), 20);
  TEST_ASSERT_TRUE(transitionActive(s)); // still going: not completed in one call
}

void test_height_change_while_standing()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Stand);
  c.height = 50;
  run(c, 20);
  float early = s.feet[0].z;
  TEST_ASSERT_TRUE(early < groundForHeight(0));
  TEST_ASSERT_TRUE(early > groundForHeight(50));
  run(c, 4000);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.5, groundForHeight(50), s.feet[i].z);
}

// --- 2.3 walk --------------------------------------------------------------

static void walkFor(uint32_t stepMs, Vector3 out[LEG_COUNT], Vector2 translate = Vector2(20, 40), float rotate = 10,
                    int gaitIndex = 0, float speed = 44)
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = translate;
  walk.rotate = rotate;
  walk.gait = gaitIndex;
  walk.speed = speed;
  run(walk, 20);  // enter Walk on an aligned step
  run(walk, 2000, stepMs);
  for (int i = 0; i < LEG_COUNT; i++)
    out[i] = s.feet[i];
}

void test_walk_is_tick_rate_independent()
{
  Vector3 at20[LEG_COUNT];
  Vector3 at50[LEG_COUNT];
  walkFor(20, at20);
  walkFor(50, at50);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(5, 0, at20[i].distanceTo(at50[i]));
}

// Settle full input while standing, then walk Bi at full speed for a short
// time: its ground phase (~49 ms) is shorter than a 50 ms tick.
static void fastBiWalk(uint32_t stepMs, uint32_t walkMs, Vector3 out[LEG_COUNT])
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Stand);
  c.translate = Vector2(0, 100);
  c.speed = 100;
  c.gait = static_cast<int>(gait::Gait::Bi);
  run(c, 1500);
  c.mode = RequestedMode::Walk;
  // Enter Walk on a tick of the size under test: a 50 ms first tick jumps
  // past the whole ground phase.
  run(c, walkMs, stepMs);
  for (int i = 0; i < LEG_COUNT; i++)
    out[i] = s.feet[i];
}

void test_fast_gait_is_tick_rate_independent()
{
  const uint32_t durations[] = {100, 300, 1000};
  for (uint32_t ms : durations)
  {
    Vector3 at20[LEG_COUNT];
    Vector3 at50[LEG_COUNT];
    fastBiWalk(20, ms, at20);
    fastBiWalk(50, ms, at50);
    for (int i = 0; i < LEG_COUNT; i++)
      TEST_ASSERT_FLOAT_WITHIN(5, 0, at20[i].distanceTo(at50[i]));
  }
}

void test_walk_is_tick_rate_independent_at_full_input()
{
  // Full input accelerates hardest from rest, where coarse integration errs most.
  Vector3 at20[LEG_COUNT];
  Vector3 at50[LEG_COUNT];
  walkFor(20, at20, Vector2(0, 100), 0);
  walkFor(50, at50, Vector2(0, 100), 0);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(5, 0, at20[i].distanceTo(at50[i]));
}

void test_gait_change_rephases_legs()
{
  bootToStand();
  s.jointBudgetDegPerS = 0; // exact rates: without the joint speed budget
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 50);
  run(walk, 500);
  walk.gait = static_cast<int>(gait::Gait::Wave);
  now += 20;
  walk.issuedAtMs = now;
  step(s, walk, now);
  TEST_ASSERT_TRUE(s.currentGait == gait::Gait::Wave);
  const gait::GaitParams &p = gait::params(gait::Gait::Wave);
  float advance = gait::progressRate(p, gait::GaitInput{s.translate, s.rotate, false, 0, 0.54f, 0.55f}) * 0.02f;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    float expected = p.phaseOffset[i] + advance;
    while (expected >= gait::CYCLE_POINTS)
      expected -= gait::CYCLE_POINTS;
    TEST_ASSERT_FLOAT_WITHIN(1, expected, s.cycleProgress[i]);
  }
}

void test_zero_input_walk_holds_and_stays_finite()
{
  bootToStand();
  run(command(RequestedMode::Walk), 100);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  Vector3 before = s.feet[0];
  float progress = s.cycleProgress[0];
  run(command(RequestedMode::Walk), 500);
  TEST_ASSERT_TRUE(allFinite());
  TEST_ASSERT_TRUE(s.feet[0] == before);
  TEST_ASSERT_EQUAL_FLOAT(progress, s.cycleProgress[0]);
}

void test_rotate_only_walk_is_finite()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.rotate = 70;
  run(c, 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  // Recorded feet stay finite even when targets are rejected, so also check
  // that every generated target was accepted and the legs actually moved.
  Vector3 start = s.feet[0];
  for (int i = 0; i < 150; i++)
  {
    run(c, 20);
    for (int leg = 0; leg < LEG_COUNT; leg++)
    {
      TEST_ASSERT_FALSE(s.rejected[leg]);
      TEST_ASSERT_TRUE(s.legUpdated[leg]);
    }
  }
  TEST_ASSERT_TRUE(allFinite());
  TEST_ASSERT_TRUE(s.feet[0].distanceTo(start) > 1);
}

// --- 2.4 smoothing ---------------------------------------------------------

void test_step_input_is_eased()
{
  bootToStand();
  s.jointBudgetDegPerS = 0; // exact rates: without the joint speed budget
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  run(c, 20);
  TEST_ASSERT_TRUE(s.translate.magnitude() > 0);
  TEST_ASSERT_TRUE(s.translate.magnitude() < 100);
  run(c, 1200);
  TEST_ASSERT_FLOAT_WITHIN(2, 100, s.translate.magnitude());
}

void test_speed_dial_and_stride_mode_are_eased()
{
  bootToStand();
  s.jointBudgetDegPerS = 0; // the raw paths
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  run(c, 1500);
  // Steady foot movement per step at dial 44.
  float steady = 0;
  Vector3 last[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
    last[i] = s.feet[i];
  for (int k = 0; k < 50; k++)
  {
    run(c, 20);
    for (int i = 0; i < LEG_COUNT; i++)
    {
      steady = fmaxf(steady, s.feet[i].distanceTo(last[i]));
      last[i] = s.feet[i];
    }
  }
  c.speed = 100;
  c.stride = StrideMode::Fixed;
  run(c, 20);
  TEST_ASSERT_TRUE(s.speed > 44 && s.speed < 100);
  TEST_ASSERT_TRUE(s.fixedStride > 0 && s.fixedStride < 1);
  // No foot jumps: the first steps move about as much as before the change.
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_TRUE(s.feet[i].distanceTo(last[i]) < steady * 1.5f);
  run(c, 1500);
  TEST_ASSERT_FLOAT_WITHIN(1, 100, s.speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1, s.fixedStride);
}

void test_sleep_resets_smoothing()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  c.rotate = 50;
  run(c, 500);
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_EQUAL_FLOAT(0, s.translate.magnitude());
  TEST_ASSERT_EQUAL_FLOAT(0, s.rotate);
}

// --- 2.5 sleep and stale commands ------------------------------------------

void test_sleep_folds_then_disables_outputs()
{
  bootToStand();
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
  TEST_ASSERT_TRUE(s.outputsEnabled);
  run(command(RequestedMode::Sleep), 5000);
  TEST_ASSERT_FALSE(s.outputsEnabled);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_TRUE(s.feet[i] == REST_POSE);
}

void test_wake_goes_through_stand_before_walk()
{
  bootToStand();
  run(command(RequestedMode::Sleep), 5000);
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 50);
  run(walk, 20);
  TEST_ASSERT_TRUE(s.outputsEnabled);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  int steps = 0;
  while (s.mode == Mode::Stand && steps++ < 500)
    run(walk, 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
}

void test_stale_command_puts_walking_body_to_sleep()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 50);
  run(walk, 500);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);

  // The controller stops: the same command keeps its old timestamp.
  for (int i = 0; i < 60; i++)
  {
    now += 20;
    step(s, walk, now);
  }
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
}

void test_stale_sleep_interrupts_stand_transition()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 80);
  run(walk, 500);
  run(command(RequestedMode::Stand), 20);
  TEST_ASSERT_TRUE(transitionActive(s));
  MovementCommand old = command(RequestedMode::Stand);
  old.issuedAtMs = now - COMMAND_TIMEOUT_MS - 1;
  now += 20;
  step(s, old, now);
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
}

// --- 2.6 calibrate ---------------------------------------------------------

void test_calibrate_raises_only_low_legs_first()
{
  bootToStand();
  // Lift legs 0-2 above the in-between height by hand.
  for (int i = 0; i < 3; i++)
    s.feet[i] = Vector3(190, 0, 10);
  run(command(RequestedMode::Calibrate), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Calibrate);
  for (int i = 0; i < 3; i++)
    TEST_ASSERT_TRUE(s.feet[i] == Vector3(190, 0, 10));
  for (int i = 3; i < 6; i++)
    TEST_ASSERT_TRUE(s.feet[i].z > groundForHeight(0));
}

void test_calibrate_approaches_from_above_without_jumping()
{
  bootToStand();
  for (int i = 0; i < LEG_COUNT; i++)
    s.feet[i] = Vector3(260, 30, 0); // x and y above their targets
  run(command(RequestedMode::Calibrate), 20);
  TEST_ASSERT_TRUE(s.feet[0].x < 260);
  TEST_ASSERT_TRUE(s.feet[0].x > CALIBRATION_POSE.x);
  TEST_ASSERT_TRUE(s.feet[0].y < 30);
  TEST_ASSERT_TRUE(s.feet[0].y > CALIBRATION_POSE.y);
  run(command(RequestedMode::Calibrate), 5000);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_TRUE(s.feet[i] == CALIBRATION_POSE);
}

void test_leaving_calibrate_passes_through_stand()
{
  bootToStand();
  run(command(RequestedMode::Calibrate), 5000);
  run(command(RequestedMode::Walk), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
}

void test_init_loads_default_offsets()
{
  init(s);
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    TEST_ASSERT_EQUAL_FLOAT(joint_calibration::DEFAULTS[leg * 3 + 0].offsetDeg, s.offsets[leg].x);
    TEST_ASSERT_EQUAL_FLOAT(joint_calibration::DEFAULTS[leg * 3 + 1].offsetDeg, s.offsets[leg].y);
    TEST_ASSERT_EQUAL_FLOAT(joint_calibration::DEFAULTS[leg * 3 + 2].offsetDeg, s.offsets[leg].z);
  }
}

void test_offset_change_resolves_a_still_leg()
{
  bootToStand();
  run(command(RequestedMode::Calibrate), 5000);
  // Sleep folds the legs and then holds them: no foot moves any more.
  run(command(RequestedMode::Sleep), 5000);
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_FALSE(s.legUpdated[1]);
  float femur = s.angles[1].femur;
  float otherFemur = s.angles[0].femur;
  Vector3 o = s.offsets[1];
  setOffsets(s, 1, Vector3(o.x, o.y + 5, o.z));
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_TRUE(s.legUpdated[1]);
  TEST_ASSERT_FALSE(s.legUpdated[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, femur + 5, s.angles[1].femur);
  TEST_ASSERT_EQUAL_FLOAT(otherFemur, s.angles[0].femur);
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_FALSE(s.legUpdated[1]); // re-solved once only
  setOffsets(s, 1, Vector3(o.x, o.y + 5, o.z)); // unchanged: no re-solve
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_FALSE(s.legUpdated[1]);
}

// --- elapsed-time bound and target rejection --------------------------------

void test_large_elapsed_time_is_bounded()
{
  bootToStand();
  s.jointBudgetDegPerS = 0; // exact rates: without the joint speed budget
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 100);
  run(walk, 1500);
  float before = s.cycleProgress[0];
  now += 5000; // a long stall between steps
  walk.issuedAtMs = now;
  step(s, walk, now);
  float advanced = s.cycleProgress[0] - before;
  if (advanced < 0)
    advanced += gait::CYCLE_POINTS;
  float maxAdvance = gait::progressRate(gait::params(s.currentGait),
                                        gait::GaitInput{s.translate, s.rotate, false, 0, 0.54f, 0.55f}) *
                     (MAX_STEP_MS / 1000.0f);
  TEST_ASSERT_FLOAT_WITHIN(1, maxAdvance, advanced);
}

void test_rejected_target_keeps_position_and_reports_once()
{
  bootToStand();
  // 340.3 mm from the pivot: Calibrate's raise phase keeps asking for
  // targets just as far out, so every one is rejected.
  const Vector3 outOfReach(335, 0, -60);
  s.feet[0] = outOfReach;
  MovementCommand c = command(RequestedMode::Calibrate);

  int reports = 0;
  for (int i = 0; i < 10; i++)
  {
    run(c, 20);
    TEST_ASSERT_TRUE(s.mode == Mode::Calibrate);
    TEST_ASSERT_TRUE(s.rejected[0]);
    TEST_ASSERT_FALSE(s.legUpdated[0]);
    TEST_ASSERT_TRUE(s.feet[0] == outOfReach);
    if (s.newlyRejected[0])
      reports++;
  }
  TEST_ASSERT_EQUAL_INT(1, reports);
  // Other legs keep moving normally.
  TEST_ASSERT_TRUE(s.feet[1].z > groundForHeight(0));
}

// --- review regressions ----------------------------------------------------

void test_initialise_holds_second_pose_for_full_duration()
{
  init(s);
  now = 1000;
  step(s, command(RequestedMode::Stand), now);
  // First hold (25 ms), then the second pose must be written on the tick its
  // hold starts and held for 500 ms before Stand begins.
  uint32_t secondPoseAt = 0;
  for (int i = 0; i < 100 && s.mode == Mode::Initialise; i++)
  {
    run(command(RequestedMode::Stand), 20);
    if (secondPoseAt == 0 && s.initialiseStep == 1)
    {
      secondPoseAt = now;
      TEST_ASSERT_TRUE(s.legUpdated[0]);
      TEST_ASSERT_TRUE(s.feet[0] == Vector3(225, 0, 115));
    }
  }
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_TRUE(now - secondPoseAt >= 500);
}

void test_stand_from_walk_finishes_at_current_height()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 80);
  run(walk, 1300);
  MovementCommand stand = command(RequestedMode::Stand);
  stand.height = 60; // height changes while the transition runs
  int steps = 0;
  run(stand, 20);
  while (transitionActive(s) && steps++ < 500)
    run(stand, 20);
  TEST_ASSERT_FALSE(transitionActive(s));
  float expectedZ = groundForHeight(s.height);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedZ, s.feet[i].z);
}

void test_stand_with_empty_first_group_moves_every_leg()
{
  bootToStand();
  // Walk with no input: feet stay exactly at the standing point.
  run(command(RequestedMode::Walk), 100);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  MovementCommand stand = command(RequestedMode::Stand);
  stand.height = 60;
  int steps = 0;
  run(stand, 20);
  while (transitionActive(s) && steps++ < 1000)
  {
    Vector3 before[LEG_COUNT];
    for (int i = 0; i < LEG_COUNT; i++)
      before[i] = s.feet[i];
    run(stand, 20);
    for (int i = 0; i < LEG_COUNT; i++)
      TEST_ASSERT_TRUE(s.feet[i].distanceTo(before[i]) < 30); // no jumps
  }
  TEST_ASSERT_FALSE(transitionActive(s));
  // The step after the transition must not jump either.
  Vector3 before[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
    before[i] = s.feet[i];
  run(stand, 20);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_TRUE(s.feet[i].distanceTo(before[i]) < 5);
}

void test_wake_waits_for_sleep_fold()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 60);
  run(walk, 500);
  run(command(RequestedMode::Sleep), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
  // A wake request on the very next tick must not abandon the fold.
  run(walk, 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
  int steps = 0;
  while (s.mode == Mode::Sleep && steps++ < 500)
  {
    run(walk, 20);
    if (s.mode == Mode::Sleep)
      TEST_ASSERT_TRUE(transitionActive(s) || !s.outputsEnabled);
  }
  // Woke only after folding completed and outputs were switched off.
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_TRUE(s.outputsEnabled);
}

void test_command_stamped_after_now_is_not_stale()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Stand);
  c.issuedAtMs = now + 25; // produced by another thread after we read millis()
  now += 20;
  step(s, c, now);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
}

void test_stale_command_never_becomes_fresh_again()
{
  bootToStand();
  MovementCommand walk = command(RequestedMode::Walk);
  walk.translate = Vector2(0, 50);
  run(walk, 500);
  // The controller stops; its last command keeps its stamp.
  for (int i = 0; i < 300; i++)
  {
    now += 20;
    step(s, walk, now);
  }
  // Fully folded and limp, so a revived command could wake it immediately.
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
  TEST_ASSERT_FALSE(s.outputsEnabled);
  // Days later, including past the signed and unsigned wrap points of millis().
  const uint32_t laterOffsets[] = {0x7FFFFFFFu, 0x80000010u, 0xC0000000u, 0xFFFFFF00u};
  for (uint32_t offset : laterOffsets)
  {
    uint32_t t = walk.issuedAtMs + offset;
    for (int i = 0; i < 5; i++)
      step(s, walk, t + i * 20);
    TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
    TEST_ASSERT_FALSE(s.outputsEnabled);
  }
  // A genuinely new command wakes it.
  now = walk.issuedAtMs + 0xFFFFFFFFu + 200;
  run(walk, 5000);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
}

// Boot and walk entirely at one tick size: transitions (Initialise holds,
// Stand groups, entering Walk) must land on the same time grid.
static void bootAndWalk(uint32_t stepMs, Vector3 out[LEG_COUNT])
{
  init(s);
  now = 0;
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  c.speed = 100;
  c.gait = static_cast<int>(gait::Gait::Bi);
  step(s, c, now);
  run(c, 3000, stepMs);
  for (int i = 0; i < LEG_COUNT; i++)
    out[i] = s.feet[i];
}

void test_boot_and_walk_is_tick_rate_independent()
{
  Vector3 at20[LEG_COUNT];
  Vector3 at50[LEG_COUNT];
  bootAndWalk(20, at20);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  bootAndWalk(50, at50);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  for (int i = 0; i < LEG_COUNT; i++)
    TEST_ASSERT_FLOAT_WITHIN(5, 0, at20[i].distanceTo(at50[i]));
}

// --- Joint speed budget ----------------------------------------------------

// The largest joint angle change between two sets of angles, in degrees.
static float largestChange(const JointAngles *a, const JointAngles *b)
{
  float m = 0;
  for (int i = 0; i < LEG_COUNT; i++)
    m = fmaxf(m, fmaxf(fabsf(a[i].coxa - b[i].coxa), fmaxf(fabsf(a[i].femur - b[i].femur), fabsf(a[i].tibia - b[i].tibia))));
  return m;
}

// Steps @p cmd every 20 ms for @p durationMs, counting the steps and those
// in which a joint turned faster than @p budget times @p factor; returns the
// fastest joint speed seen, in degrees per second.
static float fastestJoint(MovementCommand cmd, uint32_t durationMs, float budget = 0, float factor = 1,
                          int *steps = nullptr, int *over = nullptr)
{
  float fastest = 0;
  JointAngles before[LEG_COUNT];
  for (uint32_t t = 0; t < durationMs; t += 20)
  {
    for (int i = 0; i < LEG_COUNT; i++)
      before[i] = s.angles[i];
    now += 20;
    cmd.issuedAtMs = now;
    step(s, cmd, now);
    float speed = largestChange(before, s.angles) / 0.02f;
    fastest = fmaxf(fastest, speed);
    if (steps)
      (*steps)++;
    if (over && speed > budget * factor)
      (*over)++;
  }
  return fastest;
}

// The governor estimates each piece from at most three runs, so it keeps
// within 5 % of the budget nearly always; near awkward poses at the highest
// dial settings it can miss, by bounded amounts the output backstop takes.
static const float BUDGET_TOLERANCE = 1.05f;
static const float MAX_MISSED_SHARE = 0.005f;
static const float WORST_MISS = 1.6f;

void test_walking_stays_within_the_budget()
{
  const float speeds[] = {0, 44, 100};
  const float heights[] = {0, 50, 100};
  const float rotates[] = {0, 100};
  int steps = 0, over = 0;
  for (int g = 0; g < gait::GAIT_COUNT; g++)
    for (float speed : speeds)
      for (float height : heights)
        for (float rotate : rotates)
        {
          bootToStand();
          float budget = s.jointBudgetDegPerS;
          MovementCommand c = command(RequestedMode::Stand);
          c.gait = g;
          c.speed = speed;
          c.height = height;
          run(c, 1500); // settle the height
          c.mode = RequestedMode::Walk;
          c.translate = Vector2(0, 100);
          c.rotate = rotate;
          float fastest = fastestJoint(c, 3000, budget, BUDGET_TOLERANCE, &steps, &over);
          TEST_ASSERT_TRUE(s.mode == Mode::Walk);
          char msg[96];
          snprintf(msg, sizeof msg, "gait %d speed %.0f height %.0f rotate %.0f: %.2f x budget", g, speed, height, rotate,
                   fastest / budget);
          TEST_ASSERT_TRUE_MESSAGE(fastest <= budget * WORST_MISS, msg);
        }
  TEST_ASSERT_TRUE(over <= steps * MAX_MISSED_SHARE);
}

void test_slow_walking_is_not_slowed()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 30);
  c.speed = 0;
  for (int i = 0; i < 150; i++)
  {
    run(c, 20);
    TEST_ASSERT_EQUAL_FLOAT(1, s.motionScale);
  }
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
}

void test_fast_walking_is_slowed()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  c.speed = 100;
  run(c, 1000);
  float slowest = 1;
  for (int i = 0; i < 50; i++)
  {
    run(c, 20);
    slowest = fminf(slowest, s.motionScale);
  }
  TEST_ASSERT_TRUE(slowest < 1);
}

// Walks fast, then stops sending: returns the time from the last command to
// the step that starts Sleep.
static uint32_t timeToSleepAfterLastCommand(float budget)
{
  bootToStand();
  s.jointBudgetDegPerS = budget;
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  c.speed = 100;
  run(c, 1000);
  uint32_t last = now;
  c.issuedAtMs = last; // the last command sent
  for (uint32_t t = 20; t < 3000; t += 20)
  {
    now += 20;
    step(s, c, now); // the same, ever older command
    if (s.mode == Mode::Sleep)
      return now - last;
  }
  return 0;
}

void test_stale_commands_time_out_on_wall_time_under_the_budget()
{
  uint32_t without = timeToSleepAfterLastCommand(0);
  uint32_t with = timeToSleepAfterLastCommand(servo_profile::SELECTED.budgetDegPerS());
  TEST_ASSERT_TRUE(without > COMMAND_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_UINT32(without, with);
}

// Boots and returns the time from the first step to the step that enters Stand.
static uint32_t initialiseDuration(float budget)
{
  init(s);
  s.jointBudgetDegPerS = budget;
  now = 1000;
  step(s, command(RequestedMode::Stand), now);
  for (uint32_t t = 20; t < 2000; t += 20)
  {
    now += 20;
    step(s, command(RequestedMode::Stand), now);
    if (s.mode != Mode::Initialise)
      return t;
  }
  return 0;
}

void test_initialise_is_not_slowed()
{
  uint32_t without = initialiseDuration(0);
  TEST_ASSERT_TRUE(without > 0);
  TEST_ASSERT_EQUAL_UINT32(without, initialiseDuration(servo_profile::SELECTED.budgetDegPerS()));
}

// --- Gestures --------------------------------------------------------------

static MovementCommand gestureCommand(RequestedMode mode, gesture::Gesture g, uint32_t seq)
{
  MovementCommand c = command(mode);
  c.gesture = (uint8_t)g;
  c.gestureSeq = seq;
  return c;
}

static uint32_t scriptMs(gesture::Gesture g)
{
  gesture::Script sc = gesture::script(g);
  uint32_t total = 0;
  for (int k = 0; k < sc.count; k++)
    total += sc.frames[k].durationMs;
  return total;
}

// Distance from @p p to the polyline through @p path.
static float distanceToPath(Vector3 p, const std::vector<Vector3> &path)
{
  float best = INFINITY;
  for (size_t i = 1; i < path.size(); i++)
  {
    Vector3 a = path[i - 1], ab = path[i] - a;
    float len2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
    float t = len2 > 0 ? ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y + (p.z - a.z) * ab.z) / len2 : 0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    best = fminf(best, p.distanceTo(a + ab * t));
  }
  return best;
}

void test_paths_are_kept_under_the_budget()
{
  // The reference: the same walk without a budget, sampled finely.
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  c.speed = 100;
  bootToStand();
  s.jointBudgetDegPerS = 0;
  BodyState start = s;
  uint32_t startNow = now;
  std::vector<Vector3> reference[LEG_COUNT];
  for (int i = 0; i < 2000; i++)
  {
    run(c, 1, 1);
    for (int leg = 0; leg < LEG_COUNT; leg++)
      reference[leg].push_back(s.feet[leg]);
  }
  // Under the budget from the same state: slower, along the same paths.
  s = start;
  now = startNow;
  s.jointBudgetDegPerS = servo_profile::SELECTED.budgetDegPerS();
  float slowest = 1, worst = 0, worstAtLimit = 0;
  for (int i = 0; i < 100; i++)
  {
    run(c, 20);
    slowest = fminf(slowest, s.motionScale);
    for (int leg = 0; leg < LEG_COUNT; leg++)
    {
      // Where a joint is held at its servo limit the foot is fitted to it,
      // which shows small timing differences a little more.
      const JointAngles &a = s.angles[leg];
      bool atLimit = fminf(fminf(a.coxa, a.femur), a.tibia) < 0.5f || fmaxf(fmaxf(a.coxa, a.femur), a.tibia) > 179.5f;
      float d = distanceToPath(s.feet[leg], reference[leg]);
      if (atLimit)
        worstAtLimit = fmaxf(worstAtLimit, d);
      else
        worst = fmaxf(worst, d);
    }
  }
  TEST_ASSERT_TRUE(slowest < 0.5f); // the budget did slow it
  TEST_ASSERT_FLOAT_WITHIN(1, 0, worst);
  TEST_ASSERT_FLOAT_WITHIN(2, 0, worstAtLimit);
}

void test_transitions_and_gestures_stay_within_the_budget()
{
  bootToStand();
  float limit = s.jointBudgetDegPerS * BUDGET_TOLERANCE;
  MovementCommand c = command(RequestedMode::Sleep);
  TEST_ASSERT_TRUE(fastestJoint(c, 3000) <= limit); // Stand -> Sleep
  c.mode = RequestedMode::Stand;
  TEST_ASSERT_TRUE(fastestJoint(c, 3000) <= limit); // Sleep -> Stand
  c.height = 100;
  TEST_ASSERT_TRUE(fastestJoint(c, 4000) <= limit); // height 0 -> 100
  c.height = 0;
  TEST_ASSERT_TRUE(fastestJoint(c, 4000) <= limit);
  c.mode = RequestedMode::Walk;
  c.translate = Vector2(0, 60);
  fastestJoint(c, 1000);
  c.mode = RequestedMode::Stand;
  TEST_ASSERT_TRUE(fastestJoint(c, 3000) <= limit); // Walk -> Stand
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  c = command(RequestedMode::Calibrate);
  TEST_ASSERT_TRUE(fastestJoint(c, 6000) <= limit); // Stand -> Calibrate
  TEST_ASSERT_TRUE(s.mode == Mode::Calibrate);
  c.mode = RequestedMode::Stand;
  TEST_ASSERT_TRUE(fastestJoint(c, 3000) <= limit); // Calibrate -> Stand
  for (gesture::Gesture g : {gesture::Gesture::SayHi, gesture::Gesture::Wiggle})
  {
    static uint32_t seq = 1;
    MovementCommand play = gestureCommand(RequestedMode::Stand, g, seq++);
    TEST_ASSERT_TRUE(fastestJoint(play, 20) <= limit);
    TEST_ASSERT_TRUE(s.mode == Mode::Perform);
    TEST_ASSERT_TRUE(fastestJoint(play, 2 * scriptMs(g) + 2000) <= limit);
    TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  }
}

void test_a_tick_without_a_whole_piece_moves_nothing()
{
  // 10 ms ticks under the budget: every other tick has no whole 20 ms piece.
  // A command change on such a tick (here the speed dial and stride mode)
  // must not move a joint before a governed piece runs.
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 80);
  run(c, 1000, 10);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  if (s.motionPendingUs != 0)
    run(c, 10, 10); // so that the next tick is one without a whole piece
  JointAngles before[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
    before[i] = s.angles[i];
  c.speed = 100;
  c.stride = StrideMode::Fixed;
  run(c, 10, 10);
  TEST_ASSERT_EQUAL_FLOAT(0, largestChange(before, s.angles));
  run(c, 10, 10);
  TEST_ASSERT_TRUE(largestChange(before, s.angles) > 0);
}

void test_a_gesture_request_on_a_tick_without_a_piece_still_plays()
{
  bootToStand();
  TEST_ASSERT_EQUAL_UINT32(0, s.motionPendingUs);
  // Requested on a 10 ms tick, without a whole piece: nothing moves yet...
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 10, 10);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  // ...and the next piece plays it, though the request is not repeated.
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 10, 10);
  TEST_ASSERT_TRUE(s.mode == Mode::Perform);
}

void test_an_overrun_advances_at_most_three_pieces()
{
  bootToStand();
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 60);
  run(c, 1000);
  float before = s.cycleProgress[0];
  run(c, 100, 100); // one 100 ms step: three pieces, the rest dropped
  TEST_ASSERT_EQUAL_UINT32(0, s.motionPendingUs);
  float advanced = s.cycleProgress[0] - before;
  if (advanced < 0)
    advanced += gait::CYCLE_POINTS;
  float maxPerSecond = gait::progressRate(gait::params(s.currentGait),
                                          gait::GaitInput{s.translate, s.rotate, 0, 0, 0.54f, 0.55f});
  TEST_ASSERT_TRUE(advanced <= maxPerSecond * 0.06f + 1);
}

static bool inTravel(const JointAngles &a)
{
  return a.coxa >= 0 && a.coxa <= 180 && a.femur >= 0 && a.femur <= 180 && a.tibia >= 0 && a.tibia <= 180;
}

static void runInTravel(MovementCommand c, uint32_t durationMs, const char *what)
{
  for (uint32_t t = 0; t < durationMs; t += 20)
  {
    run(c, 20);
    for (int leg = 0; leg < LEG_COUNT; leg++)
      TEST_ASSERT_TRUE_MESSAGE(inTravel(s.angles[leg]), what);
  }
}

void test_every_pose_stays_within_servo_travel()
{
  // With the compiled defaults (femur offsets near 90 deg, so a lifted femur
  // soon needs more than 180 deg), every mode but Calibrate fits the feet.
  const float speeds[] = {0, 44, 100};
  const float heights[] = {0, 50, 100};
  const float rotates[] = {0, 100};
  for (int g = 0; g < gait::GAIT_COUNT; g++)
    for (float speed : speeds)
      for (float height : heights)
        for (float rotate : rotates)
        {
          init(s);
          now = 1000;
          MovementCommand c = command(RequestedMode::Stand);
          c.gait = g;
          c.speed = speed;
          c.height = height;
          step(s, c, now);
          runInTravel(c, 3000, "boot and stand");
          c.mode = RequestedMode::Walk;
          c.translate = Vector2(0, 100);
          c.rotate = rotate;
          runInTravel(c, 2000, "walk");
          c.mode = RequestedMode::Stand;
          c.translate = Vector2();
          c.rotate = 0;
          runInTravel(c, 2000, "walk to stand");
          c.mode = RequestedMode::Sleep;
          runInTravel(c, 2000, "sleep");
        }
}

void test_sleep_folds_with_the_rest_pose_out_of_travel()
{
  // The calibration guards allow a femur offset 30 deg above its default:
  // the rest pose then needs more than 180 deg. Sleep folds to the fitted
  // rest pose and switches the outputs off all the same.
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    bootToStand();
    setOffsets(s, leg, Vector3(s.offsets[leg].x, s.offsets[leg].y + 30, s.offsets[leg].z));
    JointAngles plain;
    kinematics::solveLeg(REST_POSE, s.offsets[leg], plain);
    TEST_ASSERT_TRUE(plain.femur > 180);
    run(command(RequestedMode::Sleep), 3000);
    TEST_ASSERT_FALSE(s.outputsEnabled);
    TEST_ASSERT_TRUE(inTravel(s.angles[leg]));
  }
}

void test_gesture_plays_from_stand_and_returns_to_stand()
{
  bootToStand();
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::Wiggle, 1), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Perform);
  TEST_ASSERT_EQUAL((uint8_t)gesture::Gesture::Wiggle, playingGesture(s));
  TEST_ASSERT_TRUE(transitionActive(s));
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::Wiggle, 1), scriptMs(gesture::Gesture::Wiggle));
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_EQUAL(0, playingGesture(s));
  TEST_ASSERT_TRUE(allFinite());
  // The same request is not replayed.
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::Wiggle, 1), 500);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
}

void test_first_gesture_sequence_is_only_a_baseline()
{
  init(s);
  now = 1000;
  // A restarted body sees an old request value: it must not play it.
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 7), 2000);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_EQUAL(0, playingGesture(s));
}

void test_gesture_request_while_walking_is_dropped()
{
  bootToStand();
  MovementCommand walk = gestureCommand(RequestedMode::Walk, gesture::Gesture::None, 0);
  walk.translate = Vector2(0, 50);
  run(walk, 500);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  walk.gesture = (uint8_t)gesture::Gesture::SayHi;
  walk.gestureSeq = 1;
  run(walk, 200);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  // Back in Stand later, the old request does not play.
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 3000);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
}

void test_gesture_request_during_a_stand_transition_is_dropped()
{
  init(s);
  now = 1000;
  step(s, gestureCommand(RequestedMode::Stand, gesture::Gesture::None, 0), now);
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::None, 0), 600); // into the stand transition
  TEST_ASSERT_TRUE(transitionActive(s));
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 20);
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 3000);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_EQUAL(0, playingGesture(s));
}

void test_unavailable_gesture_is_ignored()
{
  bootToStand();
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::Dance, 1), 100);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
}

void test_sleep_interrupts_a_gesture_at_once()
{
  bootToStand();
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 800);
  TEST_ASSERT_TRUE(s.mode == Mode::Perform);
  run(gestureCommand(RequestedMode::Sleep, gesture::Gesture::SayHi, 1), 20);
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
}

void test_stale_command_interrupts_a_gesture()
{
  bootToStand();
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 800);
  TEST_ASSERT_TRUE(s.mode == Mode::Perform);
  MovementCommand old = gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1);
  old.issuedAtMs = now;
  for (int i = 0; i < 60; i++) // 1.2 s without a fresh command
  {
    now += 20;
    step(s, old, now);
  }
  TEST_ASSERT_TRUE(s.mode == Mode::Sleep);
}

void test_walk_request_ends_a_gesture_through_stand()
{
  bootToStand();
  run(gestureCommand(RequestedMode::Stand, gesture::Gesture::SayHi, 1), 100);
  TEST_ASSERT_TRUE(s.mode == Mode::Perform);
  MovementCommand walk = gestureCommand(RequestedMode::Walk, gesture::Gesture::SayHi, 1);
  walk.translate = Vector2(0, 50);
  // Within the first keyframe (600 ms) plus the return (400 ms), and a
  // little for Stand to hand over, the body walks.
  uint32_t t = 0;
  while (s.mode == Mode::Perform && t < 5000)
  {
    run(walk, 20);
    t += 20;
  }
  TEST_ASSERT_TRUE(t <= 600 + gesture::RETURN_MS + 40);
  run(walk, 100);
  TEST_ASSERT_TRUE(s.mode == Mode::Walk);
  TEST_ASSERT_TRUE(allFinite());
}

void test_height_change_during_a_gesture_waits_for_stand()
{
  bootToStand();
  MovementCommand c = gestureCommand(RequestedMode::Stand, gesture::Gesture::Wiggle, 1);
  run(c, 100);
  TEST_ASSERT_TRUE(s.mode == Mode::Perform);
  c.height = 100; // the dial moves mid-gesture
  Vector3 prev[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
    prev[i] = s.feet[i];
  // Through the gesture: no foot faster than the gesture limit, and the
  // height is held. Then Stand eases to the new height as it always does:
  // its first step is its largest, so it is the one to bound.
  float heldZ = s.feet[1].z;
  bool first = true;
  for (int n = 0; n < 400; n++)
  {
    bool performing = s.mode == Mode::Perform;
    run(c, 20);
    float limit = performing && s.mode == Mode::Perform ? gesture::MAX_FOOT_SPEED_MM_S * 0.02f * 1.1f : 1e9f;
    if (performing && s.mode == Mode::Stand && first)
    {
      // The hand-over step: Stand starts from the held height.
      first = false;
      TEST_ASSERT_FLOAT_WITHIN(12, heldZ, s.feet[1].z);
    }
    for (int i = 0; i < LEG_COUNT; i++)
    {
      TEST_ASSERT_TRUE_MESSAGE(s.feet[i].distanceTo(prev[i]) <= limit, "foot jumped");
      prev[i] = s.feet[i];
    }
  }
  TEST_ASSERT_FALSE(first);
  TEST_ASSERT_TRUE(s.mode == Mode::Stand);
  TEST_ASSERT_FLOAT_WITHIN(1, groundForHeight(100), s.feet[0].z);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_clamp_command_ranges);
  RUN_TEST(test_clamp_command_non_finite_values);
  RUN_TEST(test_boot_initialises_then_stands);
  RUN_TEST(test_unknown_gait_is_ignored);
  RUN_TEST(test_walk_request_waits_for_stand_transition);
  RUN_TEST(test_stand_from_walk_moves_at_most_three_legs);
  RUN_TEST(test_stand_transition_spans_several_steps);
  RUN_TEST(test_height_change_while_standing);
  RUN_TEST(test_walk_is_tick_rate_independent);
  RUN_TEST(test_walk_is_tick_rate_independent_at_full_input);
  RUN_TEST(test_fast_gait_is_tick_rate_independent);
  RUN_TEST(test_boot_and_walk_is_tick_rate_independent);
  RUN_TEST(test_walking_stays_within_the_budget);
  RUN_TEST(test_slow_walking_is_not_slowed);
  RUN_TEST(test_fast_walking_is_slowed);
  RUN_TEST(test_stale_commands_time_out_on_wall_time_under_the_budget);
  RUN_TEST(test_initialise_is_not_slowed);
  RUN_TEST(test_paths_are_kept_under_the_budget);
  RUN_TEST(test_transitions_and_gestures_stay_within_the_budget);
  RUN_TEST(test_every_pose_stays_within_servo_travel);
  RUN_TEST(test_sleep_folds_with_the_rest_pose_out_of_travel);
  RUN_TEST(test_a_tick_without_a_whole_piece_moves_nothing);
  RUN_TEST(test_an_overrun_advances_at_most_three_pieces);
  RUN_TEST(test_a_gesture_request_on_a_tick_without_a_piece_still_plays);
  RUN_TEST(test_gait_change_rephases_legs);
  RUN_TEST(test_zero_input_walk_holds_and_stays_finite);
  RUN_TEST(test_rotate_only_walk_is_finite);
  RUN_TEST(test_step_input_is_eased);
  RUN_TEST(test_speed_dial_and_stride_mode_are_eased);
  RUN_TEST(test_sleep_resets_smoothing);
  RUN_TEST(test_sleep_folds_then_disables_outputs);
  RUN_TEST(test_wake_goes_through_stand_before_walk);
  RUN_TEST(test_stale_command_puts_walking_body_to_sleep);
  RUN_TEST(test_stale_sleep_interrupts_stand_transition);
  RUN_TEST(test_calibrate_raises_only_low_legs_first);
  RUN_TEST(test_calibrate_approaches_from_above_without_jumping);
  RUN_TEST(test_leaving_calibrate_passes_through_stand);
  RUN_TEST(test_init_loads_default_offsets);
  RUN_TEST(test_offset_change_resolves_a_still_leg);
  RUN_TEST(test_large_elapsed_time_is_bounded);
  RUN_TEST(test_rejected_target_keeps_position_and_reports_once);
  RUN_TEST(test_initialise_holds_second_pose_for_full_duration);
  RUN_TEST(test_stand_from_walk_finishes_at_current_height);
  RUN_TEST(test_stand_with_empty_first_group_moves_every_leg);
  RUN_TEST(test_wake_waits_for_sleep_fold);
  RUN_TEST(test_command_stamped_after_now_is_not_stale);
  RUN_TEST(test_stale_command_never_becomes_fresh_again);
  RUN_TEST(test_gesture_plays_from_stand_and_returns_to_stand);
  RUN_TEST(test_first_gesture_sequence_is_only_a_baseline);
  RUN_TEST(test_gesture_request_while_walking_is_dropped);
  RUN_TEST(test_gesture_request_during_a_stand_transition_is_dropped);
  RUN_TEST(test_unavailable_gesture_is_ignored);
  RUN_TEST(test_sleep_interrupts_a_gesture_at_once);
  RUN_TEST(test_stale_command_interrupts_a_gesture);
  RUN_TEST(test_walk_request_ends_a_gesture_through_stand);
  RUN_TEST(test_height_change_during_a_gesture_waits_for_stand);
  return UNITY_END();
}
