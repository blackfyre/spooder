#include <unity.h>
#include "movement/body.h"

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
  MovementCommand c = command(RequestedMode::Walk);
  c.translate = Vector2(0, 100);
  run(c, 20);
  TEST_ASSERT_TRUE(s.translate.magnitude() > 0);
  TEST_ASSERT_TRUE(s.translate.magnitude() < 100);
  run(c, 1200);
  TEST_ASSERT_FLOAT_WITHIN(2, 100, s.translate.magnitude());
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

// --- elapsed-time bound and target rejection --------------------------------

void test_large_elapsed_time_is_bounded()
{
  bootToStand();
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
  RUN_TEST(test_gait_change_rephases_legs);
  RUN_TEST(test_zero_input_walk_holds_and_stays_finite);
  RUN_TEST(test_rotate_only_walk_is_finite);
  RUN_TEST(test_step_input_is_eased);
  RUN_TEST(test_sleep_resets_smoothing);
  RUN_TEST(test_sleep_folds_then_disables_outputs);
  RUN_TEST(test_wake_goes_through_stand_before_walk);
  RUN_TEST(test_stale_command_puts_walking_body_to_sleep);
  RUN_TEST(test_stale_sleep_interrupts_stand_transition);
  RUN_TEST(test_calibrate_raises_only_low_legs_first);
  RUN_TEST(test_calibrate_approaches_from_above_without_jumping);
  RUN_TEST(test_leaving_calibrate_passes_through_stand);
  RUN_TEST(test_large_elapsed_time_is_bounded);
  RUN_TEST(test_rejected_target_keeps_position_and_reports_once);
  RUN_TEST(test_initialise_holds_second_pose_for_full_duration);
  RUN_TEST(test_stand_from_walk_finishes_at_current_height);
  RUN_TEST(test_stand_with_empty_first_group_moves_every_leg);
  RUN_TEST(test_wake_waits_for_sleep_fold);
  RUN_TEST(test_command_stamped_after_now_is_not_stale);
  RUN_TEST(test_stale_command_never_becomes_fresh_again);
  return UNITY_END();
}
