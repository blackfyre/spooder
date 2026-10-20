#include "body.h"
#include "servo_profile.h"

namespace body
{

using gait::CYCLE_POINTS;
using gait::Gait;
using gait::LegPhase;

// Initialise poses and how long each is held.
static const Vector3 INIT_POSES[2] = {Vector3(160, 0, 0), Vector3(225, 0, 115)};
static const uint32_t INIT_HOLD_MS[2] = {25, 500};

// Height dial 0..100 maps to foot height below the coxa pivot.
constexpr float GROUND_BASE = -60;
constexpr float GROUND_PER_HEIGHT = -1.7f;

// Time-based rates (design D5), converted from the original per-loop values
// at a 20 ms reference step. Tune on hardware.
constexpr float STAND_RATE = 1250;            // progress units per second
constexpr float TRANSLATE_TAU_S = 0.25f;
constexpr float ROTATE_TAU_S = 0.16f;
constexpr float HEIGHT_TAU_S = 0.5f;
constexpr float SPEED_TAU_S = 0.25f;
constexpr float STRIDE_MODE_TAU_S = 0.25f;
constexpr float SLEEP_TAU_S = 0.33f;
constexpr float SLEEP_SNAP_MM = 1;
constexpr float CALIBRATION_RATE = 50;        // mm per second
constexpr float CALIBRATION_INBETWEEN_Z = -20;
constexpr float STAND_LIFT_CLEARANCE = 50;    // minimum lift of the in-between point
constexpr float AT_POSITION_MM = 0.01f;
// Internal sub-step: the state machine advances in these increments.
constexpr uint32_t INTEGRATION_STEP_US = 5000;

// Joint speed budget (design: the motion clock). A step that would turn a
// joint faster than the budget is run again over less motion time, aiming
// this far below the budget; at most this many runs; never less than this
// share of the elapsed time.
constexpr float BUDGET_AIM = 0.9f;
constexpr int MAX_BUDGET_RUNS = 3;
constexpr float LAST_RUN_MARGIN = 0.7f;
constexpr float MIN_MOTION_SCALE = 0.02f;
// Under a budget, motion advances in pieces of this much wall time, each
// governed on its own.
constexpr uint32_t BUDGET_INTERVAL_US = 20000;
// At most this many pieces per step (50 ms ticks need three): after an
// overrun the rest is dropped, as MAX_STEP_MS drops a long stall, so catching
// up cannot make the next step longer still.
constexpr uint32_t MAX_PIECES_PER_STEP = 3;

// Speed dial 0..100 → speed scale, as upstream; rotation scale is fixed.
constexpr float ROTATION_SCALE = 0.55f;

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// Adds @p dtUs to @p carryUs and takes the whole milliseconds out of it, for
// the parts that count time in milliseconds.
static uint32_t takeWholeMs(uint32_t &carryUs, uint32_t dtUs)
{
  carryUs += dtUs;
  uint32_t ms = carryUs / 1000;
  carryUs -= ms * 1000;
  return ms;
}

// First-order smoothing factor for a time constant, independent of step size.
// Sub-steps mostly repeat a few lengths, so the last factor per time constant
// is kept: expf is costly on the board.
static float smoothing(float dtS, float tauS)
{
  struct Cached
  {
    float tauS, dtS, factor;
  };
  static Cached cache[8];
  static int next = 0;
  for (Cached &c : cache)
    if (c.tauS == tauS)
    {
      if (c.dtS != dtS)
      {
        c.dtS = dtS;
        c.factor = 1.0f - expf(-dtS / tauS);
      }
      return c.factor;
    }
  Cached &c = cache[next];
  next = (next + 1) % 8;
  c = Cached{tauS, dtS, 1.0f - expf(-dtS / tauS)};
  return c.factor;
}

static float stepToward(float value, float target, float maxStep)
{
  float delta = target - value;
  if (fabsf(delta) <= maxStep)
    return target;
  return value + (delta > 0 ? maxStep : -maxStep);
}

float groundForHeight(float height)
{
  return GROUND_BASE + GROUND_PER_HEIGHT * height;
}

MovementCommand clampCommand(const MovementCommand &cmd)
{
  MovementCommand c = cmd;
  float mag = c.translate.magnitude();
  if (!(mag <= 100)) // also catches NaN
    c.translate = (mag > 100 && isfinite(mag)) ? c.translate * (100 / mag) : Vector2();
  c.rotate = isfinite(c.rotate) ? clampf(c.rotate, -100, 100) : 0;
  c.height = isfinite(c.height) ? clampf(c.height, 0, 100) : 0;
  c.speed = isfinite(c.speed) ? clampf(c.speed, 0, 100) : 0;
  return c;
}

// Records a foot target only when the leg can reach it. The joint angles are
// solved once at the end of the step (solveAngles).
static void setFoot(BodyState &s, int leg, Vector3 target)
{
  if (kinematics::isValidTarget(target))
  {
    s.feet[leg] = target;
    s.pendingSolve[leg] = true;
    s.rejected[leg] = false;
    return;
  }
  if (!s.rejected[leg])
  {
    s.newlyRejected[leg] = true;
    s.rejectedTarget[leg] = target;
  }
  s.rejected[leg] = true;
}

// Where a leg's foot ends up for @p p: fitted to its servo travel, as
// solveAngles fits it (outside Calibrate). A destination the body waits for
// must be the fitted one, or a leg whose offsets put it out of travel would
// never arrive.
static Vector3 fittedFor(const BodyState &s, int leg, Vector3 p)
{
  JointAngles unused;
  kinematics::solveLegInRange(p, s.offsets[leg], unused);
  return p;
}

static bool atPosition(Vector3 a, Vector3 b)
{
  return a.distanceTo(b) < AT_POSITION_MM;
}

static Vector3 standingPoint(const BodyState &s)
{
  return Vector3(gait::STANCE_DISTANCE, 0, groundForHeight(s.height));
}

// --- Stand -----------------------------------------------------------------

// Picks up to three legs not yet standing, highest first.
static void pickHighestLegs(BodyState &s)
{
  StandTransition &st = s.stand;
  Vector3 end = standingPoint(s);
  for (int j = 0; j < 3; j++)
    st.currentLegs[j] = -1;

  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < LEG_COUNT; i++)
    {
      if (st.currentLegs[0] == i || st.currentLegs[1] == i || st.currentLegs[2] == i)
        continue;
      if (st.legDone[i] || atPosition(s.feet[i], fittedFor(s, i, end)))
        continue;
      if (st.currentLegs[j] == -1 || s.feet[i].z > s.feet[st.currentLegs[j]].z)
        st.currentLegs[j] = i;
    }
  }
}

static void enterStand(BodyState &s, Mode from)
{
  StandTransition &st = s.stand;
  st.active = true;
  st.moveAllAtOnce = (from == Mode::Initialise || from == Mode::Sleep || from == Mode::Calibrate);
  st.progress = 0;
  for (int i = 0; i < LEG_COUNT; i++)
    st.legDone[i] = false;

  Vector3 end = standingPoint(s);
  for (int i = 0; i < LEG_COUNT; i++)
  {
    Vector3 start = s.feet[i];
    Vector3 mid((start.x + end.x) / 2, (start.y + end.y) / 2, (start.z + end.z) / 2);
    if (fabsf(mid.z - end.z) < STAND_LIFT_CLEARANCE)
      mid.z += STAND_LIFT_CLEARANCE;
    st.controlPoints[i][0] = start;
    st.controlPoints[i][1] = mid;
    st.controlPoints[i][2] = end;
    s.legCycles[i].phase = LegPhase::Standing;
  }
  pickHighestLegs(s);

  s.mode = Mode::Stand;
}

static void stepStand(BodyState &s, float dtS)
{
  StandTransition &st = s.stand;
  Vector3 end = standingPoint(s);
  // Track height changes, during and after the transition.
  for (int i = 0; i < LEG_COUNT; i++)
    st.controlPoints[i][2] = end;

  if (!st.active)
  {
    for (int i = 0; i < LEG_COUNT; i++)
      setFoot(s, i, end);
    return;
  }

  st.progress += STAND_RATE * dtS;
  float t = fminf(st.progress / CYCLE_POINTS, 1);

  if (st.moveAllAtOnce)
  {
    for (int i = 0; i < LEG_COUNT; i++)
      setFoot(s, i, gait::bezier(st.controlPoints[i], 3, t));
    if (st.progress >= CYCLE_POINTS)
      st.active = false;
    return;
  }

  for (int j = 0; j < 3; j++)
  {
    int leg = st.currentLegs[j];
    if (leg != -1)
      setFoot(s, leg, gait::bezier(st.controlPoints[leg], 3, t));
  }
  // Legs from earlier groups slide with height changes instead of being left
  // at the height they landed at.
  for (int i = 0; i < LEG_COUNT; i++)
    if (st.legDone[i])
      setFoot(s, i, end);

  if (st.progress >= CYCLE_POINTS)
  {
    for (int j = 0; j < 3; j++)
      if (st.currentLegs[j] != -1)
        st.legDone[st.currentLegs[j]] = true;
    st.progress -= CYCLE_POINTS;
    // Done when no unfinished leg is left. Each non-empty group marks at
    // least one leg done, and an empty group picks again, so this ends.
    pickHighestLegs(s);
    if (st.currentLegs[0] == -1)
      st.active = false;
  }
}

// --- Walk ------------------------------------------------------------------

static void rephase(BodyState &s)
{
  const gait::GaitParams &p = gait::params(s.currentGait);
  for (int i = 0; i < LEG_COUNT; i++)
  {
    s.cycleProgress[i] = p.phaseOffset[i];
    s.legCycles[i].phase = LegPhase::Reset;
  }
}

static void enterWalk(BodyState &s)
{
  s.mode = Mode::Walk;
  rephase(s);
}

static gait::GaitInput gaitInput(const BodyState &s)
{
  gait::GaitInput in;
  in.translate = s.translate;
  in.rotate = s.rotate;
  in.fixedStride = s.fixedStride;
  in.distanceFromGround = groundForHeight(s.height);
  in.speedScale = (s.speed + 10) * 0.01f;
  in.rotationScale = ROTATION_SCALE;
  return in;
}

// Advances gait progress over one integration sub-step. Zero input stops
// progress; footTarget then holds every leg.
static void advanceWalk(BodyState &s, float dtS)
{
  const gait::GaitParams &p = gait::params(s.currentGait);
  gait::GaitInput in = gaitInput(s);
  if (gait::inputWeight(in) < gait::MIN_INPUT)
    return;

  float advance = gait::progressRate(p, in) * dtS;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    s.cycleProgress[i] += advance;
    while (s.cycleProgress[i] >= CYCLE_POINTS)
      s.cycleProgress[i] -= CYCLE_POINTS;
    // Record phase changes at sub-step resolution; a fast gait's ground phase
    // can be shorter than one tick.
    gait::trackPhase(i, s.cycleProgress[i] / CYCLE_POINTS, p, in, s.legCycles[i], s.feet[i]);
  }
}

// Writes foot targets for the current progress. Called after advanceWalk, so
// targets correspond to the current time rather than lagging by one step.
static void stepWalk(BodyState &s)
{
  const gait::GaitParams &p = gait::params(s.currentGait);
  gait::GaitInput in = gaitInput(s);

  for (int i = 0; i < LEG_COUNT; i++)
  {
    float t = s.cycleProgress[i] / CYCLE_POINTS;
    setFoot(s, i, gait::footTarget(i, t, p, in, s.legCycles[i], s.feet[i]));
  }
}

// Before leaving Walk mid-step, bring the feet to where the gait has them
// now, so the next transition starts from the current position.
static void leaveWalk(BodyState &s)
{
  if (s.mode == Mode::Walk)
    stepWalk(s);
}

// --- Sleep -----------------------------------------------------------------

static void enterSleep(BodyState &s)
{
  s.mode = Mode::Sleep;
  s.sleepFolded = false;
  s.stand.active = false;
  s.translate = Vector2();
  s.rotate = 0;
}

static void stepSleep(BodyState &s, float dtS)
{
  if (s.sleepFolded)
    return;

  float a = smoothing(dtS, SLEEP_TAU_S);
  bool allThere = true;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    Vector3 rest = fittedFor(s, i, REST_POSE);
    Vector3 next = interpolate(s.feet[i], rest, a);
    if (next.distanceTo(rest) < SLEEP_SNAP_MM)
      next = rest;
    setFoot(s, i, next);
    if (!atPosition(s.feet[i], rest))
      allThere = false;
  }

  if (allThere)
  {
    s.sleepFolded = true;
    s.outputsEnabled = false;
  }
}

static void wake(BodyState &s)
{
  s.outputsEnabled = true;
  s.sleepFolded = false;
  enterStand(s, Mode::Sleep);
}

// --- Calibrate -------------------------------------------------------------

static void stepCalibrate(BodyState &s, float dtS)
{
  float maxStep = CALIBRATION_RATE * dtS;

  bool legsUp = true;
  for (int i = 0; i < LEG_COUNT; i++)
    if (s.feet[i].z < CALIBRATION_INBETWEEN_Z)
      legsUp = false;

  for (int i = 0; i < LEG_COUNT; i++)
  {
    Vector3 f = s.feet[i];
    if (!legsUp)
    {
      // Raise only the legs that are still low.
      if (f.z < CALIBRATION_INBETWEEN_Z)
        setFoot(s, i, Vector3(f.x, f.y, stepToward(f.z, CALIBRATION_INBETWEEN_Z, maxStep)));
      continue;
    }
    setFoot(s, i, Vector3(stepToward(f.x, CALIBRATION_POSE.x, maxStep),
                          stepToward(f.y, CALIBRATION_POSE.y, maxStep),
                          stepToward(f.z, CALIBRATION_POSE.z, maxStep)));
  }
}

// --- Perform ---------------------------------------------------------------

static void enterPerform(BodyState &s, gesture::Gesture g, uint32_t modeRequestSeq)
{
  gesture::start(s.perform, g, s.feet, standingPoint(s));
  s.performCarryUs = 0;
  s.performRequestSeq = modeRequestSeq;
  s.mode = Mode::Perform;
}

static void stepPerform(BodyState &s, uint32_t dtUs)
{
  Vector3 targets[LEG_COUNT];
  gesture::advance(s.perform, takeWholeMs(s.performCarryUs, dtUs), standingPoint(s), targets);
  for (int i = 0; i < LEG_COUNT; i++)
    setFoot(s, i, targets[i]);
  if (s.perform.done)
  {
    // The last keyframe (or the return) ends at the standing point.
    s.mode = Mode::Stand;
    s.stand.active = false;
    for (int i = 0; i < LEG_COUNT; i++)
      s.legCycles[i].phase = LegPhase::Standing;
  }
}

uint8_t playingGesture(const BodyState &s)
{
  return s.mode == Mode::Perform ? (uint8_t)s.perform.gesture : 0;
}

// --- Initialise ------------------------------------------------------------

static void stepInitialise(BodyState &s, uint32_t dtUs)
{
  s.initialiseElapsedMs += takeWholeMs(s.initialiseCarryUs, dtUs);
  if (s.initialiseElapsedMs >= INIT_HOLD_MS[s.initialiseStep])
  {
    s.initialiseElapsedMs -= INIT_HOLD_MS[s.initialiseStep];
    s.initialiseStep++;
    if (s.initialiseStep >= 2)
    {
      enterStand(s, Mode::Initialise);
      return;
    }
  }
  // Issue the current pose, including on the sub-step its hold starts.
  for (int i = 0; i < LEG_COUNT; i++)
    setFoot(s, i, INIT_POSES[s.initialiseStep]);
}

// --- State machine ---------------------------------------------------------

void init(BodyState &s)
{
  joint_calibration::Table defaults;
  joint_calibration::loadDefaults(defaults);
  s = BodyState();
  s.mode = Mode::Initialise;
  s.started = false;
  s.outputsEnabled = true;
  s.currentGait = Gait::Tri;
  s.speed = MovementCommand().speed;
  s.jointBudgetDegPerS = servo_profile::SELECTED.budgetDegPerS();
  s.motionScale = 1;
  s.initialiseStep = 0;
  s.initialiseElapsedMs = 0;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    s.feet[i] = INIT_POSES[1];
    s.legCycles[i] = gait::LegCycle{LegPhase::Reset, s.feet[i]};
    s.stand.controlPoints[i][0] = s.feet[i];
    s.offsets[i] = joint_calibration::legOffsets(defaults, i);
  }
  for (int j = 0; j < 3; j++)
    s.stand.currentLegs[j] = -1;
}

void setOffsets(BodyState &s, int leg, Vector3 offsets)
{
  if (leg < 0 || leg >= LEG_COUNT)
    return;
  Vector3 &o = s.offsets[leg];
  if (o.x == offsets.x && o.y == offsets.y && o.z == offsets.z)
    return;
  o = offsets;
  s.offsetsChanged[leg] = true;
}

bool transitionActive(const BodyState &s)
{
  switch (s.mode)
  {
  case Mode::Initialise:
    return true;
  case Mode::Stand:
    return s.stand.active;
  case Mode::Sleep:
    return !s.sleepFolded;
  case Mode::Perform:
    return true;
  default:
    return false;
  }
}

static void smoothCommand(BodyState &s, const MovementCommand &cmd, float dtS)
{
  s.translate = interpolate(s.translate, cmd.translate, smoothing(dtS, TRANSLATE_TAU_S));
  s.rotate = interpolate(s.rotate, cmd.rotate, smoothing(dtS, ROTATE_TAU_S));
  // A gesture plays at the height it started at; a change applies, smoothed
  // as usual, once the body is back in Stand.
  if (s.mode != Mode::Perform)
    s.height = interpolate(s.height, cmd.height, smoothing(dtS, HEIGHT_TAU_S));
  // The speed dial and the stride mode change the stride: eased, so the feet
  // do not jump.
  s.speed = interpolate(s.speed, cmd.speed, smoothing(dtS, SPEED_TAU_S));
  float fixed = cmd.stride == StrideMode::Fixed ? 1.0f : 0.0f;
  s.fixedStride = interpolate(s.fixedStride, fixed, smoothing(dtS, STRIDE_MODE_TAU_S));
}

// Resolves the requested mode against the current mode. Sleep (requested or
// stale) pre-empts everything; other requests wait for transitions to finish.
// A gesture request (@p newGesture, 0 for none) is acted on only from a
// settled Stand that stays requested; otherwise it is dropped.
static void resolveMode(BodyState &s, RequestedMode req, const MovementCommand &cmd, uint8_t newGesture)
{
  if (req == RequestedMode::Sleep)
  {
    if (s.mode != Mode::Sleep)
    {
      leaveWalk(s);
      enterSleep(s);
    }
    return;
  }

  if (s.mode == Mode::Sleep)
  {
    // Like every other transition, the fold completes before waking.
    if (s.sleepFolded)
      wake(s);
    return;
  }

  if (s.mode == Mode::Perform)
  {
    // Any other mode, or a new explicit request (Stand again included), ends
    // the gesture after its current keyframe.
    if (req != RequestedMode::Stand || cmd.modeRequestSeq != s.performRequestSeq)
      s.perform.stopRequested = true;
    return;
  }

  if (transitionActive(s))
    return;

  switch (s.mode)
  {
  case Mode::Stand:
    if (req == RequestedMode::Stand && gesture::available(newGesture))
      enterPerform(s, (gesture::Gesture)newGesture, cmd.modeRequestSeq);
    else if (req == RequestedMode::Walk)
      enterWalk(s);
    else if (req == RequestedMode::Calibrate)
      s.mode = Mode::Calibrate;
    break;
  case Mode::Walk:
    if (req != RequestedMode::Walk)
    {
      leaveWalk(s);
      enterStand(s, Mode::Walk);
    }
    break;
  case Mode::Calibrate:
    if (req != RequestedMode::Calibrate)
      enterStand(s, Mode::Calibrate);
    break;
  default:
    break;
  }
}

static void selectGait(BodyState &s, int requested)
{
  if (!gait::isValidGait(requested))
    return;
  Gait g = static_cast<Gait>(requested);
  if (g == s.currentGait)
    return;
  s.currentGait = g;
  if (s.mode == Mode::Walk)
    rephase(s);
}

static bool isStale(BodyState &s, uint32_t issuedAtMs, uint32_t nowMs)
{
  if (s.staleLatched && issuedAtMs == s.staleStamp)
    return true;
  s.staleLatched = false;

  bool aheadOfNow = issuedAtMs - nowMs <= MAX_COMMAND_LEAD_MS;
  if (aheadOfNow || nowMs - issuedAtMs <= COMMAND_TIMEOUT_MS)
    return false;

  s.staleLatched = true;
  s.staleStamp = issuedAtMs;
  return true;
}

// One fixed sub-step of the whole state machine.
static void subStep(BodyState &s, const MovementCommand &cmd, RequestedMode req, uint8_t newGesture, uint32_t dtUs)
{
  float dtS = dtUs / 1e6f;

  resolveMode(s, req, cmd, newGesture);
  selectGait(s, cmd.gait);
  if (s.mode != Mode::Sleep)
    smoothCommand(s, cmd, dtS);

  switch (s.mode)
  {
  case Mode::Initialise:
    stepInitialise(s, dtUs);
    break;
  case Mode::Stand:
    stepStand(s, dtS);
    break;
  case Mode::Walk:
    advanceWalk(s, dtS);
    break;
  case Mode::Sleep:
    stepSleep(s, dtS);
    break;
  case Mode::Calibrate:
    stepCalibrate(s, dtS);
    break;
  case Mode::Perform:
    stepPerform(s, dtUs);
    break;
  }
}

// Solves joint angles for every leg recorded during this step.
static void solveAngles(BodyState &s)
{
  for (int i = 0; i < LEG_COUNT; i++)
  {
    if (!s.pendingSolve[i] && !s.offsetsChanged[i])
      continue;
    s.pendingSolve[i] = false;
    s.offsetsChanged[i] = false;
    // setFoot only records targets solveLeg accepts, so this succeeds.
    // Outside Calibrate the foot is fitted into the servos' travel (the
    // recorded foot follows); Calibrate shows each joint as calibrated.
    bool solved = s.mode == Mode::Calibrate ? kinematics::solveLeg(s.feet[i], s.offsets[i], s.angles[i])
                                            : kinematics::solveLegInRange(s.feet[i], s.offsets[i], s.angles[i]);
    if (solved)
      s.legUpdated[i] = true;
  }
}

// The walk's foot paths and the joint angles, after a step's sub-steps.
static void finishMotion(BodyState &s)
{
  // Walk evaluates its foot paths once per step; phases were tracked per sub-step.
  if (s.mode == Mode::Walk)
    stepWalk(s);

  solveAngles(s);
}

// Runs the state machine over @p motionUs of motion time, then finishes the
// step. With @p halfway, also leaves there the state finished at the first
// sub-step boundary from half the time on, and returns that share of the
// time (0 without).
static float advanceMotion(BodyState &s, const MovementCommand &cmd, RequestedMode req, uint8_t newGesture,
                           uint32_t motionUs, BodyState *halfway = nullptr)
{
  float share = 0;
  // At least one (possibly zero-length) sub-step, so the first call acts.
  uint32_t done = 0;
  do
  {
    uint32_t h = motionUs - done < INTEGRATION_STEP_US ? motionUs - done : INTEGRATION_STEP_US;
    subStep(s, cmd, req, newGesture, h);
    newGesture = 0;
    done += h;
    if (halfway && share == 0 && done < motionUs && 2 * done >= motionUs)
    {
      *halfway = s;
      finishMotion(*halfway);
      share = (float)done / motionUs;
    }
  } while (done < motionUs);

  finishMotion(s);
  return share;
}

// The largest joint angle change from @p before to @p after, in degrees.
static float largestJointChange(const BodyState &before, const BodyState &after)
{
  float largest = 0;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    const JointAngles &a = before.angles[i], &b = after.angles[i];
    largest = fmaxf(largest, fmaxf(fabsf(b.coxa - a.coxa), fmaxf(fabsf(b.femur - a.femur), fabsf(b.tibia - a.tibia))));
  }
  return largest;
}

// The trial runs of a governed step, the first run's part-way state and the
// closest run so far: a BodyState is too large for the board's 2 KB main
// stack. One body steps at a time.
static BodyState trial, halfway, best;

void step(BodyState &s, const MovementCommand &rawCmd, uint32_t nowMs)
{
  uint32_t dtMs = s.started ? nowMs - s.lastStepMs : 0;
  if (dtMs > MAX_STEP_MS)
    dtMs = MAX_STEP_MS;
  bool first = !s.started;
  s.started = true;
  s.lastStepMs = nowMs;

  for (int i = 0; i < LEG_COUNT; i++)
  {
    s.legUpdated[i] = false;
    s.newlyRejected[i] = false;
    s.pendingSolve[i] = false;
  }

  MovementCommand cmd = clampCommand(rawCmd);
  // Staleness is judged on wall time, whatever the motion clock does.
  bool stale = isStale(s, cmd.issuedAtMs, nowMs);
  RequestedMode req = stale ? RequestedMode::Sleep : cmd.mode;

  // A new gesture request is offered to the first sub-step only (under a
  // budget, the next piece's): acted on there or dropped. The first sequence
  // value seen is only a baseline.
  uint8_t newGesture = 0;
  if (s.gestureSeqKnown && cmd.gestureSeq != s.lastGestureSeq)
    newGesture = cmd.gesture;
  s.gestureSeqKnown = true;
  s.lastGestureSeq = cmd.gestureSeq;

  uint32_t wallUs = dtMs * 1000;
  s.motionScale = 1;
  if (s.jointBudgetDegPerS <= 0)
  {
    advanceMotion(s, cmd, req, newGesture, wallUs);
    return;
  }

  // Under a budget, motion advances in whole grid pieces from the first
  // step on; the rest of the elapsed time waits for the next step. Every
  // tick length then governs the same pieces.
  // A gesture request waits for the next piece rather than being lost.
  if (newGesture)
    s.pendingGesture = newGesture;
  if (first)
  {
    // The first step acts at once (Initialise's first pose).
    advanceMotion(s, cmd, req, s.pendingGesture, 0);
    s.pendingGesture = 0;
    return;
  }
  s.motionPendingUs += wallUs;
  if (s.motionPendingUs > MAX_PIECES_PER_STEP * BUDGET_INTERVAL_US)
    s.motionPendingUs = MAX_PIECES_PER_STEP * BUDGET_INTERVAL_US;

  const float allowed = s.jointBudgetDegPerS * (BUDGET_INTERVAL_US / 1e6f); // degrees per piece
  newGesture = s.pendingGesture;
  float slowest = 1;
  for (; s.motionPendingUs >= BUDGET_INTERVAL_US; s.motionPendingUs -= BUDGET_INTERVAL_US)
  {
    const float target = BUDGET_AIM * allowed;
    float scale = 1, bestChange = INFINITY, bestScale = 1;
    const BodyState *result = &trial;
    for (int run = 1;; run++)
    {
      trial = s;
      float share = advanceMotion(trial, cmd, req, newGesture, (uint32_t)lroundf(BUDGET_INTERVAL_US * scale),
                                  run == 1 ? &halfway : nullptr);
      // Initialise sets its poses as step changes: never slowed.
      if (s.mode == Mode::Initialise)
        break;
      float change = largestJointChange(s, trial);
      if (change <= allowed)
        break;
      // Near an awkward pose less time can even mean more change: if no run
      // keeps within the budget, the one that came closest is kept.
      if (change < bestChange)
      {
        best = trial;
        bestChange = change;
        bestScale = scale;
      }
      if (run == MAX_BUDGET_RUNS || scale <= MIN_MOTION_SCALE)
      {
        result = &best;
        scale = bestScale;
        break;
      }
      float next = scale * target / change; // as if change were proportional to time
      if (share > 0)
      {
        // The first run also measured part-way: interpolate through no
        // change at no time, the part-way change and the full change, which
        // also fits a piece whose fast part comes early or late.
        float mid = largestJointChange(s, halfway);
        next = mid >= target ? share * target / mid
                             : share + (1 - share) * (target - mid) / fmaxf(change - mid, 1e-6f);
      }
      // The last run gets a margin, being the last chance to keep within.
      if (run + 1 == MAX_BUDGET_RUNS)
        next *= LAST_RUN_MARGIN;
      scale = fmaxf(MIN_MOTION_SCALE, fminf(next, scale));
    }
    s = *result;
    slowest = fminf(slowest, scale);
    newGesture = 0;
    s.pendingGesture = 0;
  }
  s.motionScale = slowest;
}

} // namespace body
