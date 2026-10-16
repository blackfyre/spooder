#include "body.h"

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
constexpr float SLEEP_TAU_S = 0.33f;
constexpr float SLEEP_SNAP_MM = 1;
constexpr float CALIBRATION_RATE = 50;        // mm per second
constexpr float CALIBRATION_INBETWEEN_Z = -20;
constexpr float STAND_LIFT_CLEARANCE = 50;    // minimum lift of the in-between point
constexpr float AT_POSITION_MM = 0.01f;

// Speed dial 0..100 → speed scale, as upstream; rotation scale is fixed.
constexpr float ROTATION_SCALE = 0.55f;

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// First-order smoothing factor for a time constant, independent of step size.
static float smoothing(float dtS, float tauS)
{
  return 1.0f - expf(-dtS / tauS);
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

// Records a foot target only when the leg can reach it.
static void setFoot(BodyState &s, int leg, Vector3 target)
{
  JointAngles angles;
  if (kinematics::solveLeg(target, kinematics::LEG_OFFSETS[leg], angles))
  {
    s.feet[leg] = target;
    s.angles[leg] = angles;
    s.legUpdated[leg] = true;
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
      if (atPosition(s.feet[i], end))
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
  st.legsDone = 0;
  st.progress = 0;

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

  if (st.progress >= CYCLE_POINTS)
  {
    st.progress = 0;
    st.legsDone += 3;
    pickHighestLegs(s);
    if (st.legsDone >= LEG_COUNT || st.currentLegs[0] == -1)
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

static gait::GaitInput gaitInput(const BodyState &s, const MovementCommand &cmd)
{
  gait::GaitInput in;
  in.translate = s.translate;
  in.rotate = s.rotate;
  in.fixedStride = cmd.stride == StrideMode::Fixed;
  in.distanceFromGround = groundForHeight(s.height);
  in.speedScale = (cmd.speed + 10) * 0.01f;
  in.rotationScale = ROTATION_SCALE;
  return in;
}

static void stepWalk(BodyState &s, const MovementCommand &cmd, float dtS)
{
  const gait::GaitParams &p = gait::params(s.currentGait);
  gait::GaitInput in = gaitInput(s, cmd);

  // Advance first, so the foot targets correspond to the current time and do
  // not lag by one step. Zero input stops progress; footTarget then holds.
  if (gait::inputWeight(in) >= gait::MIN_INPUT)
  {
    float advance = gait::progressRate(p, in) * dtS;
    for (int i = 0; i < LEG_COUNT; i++)
    {
      s.cycleProgress[i] += advance;
      while (s.cycleProgress[i] >= CYCLE_POINTS)
        s.cycleProgress[i] -= CYCLE_POINTS;
    }
  }

  for (int i = 0; i < LEG_COUNT; i++)
  {
    float t = s.cycleProgress[i] / CYCLE_POINTS;
    setFoot(s, i, gait::footTarget(i, t, p, in, s.legCycles[i], s.feet[i]));
  }
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
    Vector3 next = lerp(s.feet[i], REST_POSE, a);
    if (next.distanceTo(REST_POSE) < SLEEP_SNAP_MM)
      next = REST_POSE;
    setFoot(s, i, next);
    if (!atPosition(s.feet[i], REST_POSE))
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

// --- Initialise ------------------------------------------------------------

static void stepInitialise(BodyState &s, uint32_t nowMs)
{
  if (s.initialiseStep < 2)
  {
    for (int i = 0; i < LEG_COUNT; i++)
      setFoot(s, i, INIT_POSES[s.initialiseStep]);
    if (nowMs - s.modeEnteredMs < INIT_HOLD_MS[s.initialiseStep])
      return;
    s.initialiseStep++;
    s.modeEnteredMs = nowMs;
    if (s.initialiseStep < 2)
      return;
  }
  enterStand(s, Mode::Initialise);
}

// --- State machine ---------------------------------------------------------

void init(BodyState &s)
{
  s = BodyState();
  s.mode = Mode::Initialise;
  s.started = false;
  s.outputsEnabled = true;
  s.currentGait = Gait::Tri;
  s.initialiseStep = 0;
  for (int i = 0; i < LEG_COUNT; i++)
  {
    s.feet[i] = INIT_POSES[1];
    s.legCycles[i] = gait::LegCycle{LegPhase::Reset, s.feet[i]};
    s.stand.controlPoints[i][0] = s.feet[i];
  }
  for (int j = 0; j < 3; j++)
    s.stand.currentLegs[j] = -1;
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
  default:
    return false;
  }
}

static void smoothCommand(BodyState &s, const MovementCommand &cmd, float dtS)
{
  s.translate = lerp(s.translate, cmd.translate, smoothing(dtS, TRANSLATE_TAU_S));
  s.rotate = lerp(s.rotate, cmd.rotate, smoothing(dtS, ROTATE_TAU_S));
  s.height = lerp(s.height, cmd.height, smoothing(dtS, HEIGHT_TAU_S));
}

// Resolves the requested mode against the current mode. Sleep (requested or
// stale) pre-empts everything; other requests wait for transitions to finish.
static void resolveMode(BodyState &s, RequestedMode req)
{
  if (req == RequestedMode::Sleep)
  {
    if (s.mode != Mode::Sleep)
      enterSleep(s);
    return;
  }

  if (s.mode == Mode::Sleep)
  {
    wake(s);
    return;
  }

  if (transitionActive(s))
    return;

  switch (s.mode)
  {
  case Mode::Stand:
    if (req == RequestedMode::Walk)
      enterWalk(s);
    else if (req == RequestedMode::Calibrate)
      s.mode = Mode::Calibrate;
    break;
  case Mode::Walk:
    if (req != RequestedMode::Walk)
      enterStand(s, Mode::Walk);
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

void step(BodyState &s, const MovementCommand &rawCmd, uint32_t nowMs)
{
  uint32_t dtMs = s.started ? nowMs - s.lastStepMs : 0;
  if (dtMs > MAX_STEP_MS)
    dtMs = MAX_STEP_MS;
  if (!s.started)
    s.modeEnteredMs = nowMs;
  s.started = true;
  s.lastStepMs = nowMs;
  float dtS = dtMs / 1000.0f;

  for (int i = 0; i < LEG_COUNT; i++)
  {
    s.legUpdated[i] = false;
    s.newlyRejected[i] = false;
  }

  MovementCommand cmd = clampCommand(rawCmd);
  bool stale = nowMs - cmd.issuedAtMs > COMMAND_TIMEOUT_MS;
  RequestedMode req = stale ? RequestedMode::Sleep : cmd.mode;

  resolveMode(s, req);
  selectGait(s, cmd.gait);
  if (s.mode != Mode::Sleep)
    smoothCommand(s, cmd, dtS);

  switch (s.mode)
  {
  case Mode::Initialise:
    stepInitialise(s, nowMs);
    break;
  case Mode::Stand:
    stepStand(s, dtS);
    break;
  case Mode::Walk:
    stepWalk(s, cmd, dtS);
    break;
  case Mode::Sleep:
    stepSleep(s, dtS);
    break;
  case Mode::Calibrate:
    stepCalibrate(s, dtS);
    break;
  }
}

} // namespace body
