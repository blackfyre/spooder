#include "joint_calibration.h"

#include <math.h>

namespace joint_calibration
{

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// Moves v towards target by at most maxStep, landing on it exactly.
static float approach(float v, float target, float maxStep)
{
  if (fabsf(target - v) <= maxStep)
    return target;
  return v < target ? v + maxStep : v - maxStep;
}

void loadDefaults(Table &t)
{
  for (int i = 0; i < JOINT_COUNT; i++)
    t.joints[i] = DEFAULTS[i];
}

bool clampJoint(Joint &j, const Joint &defaults)
{
  Joint in = j;
  j.offsetDeg = clampf(j.offsetDeg, defaults.offsetDeg - MAX_OFFSET_FROM_DEFAULT_DEG,
                       defaults.offsetDeg + MAX_OFFSET_FROM_DEFAULT_DEG);
  j.pulse0Us = clampf(j.pulse0Us, MIN_PULSE_US, MAX_PULSE_US);
  j.pulse180Us = clampf(j.pulse180Us, MIN_PULSE_US, MAX_PULSE_US);
  if (fabsf(j.pulse180Us - j.pulse0Us) < MIN_PULSE_SPAN_US)
  {
    // Keep the joint's direction of travel: equal pulses count as rising.
    if (j.pulse180Us >= j.pulse0Us)
    {
      j.pulse180Us = j.pulse0Us + MIN_PULSE_SPAN_US;
      if (j.pulse180Us > MAX_PULSE_US)
      {
        j.pulse180Us = MAX_PULSE_US;
        j.pulse0Us = MAX_PULSE_US - MIN_PULSE_SPAN_US;
      }
    }
    else
    {
      j.pulse180Us = j.pulse0Us - MIN_PULSE_SPAN_US;
      if (j.pulse180Us < MIN_PULSE_US)
      {
        j.pulse180Us = MIN_PULSE_US;
        j.pulse0Us = MIN_PULSE_US + MIN_PULSE_SPAN_US;
      }
    }
  }
  return j.offsetDeg != in.offsetDeg || j.pulse0Us != in.pulse0Us || j.pulse180Us != in.pulse180Us;
}

static void stepJoint(Joint &a, const Joint &r, float dtS)
{
  // One quantity at a time, so the output's rate is never the sum of two.
  if (a.offsetDeg != r.offsetDeg)
  {
    a.offsetDeg = approach(a.offsetDeg, r.offsetDeg, MAX_RATE_DEG_PER_S * dtS);
    return;
  }

  // A reversed joint with end points (p0, p180) outputs exactly what a normal
  // one with (p180, p0) does: approach the requested end points in the
  // applied direction, then flip the direction without moving the output.
  bool flip = a.reversed != r.reversed;
  float target0 = flip ? r.pulse180Us : r.pulse0Us;
  float target180 = flip ? r.pulse0Us : r.pulse180Us;
  if (a.pulse0Us != target0 || a.pulse180Us != target180)
  {
    // The output lies between the end points, so it moves no more than the
    // end points do. 60°/s through the current span; the floor keeps a span
    // that briefly narrows while the end points cross from stalling.
    float span = fabsf(a.pulse180Us - a.pulse0Us);
    if (span < MIN_PULSE_SPAN_US)
      span = MIN_PULSE_SPAN_US;
    float maxStep = MAX_RATE_DEG_PER_S * span / 180.0f * dtS;
    a.pulse0Us = approach(a.pulse0Us, target0, maxStep);
    a.pulse180Us = approach(a.pulse180Us, target180, maxStep);
    return;
  }

  if (flip)
    a = r;
}

void step(Table &applied, const Table &requested, float dtS)
{
  if (!(dtS > 0))
    return;
  for (int i = 0; i < JOINT_COUNT; i++)
    stepJoint(applied.joints[i], requested.joints[i], dtS);
}

uint16_t toPulse(const Joint &j, float angleDeg)
{
  // solveLeg never produces a non-finite angle; mid-travel keeps it defined.
  float a = isfinite(angleDeg) ? clampf(angleDeg, 0, 180) : 90;
  if (j.reversed)
    a = 180 - a;
  float us = j.pulse0Us + (j.pulse180Us - j.pulse0Us) * a / 180.0f;
  return (uint16_t)(us + 0.5f);
}

Vector3 legOffsets(const Table &t, int leg)
{
  return Vector3(t.joints[kinematics::servoOutputIndex(leg, 0)].offsetDeg,
                 t.joints[kinematics::servoOutputIndex(leg, 1)].offsetDeg,
                 t.joints[kinematics::servoOutputIndex(leg, 2)].offsetDeg);
}

} // namespace joint_calibration
