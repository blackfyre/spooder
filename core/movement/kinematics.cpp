#include "kinematics.h"

namespace kinematics
{

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

static float toDegrees(float radians)
{
  return radians * 180.0f / (float)M_PI;
}

// Distance from the femur joint to the foot.
static float kneeSpan(Vector3 target)
{
  float l1 = sqrtf(target.x * target.x + target.y * target.y) - COXA_LENGTH;
  return sqrtf(l1 * l1 + target.z * target.z);
}

bool isValidTarget(Vector3 target)
{
  if (!target.isFinite())
    return false;
  // kneeSpan(target) <= MAX_KNEE_SPAN, without the second square root.
  float l1 = sqrtf(target.x * target.x + target.y * target.y) - COXA_LENGTH;
  return l1 * l1 + target.z * target.z <= MAX_KNEE_SPAN * MAX_KNEE_SPAN;
}

bool solveLeg(Vector3 target, Vector3 offsets, JointAngles &out)
{
  if (!isValidTarget(target))
    return false;

  const float a1 = COXA_LENGTH;
  const float a2 = FEMUR_LENGTH;
  const float a3 = TIBIA_LENGTH;

  float x = target.x;
  float y = target.y;
  float z = target.z;

  float l1 = sqrtf(x * x + y * y) - a1; // x and y extension beyond the coxa
  float h = kneeSpan(target);

  float phi1 = acosf(clampf((h * h + a2 * a2 - a3 * a3) / (2 * h * a2), -1, 1));
  float phi2 = atan2f(z, l1);
  float phi3 = acosf(clampf((a2 * a2 + a3 * a3 - h * h) / (2 * a2 * a3), -1, 1));

  out.coxa = toDegrees(atan2f(y, x)) + offsets.x;
  out.femur = toDegrees(phi1 + phi2) + offsets.y;
  out.tibia = 180 - toDegrees(phi3) + offsets.z;
  return true;
}

static float toRadians(float degrees)
{
  return degrees * (float)M_PI / 180.0f;
}

// Fitted angles stay this far inside the 0-180° travel, so rounding never
// puts one just outside.
constexpr float RANGE_INSET_DEG = 0.01f;

// The femur joint to foot distances the tibia's servo travel allows: its
// angle is 180° - knee + offset, so the knee lies within [offset, 180° +
// offset] (and 0-180°), and the distance grows with the knee angle.
struct SpanLimits
{
  float offset, min, max;
};

static SpanLimits computeSpanLimits(float tibiaOffset)
{
  const float a2 = FEMUR_LENGTH, a3 = TIBIA_LENGTH;
  float lo = clampf(tibiaOffset, 0, 180) + RANGE_INSET_DEG, hi = clampf(180 + tibiaOffset, 0, 180) - RANGE_INSET_DEG;
  return SpanLimits{tibiaOffset, sqrtf(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cosf(toRadians(lo))),
                    sqrtf(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cosf(toRadians(hi)))};
}

// Offsets change rarely and legs have their own: the limits are kept per
// offset value, sparing the cosines on every solve.
static const SpanLimits &spanLimits(float tibiaOffset)
{
  constexpr int SLOTS = 8;
  static SpanLimits cache[SLOTS];
  static int used = 0, next = 0;
  for (int i = 0; i < used; i++)
    if (cache[i].offset == tibiaOffset)
      return cache[i];
  int slot = used < SLOTS ? used++ : next;
  next = (slot + 1) % SLOTS;
  cache[slot] = computeSpanLimits(tibiaOffset);
  return cache[slot];
}

bool solveLegInRange(Vector3 &target, Vector3 offsets, JointAngles &out)
{
  if (!isValidTarget(target))
    return false;

  const float a1 = COXA_LENGTH;
  const float a2 = FEMUR_LENGTH;
  const float a3 = TIBIA_LENGTH;
  bool moved = false;

  // Coxa: the foot's direction.
  float azimuth = atan2f(target.y, target.x);
  float azLo = toRadians(RANGE_INSET_DEG - offsets.x), azHi = toRadians(180 - RANGE_INSET_DEG - offsets.x);
  if (azimuth < azLo || azimuth > azHi)
  {
    azimuth = clampf(azimuth, azLo, azHi);
    moved = true;
  }

  // Tibia: the distance from the femur joint, along the same elevation.
  float l1 = sqrtf(target.x * target.x + target.y * target.y) - a1;
  float z = target.z;
  float h = sqrtf(l1 * l1 + z * z);
  const SpanLimits &span = spanLimits(offsets.z);
  if (h < span.min || h > span.max)
  {
    h = clampf(h, span.min, span.max);
    moved = true;
  }

  // Femur: the elevation about the femur joint, at that distance.
  float elevation = atan2f(z, l1);
  float phi1 = acosf(clampf((h * h + a2 * a2 - a3 * a3) / (2 * h * a2), -1, 1));
  float elLo = toRadians(RANGE_INSET_DEG - offsets.y) - phi1, elHi = toRadians(180 - RANGE_INSET_DEG - offsets.y) - phi1;
  if (elevation < elLo || elevation > elHi)
  {
    elevation = clampf(elevation, elLo, elHi);
    moved = true;
  }

  if (moved)
  {
    float l = h * cosf(elevation) + a1;
    target = Vector3(l * cosf(azimuth), l * sinf(azimuth), h * sinf(elevation));
  }

  float phi3 = acosf(clampf((a2 * a2 + a3 * a3 - h * h) / (2 * a2 * a3), -1, 1));
  out.coxa = toDegrees(azimuth) + offsets.x;
  out.femur = toDegrees(phi1 + elevation) + offsets.y;
  out.tibia = 180 - toDegrees(phi3) + offsets.z;
  return true;
}

} // namespace kinematics
