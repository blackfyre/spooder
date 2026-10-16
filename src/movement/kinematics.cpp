#include "kinematics.h"

namespace kinematics
{

const Vector3 LEG_OFFSETS[LEG_COUNT] = {
    {90, 75, -18},
    {93, 75, -15},
    {93, 75, -18},
    {87, 80, -26},
    {85, 89, -16},
    {93, 85, -24},
};

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

static float toDegrees(float radians)
{
  return radians * 180.0f / (float)M_PI;
}

bool isValidTarget(Vector3 target)
{
  return target.isFinite() && target.magnitude() <= LEG_REACH;
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

  float l = sqrtf(x * x + y * y); // x and y extension
  float l1 = l - a1;
  float h = sqrtf(l1 * l1 + z * z);

  float phi1 = acosf(clampf((h * h + a2 * a2 - a3 * a3) / (2 * h * a2), -1, 1));
  float phi2 = atan2f(z, l1);
  float phi3 = acosf(clampf((a2 * a2 + a3 * a3 - h * h) / (2 * a2 * a3), -1, 1));

  out.coxa = toDegrees(atan2f(y, x)) + offsets.x;
  out.femur = toDegrees(phi1 + phi2) + offsets.y;
  out.tibia = 180 - toDegrees(phi3) + offsets.z;
  return true;
}

uint16_t angleToMicroseconds(float angle)
{
  float clamped = clampf(angle, 0, 180);
  return (uint16_t)(500.0f + ((2500.0f - 500.0f) / 180.0f) * clamped);
}

} // namespace kinematics
