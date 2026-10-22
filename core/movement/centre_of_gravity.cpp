#include "centre_of_gravity.h"
#include "gait.h"
#include <math.h>

namespace centre_of_gravity
{

static float rad(float deg)
{
  return deg * (float)M_PI / 180.0f;
}

Vector3 toBody(int leg, Vector3 p)
{
  // The leg frame's x axis points along the mount angle; its y axis is x
  // turned 90° clockwise (legview.Mount).
  float a = rad(gait::mountAngleDeg(leg));
  float c = cosf(a), s = sinf(a);
  return Vector3(PIVOTS[leg].x + p.x * c + p.y * s, PIVOTS[leg].y + p.x * s - p.y * c, p.z);
}

LegPoints legInBody(int leg, kinematics::JointAngles a)
{
  // In the leg's vertical plane (outward, up): the femur's elevation is its
  // angle, the tibia's the femur's less the tibia's (legview.SideView).
  float femur = rad(a.femur), tibia = rad(a.femur - a.tibia);
  float outward[4], up[4];
  outward[0] = 0;
  up[0] = 0;
  outward[1] = kinematics::COXA_LENGTH;
  up[1] = 0;
  outward[2] = outward[1] + kinematics::FEMUR_LENGTH * cosf(femur);
  up[2] = kinematics::FEMUR_LENGTH * sinf(femur);
  outward[3] = outward[2] + kinematics::TIBIA_LENGTH * cosf(tibia);
  up[3] = up[2] + kinematics::TIBIA_LENGTH * sinf(tibia);
  // The plane turned by the coxa angle from x towards +y.
  float c = cosf(rad(a.coxa)), s = sinf(rad(a.coxa));
  LegPoints out;
  for (int i = 0; i < 4; i++)
    out.points[i] = toBody(leg, Vector3(outward[i] * c, outward[i] * s, up[i]));
  return out;
}

Estimate estimate(const kinematics::JointAngles legs[LEG_COUNT], bool battery)
{
  Vector3 moment = BODY_AT * BODY_G;
  float mass = BODY_G;
  auto add = [&](Vector3 at, float g) {
    moment = moment + at * g;
    mass += g;
  };
  if (battery)
    add(BATTERY_AT, BATTERY_G);
  for (int leg = 0; leg < LEG_COUNT; leg++)
  {
    LegPoints l = legInBody(leg, legs[leg]);
    const Vector3 *p = l.points;
    add(p[0], SERVO_G); // coxa servo
    add(p[1], SERVO_G); // femur servo
    add(p[2], SERVO_G); // tibia servo
    add(interpolate(p[0], p[1], 0.5f), COXA_LINK_G);
    add(interpolate(p[1], p[2], 0.5f), FEMUR_LINK_G);
    add(interpolate(p[2], p[3], TIBIA_LINK_AT), TIBIA_LINK_G);
  }
  return Estimate{moment / mass, mass};
}

static float cross(Vector2 o, Vector2 a, Vector2 b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

int supportHull(const Vector3 feet[LEG_COUNT], Vector2 hull[LEG_COUNT])
{
  float lowest = feet[0].z;
  for (int i = 1; i < LEG_COUNT; i++)
    lowest = fminf(lowest, feet[i].z);
  Vector2 pts[LEG_COUNT];
  int n = 0;
  for (int i = 0; i < LEG_COUNT; i++)
    if (feet[i].z <= lowest + CONTACT_MM)
      pts[n++] = Vector2(feet[i].x, feet[i].y);
  if (n < 3)
    return 0;
  // Sorted by x, then y (insertion sort: at most six points).
  for (int i = 1; i < n; i++)
    for (int j = i; j > 0 && (pts[j].x < pts[j - 1].x || (pts[j].x == pts[j - 1].x && pts[j].y < pts[j - 1].y)); j--)
    {
      Vector2 t = pts[j];
      pts[j] = pts[j - 1];
      pts[j - 1] = t;
    }
  // Andrew's monotone chain, collinear points dropped.
  Vector2 h[2 * LEG_COUNT];
  int k = 0;
  for (int i = 0; i < n; i++)
  {
    while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0)
      k--;
    h[k++] = pts[i];
  }
  for (int i = n - 2, lower = k + 1; i >= 0; i--)
  {
    while (k >= lower && cross(h[k - 2], h[k - 1], pts[i]) <= 0)
      k--;
    h[k++] = pts[i];
  }
  k--; // the last point repeats the first
  if (k < 3)
    return 0;
  for (int i = 0; i < k; i++)
    hull[i] = h[i];
  return k;
}

bool supportMargin(const Vector3 feet[LEG_COUNT], Vector3 point, float &margin)
{
  Vector2 hull[LEG_COUNT];
  int n = supportHull(feet, hull);
  if (n == 0)
    return false;
  float m = INFINITY;
  for (int i = 0; i < n; i++)
  {
    Vector2 a = hull[i], b = hull[(i + 1) % n];
    float ex = b.x - a.x, ey = b.y - a.y;
    m = fminf(m, (ex * (point.y - a.y) - ey * (point.x - a.x)) / sqrtf(ex * ex + ey * ey));
  }
  margin = m;
  return true;
}

} // namespace centre_of_gravity
