#ifndef VECTORS_H
#define VECTORS_H

// Hardware-free: compiled for the board and for host tests. Do not include
// Arduino headers here; string formatting lives in reporting/.
#include <math.h>

/**
 * @brief Represents a 2D vector.
 */
class Vector2
{
public:
  float x; /**< The x-coordinate of the vector. */
  float y; /**< The y-coordinate of the vector. */

  Vector2() : x(0), y(0) {}

  Vector2(float newX, float newY) : x(newX), y(newY) {}

  float magnitude() const
  {
    return sqrtf(x * x + y * y);
  }

  /**
   * @brief Scales the vector to unit length. A zero vector is left unchanged.
   */
  void normalize()
  {
    float mag = magnitude();
    if (mag > 0)
    {
      x /= mag;
      y /= mag;
    }
  }

  Vector2 operator+(Vector2 val) const
  {
    return Vector2(x + val.x, y + val.y);
  }

  Vector2 operator*(float val) const
  {
    return Vector2(x * val, y * val);
  }

  Vector2 operator*(Vector2 val) const
  {
    return Vector2(x * val.x, y * val.y);
  }
};

/**
 * @brief Represents a 3D vector.
 */
class Vector3
{
public:
  float x; /**< The x-coordinate of the vector. */
  float y; /**< The y-coordinate of the vector. */
  float z; /**< The z-coordinate of the vector. */

  Vector3() : x(0), y(0), z(0) {}

  Vector3(float newX, float newY, float newZ) : x(newX), y(newY), z(newZ) {}

  bool operator!=(Vector3 val) const
  {
    return (x != val.x || y != val.y || z != val.z);
  }

  bool operator==(Vector3 val) const
  {
    return (x == val.x && y == val.y && z == val.z);
  }

  Vector3 operator*(float val) const
  {
    return Vector3(x * val, y * val, z * val);
  }

  Vector3 operator*(Vector3 val) const
  {
    return Vector3(x * val.x, y * val.y, z * val.z);
  }

  Vector3 operator/(Vector3 val) const
  {
    return Vector3(x / val.x, y / val.y, z / val.z);
  }

  Vector3 operator/(float val) const
  {
    return Vector3(x / val, y / val, z / val);
  }

  Vector3 operator+(Vector3 val) const
  {
    return Vector3(x + val.x, y + val.y, z + val.z);
  }

  Vector3 operator-(Vector3 val) const
  {
    return Vector3(x - val.x, y - val.y, z - val.z);
  }

  /**
   * @brief Rotates the point around a pivot in the XY plane.
   *
   * The rotated x and y are truncated to whole millimetres, as in the
   * original gait code.
   *
   * @param angle Rotation in degrees.
   * @param pivot The pivot point.
   */
  Vector3 rotate(int angle, Vector2 pivot) const
  {
    if (angle == 0)
      return *this;

    float px = x - pivot.x;
    float py = y - pivot.y;
    float angleRad = angle * (float)M_PI / 180.0f;

    int x_rotated = px * cosf(angleRad) - py * sinf(angleRad);
    int y_rotated = px * sinf(angleRad) + py * cosf(angleRad);

    return Vector3(x_rotated + pivot.x, y_rotated + pivot.y, z);
  }

  float distanceTo(Vector3 v) const
  {
    float dx = v.x - x;
    float dy = v.y - y;
    float dz = v.z - z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
  }

  float magnitude() const
  {
    return Vector3().distanceTo(*this);
  }

  bool isFinite() const
  {
    return isfinite(x) && isfinite(y) && isfinite(z);
  }
};

/**
 * @brief Linearly interpolates between two values (f from 0.0 to 1.0).
 */
inline float interpolate(float a, float b, float f)
{
  return a * (1.0f - f) + (b * f);
}

inline Vector2 interpolate(Vector2 a, Vector2 b, float f)
{
  return Vector2(interpolate(a.x, b.x, f), interpolate(a.y, b.y, f));
}

inline Vector3 interpolate(Vector3 a, Vector3 b, float f)
{
  return Vector3(interpolate(a.x, b.x, f), interpolate(a.y, b.y, f), interpolate(a.z, b.z, f));
}

/**
 * @brief Maps a value from one range to another range.
 */
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
