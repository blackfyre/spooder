/**
 * @brief Represents a 2D vector.
 */
class Vector2
{
public:
  float x; /**< The x-coordinate of the vector. */
  float y; /**< The y-coordinate of the vector. */

  /**
   * @brief Default constructor.
   * Initializes the vector with x = 0 and y = 0.
   */
  Vector2()
  {
    x = 0;
    y = 0;
  }

  /**
   * @brief Constructor with specified coordinates.
   * @param newX The x-coordinate of the vector.
   * @param newY The y-coordinate of the vector.
   */
  Vector2(float newX, float newY)
  {
    x = newX;
    y = newY;
  }

  /**
   * @brief Converts the vector to a string representation.
   * @return The string representation of the vector in the format "(x, y)".
   */
  String toString()
  {
    String xs = String(x);
    String ys = String(y);

    String ret = "(" + xs + "," + ys + ")";
    return ret;
  }

  /**
   * @brief Adds two vectors together.
   * @param val The vector to be added.
   * @return The resulting vector after addition.
   */
  Vector2 operator+(Vector2 val)
  {
    Vector2 result;
    result.x = x + val.x;
    result.y = y + val.y;
    return result;
  }

  /**
   * @brief Multiplies the vector by a scalar value.
   * @param val The scalar value to multiply the vector by.
   * @return The resulting vector after multiplication.
   */
  Vector2 operator*(float val)
  {
    Vector2 result;
    result.x = x * val;
    result.y = y * val;
    return result;
  }

  /**
   * @brief Multiplies the vector component-wise with another vector.
   * @param val The vector to multiply with.
   * @return The resulting vector after multiplication.
   */
  Vector2 operator*(Vector2 val)
  {
    Vector2 result;
    result.x = x * val.x;
    result.y = y * val.y;
    return result;
  }

  /**
   * @brief Rotates the vector around a pivot point.
   * @param angle The angle of rotation in degrees.
   * @param pivot The pivot point to rotate around.
   * @return The resulting vector after rotation.
   */
  Vector2 rotate(int angle, Vector2 pivot)
  {
    x -= pivot.x;
    y -= pivot.y;

    int x_rotated = x * cos(angle) - y * sin(angle);
    int y_rotated = x * sin(angle) + y * cos(angle);

    x = x_rotated + pivot.x;
    y = y_rotated + pivot.y;
    return Vector2(x, y);
  }
};

/**
 * @brief Represents a 3D vector with x, y, and z components.
 */
class Vector3
{
public:
  float x; /**< The x component of the vector. */
  float y; /**< The y component of the vector. */
  float z; /**< The z component of the vector. */

  /**
   * @brief Default constructor that initializes the vector to (0, 0, 0).
   */
  Vector3()
  {
    x = 0;
    y = 0;
    z = 0;
  }

  /**
   * @brief Constructor that initializes the vector with the given x, y, and z values.
   * @param newX The x component of the vector.
   * @param newY The y component of the vector.
   * @param newZ The z component of the vector.
   */
  Vector3(float newX, float newY, float newZ)
  {
    x = newX;
    y = newY;
    z = newZ;
  }

  /**
   * @brief Overloaded inequality operator.
   * @param val The vector to compare against.
   * @return True if the vectors are not equal, false otherwise.
   */
  bool operator!=(Vector3 val)
  {
    return (x != val.x || y != val.y || z != val.z);
  }

  /**
   * @brief Overloaded equality operator.
   * @param val The vector to compare against.
   * @return True if the vectors are equal, false otherwise.
   */
  bool operator==(Vector3 val)
  {
    return (x == val.x && y == val.y && z == val.z);
  }

  /**
   * @brief Overloaded multiplication operator.
   * @param val The scalar value to multiply the vector by.
   * @return The resulting vector after multiplication.
   */
  Vector3 operator*(float val)
  {
    return Vector3(x * val, y * val, z * val);
  }

  /**
   * @brief Overloaded multiplication operator.
   * @param val The vector to multiply with.
   * @return The resulting vector after element-wise multiplication.
   */
  Vector3 operator*(Vector3 val)
  {
    return Vector3(x * val.x, y * val.y, z * val.z);
  }

  /**
   * @brief Overloaded division operator.
   * @param val The vector to divide by.
   * @return The resulting vector after element-wise division.
   */
  Vector3 operator/(Vector3 val)
  {
    return Vector3(x / val.x, y / val.y, z / val.z);
  }

  /**
   * @brief Overloaded division operator.
   * @param val The scalar value to divide the vector by.
   * @return The resulting vector after division.
   */
  Vector3 operator/(float val)
  {
    return Vector3(x / val, y / val, z / val);
  }

  /**
   * @brief Overloaded addition operator.
   * @param val The vector to add.
   * @return The resulting vector after addition.
   */
  Vector3 operator+(Vector3 val)
  {
    return Vector3(x + val.x, y + val.y, z + val.z);
  }

  /**
   * @brief Converts the vector to a string representation.
   * @return The string representation of the vector.
   */
  String toString()
  {
    String xs = String(x);
    String ys = String(y);
    String zs = String(z);

    String ret = "(" + xs + "," + ys + "," + zs + ")";
    return ret;
  }

  /**
   * @brief Rotates the vector around a pivot point by a given angle.
   * @param angle The angle of rotation in degrees.
   * @param pivot The pivot point to rotate around.
   * @return The resulting rotated vector.
   */
  Vector3 rotate(int angle, Vector2 pivot)
  {
    // Translate line so pivot point is at the origin
    if (angle == 0)
      return Vector3(x, y, z);

    x -= pivot.x;
    y -= pivot.y;
    float angleRad = radians(angle);

    // Rotate point by angle
    int x_rotated = x * cos(angleRad) - y * sin(angleRad);
    int y_rotated = x * sin(angleRad) + y * cos(angleRad);

    // Translate point back to original position
    x = x_rotated + pivot.x;
    y = y_rotated + pivot.y;

    return Vector3(x, y, z);
  }

  /**
   * @brief Calculates the Euclidean distance between this vector and another vector.
   * @param v The vector to calculate the distance to.
   * @return The Euclidean distance between the two vectors.
   */
  float distanceTo(Vector3 v)
  {
    double dx = v.x - x;
    double dy = v.y - y;
    double dz = v.z - z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }
};
