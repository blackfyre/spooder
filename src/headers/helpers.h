/**
 * Linearly interpolates between two values.
 *
 * @param a The starting value.
 * @param b The ending value.
 * @param f The interpolation factor (0.0 to 1.0).
 * @return The interpolated value between a and b.
 */
float lerp(float a, float b, float f)
{
    return a * (1.0 - f) + (b * f);
}

/**
 * Linearly interpolates between two vectors.
 *
 * @param a The starting vector.
 * @param b The ending vector.
 * @param f The interpolation factor (0.0 to 1.0).
 * @return The interpolated vector.
 */
Vector2 lerp(Vector2 a, Vector2 b, float f)
{
    return Vector2(lerp(a.x, b.x, f), lerp(a.y, b.y, f));
}

/**
 * Calculates the hypotenuse of a right triangle using the Pythagorean theorem.
 *
 * @param x The length of one side of the triangle.
 * @param y The length of the other side of the triangle.
 * @return The length of the hypotenuse.
 */
float calculateHypotenuse(float x, float y)
{
    float result = sqrt(pow(x, 2) + pow(y, 2));
    return result;
}

/**
 * Maps a value from one range to another range.
 *
 * @param x The value to be mapped.
 * @param in_min The minimum value of the input range.
 * @param in_max The maximum value of the input range.
 * @param out_min The minimum value of the output range.
 * @param out_max The maximum value of the output range.
 * @return The mapped value.
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Converts an angle to a pulse value.
 *
 * @param ang The angle to be converted.
 * @return The pulse value corresponding to the given angle.
 */
int angleToPulse(int ang)
{
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print(" pulse: ");
    Serial.println(pulse);
    return pulse;
}