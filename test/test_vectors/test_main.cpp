#include <unity.h>
#include "movement/vectors.h"

void setUp() {}
void tearDown() {}

void test_lerp_vector3_end_points_and_midpoint()
{
  Vector3 a(0, 10, -20);
  Vector3 b(100, -10, 20);
  TEST_ASSERT_TRUE(lerp(a, b, 0) == a);
  TEST_ASSERT_TRUE(lerp(a, b, 1) == b);
  Vector3 mid = lerp(a, b, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 50, mid.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0, mid.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0, mid.z);
}

void test_normalize_gives_unit_length()
{
  Vector2 v(30, 40);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 50, v.magnitude());
  v.normalize();
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, v.magnitude());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.6f, v.x);
}

void test_normalize_zero_vector_is_unchanged()
{
  Vector2 v;
  v.normalize();
  TEST_ASSERT_EQUAL_FLOAT(0, v.x);
  TEST_ASSERT_EQUAL_FLOAT(0, v.y);
}

void test_rotate_about_pivot()
{
  Vector3 p = Vector3(290, 0, 5).rotate(90, Vector2(190, 0));
  TEST_ASSERT_FLOAT_WITHIN(1, 190, p.x);
  TEST_ASSERT_FLOAT_WITHIN(1, 100, p.y);
  TEST_ASSERT_EQUAL_FLOAT(5, p.z);
}

void test_is_finite()
{
  TEST_ASSERT_TRUE(Vector3(1, 2, 3).isFinite());
  TEST_ASSERT_FALSE(Vector3(NAN, 2, 3).isFinite());
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_lerp_vector3_end_points_and_midpoint);
  RUN_TEST(test_normalize_gives_unit_length);
  RUN_TEST(test_normalize_zero_vector_is_unchanged);
  RUN_TEST(test_rotate_about_pivot);
  RUN_TEST(test_is_finite);
  return UNITY_END();
}
