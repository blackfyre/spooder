#include <unity.h>
#include "movement/servo_profile.h"

using namespace servo_profile;

void setUp() {}
void tearDown() {}

void test_mg996r_budget()
{
  int i = indexOf("MG996R");
  TEST_ASSERT_TRUE(i >= 0);
  // 0.14 s/60° is about 429°/s; 60 % of it under load.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 257.1f, PROFILES[i].budgetDegPerS());
}

void test_default_profile_is_mg996r()
{
  TEST_ASSERT_EQUAL_STRING("MG996R", SELECTED.name);
}

void test_unknown_name_is_not_found()
{
  TEST_ASSERT_EQUAL_INT(-1, indexOf("NOPE"));
  TEST_ASSERT_EQUAL_INT(-1, indexOf("MG996"));
  TEST_ASSERT_EQUAL_INT(-1, indexOf("MG996RX"));
  TEST_ASSERT_EQUAL_INT(-1, indexOf(""));
}

void test_every_profile_has_a_positive_budget()
{
  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    TEST_ASSERT_TRUE(PROFILES[i].budgetDegPerS() > 0);
    TEST_ASSERT_TRUE(PROFILES[i].loadShare > 0 && PROFILES[i].loadShare <= 1);
    TEST_ASSERT_EQUAL_INT(i, indexOf(PROFILES[i].name)); // names are unique
  }
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_mg996r_budget);
  RUN_TEST(test_default_profile_is_mg996r);
  RUN_TEST(test_unknown_name_is_not_found);
  RUN_TEST(test_every_profile_has_a_positive_budget);
  return UNITY_END();
}
