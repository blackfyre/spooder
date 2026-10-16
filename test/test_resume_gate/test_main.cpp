#include <unity.h>
#include "protection/resume_gate.h"

using namespace protection;

static ResumeGate g;

void setUp() { g = ResumeGate(); }
void tearDown() {}

void test_no_fault_passes_request_through()
{
  TEST_ASSERT_TRUE(resumeGate(g, false, false, RequestedMode::Walk, 3) == RequestedMode::Walk);
}

void test_fault_forces_sleep_while_latched()
{
  TEST_ASSERT_TRUE(resumeGate(g, true, true, RequestedMode::Walk, 3) == RequestedMode::Sleep);
  TEST_ASSERT_TRUE(resumeGate(g, false, true, RequestedMode::Walk, 3) == RequestedMode::Sleep);
}

void test_clear_alone_does_not_resume_repeated_request()
{
  // Walking, a fault latches and is cleared; the input keeps repeating Walk.
  resumeGate(g, true, true, RequestedMode::Walk, 3);
  for (int i = 0; i < 10; i++)
    TEST_ASSERT_TRUE(resumeGate(g, false, false, RequestedMode::Walk, 3) == RequestedMode::Sleep);
}

void test_new_request_after_clear_resumes()
{
  resumeGate(g, true, true, RequestedMode::Walk, 3);
  resumeGate(g, false, false, RequestedMode::Walk, 3);
  TEST_ASSERT_TRUE(resumeGate(g, false, false, RequestedMode::Walk, 4) == RequestedMode::Walk);
  // Stays resumed afterwards.
  TEST_ASSERT_TRUE(resumeGate(g, false, false, RequestedMode::Walk, 4) == RequestedMode::Walk);
}

void test_request_made_while_fault_active_still_counts()
{
  // The operator presses Stand while the fault is still latched, then clears.
  resumeGate(g, true, true, RequestedMode::Walk, 3);
  TEST_ASSERT_TRUE(resumeGate(g, false, true, RequestedMode::Stand, 4) == RequestedMode::Sleep);
  TEST_ASSERT_TRUE(resumeGate(g, false, false, RequestedMode::Stand, 4) == RequestedMode::Stand);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_no_fault_passes_request_through);
  RUN_TEST(test_fault_forces_sleep_while_latched);
  RUN_TEST(test_clear_alone_does_not_resume_repeated_request);
  RUN_TEST(test_new_request_after_clear_resumes);
  RUN_TEST(test_request_made_while_fault_active_still_counts);
  return UNITY_END();
}
