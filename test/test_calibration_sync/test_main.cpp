#include <unity.h>
#include "setup/calibration_sync.h"

using namespace calibration_sync;

void setUp() {}
void tearDown() {}

// A report of a plain table: offset 90.00°, 500/2500 µs, normal direction.
static protocol::CalibrationReport baseReport(uint8_t lastId = 7,
                                              protocol::CalibrationResult result = protocol::CalibrationResult::Accepted)
{
  protocol::CalibrationReport r = {};
  r.lastRequestId = lastId;
  r.result = (uint8_t)result;
  for (int i = 0; i < JOINT_COUNT; i++)
    r.joints[i] = {9000, 500, 2500, 0};
  return r;
}

// The robot's answer to a set: its values taken as they are.
static protocol::CalibrationReport confirm(const protocol::CalibrationReport &before, const protocol::CalibrationSet &s,
                                           protocol::CalibrationResult result = protocol::CalibrationResult::Accepted)
{
  protocol::CalibrationReport r = before;
  r.lastRequestId = s.requestId;
  r.result = (uint8_t)result;
  if (result != protocol::CalibrationResult::Rejected)
    r.joints[s.joint] = {s.offsetCentideg, s.pulse0Us, s.pulse180Us, s.flags};
  return r;
}

void test_no_edits_before_a_report()
{
  Sync s;
  protocol::CalibrationJoint j;
  TEST_ASSERT_FALSE(wanted(s, 0, j));
  TEST_ASSERT_FALSE(change(s, 0, Field::Offset, 100, true));
  protocol::CalibrationSet out;
  TEST_ASSERT_FALSE(next(s, 0, out));
}

void test_bad_joint_is_refused()
{
  Sync s;
  onReport(s, baseReport());
  TEST_ASSERT_FALSE(change(s, -1, Field::Offset, 1, true));
  TEST_ASSERT_FALSE(change(s, JOINT_COUNT, Field::Offset, 1, true));
  TEST_ASSERT_FALSE(pending(s, JOINT_COUNT));
}

void test_one_set_per_call_and_only_one_in_flight()
{
  Sync s;
  onReport(s, baseReport());
  change(s, 3, Field::Offset, 100, true);
  change(s, 4, Field::Offset, 100, true);
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  TEST_ASSERT_EQUAL_UINT8(3, out.joint);
  // Nothing more until it is confirmed or the resend time comes.
  TEST_ASSERT_FALSE(next(s, 20, out));
  TEST_ASSERT_FALSE(next(s, 40, out));
}

void test_oldest_pending_first_and_latest_value_only()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport();
  onReport(s, rep);
  change(s, 10, Field::Pulse0, 600, false);
  change(s, 2, Field::Offset, 100, true);
  change(s, 10, Field::Pulse0, 700, false); // replaces 600, keeps its age
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  TEST_ASSERT_EQUAL_UINT8(10, out.joint);
  TEST_ASSERT_EQUAL_UINT16(700, out.pulse0Us);
  rep = confirm(rep, out);
  onReport(s, rep);
  TEST_ASSERT_TRUE(next(s, 20, out));
  TEST_ASSERT_EQUAL_UINT8(2, out.joint);
  TEST_ASSERT_EQUAL_INT16(9100, out.offsetCentideg);
  onReport(s, confirm(rep, out));
  TEST_ASSERT_FALSE(next(s, 40, out));
}

void test_confirmation_by_request_id()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport();
  onReport(s, rep);
  change(s, 5, Field::Offset, 50, true);
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  TEST_ASSERT_TRUE(pending(s, 5));
  // A periodic report with an older id confirms nothing.
  onReport(s, rep);
  TEST_ASSERT_TRUE(pending(s, 5));
  TEST_ASSERT_EQUAL_INT16(9000, s.report.joints[5].offsetCentideg);
  onReport(s, confirm(rep, out));
  TEST_ASSERT_FALSE(pending(s, 5));
  protocol::CalibrationJoint j;
  TEST_ASSERT_TRUE(wanted(s, 5, j));
  TEST_ASSERT_EQUAL_INT16(9050, j.offsetCentideg);
}

void test_a_none_yet_report_never_confirms()
{
  Sync s;
  onReport(s, baseReport(0, protocol::CalibrationResult::NoneYet));
  change(s, 0, Field::Offset, 50, true);
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  protocol::CalibrationReport r = baseReport(out.requestId, protocol::CalibrationResult::NoneYet);
  onReport(s, r);
  TEST_ASSERT_TRUE(pending(s, 0));
}

void test_resend_after_200_ms_with_the_same_id()
{
  Sync s;
  onReport(s, baseReport());
  change(s, 1, Field::Pulse180, -50, true);
  protocol::CalibrationSet first, again;
  TEST_ASSERT_TRUE(next(s, 1000, first));
  TEST_ASSERT_FALSE(next(s, 1000 + RESEND_MS - 1, again));
  TEST_ASSERT_TRUE(next(s, 1000 + RESEND_MS, again));
  TEST_ASSERT_EQUAL_UINT8(first.requestId, again.requestId);
  TEST_ASSERT_EQUAL_UINT8(first.joint, again.joint);
  TEST_ASSERT_EQUAL_UINT16(first.pulse180Us, again.pulse180Us);
  // The resend restarts the wait.
  TEST_ASSERT_FALSE(next(s, 1000 + RESEND_MS + 20, again));
  TEST_ASSERT_TRUE(next(s, 1000 + 2 * RESEND_MS, again));
  TEST_ASSERT_EQUAL_UINT8(first.requestId, again.requestId);
}

void test_resend_keeps_the_sent_value_after_a_new_edit()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport();
  onReport(s, rep);
  change(s, 1, Field::Offset, 100, true);
  protocol::CalibrationSet first, again;
  TEST_ASSERT_TRUE(next(s, 0, first));
  change(s, 1, Field::Offset, 100, true); // edited in flight: 92.00 wanted
  TEST_ASSERT_TRUE(next(s, RESEND_MS, again));
  TEST_ASSERT_EQUAL_UINT8(first.requestId, again.requestId);
  TEST_ASSERT_EQUAL_INT16(9100, again.offsetCentideg);
  // Confirming the first value keeps the newer edit pending, then sends it.
  rep = confirm(rep, again);
  onReport(s, rep);
  TEST_ASSERT_TRUE(pending(s, 1));
  protocol::CalibrationSet newer;
  TEST_ASSERT_TRUE(next(s, RESEND_MS + 20, newer));
  TEST_ASSERT_NOT_EQUAL(first.requestId, newer.requestId);
  TEST_ASSERT_EQUAL_INT16(9200, newer.offsetCentideg);
  onReport(s, confirm(rep, newer));
  TEST_ASSERT_FALSE(pending(s, 1));
}

void test_new_ids_differ_from_the_last_reported_id()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport(41);
  onReport(s, rep);
  change(s, 0, Field::Offset, 1, true);
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  TEST_ASSERT_EQUAL_UINT8(42, out.requestId); // continues after the robot's id
  rep = confirm(rep, out);
  onReport(s, rep);
  // The robot reports some other id meanwhile (another front-end restarted
  // ours, say): the next id must still differ from it.
  rep.lastRequestId = 43;
  onReport(s, rep);
  change(s, 0, Field::Offset, 1, true);
  TEST_ASSERT_TRUE(next(s, 20, out));
  TEST_ASSERT_NOT_EQUAL(43, out.requestId);
}

void test_ids_wrap()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport(254);
  onReport(s, rep);
  uint8_t ids[3];
  for (int i = 0; i < 3; i++)
  {
    change(s, 0, Field::Offset, 1, true);
    protocol::CalibrationSet out;
    TEST_ASSERT_TRUE(next(s, 20u * i, out));
    ids[i] = out.requestId;
    rep = confirm(rep, out);
    onReport(s, rep);
  }
  TEST_ASSERT_EQUAL_UINT8(255, ids[0]);
  TEST_ASSERT_EQUAL_UINT8(0, ids[1]);
  TEST_ASSERT_EQUAL_UINT8(1, ids[2]);
}

void test_clamped_result_replaces_the_shown_value()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport();
  onReport(s, rep);
  change(s, 7, Field::Offset, 4000, true); // +40°: the robot allows ±30°
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  TEST_ASSERT_EQUAL_INT16(13000, out.offsetCentideg);
  rep.lastRequestId = out.requestId;
  rep.result = (uint8_t)protocol::CalibrationResult::Clamped;
  rep.joints[7].offsetCentideg = 12000;
  onReport(s, rep);
  TEST_ASSERT_FALSE(pending(s, 7));
  protocol::CalibrationJoint j;
  wanted(s, 7, j);
  TEST_ASSERT_EQUAL_INT16(12000, j.offsetCentideg);
}

void test_rejected_result_shows_the_unchanged_value()
{
  Sync s;
  protocol::CalibrationReport rep = baseReport();
  onReport(s, rep);
  change(s, 7, Field::Pulse0, 600, false);
  protocol::CalibrationSet out;
  TEST_ASSERT_TRUE(next(s, 0, out));
  onReport(s, confirm(rep, out, protocol::CalibrationResult::Rejected));
  TEST_ASSERT_FALSE(pending(s, 7));
  protocol::CalibrationJoint j;
  wanted(s, 7, j);
  TEST_ASSERT_EQUAL_UINT16(500, j.pulse0Us);
}

void test_edits_are_relative_to_the_wanted_value()
{
  Sync s;
  onReport(s, baseReport());
  change(s, 0, Field::Offset, 100, true);
  change(s, 0, Field::Offset, 50, true);
  change(s, 0, Field::Pulse0, -10, true);
  protocol::CalibrationJoint j;
  wanted(s, 0, j);
  TEST_ASSERT_EQUAL_INT16(9150, j.offsetCentideg);
  TEST_ASSERT_EQUAL_UINT16(490, j.pulse0Us);
  TEST_ASSERT_EQUAL_UINT16(2500, j.pulse180Us);
  // The shown value is still the reported one.
  TEST_ASSERT_EQUAL_INT16(9000, s.report.joints[0].offsetCentideg);
}

void test_fields_saturate_to_their_wire_range()
{
  Sync s;
  onReport(s, baseReport());
  change(s, 0, Field::Offset, 2000000000, true);
  change(s, 0, Field::Pulse0, -100000, true);
  change(s, 0, Field::Pulse180, 100000, false);
  protocol::CalibrationJoint j;
  wanted(s, 0, j);
  TEST_ASSERT_EQUAL_INT16(32767, j.offsetCentideg);
  TEST_ASSERT_EQUAL_UINT16(0, j.pulse0Us);
  TEST_ASSERT_EQUAL_UINT16(65535, j.pulse180Us);
  change(s, 0, Field::Offset, -2000000000, true);
  wanted(s, 0, j);
  TEST_ASSERT_EQUAL_INT16(-32768, j.offsetCentideg);
}

void test_direction_set_and_toggle()
{
  Sync s;
  onReport(s, baseReport());
  protocol::CalibrationJoint j;
  change(s, 2, Field::Reversed, 1, false);
  wanted(s, 2, j);
  TEST_ASSERT_EQUAL_UINT8(protocol::CALIBRATION_REVERSED, j.flags);
  change(s, 2, Field::Reversed, 1, true);
  wanted(s, 2, j);
  TEST_ASSERT_EQUAL_UINT8(0, j.flags);
  change(s, 2, Field::Reversed, 0, true); // no change
  wanted(s, 2, j);
  TEST_ASSERT_EQUAL_UINT8(0, j.flags);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_no_edits_before_a_report);
  RUN_TEST(test_bad_joint_is_refused);
  RUN_TEST(test_one_set_per_call_and_only_one_in_flight);
  RUN_TEST(test_oldest_pending_first_and_latest_value_only);
  RUN_TEST(test_confirmation_by_request_id);
  RUN_TEST(test_a_none_yet_report_never_confirms);
  RUN_TEST(test_resend_after_200_ms_with_the_same_id);
  RUN_TEST(test_resend_keeps_the_sent_value_after_a_new_edit);
  RUN_TEST(test_new_ids_differ_from_the_last_reported_id);
  RUN_TEST(test_ids_wrap);
  RUN_TEST(test_clamped_result_replaces_the_shown_value);
  RUN_TEST(test_rejected_result_shows_the_unchanged_value);
  RUN_TEST(test_edits_are_relative_to_the_wanted_value);
  RUN_TEST(test_fields_saturate_to_their_wire_range);
  RUN_TEST(test_direction_set_and_toggle);
  return UNITY_END();
}
