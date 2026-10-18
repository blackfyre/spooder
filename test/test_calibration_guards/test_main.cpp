#include <unity.h>
#include <math.h>
#include "protocol/link_receiver.h"

using namespace link_receiver;
using joint_calibration::DEFAULTS;
using protocol::CalibrationResult;

static Receiver r;
static joint_calibration::Table requested;

void setUp()
{
  r = Receiver();
  joint_calibration::loadDefaults(requested);
}
void tearDown() {}

static protocol::CalibrationSet set(uint8_t id, uint8_t joint, int16_t offsetCentideg, uint16_t p0 = 500,
                                    uint16_t p180 = 2500, bool reversed = false)
{
  protocol::CalibrationSet c = {};
  c.requestId = id;
  c.joint = joint;
  c.offsetCentideg = offsetCentideg;
  c.pulse0Us = p0;
  c.pulse180Us = p180;
  c.flags = reversed ? protocol::CALIBRATION_REVERSED : 0;
  return c;
}

static int16_t defaultCentideg(int joint)
{
  return (int16_t)lroundf(DEFAULTS[joint].offsetDeg * 100);
}

// Queues one set and processes it in Calibrate with output allowed (unless
// told otherwise); returns the link's result.
static CalibrationResult send(const protocol::CalibrationSet &c, bool calibrating = true, bool allowOutput = true)
{
  acceptCalibration(r, c);
  TEST_ASSERT_EQUAL_INT(1, processCalibration(r, requested, calibrating, allowOutput));
  return r.lastResult;
}

static bool tableIsDefault()
{
  for (int i = 0; i < joint_calibration::JOINT_COUNT; i++)
  {
    const joint_calibration::Joint &j = requested.joints[i];
    if (j.offsetDeg != DEFAULTS[i].offsetDeg || j.pulse0Us != DEFAULTS[i].pulse0Us ||
        j.pulse180Us != DEFAULTS[i].pulse180Us || j.reversed != DEFAULTS[i].reversed)
      return false;
  }
  return true;
}

// --- guards ---------------------------------------------------------------------

void test_change_outside_calibrate_is_rejected()
{
  // Spec scenario: a change while standing is rejected, the table unchanged.
  TEST_ASSERT_TRUE(send(set(1, 4, 1000), false, true) == CalibrationResult::Rejected);
  TEST_ASSERT_TRUE(tableIsDefault());
}

void test_change_without_output_is_rejected()
{
  TEST_ASSERT_TRUE(send(set(1, 4, 1000), true, false) == CalibrationResult::Rejected);
  TEST_ASSERT_TRUE(tableIsDefault());
}

void test_unknown_joint_is_rejected()
{
  TEST_ASSERT_TRUE(send(set(1, 18, 0)) == CalibrationResult::Rejected);
  TEST_ASSERT_TRUE(send(set(2, 255, 0)) == CalibrationResult::Rejected);
  TEST_ASSERT_TRUE(tableIsDefault());
}

void test_change_in_range_is_accepted()
{
  int16_t target = (int16_t)(defaultCentideg(7) + 1250);
  TEST_ASSERT_TRUE(send(set(1, 7, target, 600, 2400, true)) == CalibrationResult::Accepted);
  const joint_calibration::Joint &j = requested.joints[7];
  TEST_ASSERT_FLOAT_WITHIN(1e-3, target / 100.0f, j.offsetDeg);
  TEST_ASSERT_EQUAL_FLOAT(600, j.pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(2400, j.pulse180Us);
  TEST_ASSERT_TRUE(j.reversed);
  // Only that joint changed.
  requested.joints[7] = DEFAULTS[7];
  TEST_ASSERT_TRUE(tableIsDefault());
}

void test_offset_clamped_and_reported()
{
  // Spec scenarios: 45° above the default becomes default + 30°, and the
  // report shows the clamped value.
  TEST_ASSERT_TRUE(send(set(9, 1, (int16_t)(defaultCentideg(1) + 4500))) == CalibrationResult::Clamped);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULTS[1].offsetDeg + 30, requested.joints[1].offsetDeg);
  protocol::CalibrationReport rep = makeReport(r, requested);
  TEST_ASSERT_EQUAL_UINT8(9, rep.lastRequestId);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)CalibrationResult::Clamped, rep.result);
  TEST_ASSERT_EQUAL_INT16(defaultCentideg(1) + 3000, rep.joints[1].offsetCentideg);
}

void test_pulses_clamped_with_minimum_span()
{
  TEST_ASSERT_TRUE(send(set(1, 0, defaultCentideg(0), 100, 3000)) == CalibrationResult::Clamped);
  TEST_ASSERT_EQUAL_FLOAT(400, requested.joints[0].pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(2600, requested.joints[0].pulse180Us);
  TEST_ASSERT_TRUE(send(set(2, 0, defaultCentideg(0), 1500, 1900)) == CalibrationResult::Clamped);
  TEST_ASSERT_EQUAL_FLOAT(1500, requested.joints[0].pulse0Us);
  TEST_ASSERT_EQUAL_FLOAT(2500, requested.joints[0].pulse180Us);
  TEST_ASSERT_TRUE(send(set(3, 0, defaultCentideg(0), 2000, 900)) == CalibrationResult::Accepted);
  TEST_ASSERT_EQUAL_FLOAT(900, requested.joints[0].pulse180Us);
}

void test_repeated_request_id_is_a_retransmission()
{
  TEST_ASSERT_TRUE(send(set(5, 2, (int16_t)(defaultCentideg(2) + 4500))) == CalibrationResult::Clamped);
  float clamped = requested.joints[2].offsetDeg;
  // Same id, different values: reported again, nothing changes.
  TEST_ASSERT_TRUE(send(set(5, 2, defaultCentideg(2))) == CalibrationResult::Clamped);
  TEST_ASSERT_EQUAL_FLOAT(clamped, requested.joints[2].offsetDeg);
  TEST_ASSERT_EQUAL_UINT8(5, makeReport(r, requested).lastRequestId);
  // A new id is processed.
  TEST_ASSERT_TRUE(send(set(6, 2, defaultCentideg(2))) == CalibrationResult::Accepted);
  TEST_ASSERT_EQUAL_FLOAT(DEFAULTS[2].offsetDeg, requested.joints[2].offsetDeg);
}

void test_retransmission_of_a_rejection_stays_rejected()
{
  TEST_ASSERT_TRUE(send(set(7, 3, 0), false, true) == CalibrationResult::Rejected);
  TEST_ASSERT_TRUE(send(set(7, 3, 0)) == CalibrationResult::Rejected);
  TEST_ASSERT_TRUE(tableIsDefault());
}

void test_first_request_id_zero_is_processed()
{
  TEST_ASSERT_TRUE(send(set(0, 3, (int16_t)(defaultCentideg(3) + 100))) == CalibrationResult::Accepted);
  TEST_ASSERT_FALSE(tableIsDefault());
}

void test_request_ids_are_per_link()
{
  Receiver other;
  TEST_ASSERT_TRUE(send(set(4, 5, (int16_t)(defaultCentideg(5) + 100))) == CalibrationResult::Accepted);
  acceptCalibration(other, set(4, 5, (int16_t)(defaultCentideg(5) + 200)));
  TEST_ASSERT_EQUAL_INT(1, processCalibration(other, requested, true, true));
  TEST_ASSERT_TRUE(other.lastResult == CalibrationResult::Accepted);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, DEFAULTS[5].offsetDeg + 2, requested.joints[5].offsetDeg);
}

void test_sets_in_one_tick_are_processed_in_order()
{
  acceptCalibration(r, set(1, 6, (int16_t)(defaultCentideg(6) + 100)));
  acceptCalibration(r, set(2, 6, (int16_t)(defaultCentideg(6) + 200)));
  acceptCalibration(r, set(2, 6, (int16_t)(defaultCentideg(6) + 900))); // retransmission
  TEST_ASSERT_EQUAL_INT(3, processCalibration(r, requested, true, true));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, DEFAULTS[6].offsetDeg + 2, requested.joints[6].offsetDeg);
  TEST_ASSERT_EQUAL_INT(0, processCalibration(r, requested, true, true)); // queue emptied
}

void test_queue_overflow_drops_extra_sets()
{
  for (int i = 0; i < MAX_PENDING_SETS + 2; i++)
    acceptCalibration(r, set((uint8_t)(10 + i), 0, defaultCentideg(0)));
  TEST_ASSERT_EQUAL_INT(MAX_PENDING_SETS, processCalibration(r, requested, true, true));
  TEST_ASSERT_EQUAL_UINT8(10 + MAX_PENDING_SETS - 1, r.lastRequestId);
}

// --- reports --------------------------------------------------------------------

void test_report_before_any_set()
{
  protocol::CalibrationReport rep = makeReport(r, requested);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)CalibrationResult::NoneYet, rep.result);
  TEST_ASSERT_EQUAL_UINT8(0, rep.lastRequestId);
  for (int i = 0; i < joint_calibration::JOINT_COUNT; i++)
  {
    TEST_ASSERT_EQUAL_INT16(defaultCentideg(i), rep.joints[i].offsetCentideg);
    TEST_ASSERT_EQUAL_UINT16(lroundf(DEFAULTS[i].pulse0Us), rep.joints[i].pulse0Us);
    TEST_ASSERT_EQUAL_UINT16(lroundf(DEFAULTS[i].pulse180Us), rep.joints[i].pulse180Us);
    TEST_ASSERT_EQUAL_UINT8(DEFAULTS[i].reversed ? protocol::CALIBRATION_REVERSED : 0, rep.joints[i].flags);
  }
}

void test_report_rounds_values()
{
  joint_calibration::Table t = requested;
  t.joints[0].offsetDeg = -12.345f;
  t.joints[0].pulse0Us = 612.6f;
  t.joints[0].pulse180Us = 2399.4f;
  t.joints[0].reversed = true;
  protocol::CalibrationReport rep = makeReport(r, t);
  TEST_ASSERT_EQUAL_INT16(-1235, rep.joints[0].offsetCentideg);
  TEST_ASSERT_EQUAL_UINT16(613, rep.joints[0].pulse0Us);
  TEST_ASSERT_EQUAL_UINT16(2399, rep.joints[0].pulse180Us);
  TEST_ASSERT_EQUAL_UINT8(protocol::CALIBRATION_REVERSED, rep.joints[0].flags);
}

void test_report_result_codes()
{
  send(set(1, 0, defaultCentideg(0)));
  TEST_ASSERT_EQUAL_UINT8(0, makeReport(r, requested).result);
  send(set(2, 0, (int16_t)(defaultCentideg(0) + 9000)));
  TEST_ASSERT_EQUAL_UINT8(1, makeReport(r, requested).result);
  send(set(3, 0, defaultCentideg(0)), false, true);
  TEST_ASSERT_EQUAL_UINT8(2, makeReport(r, requested).result);
  TEST_ASSERT_EQUAL_UINT8(3, (uint8_t)CalibrationResult::NoneYet);
}

void test_report_after_a_set_is_due_at_once()
{
  ReportSchedule s;
  TEST_ASSERT_FALSE(reportDue(s, false, 1000));
  requestReport(s);
  TEST_ASSERT_TRUE(reportDue(s, false, 1000));
  reportSent(s, 1000);
  TEST_ASSERT_FALSE(reportDue(s, false, 1020));
}

void test_reports_capped_at_five_per_second()
{
  ReportSchedule s;
  requestReport(s);
  TEST_ASSERT_TRUE(reportDue(s, false, 1000));
  reportSent(s, 1000);
  requestReport(s);
  TEST_ASSERT_FALSE(reportDue(s, false, 1100));
  TEST_ASSERT_FALSE(reportDue(s, false, 1199));
  TEST_ASSERT_TRUE(reportDue(s, false, 1200)); // held, then sent
  reportSent(s, 1200);
  TEST_ASSERT_FALSE(reportDue(s, false, 1300));
}

void test_reports_at_one_hertz_in_calibrate()
{
  ReportSchedule s;
  TEST_ASSERT_TRUE(reportDue(s, true, 5000)); // on entering Calibrate
  reportSent(s, 5000);
  TEST_ASSERT_FALSE(reportDue(s, true, 5980));
  TEST_ASSERT_TRUE(reportDue(s, true, 6000));
  reportSent(s, 6000);
  // A report that could not be queued stays due.
  TEST_ASSERT_TRUE(reportDue(s, true, 7000));
  TEST_ASSERT_TRUE(reportDue(s, true, 7020));
  // Outside Calibrate the periodic report stops.
  reportSent(s, 7020);
  TEST_ASSERT_FALSE(reportDue(s, false, 9000));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_change_outside_calibrate_is_rejected);
  RUN_TEST(test_change_without_output_is_rejected);
  RUN_TEST(test_unknown_joint_is_rejected);
  RUN_TEST(test_change_in_range_is_accepted);
  RUN_TEST(test_offset_clamped_and_reported);
  RUN_TEST(test_pulses_clamped_with_minimum_span);
  RUN_TEST(test_repeated_request_id_is_a_retransmission);
  RUN_TEST(test_retransmission_of_a_rejection_stays_rejected);
  RUN_TEST(test_first_request_id_zero_is_processed);
  RUN_TEST(test_request_ids_are_per_link);
  RUN_TEST(test_sets_in_one_tick_are_processed_in_order);
  RUN_TEST(test_queue_overflow_drops_extra_sets);
  RUN_TEST(test_report_before_any_set);
  RUN_TEST(test_report_rounds_values);
  RUN_TEST(test_report_result_codes);
  RUN_TEST(test_report_after_a_set_is_due_at_once);
  RUN_TEST(test_reports_capped_at_five_per_second);
  RUN_TEST(test_reports_at_one_hertz_in_calibrate);
  return UNITY_END();
}
