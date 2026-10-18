package calib

import (
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

var t0 = time.Date(2026, 10, 18, 12, 0, 0, 0, time.UTC)

// defaults is a report of the compiled defaults' shape: 500/2500 µs, normal.
func defaults(lastID uint8, result protocol.CalibrationResult) protocol.CalibrationReport {
	r := protocol.CalibrationReport{LastRequestID: lastID, Result: uint8(result)}
	for i := range r.Joints {
		r.Joints[i] = protocol.CalibrationJoint{OffsetCentideg: 9000, Pulse0Us: 500, Pulse180Us: 2500}
	}
	return r
}

// reportFor is a report confirming set with the table holding value.
func reportFor(set protocol.CalibrationSet, result protocol.CalibrationResult, value protocol.CalibrationJoint) protocol.CalibrationReport {
	r := defaults(set.RequestID, result)
	r.Joints[set.Joint] = value
	return r
}

func wire(set protocol.CalibrationSet) protocol.CalibrationJoint {
	return protocol.CalibrationJoint{OffsetCentideg: set.OffsetCentideg, Pulse0Us: set.Pulse0Us, Pulse180Us: set.Pulse180Us, Flags: set.Flags}
}

const leg3Tibia = 8 // joint index: leg 3 (index 2) * 3 + tibia

func TestJointNames(t *testing.T) {
	if Leg(leg3Tibia) != 3 || Name(leg3Tibia) != "tibia" || Leg(0) != 1 || Name(0) != "coxa" || Leg(17) != 6 {
		t.Fatal("joint naming")
	}
}

// Spec: nudge leg 3's tibia 0° pulse by +10 µs in Calibrate; it is pending
// until the robot's report confirms it, and then shows the reported value.
func TestNudgeIsPendingUntilConfirmed(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(leg3Tibia, FieldPulse0, 1, false)
	if v, ok := m.Pending(leg3Tibia); !ok || v.Pulse0Us != 510 {
		t.Fatalf("pending %+v %v", v, ok)
	}
	if m.Reported(leg3Tibia).Pulse0Us != 500 {
		t.Fatal("shown value changed before confirmation")
	}
	set, ok := m.Next(t0)
	if !ok || set.Joint != leg3Tibia || set.Pulse0Us != 510 || set.Pulse180Us != 2500 || set.OffsetCentideg != 9000 || set.RequestID != 1 {
		t.Fatalf("set %+v %v", set, ok)
	}
	if _, ok := m.Next(t0.Add(20 * time.Millisecond)); ok {
		t.Fatal("sent again before the resend time")
	}
	// A report for an older request does not confirm it.
	m.Report(defaults(0, protocol.ResultAccepted))
	if _, ok := m.Pending(leg3Tibia); !ok {
		t.Fatal("confirmed by the wrong request id")
	}
	m.Report(reportFor(set, protocol.ResultAccepted, wire(set)))
	if _, ok := m.Pending(leg3Tibia); ok {
		t.Fatal("still pending after confirmation")
	}
	if m.Reported(leg3Tibia).Pulse0Us != 510 || m.Result(leg3Tibia) != ResultAccepted {
		t.Fatalf("shown %+v result %v", m.Reported(leg3Tibia), m.Result(leg3Tibia))
	}
}

func TestResendWithTheSameID(t *testing.T) {
	var m Model
	m.Report(defaults(41, protocol.ResultAccepted))
	m.Nudge(0, FieldOffset, 1, true)
	first, _ := m.Next(t0)
	if first.RequestID != 42 || first.OffsetCentideg != 9500 {
		t.Fatalf("first %+v", first)
	}
	if _, ok := m.Next(t0.Add(ResendAfter - time.Millisecond)); ok {
		t.Fatal("resent early")
	}
	again, ok := m.Next(t0.Add(ResendAfter))
	if !ok || again != first {
		t.Fatalf("resend %+v, want %+v", again, first)
	}
	// After another resend period without confirmation, again the same.
	if again2, _ := m.Next(t0.Add(2 * ResendAfter)); again2 != first {
		t.Fatal("second resend differs")
	}
}

func TestLatestValueOnlyAndANewIDForANewValue(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(5, FieldPulse180, 1, false)
	m.Nudge(5, FieldPulse180, 1, false) // before it is sent: one entry, latest value
	set1, _ := m.Next(t0)
	if set1.Pulse180Us != 2520 {
		t.Fatalf("set %+v", set1)
	}
	m.Nudge(5, FieldPulse180, 1, true) // while in flight: a new value
	set2, ok := m.Next(t0.Add(20 * time.Millisecond))
	if !ok || set2.Pulse180Us != 2570 || set2.RequestID == set1.RequestID {
		t.Fatalf("set2 %+v", set2)
	}
	// The report for the first value does not confirm the second.
	m.Report(reportFor(set1, protocol.ResultAccepted, wire(set1)))
	if v, ok := m.Pending(5); !ok || v.Pulse180Us != 2570 {
		t.Fatal("second value lost")
	}
	if m.Reported(5).Pulse180Us != 2520 {
		t.Fatal("report not shown")
	}
	m.Report(reportFor(set2, protocol.ResultAccepted, wire(set2)))
	if m.PendingCount() != 0 {
		t.Fatal("not confirmed")
	}
}

func TestOldestPendingFirst(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(9, FieldOffset, 1, false)
	m.Nudge(2, FieldOffset, 1, false)
	m.Nudge(9, FieldOffset, 1, false) // updating 9 keeps its place
	a, _ := m.Next(t0)
	if a.Joint != 9 || a.OffsetCentideg != 9100 {
		t.Fatalf("first %+v", a)
	}
	// One change in flight: joint 2 waits until joint 9 is confirmed.
	if _, ok := m.Next(t0.Add(20 * time.Millisecond)); ok {
		t.Fatal("a second change sent while the first is in flight")
	}
	if r, _ := m.Next(t0.Add(ResendAfter)); r != a {
		t.Fatalf("resend %+v", r)
	}
	m.Report(reportFor(a, protocol.ResultAccepted, wire(a)))
	b, ok := m.Next(t0.Add(ResendAfter + 20*time.Millisecond))
	if !ok || b.Joint != 2 || b.RequestID == a.RequestID {
		t.Fatalf("second %+v", b)
	}
}

// Review finding: with two changes in flight, resending the older one
// after the robot processed the newer would be processed again (the robot
// knows only its last id) and could be reported as rejected though it was
// applied. One change in flight avoids that: the resend repeats the
// robot's last id and is answered with its stored result.
func TestResendIsAlwaysTheRobotsLastRequest(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(0, FieldOffset, 1, false)
	m.Nudge(1, FieldOffset, 1, false)
	a, _ := m.Next(t0)
	// The report for a is lost; until it is confirmed, only a is sent.
	for d := 20 * time.Millisecond; d < 2*time.Second; d += 20 * time.Millisecond {
		if set, ok := m.Next(t0.Add(d)); ok && set != a {
			t.Fatalf("sent %+v while %d is unconfirmed", set, a.RequestID)
		}
	}
}

// Spec (joint calibration): an offset 45° above the default is clamped by
// the robot to default + 30°, and the clamped value is shown.
func TestClampedValueShownAsReported(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	for i := 0; i < 9; i++ {
		m.Nudge(1, FieldOffset, 1, true) // +45°
	}
	set, _ := m.Next(t0)
	if set.OffsetCentideg != 9000+4500 {
		t.Fatalf("set %+v", set)
	}
	clamped := wire(set)
	clamped.OffsetCentideg = 9000 + 3000
	m.Report(reportFor(set, protocol.ResultClamped, clamped))
	if m.Reported(1).OffsetCentideg != 12000 || m.Result(1) != ResultClamped || m.PendingCount() != 0 {
		t.Fatalf("shown %+v result %v", m.Reported(1), m.Result(1))
	}
}

func TestRejectionShownAsReported(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(3, FieldDirection, 1, false)
	set, _ := m.Next(t0)
	if set.Flags != protocol.CalibrationReversed {
		t.Fatalf("set %+v", set)
	}
	m.Report(reportFor(set, protocol.ResultRejected, protocol.CalibrationJoint{OffsetCentideg: 9000, Pulse0Us: 500, Pulse180Us: 2500}))
	if m.Reported(3).Reversed || m.Result(3) != ResultRejected || m.PendingCount() != 0 {
		t.Fatalf("shown %+v result %v", m.Reported(3), m.Result(3))
	}
}

// A report with no set processed yet (result NoneYet, id 0) never confirms
// a change, and the first request id follows the robot's last one.
func TestNoneYetNeverConfirms(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(0, FieldPulse0, -1, false)
	set, _ := m.Next(t0)
	if set.RequestID != 1 {
		t.Fatalf("id %d", set.RequestID)
	}
	r := defaults(1, protocol.ResultNoneYet)
	m.Report(r)
	if m.PendingCount() != 1 {
		t.Fatal("confirmed by a NoneYet report")
	}
}

// Request ids wrap, but a new change never repeats the robot's last id,
// which it would take for a retransmission.
func TestRequestIDsWrapAndSkipTheRobotsLastID(t *testing.T) {
	var m Model
	m.Report(defaults(254, protocol.ResultAccepted))
	now := t0
	last := uint8(254)
	for i := 0; i < 600; i++ {
		now = now.Add(20 * time.Millisecond)
		m.Nudge(i%Joints, FieldOffset, 1, false)
		set, ok := m.Next(now)
		if !ok {
			t.Fatalf("round %d: nothing sent", i)
		}
		if set.RequestID == last {
			t.Fatalf("round %d: id %d repeats the robot's last", i, set.RequestID)
		}
		last = set.RequestID
		m.Report(reportFor(set, protocol.ResultAccepted, wire(set)))
	}
	// An earlier session left the robot at id 7 while the model's counter
	// would hand out 7 next: it is skipped.
	var m2 Model
	m2.Report(defaults(6, protocol.ResultAccepted)) // next would be 7
	m2.Report(defaults(7, protocol.ResultAccepted)) // a later report: the robot's last is 7
	m2.Nudge(0, FieldPulse0, 1, false)
	if set, _ := m2.Next(t0); set.RequestID == 7 {
		t.Fatal("id 7 reused")
	}
}

func TestNudgeStaysWithinWireTypes(t *testing.T) {
	var m Model
	r := defaults(0, protocol.ResultNoneYet)
	r.Joints[0] = protocol.CalibrationJoint{OffsetCentideg: 32700, Pulse0Us: 20, Pulse180Us: 65500}
	m.Report(r)
	m.Nudge(0, FieldOffset, 1, true)
	m.Nudge(0, FieldPulse0, -1, true)
	m.Nudge(0, FieldPulse180, 1, true)
	v, _ := m.Pending(0)
	if v.OffsetCentideg != 32767 || v.Pulse0Us != 0 || v.Pulse180Us != 65535 {
		t.Fatalf("value %+v", v)
	}
}

func TestSetBackToTheReportedValueStillSends(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	if m.Set(4, m.Reported(4)); m.PendingCount() != 0 {
		t.Fatal("an unchanged value became pending")
	}
	m.Nudge(4, FieldPulse0, 1, false)
	m.Nudge(4, FieldPulse0, -1, false)
	// Back to the reported value while pending: the robot may already hold
	// the first value, so the return must still be sent and confirmed.
	set, ok := m.Next(t0)
	if !ok || set.Pulse0Us != 500 {
		t.Fatalf("set %+v %v", set, ok)
	}
}

func TestDropPendingAndForget(t *testing.T) {
	var m Model
	m.Report(defaults(0, protocol.ResultNoneYet))
	m.Nudge(4, FieldPulse0, 1, false)
	m.DropPending()
	if m.PendingCount() != 0 {
		t.Fatal("pending kept")
	}
	if _, ok := m.Next(t0); ok {
		t.Fatal("dropped change sent")
	}
	m.Forget()
	if m.HaveReport() {
		t.Fatal("report kept")
	}
}
