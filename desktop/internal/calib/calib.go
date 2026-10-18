// Package calib is the joint table model of the desktop calibrator (design
// D5): it shows the robot's reported calibration, holds the operator's
// changes until a report confirms them, and decides which calibration set
// the writer sends next. The rules are those of the bridge's
// calibration_sync: the latest value per joint, the oldest pending sent
// first, a request id per send, confirmation by the report's request id, a
// resend after 200 ms with the same id, and the reported value replacing the
// shown one, including clamps and rejections. One change is in flight at a
// time (see Next). Every guard is on the robot;
// this model only keeps values within their wire types.
package calib

import (
	"math"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// Joints is the number of joints, in servo output order (leg 1 coxa, femur,
// tibia, then leg 2 ...).
const Joints = protocol.CalibrationJoints

// ResendAfter is how long a sent change waits for its confirming report
// before it is sent again with the same request id.
const ResendAfter = 200 * time.Millisecond

// Nudge steps (design D5).
const (
	FineOffsetCentideg   = 50  // 0.5°
	CoarseOffsetCentideg = 500 // 5°
	FinePulseUs          = 10
	CoarsePulseUs        = 50
)

// Joint is one joint's calibration as it travels on the wire.
type Joint struct {
	OffsetCentideg int16
	Pulse0Us       uint16
	Pulse180Us     uint16
	Reversed       bool
}

// FromWire converts a report entry.
func FromWire(j protocol.CalibrationJoint) Joint {
	return Joint{OffsetCentideg: j.OffsetCentideg, Pulse0Us: j.Pulse0Us, Pulse180Us: j.Pulse180Us,
		Reversed: j.Flags&protocol.CalibrationReversed != 0}
}

// Field is one editable value of a joint.
type Field int

const (
	FieldOffset Field = iota
	FieldPulse0
	FieldPulse180
	FieldDirection
	FieldCount
)

// Names of the joints within a leg.
var JointNames = [3]string{"coxa", "femur", "tibia"}

// Leg is the joint's leg, 1..6.
func Leg(joint int) int { return joint/3 + 1 }

// Name is the joint's name within its leg.
func Name(joint int) string { return JointNames[joint%3] }

// Result is the robot's answer to the last confirmed change of a joint.
type Result int

const (
	ResultNone     Result = iota // no change confirmed in this session
	ResultAccepted               // taken as requested
	ResultClamped                // taken after clamping
	ResultRejected               // refused (not in Calibrate, or output not allowed)
)

type pending struct {
	value Joint
	order uint64    // when the joint became pending; lower is older
	sent  bool      // the current value has been sent under id
	id    uint8     // request id of the last send of the current value
	at    time.Time // time of the last send
}

// Model is the joint table. It is not safe for concurrent use; the session
// serialises access.
type Model struct {
	reported   [Joints]Joint
	haveReport bool
	results    [Joints]Result
	pending    [Joints]*pending
	order      uint64
	nextID     uint8
	seeded     bool
	lastID     uint8 // the latest report's request id
	haveLastID bool
}

// HaveReport reports whether a calibration report has arrived.
func (m *Model) HaveReport() bool { return m.haveReport }

// Reported is the joint's value from the latest report.
func (m *Model) Reported(joint int) Joint { return m.reported[joint] }

// Table is the latest reported table.
func (m *Model) Table() [Joints]Joint { return m.reported }

// Pending reports whether a change of the joint awaits confirmation, and
// the value requested.
func (m *Model) Pending(joint int) (Joint, bool) {
	if p := m.pending[joint]; p != nil {
		return p.value, true
	}
	return Joint{}, false
}

// PendingCount is the number of joints with changes awaiting confirmation.
func (m *Model) PendingCount() int {
	n := 0
	for _, p := range m.pending {
		if p != nil {
			n++
		}
	}
	return n
}

// Result is the robot's answer to the joint's last confirmed change.
func (m *Model) Result(joint int) Result { return m.results[joint] }

// Report applies a calibration report: the reported table replaces the
// shown one, and the change it confirms, if any, is done.
func (m *Model) Report(r protocol.CalibrationReport) {
	for i := range m.reported {
		m.reported[i] = FromWire(r.Joints[i])
	}
	m.haveReport = true
	m.lastID, m.haveLastID = r.LastRequestID, true
	if !m.seeded {
		// Start after the robot's last request id on this link, which may
		// be from an earlier session: a repeat would be taken as a
		// retransmission and change nothing.
		m.nextID = r.LastRequestID + 1
		m.seeded = true
	}
	if protocol.CalibrationResult(r.Result) == protocol.ResultNoneYet {
		return
	}
	for i, p := range m.pending {
		if p == nil || !p.sent || p.id != r.LastRequestID {
			continue
		}
		switch protocol.CalibrationResult(r.Result) {
		case protocol.ResultAccepted:
			m.results[i] = ResultAccepted
		case protocol.ResultClamped:
			m.results[i] = ResultClamped
		default:
			m.results[i] = ResultRejected
		}
		m.pending[i] = nil
	}
}

// base is the value an edit of the joint starts from: the pending value if
// there is one, otherwise the reported one.
func (m *Model) base(joint int) Joint {
	if p := m.pending[joint]; p != nil {
		return p.value
	}
	return m.reported[joint]
}

// Set requests a value for a joint. A value equal to the reported one with
// nothing pending changes nothing.
func (m *Model) Set(joint int, v Joint) {
	p := m.pending[joint]
	if p == nil {
		if v == m.reported[joint] {
			return
		}
		m.order++
		m.pending[joint] = &pending{value: v, order: m.order}
		return
	}
	if v != p.value {
		p.value = v
		p.sent = false // a new value needs a new request id
	}
}

// Nudge changes one field of a joint by steps fine or coarse increments
// (negative steps go down); for the direction, any step toggles it.
func (m *Model) Nudge(joint int, f Field, steps int, coarse bool) {
	v := m.base(joint)
	switch f {
	case FieldOffset:
		step := FineOffsetCentideg
		if coarse {
			step = CoarseOffsetCentideg
		}
		v.OffsetCentideg = int16(clamp(int(v.OffsetCentideg)+steps*step, math.MinInt16, math.MaxInt16))
	case FieldPulse0, FieldPulse180:
		step := FinePulseUs
		if coarse {
			step = CoarsePulseUs
		}
		if f == FieldPulse0 {
			v.Pulse0Us = uint16(clamp(int(v.Pulse0Us)+steps*step, 0, math.MaxUint16))
		} else {
			v.Pulse180Us = uint16(clamp(int(v.Pulse180Us)+steps*step, 0, math.MaxUint16))
		}
	case FieldDirection:
		if steps != 0 {
			v.Reversed = !v.Reversed
		}
	}
	m.Set(joint, v)
}

func clamp(v, lo, hi int) int {
	return min(max(v, lo), hi)
}

// Next returns the calibration set to send now, if any. One change is in
// flight at a time, because the robot recognises only its last request id
// as a retransmission: while a sent value awaits its report, only that
// change is sent again, with the same id, once it has waited ResendAfter.
// Otherwise the oldest pending joint is sent with a new id.
func (m *Model) Next(now time.Time) (protocol.CalibrationSet, bool) {
	best := -1
	for i, p := range m.pending {
		if p != nil && p.sent {
			if now.Sub(p.at) < ResendAfter {
				return protocol.CalibrationSet{}, false
			}
			best = i
			break
		}
	}
	if best < 0 {
		for i, p := range m.pending {
			if p != nil && (best < 0 || p.order < m.pending[best].order) {
				best = i
			}
		}
	}
	if best < 0 {
		return protocol.CalibrationSet{}, false
	}
	p := m.pending[best]
	if !p.sent {
		p.id = m.newID()
		p.sent = true
	}
	p.at = now
	set := protocol.CalibrationSet{RequestID: p.id, Joint: uint8(best), OffsetCentideg: p.value.OffsetCentideg,
		Pulse0Us: p.value.Pulse0Us, Pulse180Us: p.value.Pulse180Us}
	if p.value.Reversed {
		set.Flags = protocol.CalibrationReversed
	}
	return set, true
}

// newID returns the next request id, skipping the robot's last reported id
// and any id still awaiting confirmation, so a report never confirms the
// wrong change and the robot never takes a new change for a retransmission.
func (m *Model) newID() uint8 {
	for {
		id := m.nextID
		m.nextID++
		if m.haveLastID && id == m.lastID {
			continue
		}
		inUse := false
		for _, p := range m.pending {
			if p != nil && p.sent && p.id == id {
				inUse = true
				break
			}
		}
		if !inUse {
			return id
		}
	}
}

// DropPending abandons every unconfirmed change, when the app may no longer
// send calibration (control released or the link lost).
func (m *Model) DropPending() {
	m.pending = [Joints]*pending{}
}

// Forget clears the reported table, for a new connection.
func (m *Model) Forget() {
	*m = Model{}
}
