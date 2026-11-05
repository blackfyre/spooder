package udplink

import (
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// Gap is the silence after which the next valid frame rebaselines the
// sequence numbers, and the age after which a type's shown frame is taken
// again from its next frame, whatever its sequence number (design D3).
const Gap = 500 * time.Millisecond

// typeCount is the size of the per-type arrays, indexed by FrameType.
const typeCount = int(protocol.TypeDiagnostics) + 1

// RobotType tells the frame types a robot sends.
func RobotType(t protocol.FrameType) bool {
	switch t {
	case protocol.TypeTelemetry, protocol.TypeCalibrationReport, protocol.TypePose, protocol.TypeDiagnostics:
		return true
	}
	return false
}

// classify decodes a datagram that must hold exactly one valid frame and
// nothing else: no byte before or after it, no rejected candidate, no
// second frame.
func classify(b []byte) (protocol.Frame, bool) {
	stray := 0
	d := protocol.Decoder{Discard: func(byte) { stray++ }}
	for _, x := range b {
		d.Push(x)
	}
	f, ok := d.Next()
	if !ok || stray != 0 || d.Pending() != 0 || d.Discarded() != 0 || d.Valid() != 1 {
		return protocol.Frame{}, false
	}
	return f, true
}

// newer tells a sequence number ahead of ref by 1..127 (design D3).
func newer(seq, ref uint8) bool {
	d := seq - ref
	return d >= 1 && d <= 127
}

// Event is one accepted robot frame, in arrival order.
type Event struct {
	Frame protocol.Frame
	At    time.Time
	// Late: behind the link-wide newest sequence number, so not taken as
	// newest.
	Late bool
	// Rebaseline: this frame set the newest sequence number afresh (the
	// run's first frame, the first after a resume or after Gap of silence).
	Rebaseline bool
}

// Shown is what the screen shows of one frame type (design D3).
type Shown struct {
	Frame     protocol.Frame
	Have      bool
	TakenAt   time.Time // when Frame was taken
	ArrivedAt time.Time // the last valid frame of the type, taken or not
	seq       uint8     // the sequence number a frame must be newer than
}

// Quality is the link-quality part of a Status.
type Quality struct {
	Sent, SendErrors int
	Invalid          int // datagrams that were not exactly one robot frame
	Lost, Late       int // from the robot's sequence numbers, since start
	Dropped          int // events not taken before the queue filled
	// PerSecond is the robot frames received in the last whole second, by
	// type.
	PerSecond  [typeCount]int
	HaveFrame  bool
	LastValid  time.Time // the last valid frame's arrival
	Newest     uint8     // the link-wide newest sequence number
	HaveNewest bool
}

// SinceLast is the time since the last valid frame at now, or 0 before the
// first.
func (q Quality) SinceLast(now time.Time) time.Duration {
	if !q.HaveFrame {
		return 0
	}
	return now.Sub(q.LastValid)
}

// Status is a copy of the receiver's state for the screen and the app.
type Status struct {
	Quality
	Shown [typeCount]Shown
}

// Receiver validates received datagrams and keeps the link-quality counts
// and the shown frames. It is not safe for concurrent use: Link guards it.
type Receiver struct {
	q       Quality
	shown   [typeCount]Shown
	rebase  bool // the next valid frame rebaselines
	winFrom time.Time
	win     [typeCount]int // the current second's counts
}

// NewReceiver returns a receiver whose one-second windows start at start.
func NewReceiver(start time.Time) *Receiver {
	return &Receiver{rebase: true, winFrom: start}
}

// Rebaseline makes the next valid frame set the newest sequence number
// without counting anything lost or late: for a resume.
func (r *Receiver) Rebaseline() { r.rebase = true }

// roll moves the one-second windows forward to now.
func (r *Receiver) roll(now time.Time) {
	n := now.Sub(r.winFrom) / time.Second
	if n < 1 {
		return
	}
	if n == 1 {
		r.q.PerSecond = r.win
	} else {
		r.q.PerSecond = [typeCount]int{}
	}
	r.win = [typeCount]int{}
	r.winFrom = r.winFrom.Add(n * time.Second)
}

// Datagram takes one received datagram at now. It returns the event of an
// accepted robot frame; anything else is counted invalid and dropped.
func (r *Receiver) Datagram(b []byte, now time.Time) (Event, bool) {
	r.roll(now)
	f, ok := classify(b)
	if !ok || !RobotType(f.Type) {
		r.q.Invalid++
		return Event{}, false
	}
	r.win[f.Type]++
	ev := Event{Frame: f, At: now}
	if r.rebase || !r.q.HaveNewest || now.Sub(r.q.LastValid) >= Gap {
		ev.Rebaseline = true
		r.rebase = false
		r.q.Newest, r.q.HaveNewest = f.Seq, true
		for i := range r.shown {
			r.shown[i].seq = f.Seq
		}
	} else {
		switch d := f.Seq - r.q.Newest; {
		case d == 1:
			r.q.Newest = f.Seq
		case d >= 2 && d <= 127:
			r.q.Lost += int(d) - 1
			r.q.Newest = f.Seq
		default: // 0 or 128..255
			r.q.Late++
			ev.Late = true
		}
	}
	r.q.HaveFrame, r.q.LastValid = true, now

	s := &r.shown[f.Type]
	if ev.Rebaseline || !s.Have || newer(f.Seq, s.seq) || now.Sub(s.TakenAt) > Gap {
		s.Frame, s.Have, s.TakenAt, s.seq = f, true, now, f.Seq
	}
	s.ArrivedAt = now
	return ev, true
}

// Status returns the state at now.
func (r *Receiver) Status(now time.Time) Status {
	r.roll(now)
	return Status{Quality: r.q, Shown: r.shown}
}
