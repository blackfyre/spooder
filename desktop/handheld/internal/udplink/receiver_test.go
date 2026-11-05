package udplink

import (
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// rxRig feeds a Receiver frames at chosen times.
type rxRig struct {
	t   *testing.T
	r   *Receiver
	now time.Time
}

func newRxRig(t *testing.T) *rxRig {
	start := time.Unix(1000, 0)
	return &rxRig{t: t, r: NewReceiver(start), now: start}
}

func (g *rxRig) after(d time.Duration) *rxRig { g.now = g.now.Add(d); return g }

func (g *rxRig) frame(ft protocol.FrameType, seq uint8, voltage uint16) Event {
	g.t.Helper()
	var b []byte
	switch ft {
	case protocol.TypeTelemetry:
		b = protocol.EncodeTelemetry(protocol.Telemetry{VoltageMv: voltage}, seq)
	case protocol.TypePose:
		b = protocol.EncodePose(protocol.Pose{}, seq)
	case protocol.TypeDiagnostics:
		b = protocol.EncodeDiagnostics(protocol.Diagnostics{MinMv: voltage}, seq)
	case protocol.TypeCalibrationReport:
		b = protocol.EncodeCalibrationReport(protocol.CalibrationReport{}, seq)
	}
	ev, ok := g.r.Datagram(b, g.now)
	if !ok {
		g.t.Fatalf("frame type %d seq %d refused", ft, seq)
	}
	return ev
}

func (g *rxRig) tel(seqs ...uint8) {
	g.t.Helper()
	for _, s := range seqs {
		g.after(33*time.Millisecond).frame(protocol.TypeTelemetry, s, uint16(s))
	}
}

func (g *rxRig) counts(lost, late int) {
	g.t.Helper()
	st := g.r.Status(g.now)
	if st.Lost != lost || st.Late != late {
		g.t.Fatalf("lost %d late %d, want %d and %d", st.Lost, st.Late, lost, late)
	}
}

func TestLossCountedFromSequenceNumbers(t *testing.T) {
	g := newRxRig(t)
	g.tel(10, 11, 14)
	g.counts(2, 0)
}

func TestLateFrame(t *testing.T) {
	g := newRxRig(t)
	g.tel(10, 11, 14, 12)
	g.counts(2, 1)
	if st := g.r.Status(g.now); st.Newest != 14 {
		t.Fatalf("newest %d after the late 12, want 14", st.Newest)
	}
	g.tel(15)
	g.counts(2, 1) // 15 follows 14 in order
}

func TestSequenceWrap(t *testing.T) {
	g := newRxRig(t)
	g.tel(254, 255, 0, 1)
	g.counts(0, 0)
}

func TestGapRebaselines(t *testing.T) {
	g := newRxRig(t)
	g.tel(13, 14)
	g.after(500 * time.Millisecond)
	if ev := g.frame(protocol.TypeTelemetry, 200, 200); !ev.Rebaseline || ev.Late {
		t.Fatalf("the frame after 500 ms of silence: %+v, want a rebaseline", ev)
	}
	g.counts(0, 0)
	g.tel(201, 202)
	g.counts(0, 0)

	// Just under the gap a jump is counted.
	g.after(499*time.Millisecond).frame(protocol.TypeTelemetry, 210, 0)
	g.counts(7, 0)
}

func TestResumeRebaselines(t *testing.T) {
	g := newRxRig(t)
	g.tel(13, 14)
	g.r.Rebaseline()
	if ev := g.after(20*time.Millisecond).frame(protocol.TypeTelemetry, 90, 90); !ev.Rebaseline {
		t.Fatalf("the first frame after a resume: %+v, want a rebaseline", ev)
	}
	g.tel(91, 3) // 3 is behind 91 by 88: late
	g.counts(0, 1)
}

func TestRestartRebaselines(t *testing.T) {
	g := newRxRig(t)
	g.tel(100, 101)
	g.after(800 * time.Millisecond) // a robot's restart silences the link
	g.tel(0, 1, 2)
	g.counts(0, 0)
	if sh := g.r.Status(g.now).Shown[protocol.TypeTelemetry]; sh.Frame.Seq != 2 {
		t.Fatalf("shown telemetry %d after the restart, want 2", sh.Frame.Seq)
	}
}

func shownVoltage(g *rxRig) uint16 {
	return g.r.Status(g.now).Shown[protocol.TypeTelemetry].Frame.Telemetry.VoltageMv
}

func TestOlderTelemetryAfterANewerFrame(t *testing.T) {
	g := newRxRig(t)
	g.tel(10)
	g.after(10*time.Millisecond).frame(protocol.TypeDiagnostics, 14, 0)
	g.tel(13)
	if v := shownVoltage(g); v != 13 {
		t.Fatalf("shown telemetry %d, want 13", v)
	}
	g.tel(9)
	if v := shownVoltage(g); v != 13 {
		t.Fatalf("shown telemetry %d after 9, want 13", v)
	}
}

func TestOneTypeLostWhileTheOthersArrive(t *testing.T) {
	g := newRxRig(t)
	g.tel(10)
	seq := uint8(11)
	for elapsed := time.Duration(0); elapsed < 600*time.Millisecond; elapsed += 50 * time.Millisecond {
		g.after(25*time.Millisecond).frame(protocol.TypePose, seq, 0)
		g.after(25*time.Millisecond).frame(protocol.TypeDiagnostics, seq+1, 0)
		seq += 2
	}
	if st := g.r.Status(g.now); st.Late != 0 || st.Lost != 0 {
		t.Fatalf("pose and diagnostics in order counted %+v", st.Quality)
	}
	g.after(10*time.Millisecond).frame(protocol.TypeTelemetry, 140, 140)
	if v := shownVoltage(g); v != 140 {
		t.Fatalf("shown telemetry %d, want 140", v)
	}
	// Within 500 ms of its own last frame an older one is not taken.
	g.after(10*time.Millisecond).frame(protocol.TypeTelemetry, 139, 139)
	if v := shownVoltage(g); v != 140 {
		t.Fatalf("shown telemetry %d after 139, want 140", v)
	}
}

func TestFramesPerSecond(t *testing.T) {
	g := newRxRig(t)
	for i := 0; i < 10; i++ {
		g.after(100*time.Millisecond).frame(protocol.TypeTelemetry, uint8(2*i), 0)
		g.frame(protocol.TypePose, uint8(2*i+1), 0)
	}
	// The tenth pair arrived at exactly 1 s: it opens the second window.
	st := g.r.Status(g.now)
	if st.PerSecond[protocol.TypeTelemetry] != 9 || st.PerSecond[protocol.TypePose] != 9 {
		t.Fatalf("per second %v, want 9 telemetry and 9 pose", st.PerSecond)
	}
	if st := g.r.Status(g.now.Add(2 * time.Second)); st.PerSecond != [typeCount]int{} {
		t.Fatalf("per second %v after 2 s of silence, want none", st.PerSecond)
	}
	if st := g.r.Status(g.now.Add(2 * time.Second)); st.SinceLast(g.now.Add(2*time.Second)) != 2*time.Second {
		t.Fatalf("since last %v, want 2 s", st.SinceLast(g.now.Add(2*time.Second)))
	}
}
