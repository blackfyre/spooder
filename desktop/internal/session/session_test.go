package session

import (
	"errors"
	"reflect"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

type clock struct{ t time.Time }

func (c *clock) now() time.Time          { return c.t }
func (c *clock) advance(d time.Duration) { c.t = c.t.Add(d) }
func newClock() *clock                   { return &clock{t: time.Date(2026, 10, 18, 9, 0, 0, 0, time.UTC)} }
func telemetryFrame(t protocol.Telemetry) protocol.Frame {
	return protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: t}
}

var device = link.PortInfo{Name: "/dev/ttyACM0", IsUSB: true, VID: "2E8A", PID: "000A", SerialNumber: "X"}

// connected returns a session on a connected link, and its clock.
func connected(t *testing.T) (*Session, *clock) {
	t.Helper()
	c := newClock()
	s := New(c.now, nil)
	s.LinkState(link.State{Status: link.Connected, Port: device})
	return s, c
}

func report(lastID uint8, result protocol.CalibrationResult) protocol.Frame {
	r := protocol.CalibrationReport{LastRequestID: lastID, Result: uint8(result)}
	for i := range r.Joints {
		r.Joints[i] = protocol.CalibrationJoint{OffsetCentideg: 9000, Pulse0Us: 500, Pulse180Us: 2500}
	}
	return protocol.Frame{Type: protocol.TypeCalibrationReport, CalibrationReport: r}
}

// Spec "Observe only": connected without control, frames report no
// controller and the mode controls are disabled.
func TestObserveOnly(t *testing.T) {
	s, _ := connected(t)
	c := s.Command()
	want := protocol.Command{Mode: uint8(RequestSleep), Speed: 44}
	if c != want {
		t.Fatalf("command %+v, want %+v", c, want)
	}
	snap := s.Snapshot()
	if snap.Control || snap.ModesEnabled || !snap.CanControl {
		t.Fatalf("snapshot %+v", snap)
	}
	if err := s.RequestMode(RequestStand); !errors.Is(err, ErrNotInControl) {
		t.Fatalf("mode request while observing: %v", err)
	}
	if err := s.ClearFault(); !errors.Is(err, ErrNotInControl) {
		t.Fatalf("clear while observing: %v", err)
	}
	if c := s.Command(); c != want {
		t.Fatalf("counters moved: %+v", c)
	}
}

func TestTakeControlAndModeRequests(t *testing.T) {
	s, _ := connected(t)
	if err := s.TakeControl(); err != nil {
		t.Fatal(err)
	}
	c := s.Command()
	if c.Flags != protocol.FlagControllerConnected || c.Mode != uint8(RequestSleep) || c.ModeRequestCounter != 0 {
		t.Fatalf("after take: %+v", c)
	}
	for i, m := range []RequestedMode{RequestStand, RequestCalibrate, RequestCalibrate, RequestSleep} {
		if err := s.RequestMode(m); err != nil {
			t.Fatal(err)
		}
		c = s.Command()
		if c.Mode != uint8(m) || c.ModeRequestCounter != uint8(i+1) {
			t.Fatalf("request %d: %+v", i, c)
		}
		if c.TranslateX != 0 || c.TranslateY != 0 || c.Rotate != 0 {
			t.Fatal("translate or rotate not zero")
		}
	}
	if err := s.RequestMode(RequestWalk); err == nil {
		t.Fatal("walk accepted")
	}
	if !s.Snapshot().ModesEnabled {
		t.Fatal("modes disabled in control")
	}
}

// Spec "Gesture controls": one request per press, only in control.
func TestGestureRequests(t *testing.T) {
	s, _ := connected(t)
	if err := s.RequestGesture(protocol.GestureWiggle); !errors.Is(err, ErrNotInControl) {
		t.Fatalf("gesture while observing: %v", err)
	}
	if c := s.Command(); c.GestureRequestCounter != 0 {
		t.Fatalf("counter moved while observing: %+v", c)
	}
	if err := s.TakeControl(); err != nil {
		t.Fatal(err)
	}
	for i, g := range []uint8{protocol.GestureWiggle, protocol.GestureWiggle, protocol.GestureSayHi} {
		if err := s.RequestGesture(g); err != nil {
			t.Fatal(err)
		}
		c := s.Command()
		if c.Gesture != g || c.GestureRequestCounter != uint8(i+1) {
			t.Fatalf("request %d: %+v", i, c)
		}
	}
	for _, g := range []uint8{protocol.GestureNone, protocol.GestureCount, 255} {
		if err := s.RequestGesture(g); err == nil {
			t.Fatalf("gesture %d accepted", g)
		}
	}
	s.Frame(protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: protocol.Telemetry{BodyMode: uint8(BodyPerform), Gesture: protocol.GestureSayHi}}, link.Stats{})
	snap := s.Snapshot()
	if snap.Body != BodyPerform || snap.Gesture != protocol.GestureSayHi || snap.Body.String() != "Perform" {
		t.Fatalf("snapshot body %v gesture %d", snap.Body, snap.Gesture)
	}
}

func TestTakeControlNeedsConnection(t *testing.T) {
	s := New(newClock().now, nil)
	if err := s.TakeControl(); !errors.Is(err, ErrNotConnected) {
		t.Fatalf("take while disconnected: %v", err)
	}
	if s.Snapshot().CanControl {
		t.Fatal("control offered while disconnected")
	}
}

func TestReleaseLossAndShutdownStopReportingAController(t *testing.T) {
	for name, drop := range map[string]func(s *Session){
		"release":  func(s *Session) { s.ReleaseControl() },
		"shutdown": func(s *Session) { s.Shutdown() },
		"unplug":   func(s *Session) { s.LinkState(link.State{Status: link.Waiting, Port: device, Err: link.ErrLost}) },
		"disconnect": func(s *Session) {
			s.LinkState(link.State{Status: link.Disconnected, Port: device})
		},
	} {
		s, _ := connected(t)
		_ = s.TakeControl()
		_ = s.RequestMode(RequestCalibrate)
		drop(s)
		c := s.Command()
		if c.Flags&protocol.FlagControllerConnected != 0 || c.Mode != uint8(RequestSleep) {
			t.Errorf("%s: %+v", name, c)
		}
		if s.Snapshot().Control {
			t.Errorf("%s: still in control", name)
		}
		// Reconnecting does not take control again by itself.
		s.LinkState(link.State{Status: link.Connected, Port: device})
		if c := s.Command(); c.Flags&protocol.FlagControllerConnected != 0 {
			t.Errorf("%s: control after reconnect", name)
		}
	}
}

// Spec "Emergency stop": the flag toggles whether or not the app is in
// control; releasing it does not clear the fault, which needs control.
func TestEmergencyStop(t *testing.T) {
	s, _ := connected(t)
	s.ToggleEmergencyStop()
	c := s.Command()
	if c.Flags != protocol.FlagEmergencyStop || c.Mode != uint8(RequestSleep) {
		t.Fatalf("e-stop while observing: %+v", c)
	}
	if !s.Snapshot().EmergencyStop {
		t.Fatal("e-stop not shown")
	}
	s.ToggleEmergencyStop()
	if c := s.Command(); c.Flags != 0 || c.ClearCounter != 0 {
		t.Fatalf("release: %+v", c)
	}
	_ = s.TakeControl()
	s.ToggleEmergencyStop()
	if c := s.Command(); c.Flags != protocol.FlagControllerConnected|protocol.FlagEmergencyStop {
		t.Fatalf("e-stop in control: %+v", c)
	}
	if err := s.ClearFault(); err != nil {
		t.Fatal(err)
	}
	if c := s.Command(); c.ClearCounter != 1 || c.Flags&protocol.FlagEmergencyStop == 0 {
		t.Fatalf("clear: %+v", c)
	}
	// The e-stop survives losing the link and control.
	s.LinkState(link.State{Status: link.Waiting, Port: device})
	if c := s.Command(); c.Flags != protocol.FlagEmergencyStop {
		t.Fatalf("after loss: %+v", c)
	}
}

// Spec "Fault on the robot" and the telemetry view.
func TestTelemetryView(t *testing.T) {
	s, c := connected(t)
	s.Frame(telemetryFrame(protocol.Telemetry{BodyMode: uint8(BodySleep), Gait: 2, Faults: FaultUndervoltage, VoltageMv: 5480,
		CurrentMa: 120, ValidFrames: 900, DiscardedFrames: 3, Flags: protocol.TelemetryUSBInControl}), link.Stats{Valid: 10})
	snap := s.Snapshot()
	if !snap.Fresh || snap.Body != BodySleep || snap.Telemetry.VoltageMv != 5480 || snap.Source != SourceUSB {
		t.Fatalf("snapshot %+v", snap)
	}
	if !reflect.DeepEqual(snap.Faults, []string{"undervoltage"}) {
		t.Fatalf("faults %v", snap.Faults)
	}
	if GaitName(snap.Telemetry.Gait) != "Ripple" || snap.Stats.Valid != 10 {
		t.Fatal("gait or stats")
	}
	c.advance(StaleAfter)
	if !s.Snapshot().Fresh {
		t.Fatal("stale at exactly 1 s")
	}
	c.advance(time.Millisecond)
	if snap := s.Snapshot(); snap.Fresh || !snap.HaveTelemetry {
		t.Fatal("not marked stale after 1 s")
	}
}

// Spec "Conflict shown".
func TestSourceAndConflict(t *testing.T) {
	for flags, want := range map[uint8]Source{
		0:                                    SourceNone,
		protocol.TelemetryBridgeInControl:    SourceBridge,
		protocol.TelemetryUSBInControl:       SourceUSB,
		protocol.TelemetrySourceConflict:     SourceConflict,
		protocol.TelemetrySourceConflict | 6: SourceConflict,
	} {
		if got := SourceFrom(flags); got != want {
			t.Errorf("flags %d: %v, want %v", flags, got, want)
		}
	}
	s, _ := connected(t)
	_ = s.TakeControl()
	s.Frame(telemetryFrame(protocol.Telemetry{BodyMode: uint8(BodySleep), Flags: protocol.TelemetrySourceConflict}), link.Stats{})
	if s.Snapshot().Source != SourceConflict {
		t.Fatal("conflict not shown")
	}
}

func TestFaultNames(t *testing.T) {
	got := FaultNames(FaultOvercurrent | FaultEmergencyStop | 1<<9)
	want := []string{"overcurrent", "emergency stop", "fault bit 9"}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("%v", got)
	}
	if FaultNames(0) != nil {
		t.Fatal("faults without bits")
	}
}

// calibrating returns a session in control with the robot in Calibrate and
// a report received.
func calibrating(t *testing.T) (*Session, *clock) {
	s, c := connected(t)
	_ = s.TakeControl()
	_ = s.RequestMode(RequestCalibrate)
	s.Frame(telemetryFrame(protocol.Telemetry{BodyMode: uint8(BodyCalibrate), OutputsOn: true, Flags: protocol.TelemetryUSBInControl}), link.Stats{})
	s.Frame(report(0, protocol.ResultNoneYet), link.Stats{})
	return s, c
}

// Spec "Nudge a joint": leg 3's tibia 0° pulse +10 µs is pending until the
// report confirms it.
func TestNudgeAJointThroughTheSession(t *testing.T) {
	s, c := calibrating(t)
	if !s.Snapshot().Editable {
		t.Fatal("not editable in Calibrate")
	}
	if err := s.Nudge(8, calib.FieldPulse0, 1, false); err != nil {
		t.Fatal(err)
	}
	snap := s.Snapshot()
	if !snap.Joints[8].Pending || snap.Joints[8].Wanted.Pulse0Us != 510 || snap.Joints[8].Reported.Pulse0Us != 500 {
		t.Fatalf("joint %+v", snap.Joints[8])
	}
	set, ok := s.CalibrationSet(c.now())
	if !ok || set.Joint != 8 || set.Pulse0Us != 510 {
		t.Fatalf("set %+v", set)
	}
	r := report(set.RequestID, protocol.ResultAccepted)
	r.CalibrationReport.Joints[8].Pulse0Us = 510
	s.Frame(r, link.Stats{})
	snap = s.Snapshot()
	if snap.Joints[8].Pending || snap.Joints[8].Reported.Pulse0Us != 510 || snap.PendingCount != 0 {
		t.Fatalf("joint %+v", snap.Joints[8])
	}
}

// Spec "Editing outside Calibrate", and the other parts of the gate.
func TestEditingDisabled(t *testing.T) {
	cases := map[string]func(s *Session, c *clock){
		"standing": func(s *Session, c *clock) {
			s.Frame(telemetryFrame(protocol.Telemetry{BodyMode: uint8(BodyStand)}), link.Stats{})
		},
		"observing":       func(s *Session, c *clock) { s.ReleaseControl() },
		"stale telemetry": func(s *Session, c *clock) { c.advance(StaleAfter + time.Millisecond) },
		"new connection": func(s *Session, c *clock) {
			s.LinkState(link.State{Status: link.Waiting, Port: device})
			s.LinkState(link.State{Status: link.Connected, Port: device})
			_ = s.TakeControl()
		},
	}
	for name, change := range cases {
		s, c := calibrating(t)
		change(s, c)
		if s.Snapshot().Editable {
			t.Errorf("%s: editable", name)
		}
		if err := s.Nudge(0, calib.FieldOffset, 1, false); !errors.Is(err, ErrNotEditable) {
			t.Errorf("%s: nudge %v", name, err)
		}
	}
}

func TestNoCalibrationSentWithoutControl(t *testing.T) {
	s, c := calibrating(t)
	_ = s.Nudge(0, calib.FieldOffset, 1, false)
	s.ReleaseControl()
	if _, ok := s.CalibrationSet(c.now()); ok {
		t.Fatal("set sent while observing")
	}
	if s.Snapshot().PendingCount != 0 {
		t.Fatal("pending kept after release")
	}
}

func TestLogAndFrames(t *testing.T) {
	s, _ := connected(t)
	for i := 0; i < MaxLogLines+5; i++ {
		s.Line(link.Line{Text: "line"}, link.Stats{})
	}
	s.Line(link.Line{Text: `bad\xA5`, Stray: 1}, link.Stats{Stray: 1})
	s.Frame(telemetryFrame(protocol.Telemetry{}), link.Stats{Valid: 1, Stray: 1})
	snap := s.Snapshot()
	if len(snap.Log) != MaxLogLines || snap.Log[len(snap.Log)-1].Stray != 1 || snap.StrayLines != 1 || snap.Stats.Stray != 1 {
		t.Fatalf("log %d stray %d", len(snap.Log), snap.StrayLines)
	}
	if len(snap.Frames) != 1 || snap.Frames[0].Text[:9] != "telemetry" {
		t.Fatalf("frames %+v", snap.Frames)
	}
}

func TestOnChangeIsCalledWithoutTheLock(t *testing.T) {
	var s *Session
	calls := 0
	s = New(newClock().now, func() {
		calls++
		_ = s.Snapshot() // would deadlock if the lock were held
	})
	s.LinkState(link.State{Status: link.Connected})
	s.ToggleEmergencyStop()
	if calls != 2 {
		t.Fatalf("calls %d", calls)
	}
}

// Spec "Pose frames stop": the pose is fresh for a second after it
// arrives, and a new connection forgets it.
func TestPoseAndItsAge(t *testing.T) {
	s, c := connected(t)
	if snap := s.Snapshot(); snap.HavePose || snap.PoseFresh {
		t.Fatal("pose before any frame")
	}
	var p protocol.Pose
	p.Flags = protocol.PoseOutputsOn
	p.Joints[4] = protocol.PoseJoint{CommandedCentideg: 9000, ServoCentideg: 16500, PulseUs: 2333}
	s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: p}, link.Stats{})
	if snap := s.Snapshot(); !snap.HavePose || !snap.PoseFresh || snap.Pose != p || snap.PoseAge != 0 {
		t.Fatalf("snapshot %+v", snap)
	}
	c.advance(StaleAfter)
	if !s.Snapshot().PoseFresh {
		t.Fatal("stale at exactly a second")
	}
	c.advance(time.Millisecond)
	if snap := s.Snapshot(); !snap.HavePose || snap.PoseFresh || snap.PoseAge != StaleAfter+time.Millisecond {
		t.Fatalf("not stale: %v", snap.PoseAge)
	}
	s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: p}, link.Stats{})
	s.LinkState(link.State{Status: link.Waiting, Port: device})
	s.LinkState(link.State{Status: link.Connected, Port: device})
	if s.Snapshot().HavePose {
		t.Fatal("pose kept across connections")
	}
}
