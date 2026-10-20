package session

import (
	"errors"
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// standing returns a connected session in control with Stand requested.
func standing(t *testing.T) *Session {
	t.Helper()
	s, _ := connected(t)
	if err := s.TakeControl(); err != nil {
		t.Fatal(err)
	}
	if err := s.RequestMode(RequestStand); err != nil {
		t.Fatal(err)
	}
	return s
}

func input(c protocol.Command) [3]int8 { return [3]int8{c.TranslateX, c.TranslateY, c.Rotate} }

// Spec "Walking forward": W requests Walk once and walks; releasing it
// keeps Walk with zero input.
func TestDrivingForwardRequestsWalkOnce(t *testing.T) {
	s := standing(t)
	before := s.Command().ModeRequestCounter
	if err := s.SetDrive(Drive{Forward: true}); err != nil {
		t.Fatal(err)
	}
	c := s.Command()
	if c.Mode != uint8(RequestWalk) || c.ModeRequestCounter != before+1 || input(c) != [3]int8{0, 100, 0} {
		t.Fatalf("driving forward: %+v", c)
	}
	_ = s.SetDrive(Drive{Forward: true, SteerRight: true})
	if s.Command().ModeRequestCounter != before+1 {
		t.Fatal("walk requested again while walking")
	}
	_ = s.SetDrive(Drive{})
	c = s.Command()
	if c.Mode != uint8(RequestWalk) || input(c) != [3]int8{} {
		t.Fatalf("released: %+v", c)
	}
	// Stand ends walking; translate and rotate stay zero there.
	_ = s.RequestMode(RequestStand)
	_ = s.SetDrive(Drive{Back: true})
	if c = s.Command(); c.Mode != uint8(RequestWalk) || input(c) != [3]int8{0, -100, 0} {
		t.Fatalf("walking again from Stand: %+v", c)
	}
}

// Specs "Steering and turning on the spot" and "Reversing mirrors the
// steering", and stepping sideways.
func TestDriveMapping(t *testing.T) {
	cases := []struct {
		d    Drive
		want [3]int8
	}{
		{Drive{Forward: true, SteerRight: true}, [3]int8{0, 100, -100}},
		{Drive{Forward: true, SteerLeft: true}, [3]int8{0, 100, 100}},
		{Drive{SteerRight: true}, [3]int8{0, 0, -100}}, // on the spot
		{Drive{Back: true, SteerRight: true}, [3]int8{0, -100, 100}},
		{Drive{Back: true, SteerLeft: true}, [3]int8{0, -100, -100}},
		{Drive{StepLeft: true}, [3]int8{-100, 0, 0}},
		{Drive{StepRight: true, Forward: true}, [3]int8{100, 100, 0}},
		{Drive{Forward: true, Back: true, SteerLeft: true, SteerRight: true}, [3]int8{}}, // opposites cancel
	}
	for _, tc := range cases {
		s := standing(t)
		_ = s.SetDrive(tc.d)
		if got := input(s.Command()); got != tc.want {
			t.Errorf("%+v: got %v, want %v", tc.d, got, tc.want)
		}
	}
}

// The drive keys never pull the robot out of Sleep or Calibrate.
func TestDriveKeysIgnoredInSleepAndCalibrate(t *testing.T) {
	for _, m := range []RequestedMode{RequestSleep, RequestCalibrate} {
		s := standing(t)
		_ = s.RequestMode(m)
		counter := s.Command().ModeRequestCounter
		_ = s.SetDrive(Drive{Forward: true})
		c := s.Command()
		if c.Mode != uint8(m) || c.ModeRequestCounter != counter || input(c) != [3]int8{} {
			t.Fatalf("%v: %+v", m, c)
		}
	}
}

// Spec "Drive keys while observing".
func TestDriveKeysWhileObserving(t *testing.T) {
	s, _ := connected(t)
	before := s.Command()
	if err := s.SetDrive(Drive{Forward: true}); !errors.Is(err, ErrNotInControl) {
		t.Fatalf("drive while observing: %v", err)
	}
	if err := s.StepSpeed(1); !errors.Is(err, ErrNotInControl) {
		t.Fatalf("speed while observing: %v", err)
	}
	if s.Command() != before {
		t.Fatal("frame changed while observing")
	}
}

// Releasing control or losing the link drops the held keys.
func TestReleaseAndDisconnectDropTheDrive(t *testing.T) {
	s := standing(t)
	_ = s.SetDrive(Drive{Forward: true})
	s.ReleaseControl()
	if s.Snapshot().Drive != (Drive{}) {
		t.Fatal("drive kept after release")
	}
	_ = s.TakeControl()
	_ = s.RequestMode(RequestStand)
	_ = s.SetDrive(Drive{Forward: true})
	s.LinkState(link.State{Status: link.Disconnected})
	s.LinkState(link.State{Status: link.Connected, Port: device})
	_ = s.TakeControl()
	if c := s.Command(); input(c) != [3]int8{} || s.Snapshot().Drive != (Drive{}) {
		t.Fatalf("drive kept after a disconnect: %+v", c)
	}
}

// Spec "Settings": speed, height, gait and stride in every frame in control.
func TestWalkingSettings(t *testing.T) {
	s := standing(t)
	c := s.Command()
	if c.Speed != 44 || c.Height != 0 || c.Gait != 0 || c.Stride != 0 {
		t.Fatalf("defaults: %+v", c)
	}
	_ = s.StepSpeed(1)
	_ = s.StepSpeed(1)
	_ = s.StepHeight(1)
	_ = s.NextGait()
	_ = s.ToggleStride()
	c = s.Command()
	if c.Speed != 64 || c.Height != 10 || GaitName(c.Gait) != "Wave" || StrideName(c.Stride) != "fixed" {
		t.Fatalf("after the keys: %+v", c)
	}
	snap := s.Snapshot()
	if snap.Speed != 64 || snap.Height != 10 || snap.Gait != 1 || snap.Stride != 1 {
		t.Fatalf("snapshot: %+v", snap)
	}
	// Clamped to 0-100, and the gait wraps.
	for i := 0; i < 20; i++ {
		_ = s.StepSpeed(-1)
		_ = s.StepHeight(1)
	}
	for i := 0; i < 5; i++ {
		_ = s.NextGait()
	}
	if c = s.Command(); c.Speed != 0 || c.Height != 100 || c.Gait != 0 {
		t.Fatalf("clamped: %+v", c)
	}
	// Observing sends the defaults again.
	s.ReleaseControl()
	if c = s.Command(); c.Speed != 44 || c.Height != 0 || c.Gait != 0 || c.Stride != 0 {
		t.Fatalf("observing: %+v", c)
	}
}

// A key still held from before Stand, or a release, never requests Walk.
func TestOnlyAFreshPressRequestsWalk(t *testing.T) {
	s := standing(t)
	_ = s.SetDrive(Drive{Forward: true, SteerRight: true})
	_ = s.RequestMode(RequestStand)
	counter := s.Command().ModeRequestCounter
	_ = s.SetDrive(Drive{Forward: true})
	if c := s.Command(); c.Mode != uint8(RequestStand) || c.ModeRequestCounter != counter {
		t.Fatalf("a release requested Walk: %+v", c)
	}
	_ = s.SetDrive(Drive{Forward: true, StepLeft: true})
	if c := s.Command(); c.Mode != uint8(RequestWalk) || c.ModeRequestCounter != counter+1 {
		t.Fatalf("a fresh press did not: %+v", c)
	}
}
