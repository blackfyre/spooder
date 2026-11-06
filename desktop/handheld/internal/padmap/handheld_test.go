package padmap

import (
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// rig steps a Handheld at 20 ms, as the pak's sender does.
type rig struct {
	t    *testing.T
	h    *Handheld
	now  uint32
	c    Controls
	last Desired
}

func newRig(t *testing.T) *rig {
	r := &rig{t: t, h: NewHandheld(), now: 1000}
	r.h.Begin(r.now)
	r.step()
	return r
}

func (r *rig) step() Desired {
	r.now += 20
	r.last = r.h.Step(r.c, r.now)
	if !r.last.ControllerConnected {
		r.t.Fatalf("t=%d: controller not reported connected", r.now)
	}
	return r.last
}

// hold steps for ms milliseconds with the controls as they are.
func (r *rig) hold(ms uint32) Desired {
	for end := r.now + ms; r.now < end; {
		r.step()
	}
	return r.last
}

func (r *rig) down(b Buttons) Desired { r.c.Buttons |= b; return r.step() }
func (r *rig) up(b Buttons) Desired   { r.c.Buttons &^= b; return r.step() }
func (r *rig) tap(b Buttons) Desired  { r.down(b); return r.up(b) }

func (r *rig) want(mode RequestedMode, counter uint8) {
	r.t.Helper()
	if r.last.Mode != mode || r.last.ModeRequestCounter != counter {
		r.t.Fatalf("t=%d: mode %d counter %d, want mode %d counter %d", r.now, r.last.Mode,
			r.last.ModeRequestCounter, mode, counter)
	}
}

func TestStartsInSleepAndYStands(t *testing.T) {
	r := newRig(t)
	r.want(ModeSleep, 1) // Begin's Sleep request, the session's baseline frame
	r.tap(HandY)
	r.want(ModeStand, 2)
}

func TestStandAndWalk(t *testing.T) {
	r := newRig(t)
	r.tap(HandY)
	r.tap(HandStart)
	r.want(ModeWalk, 3)
}

func TestControlsHeldAtBeginAreNotPresses(t *testing.T) {
	h := NewHandheld()
	h.Begin(1000)
	held := Controls{Buttons: HandStart | HandSelect | HandY}
	for now := uint32(1020); now <= 2200; now += 20 {
		d := h.Step(held, now)
		if d.Mode != ModeSleep || d.ModeRequestCounter != 1 || d.ClearCounter != 0 || h.EmergencyStop() {
			t.Fatalf("t=%d: %+v, e-stop %v", now, d, h.EmergencyStop())
		}
	}
}

func TestGestureLayerOnAnalogueTrigger(t *testing.T) {
	r := newRig(t)
	r.c.TriggerL = 16383 // not beyond half travel
	r.step()
	r.tap(HandA)
	if r.last.GestureRequestCounter != 0 {
		t.Fatalf("gesture at 16383: %+v", r.last)
	}
	r.c.TriggerL = 16385
	r.step()
	before := r.last
	r.down(HandA)
	if r.last.Gesture != protocol.GestureSayHi || r.last.GestureRequestCounter != 1 {
		t.Fatalf("no say-hi at 16385: %+v", r.last)
	}
	after := r.last
	after.Gesture, after.GestureRequestCounter = before.Gesture, before.GestureRequestCounter
	if after != before {
		t.Fatalf("A did more than the gesture: %+v, before %+v", r.last, before)
	}
	// Y in the layer is a gesture, not a Sleep toggle.
	r.tap(HandY)
	if r.last.Gesture != protocol.GestureAttack || r.last.Mode != ModeSleep || r.last.ModeRequestCounter != 1 {
		t.Fatalf("Y in the layer: %+v", r.last)
	}
	// R2 beyond half travel is the ball gesture.
	r.c.TriggerR = 16385
	r.step()
	if r.last.Gesture != protocol.GestureBall {
		t.Fatalf("R2 in the layer: %+v", r.last)
	}
}

func TestEmergencyStopToggle(t *testing.T) {
	r := newRig(t)
	r.tap(HandY)
	r.want(ModeStand, 2)
	r.down(HandSelect)
	r.down(HandStart)
	if !r.h.EmergencyStop() {
		t.Fatal("Select + Start did not stop")
	}
	r.up(HandStart)
	r.up(HandSelect)
	for range 10 {
		if c := r.h.Command(r.step()); c.Flags&protocol.FlagEmergencyStop == 0 {
			t.Fatal("a frame without the e-stop")
		}
	}
	r.want(ModeStand, 2)
	r.down(HandSelect)
	r.tap(HandStart)
	r.up(HandSelect)
	if r.h.EmergencyStop() || r.h.Command(r.last).Flags&protocol.FlagEmergencyStop != 0 {
		t.Fatal("second Select + Start did not release")
	}
	r.want(ModeStand, 2)
}

// Select + X switches the page and leaves the stride alone, however the
// buttons are released; X alone still toggles the stride.
func TestDriveViewToggle(t *testing.T) {
	r := newRig(t)
	stride := r.last.Stride
	r.down(HandSelect)
	r.down(HandX)
	if !r.h.DriveView() {
		t.Fatal("Select + X did not show the driving view")
	}
	r.up(HandSelect) // released while X is still held
	r.hold(100)
	r.up(HandX)
	if r.last.Stride != stride || !r.h.DriveView() {
		t.Fatalf("stride %d (was %d), driving view %v", r.last.Stride, stride, r.h.DriveView())
	}
	r.down(HandSelect)
	r.tap(HandX)
	r.up(HandSelect)
	if r.h.DriveView() || r.last.Stride != stride {
		t.Fatalf("second Select + X: driving view %v, stride %d", r.h.DriveView(), r.last.Stride)
	}
	r.tap(HandX)
	if r.last.Stride == stride {
		t.Fatal("X alone did not toggle the stride")
	}
}

func TestSelectReleasedFirst(t *testing.T) {
	for _, b := range []Buttons{HandStart, HandY} {
		r := newRig(t)
		r.tap(HandY)
		r.want(ModeStand, 2)
		r.down(HandSelect)
		r.down(b)
		r.up(HandSelect) // released while b is still held
		r.want(ModeStand, 2)
		r.hold(200)
		r.want(ModeStand, 2)
		r.up(b)
		r.want(ModeStand, 2)
		r.hold(100)
		r.want(ModeStand, 2)
		// b works again once released.
		r.tap(b)
		if r.last.ModeRequestCounter != 3 {
			t.Fatalf("button %d not unmasked after release: %+v", b, r.last)
		}
	}
}

func TestSelectPressedWhileHeld(t *testing.T) {
	// Start held, then Select: no e-stop, and Start does not act again.
	r := newRig(t)
	r.tap(HandY)
	r.down(HandStart)
	r.want(ModeWalk, 3)
	r.down(HandSelect)
	r.up(HandSelect)
	r.up(HandStart)
	r.want(ModeWalk, 3)
	if r.h.EmergencyStop() {
		t.Fatal("Select after Start toggled the e-stop")
	}
	// Y held, then Select held 1.5 s: no fault clear, no further toggle.
	r.down(HandY)
	r.want(ModeSleep, 4)
	r.down(HandSelect)
	r.hold(1500)
	r.up(HandSelect)
	r.up(HandY)
	r.want(ModeSleep, 4)
	if r.last.ClearCounter != 0 {
		t.Fatal("Select after Y requested a fault clear")
	}
}

func TestFaultClearByHold(t *testing.T) {
	r := newRig(t)
	r.down(HandSelect)
	r.down(HandY)
	r.hold(960)
	if r.last.ClearCounter != 0 {
		t.Fatalf("clear before 1 s: %+v", r.last)
	}
	r.hold(60)
	if r.last.ClearCounter != 1 {
		t.Fatalf("no clear at 1 s: %+v", r.last)
	}
	r.hold(2000)
	r.up(HandY)
	r.up(HandSelect)
	if r.last.ClearCounter != 1 {
		t.Fatalf("held longer advanced the clear: %+v", r.last)
	}
	r.want(ModeSleep, 1)
}

func TestShortSelectY(t *testing.T) {
	r := newRig(t)
	r.down(HandSelect)
	r.down(HandY)
	r.hold(500)
	r.up(HandY)
	r.up(HandSelect)
	r.hold(1000)
	if r.last.ClearCounter != 0 {
		t.Fatalf("clear after 0.5 s: %+v", r.last)
	}
	r.want(ModeSleep, 1)
}

// Every combination of the buttons, held 1.2 s and released, never requests
// Calibrate: the layer never sets miscBack.
func TestNoCalibrate(t *testing.T) {
	const all = HandSelect<<1 - 1 // every Buttons bit up to Select
	for combo := Buttons(1); combo <= all; combo++ {
		h := NewHandheld()
		h.Begin(0)
		now := uint32(0)
		for _, c := range []Controls{{Buttons: 0}, {Buttons: combo, TriggerL: 32767, TriggerR: 32767}} {
			for range 60 {
				now += 20
				if d := h.Step(c, now); d.Mode == ModeCalibrate {
					t.Fatalf("combination %#x requested Calibrate", combo)
				}
			}
		}
	}
}

func TestNeutralDuringLeave(t *testing.T) {
	r := newRig(t)
	r.tap(HandY)
	r.tap(HandStart)
	r.want(ModeWalk, 3)
	r.h.RequestSleep()
	r.h.SetNeutral(true)
	r.step()
	r.want(ModeSleep, 4)
	r.tap(HandY)
	r.tap(HandStart)
	r.c.LeftY = -32768
	r.c.TriggerL = 32767
	r.down(HandA)
	r.hold(200)
	if r.last.Mode != ModeSleep || r.last.ModeRequestCounter != 4 || r.last.TranslateY != 0 ||
		r.last.GestureRequestCounter != 0 {
		t.Fatalf("a control acted during the leave: %+v", r.last)
	}
	// Select + Start still toggles the e-stop, outside the mapping.
	r.down(HandSelect)
	r.tap(HandStart)
	r.up(HandSelect)
	if !r.h.EmergencyStop() {
		t.Fatal("no e-stop during the leave")
	}
	// Back to driving (after a suspend) with Y and A still held: not presses.
	r.down(HandY)
	r.h.SetNeutral(false)
	r.step()
	r.hold(100)
	if r.last.Mode != ModeSleep || r.last.ModeRequestCounter != 4 || r.last.GestureRequestCounter != 0 {
		t.Fatalf("a held control acted after the leave: %+v", r.last)
	}
	// The stick is live again (it has no presses to mask).
	if r.last.TranslateY != 100 {
		t.Fatalf("stick after the leave: %+v", r.last)
	}
	r.up(HandY)
	// Y leaves the leave's Sleep for Stand, not for the Walk before it.
	r.tap(HandY)
	r.want(ModeStand, 5)
}

func TestRequestSleepOnceWhenAlreadyRequested(t *testing.T) {
	r := newRig(t)
	r.h.RequestSleep()
	r.step()
	r.want(ModeSleep, 1)
	// Asleep from Walk: a further request (a resume) changes no counter but
	// still makes Y return to Stand.
	r.tap(HandY)
	r.tap(HandStart)
	r.tap(HandY)
	r.want(ModeSleep, 4)
	r.h.RequestSleep()
	r.step()
	r.want(ModeSleep, 4)
	r.tap(HandY)
	r.want(ModeStand, 5)
}

// Every control reaches the mapping as its gamepad counterpart.
func TestControlsMapToGamepad(t *testing.T) {
	r := newRig(t)
	r.tap(HandY)
	r.want(ModeStand, 2)
	r.tap(HandR1)
	if r.last.Gait != 1 {
		t.Fatalf("R1: gait %d", r.last.Gait)
	}
	r.tap(HandL1)
	r.tap(HandL1)
	if r.last.Gait != GaitCount-1 {
		t.Fatalf("L1: gait %d", r.last.Gait)
	}
	r.tap(HandUp)
	if r.last.Height != 10 {
		t.Fatalf("Up: height %d", r.last.Height)
	}
	r.tap(HandDown)
	if r.last.Height != 0 {
		t.Fatalf("Down: height %d", r.last.Height)
	}
	r.tap(HandRight)
	if r.last.Speed != 54 {
		t.Fatalf("Right: speed %d", r.last.Speed)
	}
	r.tap(HandLeft)
	if r.last.Speed != 44 {
		t.Fatalf("Left: speed %d", r.last.Speed)
	}
	r.tap(HandX)
	if r.last.Stride != StrideFixed {
		t.Fatalf("X: stride %d", r.last.Stride)
	}
	// Steer: left stick X steers (right turns clockwise), right stick X
	// rotates.
	r.c.LeftX = 32767
	if d := r.step(); d.TranslateX != 0 || d.Rotate != -100 {
		t.Fatalf("steer right: %+v", d)
	}
	r.c.LeftX, r.c.RightX = 0, -32768
	if d := r.step(); d.Rotate != 100 {
		t.Fatalf("right stick left: %+v", d)
	}
	r.c.RightX = 0
	r.tap(HandB)
	if r.last.StickMode != StickCrab {
		t.Fatalf("B: stick mode %d", r.last.StickMode)
	}
	r.c.LeftX = 32767
	if d := r.step(); d.TranslateX != 100 || d.TranslateY != 0 || d.Rotate != 0 {
		t.Fatalf("crab right: %+v", d)
	}
	r.c.LeftX, r.c.RightY = 0, 32767
	if d := r.step(); d.TranslateX != 0 || d.Rotate != 0 {
		t.Fatalf("right stick Y: %+v", d)
	}
	r.c.RightY = 0
	r.tap(HandStart)
	r.want(ModeWalk, 3)
	r.tap(HandY)
	r.want(ModeSleep, 4)
	// L2 + B, X, Y and R2 are gestures, as on the gamepad.
	r.c.TriggerL = 32767
	r.step()
	for _, g := range []struct {
		b       Buttons
		gesture uint8
	}{{HandB, protocol.GestureWiggle}, {HandX, protocol.GestureDance}, {HandY, protocol.GestureAttack}} {
		r.tap(g.b)
		if r.last.Gesture != g.gesture {
			t.Fatalf("L2 + %#x: gesture %d", g.b, r.last.Gesture)
		}
	}
	if r.last.StickMode != StickCrab || r.last.Stride != StrideFixed || r.last.ModeRequestCounter != 4 {
		t.Fatalf("a gesture button kept its own function: %+v", r.last)
	}
}

func TestScaleAxis(t *testing.T) {
	for _, c := range []struct {
		in   int16
		want int32
	}{{-32768, -511}, {0, 0}, {32767, 512}, {-16384, -255}, {16384, 256}} {
		if got := ScaleAxis(c.in); got != c.want {
			t.Errorf("ScaleAxis(%d) = %d, want %d", c.in, got, c.want)
		}
	}
	// SDL's up (negative) is Bluepad32's up: full up walks forward.
	r := newRig(t)
	r.tap(HandY)
	r.c.LeftY = -32768
	if d := r.step(); d.TranslateY != 100 {
		t.Fatalf("full up: %+v", d)
	}
}
