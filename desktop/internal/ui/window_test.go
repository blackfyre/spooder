package ui

import (
	"image"
	"sync"
	"testing"
	"time"

	"gioui.org/io/input"
	"gioui.org/io/key"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/unit"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// The window is driven without a display, as bopen's tests do: frames are
// laid out against an input router.

var clock = time.Date(2026, 10, 18, 10, 0, 0, 0, time.UTC)

func nextFrameTime() time.Time {
	clock = clock.Add(16 * time.Millisecond)
	return clock
}

type fakeLink struct {
	mu        sync.Mutex
	connected []link.PortInfo
	released  int
}

func (f *fakeLink) Connect(d link.PortInfo) {
	f.mu.Lock()
	f.connected = append(f.connected, d)
	f.mu.Unlock()
}
func (f *fakeLink) Disconnect() { f.mu.Lock(); f.released++; f.mu.Unlock() }
func (f *fakeLink) Close()      { f.Disconnect() }

var servo = link.PortInfo{Name: "/dev/ttyACM0", IsUSB: true, VID: "2E8A", PID: "000A", SerialNumber: "S1"}

func testWindow(t *testing.T, dark bool) (*window, *input.Router, *fakeLink) {
	t.Helper()
	s := session.New(func() time.Time { return clock }, nil)
	fl := &fakeLink{}
	w, err := newWindow(Env{
		Session:    s,
		Link:       fl,
		Ports:      func() ([]link.PortInfo, error) { return []link.PortInfo{{Name: "/dev/ttyS0"}, servo}, nil },
		Checkouts:  []string{checkoutDir(t)},
		Appearance: appearance.Settings{Dark: dark},
	})
	if err != nil {
		t.Fatal(err)
	}
	ports, _ := w.env.Ports()
	w.setPorts(ports)
	router := new(input.Router)
	frame(w, router)
	return w, router, fl
}

func frame(w *window, router *input.Router) {
	var ops op.Ops
	gtx := layout.Context{Ops: &ops, Metric: unit.Metric{PxPerDp: 1, PxPerSp: 1},
		Constraints: layout.Exact(image.Pt(1360, 900)), Now: nextFrameTime(), Source: router.Source()}
	w.frame(gtx)
	router.Frame(&ops)
}

func press(w *window, router *input.Router, name key.Name, mods key.Modifiers) {
	router.Queue(key.Event{Name: name, Modifiers: mods, State: key.Press})
	frame(w, router)
	router.Queue(key.Event{Name: name, Modifiers: mods, State: key.Release})
	frame(w, router)
}

// pressTab delivers Tab as app.Window does: as a system event, moving the
// focus to the next button unless a handler takes it.
func pressTab(w *window, router *input.Router) {
	router.WakeupTime() // the window reads it after every frame
	router.Queue(input.SystemEvent{Event: key.Event{Name: key.NameTab, State: key.Press}})
	if _, handled := router.WakeupTime(); !handled {
		router.MoveFocus(key.FocusForward)
	}
	frame(w, router)
	router.Queue(key.Event{Name: key.NameTab, State: key.Release})
	frame(w, router)
}

// click activates a button as a pointer click would, and runs a frame.
func click(w *window, router *input.Router, c interface{ Click() }) {
	c.Click()
	frame(w, router)
	frame(w, router)
}

// calibrating brings the window's session into control in Calibrate, with
// a report, as the robot would.
func calibrating(t *testing.T, w *window, router *input.Router) {
	t.Helper()
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	click(w, router, &w.control)
	click(w, router, &w.modes[1])
	if c := s.Command(); c.Flags&protocol.FlagControllerConnected == 0 || c.Mode != uint8(session.RequestCalibrate) {
		t.Fatalf("command %+v", c)
	}
	s.Frame(protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: protocol.Telemetry{BodyMode: uint8(session.BodyCalibrate),
		OutputsOn: true, VoltageMv: 6000, Flags: protocol.TelemetryUSBInControl}}, link.Stats{})
	r := protocol.CalibrationReport{Result: uint8(protocol.ResultNoneYet)}
	for i := range r.Joints {
		r.Joints[i] = protocol.CalibrationJoint{OffsetCentideg: 9000, Pulse0Us: 500, Pulse180Us: 2500}
	}
	s.Frame(protocol.Frame{Type: protocol.TypeCalibrationReport, CalibrationReport: r}, link.Stats{})
	frame(w, router)
	if !w.snap.Editable {
		t.Fatal("not editable")
	}
}

func TestServo2040IsPreselectedAndConnects(t *testing.T) {
	w, router, fl := testWindow(t, false)
	if w.portSel != 1 {
		t.Fatalf("preselected %d", w.portSel)
	}
	click(w, router, &w.connect)
	if len(fl.connected) != 1 || fl.connected[0] != servo {
		t.Fatalf("connected %+v", fl.connected)
	}
	click(w, router, &w.connect)
	if fl.released != 1 || w.device != nil {
		t.Fatal("not disconnected")
	}
}

func TestExplicitPortIsPreselectedAheadOfAServo2040(t *testing.T) {
	explicit := link.PortInfo{Name: "/tmp/virtual-robot/usb"}
	fl := &fakeLink{}
	w, err := newWindow(Env{
		Session: session.New(func() time.Time { return clock }, nil),
		Link:    fl,
		Ports:   func() ([]link.PortInfo, error) { return []link.PortInfo{servo, explicit}, nil },
		Port:    explicit.Name,
	})
	if err != nil {
		t.Fatal(err)
	}
	ports, _ := w.env.Ports()
	w.setPorts(ports)
	if w.portSel != 1 {
		t.Fatalf("preselected %d", w.portSel)
	}
	router := new(input.Router)
	frame(w, router)
	click(w, router, &w.connect)
	if len(fl.connected) != 1 || fl.connected[0] != explicit {
		t.Fatalf("connected %+v", fl.connected)
	}
	click(w, router, &w.connect) // disconnect
	// Absent (the virtual robot not running): the Servo 2040 instead.
	w.setPorts([]link.PortInfo{servo})
	if w.portSel != 0 {
		t.Fatalf("preselected %d", w.portSel)
	}
	// Once it appears, it is preselected again over the Servo 2040.
	w.setPorts([]link.PortInfo{servo, {Name: "/dev/ttyS0"}, explicit})
	if w.portSel != 2 {
		t.Fatalf("preselected %d after the path appeared", w.portSel)
	}
	// The operator's own pick is kept.
	w.portClicks[1].Click()
	frame(w, router)
	w.setPorts([]link.PortInfo{servo, {Name: "/dev/ttyS0"}, explicit, {Name: "/dev/ttyS1"}})
	if w.portSel != 1 {
		t.Fatalf("pick lost: %d", w.portSel)
	}
}

// Space toggles the e-stop whether or not the app is in control, and even
// after Tab presses (which would otherwise focus a button).
func TestSpaceIsTheEmergencyStop(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	press(w, router, key.NameSpace, 0)
	if c := s.Command(); c.Flags != protocol.FlagEmergencyStop {
		t.Fatalf("observing: %+v", c)
	}
	for i := 0; i < 5; i++ {
		pressTab(w, router)
		src := router.Source()
		focused := src.Focused(&w.refresh) || src.Focused(&w.connect) || src.Focused(&w.control) || src.Focused(&w.estop)
		for i := range w.portClicks {
			focused = focused || src.Focused(&w.portClicks[i])
		}
		if focused {
			t.Fatal("Tab moved the keyboard focus onto a button")
		}
	}
	press(w, router, key.NameSpace, 0)
	if s.Command().Flags&protocol.FlagEmergencyStop != 0 {
		t.Fatal("space after tab did not release the e-stop")
	}
	click(w, router, &w.control)
	press(w, router, key.NameSpace, 0)
	if c := s.Command(); c.Flags != protocol.FlagControllerConnected|protocol.FlagEmergencyStop {
		t.Fatalf("in control: %+v", c)
	}
	click(w, router, &w.estop)
	if s.Command().Flags&protocol.FlagEmergencyStop != 0 {
		t.Fatal("button did not release the e-stop")
	}
}

// Review finding: holding Space (key repeat) must not toggle the e-stop
// back; a new press after a lost release still counts.
func TestHeldSpaceTogglesOnce(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	router.Queue(key.Event{Name: key.NameSpace, State: key.Press})
	frame(w, router)
	for i := 0; i < 10; i++ { // repeats while held
		router.Queue(key.Event{Name: key.NameSpace, State: key.Press})
		frame(w, router)
	}
	if s.Command().Flags&protocol.FlagEmergencyStop == 0 {
		t.Fatal("key repeat released the e-stop")
	}
	router.Queue(key.Event{Name: key.NameSpace, State: key.Release})
	frame(w, router)
	press(w, router, key.NameSpace, 0)
	if s.Command().Flags&protocol.FlagEmergencyStop != 0 {
		t.Fatal("second press did not release")
	}
	// However long the key is held, repeats never toggle it back.
	router.Queue(key.Event{Name: key.NameSpace, State: key.Press})
	frame(w, router)
	clock = clock.Add(5 * time.Second)
	router.Queue(key.Event{Name: key.NameSpace, State: key.Press})
	frame(w, router)
	if s.Command().Flags&protocol.FlagEmergencyStop == 0 {
		t.Fatal("a slow repeat released the e-stop")
	}
	// Released while Ctrl is held: the release still reaches the latch.
	router.Queue(key.Event{Name: key.NameSpace, Modifiers: key.ModCtrl, State: key.Release})
	frame(w, router)
	press(w, router, key.NameSpace, 0)
	if s.Command().Flags&protocol.FlagEmergencyStop != 0 {
		t.Fatal("press after a release with Ctrl ignored")
	}
	press(w, router, key.NameSpace, 0)
	// The window loses the focus while Space is down, so its release is
	// lost: the next press still counts.
	w.focusChanged(false)
	router.Queue(key.Event{Name: key.NameSpace, State: key.Press})
	frame(w, router)
	if s.Command().Flags&protocol.FlagEmergencyStop != 0 {
		t.Fatal("press after a lost release ignored")
	}
}

// Review finding: refreshing an unchanged port list keeps the buttons, so a
// press in progress is not lost.
func TestPortRefreshKeepsButtons(t *testing.T) {
	w, _, _ := testWindow(t, false)
	before := &w.portClicks[0]
	ports, _ := w.env.Ports()
	w.setPorts(ports)
	if &w.portClicks[0] != before {
		t.Fatal("buttons recreated for an unchanged list")
	}
	w.setPorts(ports[1:])
	if len(w.portClicks) != 1 || w.portSel != 0 {
		t.Fatalf("changed list: %d buttons, selected %d", len(w.portClicks), w.portSel)
	}
}

// Spec "Observe only": the mode buttons do nothing until control is taken.
func TestModeButtonsNeedControl(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	frame(w, router)
	click(w, router, &w.modes[0])
	click(w, router, &w.clearFault)
	if c := s.Command(); c.ModeRequestCounter != 0 || c.ClearCounter != 0 || c.Flags != 0 {
		t.Fatalf("observing: %+v", c)
	}
	click(w, router, &w.control)
	click(w, router, &w.modes[0])
	if c := s.Command(); c.ModeRequestCounter != 1 || c.Mode != uint8(session.RequestStand) {
		t.Fatalf("stand: %+v", c)
	}
	click(w, router, &w.control) // release
	if c := s.Command(); c.Flags != 0 || c.Mode != uint8(session.RequestSleep) {
		t.Fatalf("released: %+v", c)
	}
}

// The mode shortcuts act like the buttons: nothing while observing.
func TestModeKeys(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	frame(w, router)
	press(w, router, "2", 0)
	press(w, router, "F", 0)
	if c := s.Command(); c.ModeRequestCounter != 0 || c.ClearCounter != 0 {
		t.Fatalf("observing: %+v", c)
	}
	click(w, router, &w.control)
	for i, want := range modeButtons {
		press(w, router, modeKeys[i], 0)
		if c := s.Command(); c.ModeRequestCounter != uint8(i+1) || c.Mode != uint8(want) {
			t.Fatalf("key %s: %+v", modeKeys[i], c)
		}
	}
	press(w, router, clearFaultKey, 0)
	if c := s.Command(); c.ClearCounter != 1 {
		t.Fatalf("clear fault: %+v", c)
	}
}

// Spec "Gesture controls": keys 4-8 and the buttons request gestures, only
// in control.
func TestGestureKeysAndButtons(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	frame(w, router)
	press(w, router, "5", 0)
	if c := s.Command(); c.GestureRequestCounter != 0 {
		t.Fatalf("observing: %+v", c)
	}
	click(w, router, &w.control)
	for i, g := range gestureButtons {
		press(w, router, gestureKeys[i], 0)
		if c := s.Command(); c.Gesture != g.id || c.GestureRequestCounter != uint8(i+1) {
			t.Fatalf("key %s: %+v", gestureKeys[i], c)
		}
	}
	// A held key repeats presses: still one request until it is released.
	before := s.Command().GestureRequestCounter
	for i := 0; i < 5; i++ {
		router.Queue(key.Event{Name: gestureKeys[0], State: key.Press})
		frame(w, router)
	}
	if c := s.Command(); c.GestureRequestCounter != before+1 {
		t.Fatalf("key repeat: %d requests", c.GestureRequestCounter-before)
	}
	router.Queue(key.Event{Name: gestureKeys[0], State: key.Release})
	frame(w, router)
	press(w, router, gestureKeys[0], 0)
	if c := s.Command(); c.GestureRequestCounter != before+2 {
		t.Fatalf("after release: %d requests", c.GestureRequestCounter-before)
	}
	click(w, router, &w.gestures[1])
	if c := s.Command(); c.Gesture != protocol.GestureWiggle || c.GestureRequestCounter != uint8(len(gestureButtons)+3) {
		t.Fatalf("wiggle button: %+v", c)
	}
}

func TestKeyboardNudges(t *testing.T) {
	w, router, _ := testWindow(t, false)
	// Refused before Calibrate: a notice, no change.
	w.env.Session.LinkState(link.State{Status: link.Connected, Port: servo})
	press(w, router, "=", 0)
	if w.notice == "" || w.snap.PendingCount != 0 {
		t.Fatal("nudge outside Calibrate not refused")
	}
	calibrating(t, w, router)
	// Leg 3 tibia is joint 8: eight steps right, then down to the 0° pulse.
	for i := 0; i < 8; i++ {
		press(w, router, key.NameRightArrow, 0)
	}
	press(w, router, key.NameDownArrow, 0)
	press(w, router, "=", 0)
	v := w.snap.Joints[8]
	if !v.Pending || v.Wanted.Pulse0Us != 510 || v.Reported.Pulse0Us != 500 {
		t.Fatalf("joint 8 %+v", v)
	}
	press(w, router, "_", key.ModShift)
	if v := w.snap.Joints[8]; v.Wanted.Pulse0Us != 460 {
		t.Fatalf("coarse down: %+v", v)
	}
	press(w, router, "R", 0)
	if v := w.snap.Joints[8]; !v.Wanted.Reversed {
		t.Fatal("R did not flip")
	}
}

func TestPointerNudgesAndFlip(t *testing.T) {
	w, router, _ := testWindow(t, true)
	calibrating(t, w, router)
	click(w, router, &w.nudges[0][int(calib.FieldOffset)][3]) // +5°
	if v := w.snap.Joints[0]; v.Wanted.OffsetCentideg != 9500 || w.sel.joint != 0 {
		t.Fatalf("joint 0 %+v", v)
	}
	click(w, router, &w.flips[17])
	if v := w.snap.Joints[17]; !v.Wanted.Reversed || w.sel.joint != 17 || w.sel.field != calib.FieldDirection {
		t.Fatalf("joint 17 %+v", v)
	}
}

// Every pane lays out in both palettes and in every state without
// panicking.
func TestLayoutsInEveryState(t *testing.T) {
	for _, dark := range []bool{false, true} {
		w, router, _ := testWindow(t, dark)
		for tab := 0; tab < tabCount; tab++ {
			w.tab = tab
			frame(w, router)
		}
		calibrating(t, w, router)
		s := w.env.Session
		s.Line(link.Line{Text: `boot \xA5`, Stray: 1}, link.Stats{Stray: 1})
		s.ToggleEmergencyStop()
		s.Frame(protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: protocol.Telemetry{BodyMode: uint8(session.BodyCalibrate),
			Faults: 0x24, Flags: protocol.TelemetrySourceConflict}}, link.Stats{})
		_ = s.Nudge(4, calib.FieldPulse180, 1, false)
		for tab := 0; tab < tabCount; tab++ {
			w.tab = tab
			frame(w, router)
		}
		click(w, router, &w.prepare)
		if w.exp.plan == nil {
			t.Fatalf("no plan: %s", w.exp.status)
		}
		frame(w, router)
		clock = clock.Add(2 * time.Second) // telemetry goes stale
		frame(w, router)
		if w.snap.Fresh {
			t.Fatal("still fresh")
		}
		s.LinkState(link.State{Status: link.Waiting, Port: servo, Err: link.ErrLost})
		frame(w, router)
	}
}
