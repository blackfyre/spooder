package e2e

import (
	"context"
	"fmt"
	"io"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
)

// Pak is the handheld's run loop as cmd/spooder-handheld starts it, without
// SDL: the flags, Start and Run, with the controls set by the test.
type Pak struct {
	t    *testing.T
	S    *app.Session
	done chan error
}

// StartPak starts a run against the virtual bridge with --addr.
func StartPak(t *testing.T, b *Bridge) *Pak {
	t.Helper()
	cfg, err := app.ParseFlags([]string{"--addr", b.Addr}, io.Discard)
	if err != nil {
		t.Fatal(err)
	}
	s, err := app.Start(context.Background(), cfg, app.OS())
	if err != nil {
		t.Fatal(err)
	}
	p := &Pak{t: t, S: s, done: make(chan error, 1)}
	ctx, cancel := context.WithCancel(context.Background())
	go func() { p.done <- s.Run(ctx) }()
	t.Cleanup(func() { cancel(); <-p.done; s.Close() })
	return p
}

// Tap presses and releases a button, held for 100 ms.
func (p *Pak) Tap(b padmap.Buttons) {
	p.S.App.SetControls(padmap.Controls{Buttons: b})
	time.Sleep(100 * time.Millisecond)
	p.S.App.SetControls(padmap.Controls{})
	time.Sleep(100 * time.Millisecond)
}

// The handheld's run loop drives the virtual robot through the virtual
// bridge: Y stands it, Start walks it, Menu leaves only once Sleep is
// confirmed, the stop ends the bridge's session, and a new run (a new
// socket) starts in Sleep.
func TestPakDrivesAndLeaves(t *testing.T) {
	b := StartBridge(t)
	p := StartPak(t, b)
	b.WaitSessions(1, 2*time.Second)
	time.Sleep(200 * time.Millisecond) // Begin masks controls held at the start
	if m := b.Mode(); !resting(m) {
		t.Fatalf("robot in %s at the start, want sleep (or initialise, before any request)", m)
	}

	p.Tap(padmap.HandY)
	b.WaitMode("stand", 10*time.Second)
	p.Tap(padmap.HandStart)
	b.WaitMode("walk", 10*time.Second)

	time.Sleep(1200 * time.Millisecond) // a whole second of counts
	sc := p.S.App.Screen(time.Now())
	if sc.Robot.Stale || sc.Robot.Body != "Walk" {
		t.Errorf("screen shows body %q (stale %v), want Walk", sc.Robot.Body, sc.Robot.Stale)
	}
	if sc.Link.Telemetry < 5 || sc.Link.Pose < 5 || sc.Link.Diagnostics < 5 {
		t.Errorf("frames per second: telemetry %d, pose %d, diagnostics %d, want about 10 each",
			sc.Link.Telemetry, sc.Link.Pose, sc.Link.Diagnostics)
	}
	if sc.Link.Lost != 0 || sc.Link.Invalid != 0 {
		t.Errorf("lost %d, invalid %d, want 0", sc.Link.Lost, sc.Link.Invalid)
	}
	// The driving view's pose: fresh, every leg placed, and Select + X shows it.
	if !sc.Pose.Have || sc.Pose.Stale {
		t.Errorf("pose have %v, stale %v, want a fresh pose", sc.Pose.Have, sc.Pose.Stale)
	}
	for leg, l := range sc.Pose.Legs {
		if !l.Set {
			t.Errorf("leg %d not placed while walking", leg)
		}
	}
	p.Tap(padmap.HandSelect | padmap.HandX)
	if !p.S.App.Screen(time.Now()).DriveView {
		t.Error("Select + X did not show the driving view")
	}

	pressed := time.Now()
	p.S.App.Press(app.ActionExit, pressed)
	var err error
	select {
	case err = <-p.done:
		p.done <- err // for the cleanup
	case <-time.After(5 * time.Second):
		t.Fatal("the run did not end after Menu")
	}
	took := time.Since(pressed)
	if err != nil {
		t.Fatal(err)
	}
	if p.S.App.Ended() != app.ActionExit {
		t.Fatalf("run ended with %v, want exit", p.S.App.Ended())
	}
	// Ended by the confirmation, not by the 3 s wait, and with the robot
	// asleep (body Sleep, or output off) when it ended.
	if took >= app.LeaveTimeout {
		t.Errorf("leave took %v: ended by the timeout, not by the robot's Sleep", took)
	}
	answer, derr := b.Do("status")
	if derr != nil {
		t.Fatal(derr)
	}
	if !strings.Contains(answer, "mode=sleep") && !strings.Contains(answer, "output=off") {
		t.Errorf("robot after the leave: %s, want Sleep or output off", answer)
	}
	s, _ := b.WaitSessions(2, time.Second)
	if s[1].Start {
		t.Fatalf("sessions %v, want the run's start and end", s)
	}

	// A new run, from a new socket: a new session, starting in Sleep.
	q := StartPak(t, b)
	s, _ = b.WaitSessions(3, 2*time.Second)
	if !s[2].Start || s[2].From == s[0].From {
		t.Fatalf("sessions %v, want a new start from a new port", s)
	}
	time.Sleep(500 * time.Millisecond)
	if m := b.Mode(); !resting(m) {
		t.Errorf("robot in %s in the new session, want sleep (or initialise, before any request)", m)
	}
	if sc := q.S.App.Screen(time.Now()); sc.Requested.Mode != "Sleep" {
		t.Errorf("new run requests %q, want Sleep", sc.Requested.Mode)
	}
}

// faults is the robot's latched fault bits from its status command.
func (b *Bridge) faults() uint64 {
	b.t.Helper()
	answer, err := b.Do("status")
	if err != nil {
		b.t.Fatal(err)
	}
	for _, f := range strings.Fields(answer) {
		if v, ok := strings.CutPrefix(f, "faults="); ok {
			var n uint64
			if _, err := fmt.Sscanf(v, "0x%x", &n); err != nil {
				b.t.Fatalf("faults in %q: %v", answer, err)
			}
			return n
		}
	}
	b.t.Fatalf("no faults in %q", answer)
	return 0
}

// waitFaults waits until want reports true for the robot's fault bits.
func (b *Bridge) waitFaults(what string, want func(uint64) bool, timeout time.Duration) {
	b.t.Helper()
	deadline := time.Now().Add(timeout)
	for {
		f := b.faults()
		if want(f) {
			return
		}
		if time.Now().After(deadline) {
			b.t.Fatalf("faults never %s (last 0x%02x)", what, f)
		}
		time.Sleep(50 * time.Millisecond)
	}
}

// The e-stop from the handheld latches the robot's emergency-stop fault;
// releasing it leaves the fault latched, and Select + Y held 1 s clears it
// through the bridge's counter translation.
func TestPakEmergencyStopAndFaultClear(t *testing.T) {
	const estopFault = 0x20 // protection::EMERGENCY_STOP
	b := StartBridge(t)
	p := StartPak(t, b)
	b.WaitSessions(1, 2*time.Second)
	time.Sleep(200 * time.Millisecond) // Begin masks controls held at the start

	if src := p.S.App.Screen(time.Now()).Robot.Source; src != "bridge" {
		t.Fatalf("robot reports source %q in control, want bridge", src)
	}
	p.Tap(padmap.HandSelect | padmap.HandStart)
	if !p.S.App.Screen(time.Now()).Requested.EStop {
		t.Fatal("Select + Start did not set the handheld's e-stop")
	}
	b.waitFaults("latch the e-stop", func(f uint64) bool { return f&estopFault != 0 }, 2*time.Second)

	p.Tap(padmap.HandSelect | padmap.HandStart)
	if p.S.App.Screen(time.Now()).Requested.EStop {
		t.Fatal("Select + Start did not release the handheld's e-stop")
	}
	time.Sleep(300 * time.Millisecond)
	if b.faults()&estopFault == 0 {
		t.Fatal("the e-stop fault cleared without a fault clear")
	}

	p.S.App.SetControls(padmap.Controls{Buttons: padmap.HandSelect | padmap.HandY})
	time.Sleep(500 * time.Millisecond)
	if c := p.S.App.Screen(time.Now()).Clearing; c <= 0 || c >= 1 {
		t.Errorf("clear hold at 0.5 s shows %.2f, want between 0 and 1", c)
	}
	time.Sleep(800 * time.Millisecond)
	p.S.App.SetControls(padmap.Controls{})
	b.waitFaults("clear the e-stop", func(f uint64) bool { return f&estopFault == 0 }, 2*time.Second)
}
