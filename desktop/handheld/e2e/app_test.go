package e2e

import (
	"context"
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
