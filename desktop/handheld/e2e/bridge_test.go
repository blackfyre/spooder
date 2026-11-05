// Package e2e drives the virtual robot's virtual bridge in WiFi drive
// (tools/virtual-robot --wifi-bridge) over loopback UDP: with plain sockets
// for the bridge's rules, and with the handheld's own run loop
// (internal/app, started as cmd/spooder-handheld starts it) for driving.
// Build the robot first (`mise run virtual-robot`, or `mise run e2e` for
// both); without it the tests skip. See openspec/changes/
// add-handheld-controller, design D11.
package e2e

import (
	"fmt"
	"net"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/virtualrobot"
)

// Bridge is a running virtual robot with its virtual bridge.
type Bridge struct {
	t *testing.T
	*virtualrobot.Robot
	Addr string // the virtual bridge's UDP address
}

// StartBridge runs the virtual robot with --wifi-bridge on a free loopback
// port, or skips the test if it is not built.
func StartBridge(t *testing.T) *Bridge {
	t.Helper()
	bin, ok := virtualrobot.Binary()
	if !ok {
		t.Skip("virtual robot not built (mise run virtual-robot):", bin)
	}
	vr, err := virtualrobot.Start(bin, filepath.Join(t.TempDir(), "usb"), "--wifi-bridge", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(vr.Close)
	line, err := vr.WaitLine("wifi-bridge ", time.Second)
	if err != nil {
		t.Fatal(err)
	}
	return &Bridge{t: t, Robot: vr, Addr: strings.TrimPrefix(line, "wifi-bridge ")}
}

// Mode is the robot's body mode from its status command.
func (b *Bridge) Mode() string {
	b.t.Helper()
	answer, err := b.Do("status")
	if err != nil {
		b.t.Fatal(err)
	}
	for _, f := range strings.Fields(answer) {
		if m, ok := strings.CutPrefix(f, "mode="); ok {
			return m
		}
	}
	b.t.Fatalf("no mode in %q", answer)
	return ""
}

// WaitMode waits until the robot's body mode is mode.
func (b *Bridge) WaitMode(mode string, timeout time.Duration) {
	b.t.Helper()
	deadline := time.Now().Add(timeout)
	for {
		m := b.Mode()
		if m == mode {
			return
		}
		if time.Now().After(deadline) {
			b.t.Fatalf("robot never reached %s (last %s)", mode, m)
		}
		time.Sleep(50 * time.Millisecond)
	}
}

// resting tells a body mode that has not left Sleep: the robot starts in
// Initialise, which a mode request leaves for Sleep or Stand.
func resting(mode string) bool { return mode == "sleep" || mode == "initialise" }

// SessionLine is one session start or end the virtual robot printed.
type SessionLine struct {
	Start         bool
	T             uint32 // robot time, ms
	From          string // start only
	Accepted      uint32 // end only: the session's accepted datagrams
	BudgetMax     int    // end only: the most datagrams charged to one 20 ms window
	LongestTickMs float64
}

// Sessions returns the session lines printed so far, in order.
func (b *Bridge) Sessions() []SessionLine {
	var out []SessionLine
	for _, l := range b.Lines() {
		var s SessionLine
		switch {
		case strings.HasPrefix(l, "session start "):
			s.Start = true
			if _, err := fmt.Sscanf(l, "session start t=%d from %s", &s.T, &s.From); err != nil {
				b.t.Fatalf("bad line %q: %v", l, err)
			}
		case strings.HasPrefix(l, "session end "):
			if _, err := fmt.Sscanf(l, "session end t=%d accepted=%d budget-max=%d longest-tick-ms=%g", &s.T,
				&s.Accepted, &s.BudgetMax, &s.LongestTickMs); err != nil {
				b.t.Fatalf("bad line %q: %v", l, err)
			}
		default:
			continue
		}
		out = append(out, s)
	}
	return out
}

// WaitSessions waits until n session lines have been printed and returns
// them with the time the n-th was seen.
func (b *Bridge) WaitSessions(n int, timeout time.Duration) ([]SessionLine, time.Time) {
	b.t.Helper()
	deadline := time.Now().Add(timeout)
	for {
		s := b.Sessions()
		if len(s) >= n {
			return s, time.Now()
		}
		if time.Now().After(deadline) {
			b.t.Fatalf("waited for %d session lines, have %v", n, s)
		}
		time.Sleep(time.Millisecond)
	}
}

// Handheld is a plain socket speaking the bridge's WiFi drive: command
// frames out, the robot's relayed frames counted by type with their arrival.
type Handheld struct {
	t    *testing.T
	conn *net.UDPConn
	seq  uint8

	mu       sync.Mutex
	arrivals map[protocol.FrameType][]time.Time
	invalid  int
	done     chan struct{}
}

// Dial opens a handheld socket to the bridge.
func Dial(t *testing.T, b *Bridge) *Handheld {
	t.Helper()
	ra, err := net.ResolveUDPAddr("udp4", b.Addr)
	if err != nil {
		t.Fatal(err)
	}
	conn, err := net.DialUDP("udp4", nil, ra)
	if err != nil {
		t.Fatal(err)
	}
	h := &Handheld{t: t, conn: conn, arrivals: map[protocol.FrameType][]time.Time{}, done: make(chan struct{})}
	go h.receive()
	t.Cleanup(func() { conn.Close(); <-h.done })
	return h
}

func (h *Handheld) receive() {
	defer close(h.done)
	buf := make([]byte, 2048)
	for {
		n, err := h.conn.Read(buf)
		if err != nil {
			if strings.Contains(err.Error(), "closed") {
				return
			}
			continue // a refused send reported on the read
		}
		at := time.Now()
		var d protocol.Decoder
		var f protocol.Frame
		ok, count := false, 0
		for _, c := range buf[:n] {
			d.Push(c)
			for {
				g, more := d.Next()
				if !more {
					break
				}
				f, ok = g, true
				count++
			}
		}
		h.mu.Lock()
		if ok && count == 1 && d.Discarded() == 0 && d.Pending() == 0 {
			h.arrivals[f.Type] = append(h.arrivals[f.Type], at)
		} else {
			h.invalid++
		}
		h.mu.Unlock()
	}
}

// Send sends one command frame with the next sequence number.
func (h *Handheld) Send(c protocol.Command) {
	h.seq++
	h.SendSeq(c, h.seq)
}

// SendSeq sends one command frame with sequence number seq.
func (h *Handheld) SendSeq(c protocol.Command, seq uint8) {
	_, _ = h.conn.Write(protocol.EncodeCommand(c, seq))
}

// Drive sends cmd() every 20 ms until stop is closed.
func (h *Handheld) Drive(cmd func() protocol.Command, stop <-chan struct{}) {
	tick := time.NewTicker(20 * time.Millisecond)
	defer tick.Stop()
	for {
		h.Send(cmd())
		select {
		case <-stop:
			return
		case <-tick.C:
		}
	}
}

// Count is the frames of type ft received in [from, to).
func (h *Handheld) Count(ft protocol.FrameType, from, to time.Time) int {
	h.mu.Lock()
	defer h.mu.Unlock()
	n := 0
	for _, at := range h.arrivals[ft] {
		if !at.Before(from) && at.Before(to) {
			n++
		}
	}
	return n
}

// Total is the frames received of every type, and the invalid datagrams.
func (h *Handheld) Total() (frames, invalid int) {
	h.mu.Lock()
	defer h.mu.Unlock()
	for _, a := range h.arrivals {
		frames += len(a)
	}
	return frames, h.invalid
}

// Last is the arrival of the last frame of any type.
func (h *Handheld) Last() time.Time {
	h.mu.Lock()
	defer h.mu.Unlock()
	var last time.Time
	for _, a := range h.arrivals {
		if len(a) > 0 && a[len(a)-1].After(last) {
			last = a[len(a)-1]
		}
	}
	return last
}

const (
	modeStand = 0 // RequestedMode values
	modeWalk  = 1
	modeSleep = 2
)

// command is a connected handheld's frame requesting mode, with the mode
// request counter at counter.
func command(mode, counter uint8) protocol.Command {
	return protocol.Command{Mode: mode, Speed: 44, Flags: protocol.FlagControllerConnected, ModeRequestCounter: counter}
}

// Handheld drives the virtual robot; Handheld goes quiet.
func TestDriveThenQuiet(t *testing.T) {
	b := StartBridge(t)
	h := Dial(t, b)
	stop := make(chan struct{})
	begun := time.Now()
	var counter uint8
	var mu sync.Mutex
	done := make(chan struct{})
	go func() {
		h.Drive(func() protocol.Command {
			mu.Lock()
			defer mu.Unlock()
			if time.Since(begun) > 100*time.Millisecond {
				counter = 1 // a mode request after the baseline frames
			}
			return command(modeStand, counter)
		}, stop)
		close(done)
	}()
	b.WaitMode("stand", 10*time.Second)
	// A second of frames from the robot while standing.
	from := time.Now()
	time.Sleep(time.Second)
	to := time.Now()
	for _, ft := range []protocol.FrameType{protocol.TypeTelemetry, protocol.TypePose, protocol.TypeDiagnostics} {
		if n := h.Count(ft, from, to); n < 5 {
			t.Errorf("%s frames in 1 s: %d, want about 10", ft.Name(), n)
		}
	}
	if _, invalid := h.Total(); invalid != 0 {
		t.Errorf("invalid datagrams: %d", invalid)
	}

	// Quiet: the session ends, the robot sleeps, nothing more is sent.
	close(stop)
	<-done
	quiet := time.Now()
	s, seen := b.WaitSessions(2, time.Second)
	if s[0].Start != true || s[1].Start != false {
		t.Fatalf("sessions %v, want a start and an end", s)
	}
	if d := seen.Sub(quiet); d < 140*time.Millisecond || d > 250*time.Millisecond {
		t.Errorf("session ended %v after the last frame, want about 150 to 190 ms", d)
	}
	time.Sleep(50 * time.Millisecond) // frames sent before the end, in flight
	after := time.Now()
	b.WaitMode("sleep", 10*time.Second)
	time.Sleep(300 * time.Millisecond)
	if n := h.Count(protocol.TypeTelemetry, after, time.Now()) + h.Count(protocol.TypePose, after, time.Now()) +
		h.Count(protocol.TypeDiagnostics, after, time.Now()); n != 0 {
		t.Errorf("%d frames relayed after the session ended", n)
	}
}

// Session starts in Sleep.
func TestSessionStartsInSleep(t *testing.T) {
	b := StartBridge(t)
	h := Dial(t, b)
	stop := make(chan struct{})
	defer close(stop)
	go h.Drive(func() protocol.Command { return command(modeWalk, 7) }, stop)
	b.WaitSessions(1, time.Second)
	deadline := time.Now().Add(time.Second)
	for time.Now().Before(deadline) {
		if m := b.Mode(); !resting(m) {
			t.Fatalf("robot in %s with no change of the mode request counter, want sleep (or initialise, before any request)", m)
		}
		time.Sleep(50 * time.Millisecond)
	}
}

// seqAt is a flooding handheld's sequence number: one step per 20 ms, so
// every frame taken is newer than the last by at most a few and the
// session stays live (repeats are stale and change nothing).
func seqAt(start time.Time) uint8 { return uint8(1 + time.Since(start)/(20*time.Millisecond)) }

// Flood then silence.
func TestFloodThenSilence(t *testing.T) {
	b := StartBridge(t)
	h := Dial(t, b)
	start := time.Now()
	for i := 0; i < 500; i++ {
		h.SendSeq(command(modeSleep, 0), seqAt(start))
	}
	last := time.Now()
	var s []SessionLine
	var seen time.Time
	for {
		s, seen = b.WaitSessions(len(s)+1, time.Second)
		if !s[len(s)-1].Start {
			break
		}
	}
	if d := seen.Sub(last); d > 190*time.Millisecond {
		t.Errorf("session ended %v after the last datagram, want at most 190 ms", d)
	}
	time.Sleep(300 * time.Millisecond)
	if n := len(b.Sessions()); n != len(s) {
		t.Errorf("sessions after the end: %v", b.Sessions()[len(s):])
	}
	// At most WINDOW_BUDGET (8) datagrams charged to a 20 ms window,
	// whatever their sequence numbers.
	for i := 1; i < len(s); i += 2 {
		t.Logf("session %d: %d accepted, at most %d a window", i/2, s[i].Accepted, s[i].BudgetMax)
		if s[i].BudgetMax > 8 || s[i].BudgetMax < 1 {
			t.Errorf("session %d: %d datagrams charged to one window, want 1 to 8", i/2, s[i].BudgetMax)
		}
	}
}

// A datagram that ends a session is charged to that session's window.
func TestJumpChargedToEndingSession(t *testing.T) {
	b := StartBridge(t)
	h := Dial(t, b)
	h.SendSeq(command(modeSleep, 0), 0)
	b.WaitSessions(1, time.Second)
	time.Sleep(60 * time.Millisecond)
	h.SendSeq(command(modeSleep, 0), 0)  // stale, charged
	h.SendSeq(command(modeSleep, 0), 33) // a jump: the session ends
	s, _ := b.WaitSessions(2, time.Second)
	if s[1].Start || s[1].BudgetMax != 2 {
		t.Errorf("sessions %v, want an end with 2 datagrams charged to one window", s)
	}
	if s[1].T-s[0].T > 150 {
		t.Errorf("session ended at %d ms, after its lapse: the jump did not end it", s[1].T)
	}
}

// A datagram arriving as the previous session lapses starts the next
// session, and is counted in it only.
func TestSessionBoundaryCounts(t *testing.T) {
	b := StartBridge(t)
	h := Dial(t, b)
	n := 0
	for gap := 150; gap <= 190; gap += 5 {
		h.Send(command(modeSleep, 0))
		time.Sleep(time.Duration(gap) * time.Millisecond)
		h.Send(command(modeSleep, 0)) // newer by one: the same session if it is still live
		time.Sleep(250 * time.Millisecond)
		n += 2
	}
	s := b.Sessions()
	var accepted uint32
	for _, l := range s {
		if !l.Start {
			accepted += l.Accepted
		}
	}
	if accepted != uint32(n) {
		t.Errorf("session ends count %d accepted datagrams, %d were sent: %v", accepted, n, s)
	}
}

// Sustained flood.
func TestSustainedFlood(t *testing.T) {
	b := StartBridge(t)
	h := Dial(t, b)
	start := time.Now()
	// A session first, at the handheld's own rate.
	for time.Since(start) < 300*time.Millisecond {
		h.SendSeq(command(modeSleep, 0), seqAt(start))
		time.Sleep(20 * time.Millisecond)
	}
	from := time.Now()
	sent := 0
	for time.Since(from) < time.Second {
		h.SendSeq(command(modeSleep, 0), seqAt(start))
		sent++
	}
	to := time.Now()
	t.Logf("%d datagrams in 1 s", sent)
	if n := h.Count(protocol.TypeTelemetry, from, to); n < 8 || n > 12 {
		t.Errorf("telemetry during the flood: %d frames in 1 s, want 8 to 12", n)
	}
	s, _ := b.WaitSessions(2, time.Second)
	if len(s) != 2 || s[1].Start {
		t.Fatalf("sessions %v, want one start and its end", s)
	}
	if s[1].LongestTickMs > 40 {
		t.Errorf("longest interval between tick starts %.1f ms, want at most 40", s[1].LongestTickMs)
	}
}

// Second handheld.
func TestSecondSenderIgnored(t *testing.T) {
	b := StartBridge(t)
	first := Dial(t, b)
	stop := make(chan struct{})
	defer close(stop)
	go first.Drive(func() protocol.Command { return command(modeSleep, 0) }, stop)
	b.WaitSessions(1, time.Second)
	second := Dial(t, b)
	from := time.Now()
	for i := 0; i < 25; i++ {
		second.Send(command(modeStand, uint8(i)))
		time.Sleep(20 * time.Millisecond)
	}
	time.Sleep(200 * time.Millisecond)
	if frames, invalid := second.Total(); frames+invalid != 0 {
		t.Errorf("second handheld received %d frames and %d other datagrams, want nothing", frames, invalid)
	}
	if n := first.Count(protocol.TypeTelemetry, from, time.Now()); n == 0 {
		t.Error("first handheld received no telemetry while the second sent")
	}
	if s := b.Sessions(); len(s) != 1 {
		t.Errorf("sessions %v, want the first one's start only", s)
	}
	if m := b.Mode(); !resting(m) {
		t.Errorf("robot in %s after the second handheld's requests, want sleep (or initialise, before any request)", m)
	}
}
