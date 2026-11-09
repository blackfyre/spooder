package app

import (
	"context"
	"errors"
	"net"
	"os"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/handheld/internal/supplyband"
	"github.com/blackfyre/spooder/desktop/handheld/internal/udplink"
	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// fakeLink is a udplink.Receiver fed by the test, with the event queue
// udplink.Link keeps.
type fakeLink struct {
	rx          *udplink.Receiver
	events      []udplink.Event
	rebaselines int
}

func (l *fakeLink) TakeEvents() []udplink.Event {
	ev := l.events
	l.events = nil
	return ev
}
func (l *fakeLink) Status(now time.Time) udplink.Status { return l.rx.Status(now) }
func (l *fakeLink) Rebaseline()                         { l.rebaselines++; l.rx.Rebaseline() }

// fakeRunner records every command, power save's included, in order.
type fakeRunner struct {
	mu       sync.Mutex
	calls    []string
	link     string // iw dev wlan0 link's output
	linkFail bool   // iw dev wlan0 link fails
}

func (f *fakeRunner) Run(_ context.Context, name string, args ...string) ([]byte, error) {
	cmd := strings.Join(append([]string{name}, args...), " ")
	f.mu.Lock()
	f.calls = append(f.calls, cmd)
	f.mu.Unlock()
	switch cmd {
	case "iw dev wlan0 get power_save":
		return []byte("Power save: on\n"), nil
	case "iw dev wlan0 link":
		f.mu.Lock()
		defer f.mu.Unlock()
		if f.linkFail {
			return nil, errors.New("exit status 1")
		}
		return []byte(f.link), nil
	}
	return nil, nil
}

func (f *fakeRunner) log() string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return strings.Join(f.calls, "\n")
}

// rig runs an App with a fake clock, link and runner, one 20 ms tick per
// step, as udplink.Link.Run does.
type rig struct {
	t    *testing.T
	a    *App
	link *fakeLink
	run  *fakeRunner
	now  time.Time
	seq  uint8 // the robot's next sequence number
	c    padmap.Controls
	cmd  protocol.Command
	stop bool
	held bool // the last step held its frame
}

func newRig(t *testing.T) *rig {
	start := time.Unix(5000, 0)
	r := &rig{t: t, link: &fakeLink{rx: udplink.NewReceiver(start)}, run: &fakeRunner{}, now: start}
	ps, err := wlan.ReadPowerSave(context.Background(), r.run)
	if err != nil {
		t.Fatal(err)
	}
	r.a = New(r.link, r.run, ps, nil)
	r.step()
	return r
}

func (r *rig) step() protocol.Command {
	r.t.Helper()
	if r.stop {
		r.t.Fatal("stepped after the run stopped")
	}
	r.now = r.now.Add(udplink.Period)
	r.a.SetControls(r.c)
	var out udplink.Outcome
	r.cmd, out = r.a.Step(r.now)
	r.stop = out == udplink.Stop
	r.held = out == udplink.Hold
	return r.cmd
}

func (r *rig) steps(n int) {
	r.t.Helper()
	for i := 0; i < n && !r.stop; i++ {
		r.step()
	}
}

// press taps a button for two ticks.
func (r *rig) press(b padmap.Buttons) {
	r.t.Helper()
	r.c.Buttons |= b
	r.steps(2)
	r.c.Buttons &^= b
	r.steps(2)
}

// telemetry delivers one telemetry frame with sequence seq.
func (r *rig) telemetry(seq uint8, body session.BodyMode, outputs bool) {
	r.t.Helper()
	b := protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: uint8(body), OutputsOn: outputs}, seq)
	ev, ok := r.link.rx.Datagram(b, r.now)
	if !ok {
		r.t.Fatal("telemetry refused")
	}
	r.link.events = append(r.link.events, ev)
}

// robot delivers the next telemetry frame in sequence.
func (r *rig) robot(body session.BodyMode, outputs bool) {
	r.t.Helper()
	r.telemetry(r.seq, body, outputs)
	r.seq++
}

// walking drives the rig to Walk with the robot reporting it.
func walking(t *testing.T) *rig {
	r := newRig(t)
	if r.cmd.Mode != uint8(padmap.ModeSleep) {
		t.Fatalf("first command mode %d, want Sleep", r.cmd.Mode)
	}
	r.seq = 40
	r.press(padmap.HandY) // Sleep -> Stand
	r.press(padmap.HandStart)
	if r.cmd.Mode != uint8(padmap.ModeWalk) {
		t.Fatalf("mode %d after Y and Start, want Walk", r.cmd.Mode)
	}
	for i := 0; i < 3; i++ {
		r.robot(session.BodyWalk, true)
		r.steps(2)
	}
	return r
}

func (r *rig) wantRunning(what string) {
	r.t.Helper()
	if r.stop {
		r.t.Fatalf("%s: the run stopped", what)
	}
	if r.cmd.Mode != uint8(padmap.ModeSleep) {
		r.t.Fatalf("%s: mode %d, want Sleep", what, r.cmd.Mode)
	}
}

func TestMenuLeavesOnTwoNewerSleepFrames(t *testing.T) {
	r := walking(t)
	counter := r.cmd.ModeRequestCounter
	r.a.Press(ActionExit, r.now)
	r.step()
	r.wantRunning("after Menu")
	if r.cmd.ModeRequestCounter != counter+1 {
		t.Fatalf("mode-request counter %d, want %d", r.cmd.ModeRequestCounter, counter+1)
	}
	if r.a.State() != Leaving {
		t.Fatalf("state %d, want Leaving", r.a.State())
	}
	r.robot(session.BodySleep, true)
	r.step()
	r.wantRunning("after one Sleep frame")
	r.robot(session.BodySleep, true)
	r.step()
	if !r.stop || r.a.Ended() != ActionExit {
		t.Fatalf("stop %v ended %v after two Sleep frames, want an exit", r.stop, r.a.Ended())
	}
}

func TestLeaveConfirmedByOutputOff(t *testing.T) {
	r := walking(t)
	r.a.Press(ActionExit, r.now)
	r.step()
	r.robot(session.BodyWalk, false)
	r.robot(session.BodyWalk, false)
	r.step()
	if !r.stop {
		t.Fatal("two output-off frames did not end the leave")
	}
}

func TestLeaveIgnoresAFrameOlderThanThePress(t *testing.T) {
	r := walking(t)
	newest := r.seq - 1
	r.a.Press(ActionExit, r.now)
	r.step()
	// A Sleep frame from before the press, delayed: older than the newest.
	r.telemetry(newest-1, session.BodySleep, true)
	r.robot(session.BodySleep, true)
	r.step()
	r.wantRunning("after an old and one new Sleep frame")
	// The same frame number as the newest before the press (a duplicate)
	// does not count either.
	r.telemetry(newest, session.BodySleep, true)
	r.step()
	r.wantRunning("after a duplicate")
}

func TestLeaveNeedsTheSleepFramesInARow(t *testing.T) {
	r := walking(t)
	r.a.Press(ActionExit, r.now)
	r.step()
	r.robot(session.BodySleep, true)
	r.robot(session.BodyWalk, true)
	r.robot(session.BodySleep, true)
	r.step()
	r.wantRunning("after Sleep, Walk, Sleep")
	r.robot(session.BodySleep, true)
	r.step()
	if !r.stop {
		t.Fatal("two Sleep frames in a row did not end the leave")
	}
}

func TestQueuedFrameFromBeforeThePress(t *testing.T) {
	r := walking(t)
	// Received before the press but taken by the app after it.
	r.robot(session.BodySleep, true)
	r.a.Press(ActionExit, r.now)
	r.robot(session.BodySleep, true)
	r.step()
	r.wantRunning("after a queued frame and one newer")
}

func TestQueuedRebaselineFrameFromBeforeThePress(t *testing.T) {
	r := newRig(t)
	r.seq = 40
	// The run's first frame (a rebaseline), received before the press but
	// taken by the app after it.
	r.robot(session.BodySleep, true)
	r.a.Press(ActionExit, r.now)
	r.robot(session.BodySleep, true)
	r.step()
	r.wantRunning("after the queued first frame and one newer")
	r.robot(session.BodySleep, true)
	r.step()
	if !r.stop {
		t.Fatal("two Sleep frames newer than the press did not end the leave")
	}
}

func TestRebaselineAfterThePressCounts(t *testing.T) {
	r := walking(t)
	r.a.Press(ActionExit, r.now)
	r.steps(30) // 600 ms without a frame
	r.seq = 3   // the robot restarted
	r.robot(session.BodySleep, true)
	r.step()
	r.wantRunning("after the rebaselining Sleep frame")
	r.robot(session.BodySleep, true)
	r.step()
	if !r.stop {
		t.Fatal("two Sleep frames after a rebaseline did not end the leave")
	}
}

func TestControlsDuringTheWait(t *testing.T) {
	r := walking(t)
	r.a.Press(ActionExit, r.now)
	r.step()
	r.press(padmap.HandY)
	r.wantRunning("after Y")
	r.press(padmap.HandStart)
	r.wantRunning("after Start")
	r.c.LeftY = -32768
	r.steps(5)
	r.wantRunning("with the stick pushed")
	if r.cmd.TranslateY != 0 || r.cmd.TranslateX != 0 || r.cmd.Rotate != 0 {
		t.Fatalf("command %+v moves during the wait", r.cmd)
	}
	r.c.LeftY = 0
	r.c.Buttons = padmap.HandSelect
	r.steps(2)
	r.press(padmap.HandStart)
	if r.cmd.Flags&protocol.FlagEmergencyStop == 0 {
		t.Fatal("Select + Start during the wait did not set the e-stop")
	}
	r.wantRunning("after Select + Start")
}

func TestLeaveTimesOutAtThreeSeconds(t *testing.T) {
	r := walking(t)
	pressed := r.now
	r.a.Press(ActionExit, pressed)
	for r.now.Add(udplink.Period).Sub(pressed) < 2900*time.Millisecond {
		r.step()
	}
	r.step() // 2.9 s
	if d := r.now.Sub(pressed); d != 2900*time.Millisecond {
		t.Fatalf("at %v, want 2.9 s", d)
	}
	r.wantRunning("at 2.9 s")
	for r.now.Sub(pressed) < LeaveTimeout-udplink.Period {
		r.step()
	}
	r.step()
	if !r.stop || r.now.Sub(pressed) != LeaveTimeout {
		t.Fatalf("stop %v at %v, want a stop at 3 s", r.stop, r.now.Sub(pressed))
	}
}

func TestSecondPressIgnored(t *testing.T) {
	r := walking(t)
	pressed := r.now
	r.a.Press(ActionExit, pressed)
	r.step()
	r.now = r.now.Add(time.Second)
	r.a.Press(ActionPowerOff, r.now)
	for !r.stop {
		r.step()
	}
	if r.a.Ended() != ActionExit || r.now.Sub(pressed) > LeaveTimeout+udplink.Period {
		t.Fatalf("ended %v at %v, want an exit at the first press's timeout", r.a.Ended(), r.now.Sub(pressed))
	}
	if strings.Contains(r.run.log(), PowerOffCommand) {
		t.Fatal("the second press powered off")
	}
}

func TestShortPowerSuspendsAndReturnsToDriving(t *testing.T) {
	r := walking(t)
	r.a.Press(ActionSuspend, r.now)
	r.step()
	r.robot(session.BodySleep, false)
	r.robot(session.BodySleep, false)
	r.step()
	if r.stop {
		t.Fatal("the suspend ended the run")
	}
	if r.a.State() != Driving || r.cmd.Mode != uint8(padmap.ModeSleep) {
		t.Fatalf("state %d mode %d after the suspend, want driving with Sleep", r.a.State(), r.cmd.Mode)
	}
	want := strings.Join([]string{
		"iw dev wlan0 get power_save",
		"iw dev wlan0 set power_save on",
		SuspendScript,
		"iw dev wlan0 set power_save off",
	}, "\n")
	if got := r.run.log(); got != want {
		t.Fatalf("ran\n%s\nwant\n%s", got, want)
	}
	if r.link.rebaselines != 1 {
		t.Fatalf("%d rebaselines after the resume, want 1", r.link.rebaselines)
	}
	// Driving again: Y stands, and a new leave can start.
	r.press(padmap.HandY)
	if r.cmd.Mode != uint8(padmap.ModeStand) {
		t.Fatalf("mode %d after Y on resume, want Stand", r.cmd.Mode)
	}
	r.a.Press(ActionExit, r.now)
	r.step()
	if r.a.State() != Leaving {
		t.Fatal("no leave after the resume")
	}
}

func TestLongPowerPowersOffAfterRestoring(t *testing.T) {
	r := walking(t)
	r.a.Press(ActionPowerOff, r.now)
	r.step()
	r.robot(session.BodySleep, true)
	r.robot(session.BodySleep, true)
	r.step()
	if !r.stop || r.a.Ended() != ActionPowerOff {
		t.Fatalf("stop %v ended %v, want a power-off", r.stop, r.a.Ended())
	}
	want := strings.Join([]string{
		"iw dev wlan0 get power_save",
		"iw dev wlan0 set power_save on",
		PowerOffCommand,
	}, "\n")
	if got := r.run.log(); got != want {
		t.Fatalf("ran\n%s\nwant\n%s", got, want)
	}
}

func TestScreenStaleAndFaults(t *testing.T) {
	r := newRig(t)
	if sc := r.a.Screen(r.now); !sc.Robot.Stale || sc.Robot.Have {
		t.Fatalf("robot %+v before any telemetry, want stale", sc.Robot)
	}
	b := protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: uint8(session.BodyStand), OutputsOn: true,
		Faults: session.FaultOvercurrent, VoltageMv: 7400}, 3)
	if _, ok := r.link.rx.Datagram(b, r.now); !ok {
		t.Fatal("telemetry refused")
	}
	sc := r.a.Screen(r.now.Add(499 * time.Millisecond))
	if sc.Robot.Stale || sc.Robot.Body != "Stand" || sc.Robot.VoltageMv != 7400 {
		t.Fatalf("robot %+v at 499 ms, want fresh Stand at 7400 mV", sc.Robot)
	}
	if len(sc.Robot.Faults) != 1 || sc.Robot.Faults[0] != "overcurrent" {
		t.Fatalf("faults %v, want overcurrent", sc.Robot.Faults)
	}
	if sc.Requested.Mode != "Sleep" || sc.Requested.EStop {
		t.Fatalf("requested %+v, want Sleep without the e-stop", sc.Requested)
	}
	sc = r.a.Screen(r.now.Add(StaleAfter))
	if !sc.Robot.Stale || sc.Robot.Body != "Stand" {
		t.Fatalf("robot %+v at 500 ms, want stale, still showing Stand", sc.Robot)
	}
	if sc.Link.SinceLast != StaleAfter {
		t.Fatalf("since last %v, want 500 ms", sc.Link.SinceLast)
	}
	if later := r.a.Screen(r.now.Add(time.Second)); later.Link.SinceLast != time.Second {
		t.Fatalf("since last %v at 1 s, want it rising", later.Link.SinceLast)
	}
}

// The robot's bench simulation flag reaches the screen, and only when set.
func TestScreenSimulated(t *testing.T) {
	r := newRig(t)
	for i, flags := range []uint8{protocol.TelemetrySimulated | protocol.TelemetryBridgeInControl, protocol.TelemetryBridgeInControl} {
		b := protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: uint8(session.BodyWalk), OutputsOn: true, Flags: flags}, uint8(3+i))
		if _, ok := r.link.rx.Datagram(b, r.now); !ok {
			t.Fatal("telemetry refused")
		}
		if sc := r.a.Screen(r.now); sc.Robot.Simulated != (i == 0) {
			t.Fatalf("flags 0x%02x: simulated %v", flags, sc.Robot.Simulated)
		}
	}
}

// The driving view's legs come from the shown pose frame: a leg with a pulse
// set is placed, one never set is not, and the pose goes stale as telemetry
// does.
func TestScreenPose(t *testing.T) {
	r := newRig(t)
	if sc := r.a.Screen(r.now); sc.Pose.Have || !sc.Pose.Stale {
		t.Fatalf("pose %+v before any frame, want none, stale", sc.Pose)
	}
	var p protocol.Pose
	for j := 0; j < 3; j++ { // leg 0 only
		p.Joints[j] = protocol.PoseJoint{CommandedCentideg: 9000, ServoCentideg: 9000, PulseUs: 1500}
	}
	if _, ok := r.link.rx.Datagram(protocol.EncodePose(p, 4), r.now); !ok {
		t.Fatal("pose refused")
	}
	sc := r.a.Screen(r.now.Add(499 * time.Millisecond))
	if !sc.Pose.Have || sc.Pose.Stale || !sc.Pose.Legs[0].Set || sc.Pose.Legs[1].Set {
		t.Fatalf("pose %+v at 499 ms, want fresh with leg 0 alone set", sc.Pose)
	}
	if sc.Pose.Legs[0].Points[3] == sc.Pose.Legs[0].Points[0] {
		t.Fatal("leg 0's foot at its pivot")
	}
	if sc = r.a.Screen(r.now.Add(StaleAfter)); !sc.Pose.Stale || !sc.Pose.Legs[0].Set {
		t.Fatalf("pose %+v at 500 ms, want stale, still placed", sc.Pose)
	}
}

// The robot's protection limits come from the newest diagnostics frame
// and stay once diagnostics are stale (design D3 of handheld-observations).
func TestScreenSupplyLimits(t *testing.T) {
	r := newRig(t)
	if sc := r.a.Screen(r.now); sc.Robot.SupplyLimits.Valid() {
		t.Fatalf("limits %+v before any diagnostics, want none", sc.Robot.SupplyLimits)
	}
	d := protocol.Diagnostics{BrownoutMv: 4800, MinMv: 5200, MaxMv: 6800, SustainedCurrentMa: 6000, TripCurrentMa: 9000}
	if _, ok := r.link.rx.Datagram(protocol.EncodeDiagnostics(d, 7), r.now); !ok {
		t.Fatal("diagnostics refused")
	}
	want := supplyband.Limits{BrownoutMv: 4800, MinMv: 5200, MaxMv: 6800, SustainedCurrentMa: 6000, TripCurrentMa: 9000}
	if got := r.a.Screen(r.now).Robot.SupplyLimits; got != want {
		t.Errorf("limits %+v, want %+v", got, want)
	}
	if got := r.a.Screen(r.now.Add(10 * StaleAfter)).Robot.SupplyLimits; got != want {
		t.Errorf("limits %+v with diagnostics stale, want %+v kept", got, want)
	}
}

func TestParseFlags(t *testing.T) {
	var out strings.Builder
	c, err := ParseFlags(nil, &out)
	if err != nil || c.Device || c.Addr != udplink.DefaultAddr {
		t.Fatalf("no flags: %+v, %v", c, err)
	}
	if c, err = ParseFlags([]string{"--device"}, &out); err != nil || !c.Device || c.Addr != udplink.DefaultAddr {
		t.Fatalf("--device: %+v, %v", c, err)
	}
	if c, err = ParseFlags([]string{"--addr", "127.0.0.1:9"}, &out); err != nil || c.Addr != "127.0.0.1:9" {
		t.Fatalf("--addr: %+v, %v", c, err)
	}
	if _, err = ParseFlags([]string{"--device", "--addr", "127.0.0.1:9"}, &out); err == nil {
		t.Fatal("--device with --addr accepted")
	}
	if _, err = ParseFlags([]string{"extra"}, &out); err == nil {
		t.Fatal("an argument accepted")
	}
}

// refusingRunner fails the test on any command: no WiFi step may run.
type refusingRunner struct{ t *testing.T }

func (r refusingRunner) Run(_ context.Context, name string, args ...string) ([]byte, error) {
	r.t.Errorf("ran %s %v off the device", name, args)
	return nil, errors.New("refused")
}

func TestStartWithoutDeviceUsesAddr(t *testing.T) {
	bridge, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer bridge.Close()
	cfg, err := ParseFlags([]string{"--addr", bridge.LocalAddr().String()}, &strings.Builder{})
	if err != nil {
		t.Fatal(err)
	}
	sys := System{
		Runner: refusingRunner{t},
		Addrs: func(string) ([]net.IP, error) {
			t.Error("address check off the device")
			return nil, nil
		},
		ReadFile: func(p string) ([]byte, error) {
			t.Errorf("read %s off the device", p)
			return nil, errors.New("refused")
		},
	}
	s, err := Start(context.Background(), cfg, sys)
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- s.Run(ctx) }()

	buf := make([]byte, 64)
	_ = bridge.SetReadDeadline(time.Now().Add(time.Second))
	n, from, err := bridge.ReadFromUDP(buf)
	if err != nil {
		t.Fatal(err)
	}
	if from.String() != s.Link.LocalAddr().String() || n != protocol.CommandFrameSize {
		t.Fatalf("%d bytes from %v, want a command frame from %v", n, from, s.Link.LocalAddr())
	}
	var d protocol.Decoder
	for _, b := range buf[:n] {
		d.Push(b)
	}
	f, ok := d.Next()
	if !ok || f.Command.Mode != uint8(padmap.ModeSleep) || f.Command.Flags&protocol.FlagControllerConnected == 0 {
		t.Fatalf("first frame %+v, want Sleep with a controller connected", f)
	}
	sc := s.App.Screen(time.Now())
	if sc.Link.SignalKnown {
		t.Fatal("a signal level off the device")
	}
	cancel()
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}

// iwOn is iw dev wlan0 link's output on ssid, with a signal level.
func iwOn(ssid string) string {
	return "Connected to 02:00:00:00:00:00 (on wlan0)\n\tSSID: " + ssid + "\n\tsignal: -61 dBm\n"
}

func TestStartOnDevice(t *testing.T) {
	run := &fakeRunner{link: iwOn("spooder-drive-1a2b")}
	sys := System{
		Runner:   run,
		Addrs:    func(string) ([]net.IP, error) { return []net.IP{net.ParseIP("192.168.4.2")}, nil },
		ReadFile: func(string) ([]byte, error) { return []byte(" wlan0: 0000   70.  -52.  -256 0 0 0 0 0 0\n"), nil },
	}
	s, err := Start(context.Background(), Config{Device: true, Addr: udplink.DefaultAddr}, sys)
	if err != nil {
		t.Fatal(err)
	}
	if got := s.Link.RemoteAddr().String(); got != udplink.DefaultAddr {
		t.Fatalf("link to %s, want %s", got, udplink.DefaultAddr)
	}
	if sc := s.App.Screen(time.Now()); !sc.Link.SignalKnown || sc.Link.SignalDBm != -52 {
		t.Fatalf("signal %+v, want -52 dBm", sc.Link)
	}
	if err := s.Close(); err != nil {
		t.Fatal(err)
	}
	if sc := s.App.Screen(time.Now()); sc.Link.OffNetwork || sc.Link.Network != "spooder-drive-1a2b" {
		t.Fatalf("network %+v, want on spooder-drive-1a2b", sc.Link)
	}
	want := "iw dev wlan0 link\niw dev wlan0 get power_save\niw dev wlan0 set power_save off\niw dev wlan0 link\niw dev wlan0 set power_save on"
	if got := run.log(); got != want {
		t.Fatalf("ran\n%s\nwant\n%s", got, want)
	}
}

// On the TrimUI there is no /proc/net/wireless: the signal level comes
// from iw, sampled at the start and every SignalPeriod while running.
func TestSignalFromIwOnDevice(t *testing.T) {
	run := &fakeRunner{link: iwOn("spooder-drive-1a2b")}
	sys := System{
		Runner:   run,
		Addrs:    func(string) ([]net.IP, error) { return []net.IP{net.ParseIP("192.168.4.2")}, nil },
		ReadFile: func(string) ([]byte, error) { return nil, os.ErrNotExist },
	}
	s, err := Start(context.Background(), Config{Device: true, Addr: "127.0.0.1:9"}, sys)
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	if sc := s.App.Screen(time.Now()); !sc.Link.SignalKnown || sc.Link.SignalDBm != -61 {
		t.Fatalf("signal %+v at the start, want -61 dBm", sc.Link)
	}
	run.mu.Lock()
	run.link = "Not connected.\n"
	run.mu.Unlock()
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- s.Run(ctx) }()
	deadline := time.Now().Add(3 * SignalPeriod)
	for s.App.Screen(time.Now()).Link.SignalKnown {
		if time.Now().After(deadline) {
			t.Fatal("the signal level was not sampled again while running")
		}
		time.Sleep(50 * time.Millisecond)
	}
	cancel()
	if err := <-done; err != nil && !errors.Is(err, context.Canceled) {
		t.Fatal(err)
	}
}

func TestStartOnDeviceNotJoined(t *testing.T) {
	for _, c := range []struct {
		link string
		ip   string
	}{
		{iwOn("spooder-drive-1a2b"), ""},      // no address
		{iwOn("spooder-1a2b"), "192.168.4.2"}, // the setup network, in the bridge's range
		{"Not connected.\n", "192.168.4.2"},
	} {
		run := &fakeRunner{link: c.link}
		sys := System{Runner: run, Addrs: func(string) ([]net.IP, error) {
			if c.ip == "" {
				return nil, nil
			}
			return []net.IP{net.ParseIP(c.ip)}, nil
		}}
		_, err := Start(context.Background(), Config{Device: true, Addr: udplink.DefaultAddr}, sys)
		var nj *wlan.NotJoinedError
		if !errors.As(err, &nj) {
			t.Fatalf("%q: got %v, want the join message", c.link, err)
		}
		if strings.Contains(run.log(), "power_save") {
			t.Fatalf("%q: ran\n%s\nbefore the network check passed", c.link, run.log())
		}
	}
}

// onDevice makes the rig's app check the network as a device run started
// on spooder-drive-1a2b does, wlan0 read through the rig's runner.
func (r *rig) onDevice() {
	r.run.mu.Lock()
	r.run.link = iwOn("spooder-drive-1a2b")
	r.run.mu.Unlock()
	r.a.readFile = func(string) ([]byte, error) { return nil, os.ErrNotExist }
	r.a.network = "spooder-drive-1a2b"
}

// sample sets iw's link output (or its failure) and samples wlan0.
func (r *rig) sample(link string, fail bool) {
	r.run.mu.Lock()
	r.run.link, r.run.linkFail = link, fail
	r.run.mu.Unlock()
	r.a.SampleWLAN(context.Background())
}

// Off the network the run started on, every tick is held, whatever reads
// fail or whichever bridge's network wlan0 is on; back on it, the first
// frame requests Sleep with the held control masked and the robot frames
// rebaselined (openspec change handheld-network-check).
func TestOffNetworkHoldsAndRejoinRequestsSleep(t *testing.T) {
	r := walking(t)
	r.onDevice()
	r.sample(iwOn("spooder-drive-1a2b"), false)
	r.steps(2)
	if r.held || r.cmd.Mode != uint8(padmap.ModeWalk) {
		t.Fatalf("on the network: held %v, mode %d; want Walk sent", r.held, r.cmd.Mode)
	}

	for _, c := range []struct {
		what, link string
		fail       bool
	}{
		{"another network", iwOn("Home WiFi"), false},
		{"a failed read", "", true},
		{"no network", "Not connected.\n", false},
		{"another bridge", iwOn("spooder-drive-9f00"), false},
	} {
		r.sample(c.link, c.fail)
		r.steps(3)
		if !r.held {
			t.Fatalf("%s: a frame was sent off the network", c.what)
		}
		sc := r.a.Screen(r.now)
		if !sc.Link.OffNetwork || sc.Link.Network != "spooder-drive-1a2b" {
			t.Fatalf("%s: screen %+v, want off spooder-drive-1a2b", c.what, sc.Link)
		}
	}
	if sc := r.a.Screen(r.now); sc.Link.NowOn != "spooder-drive-9f00" {
		t.Fatalf("now on %q, want spooder-drive-9f00", sc.Link.NowOn)
	}

	// Back on it, with Y held.
	before := r.link.rebaselines
	r.c.Buttons = padmap.HandY
	r.sample(iwOn("spooder-drive-1a2b"), false)
	r.step()
	if r.held || r.cmd.Mode != uint8(padmap.ModeSleep) {
		t.Fatalf("rejoined: held %v, mode %d; want Sleep sent", r.held, r.cmd.Mode)
	}
	if r.link.rebaselines != before+1 {
		t.Fatalf("%d rebaselines on rejoining, want 1", r.link.rebaselines-before)
	}
	r.steps(5)
	if r.cmd.Mode != uint8(padmap.ModeSleep) {
		t.Fatalf("Y held through the rejoin changed the mode to %d", r.cmd.Mode)
	}
	// A failed read on the network keeps sending.
	r.sample("", true)
	r.c.Buttons = 0
	r.steps(2)
	r.press(padmap.HandY)
	if r.held || r.cmd.Mode != uint8(padmap.ModeStand) {
		t.Fatalf("Y after release: held %v, mode %d; want Stand sent", r.held, r.cmd.Mode)
	}
}

// A leave off the network gets no telemetry, so it acts after the full
// wait, every tick held.
func TestLeaveOffNetworkActsAfterTheWait(t *testing.T) {
	r := walking(t)
	r.onDevice()
	r.sample(iwOn("Home WiFi"), false)
	r.step()
	r.a.Press(ActionExit, r.now)
	ticks := 0
	for !r.stop {
		r.step()
		ticks++
		if !r.stop && !r.held {
			t.Fatal("a frame was sent off the network during the leave")
		}
		if ticks > int(LeaveTimeout/udplink.Period)+2 {
			t.Fatal("the leave did not act")
		}
	}
	if ticks < int(LeaveTimeout/udplink.Period)-1 {
		t.Fatalf("the leave acted after %d ticks, want the full %v", ticks, LeaveTimeout)
	}
	if r.a.Ended() != ActionExit {
		t.Fatalf("ended %v, want exit", r.a.Ended())
	}
}

// The gesture overlay's inputs (add-handheld-gesture-overlay, D4): L2
// beyond half travel and A advance the request counter by one with say hi,
// the frames sent are the same whether or not the screen is read, and the
// telemetry's gesture reaches the screen by id.
func TestScreenGesture(t *testing.T) {
	plain, read := newRig(t), newRig(t)
	seq0 := read.a.Screen(read.now).Requested.GestureSeq
	var sent [2][]protocol.Command
	tick := func(i int, r *rig) {
		sent[i] = append(sent[i], r.step())
		if i == 1 {
			r.a.Screen(r.now)
		}
	}
	both := func(c padmap.Controls, n int) {
		for k := 0; k < n; k++ {
			for i, r := range []*rig{plain, read} {
				r.c = c
				tick(i, r)
			}
		}
	}
	both(padmap.Controls{}, 2)
	both(padmap.Controls{TriggerL: padmap.TriggerThreshold + 1}, 2)
	both(padmap.Controls{TriggerL: padmap.TriggerThreshold + 1, Buttons: padmap.HandA}, 2)
	sc := read.a.Screen(read.now)
	if sc.Held&padmap.HandL2 == 0 {
		t.Fatalf("held %b, want L2", sc.Held)
	}
	if sc.Requested.GestureSeq != seq0+1 || sc.Requested.Gesture != protocol.GestureSayHi {
		t.Fatalf("request %d #%d, want say hi #%d", sc.Requested.Gesture, sc.Requested.GestureSeq, seq0+1)
	}
	both(padmap.Controls{}, 2)
	for k := range sent[0] {
		if sent[0][k] != sent[1][k] {
			t.Fatalf("tick %d: sent %+v while reading the screen, %+v without", k, sent[1][k], sent[0][k])
		}
	}
	if last := sent[1][len(sent[1])-1]; last.Gesture != protocol.GestureSayHi || last.GestureRequestCounter != sc.Requested.GestureSeq {
		t.Fatalf("command %+v, want the say-hi request", last)
	}

	b := protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: uint8(session.BodyPerform), OutputsOn: true, Gesture: protocol.GestureWiggle}, 3)
	if _, ok := read.link.rx.Datagram(b, read.now); !ok {
		t.Fatal("telemetry refused")
	}
	if sc := read.a.Screen(read.now); sc.Robot.GestureID != protocol.GestureWiggle || sc.Robot.Body != "Perform" {
		t.Fatalf("robot %+v, want wiggle (2) in Perform", sc.Robot)
	}
}
