package link

import (
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

var servo = PortInfo{Name: "/dev/ttyACM0", IsUSB: true, VID: "2E8A", PID: "000A", SerialNumber: "E66138935F2A6B24"}

var fast = Options{CommandPeriod: 5 * time.Millisecond, RetryInterval: 10 * time.Millisecond, ReadTimeout: 5 * time.Millisecond}

func telemetry(seq uint8) []byte {
	return protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: 3, VoltageMv: 6000}, seq)
}

// start connects a link to a fake Servo 2040 and returns its first port.
func start(t *testing.T, src Source) (*Link, *fakeSystem, *recorder, *fakePort) {
	t.Helper()
	sys := newFakeSystem(servo)
	rec := newRecorder()
	if src == nil {
		src = &staticSource{}
	}
	l := New(sys, rec, src, fast)
	l.Connect(servo)
	t.Cleanup(l.Close)
	select {
	case p := <-sys.opened:
		return l, sys, rec, p
	case <-time.After(time.Second):
		t.Fatal("port not opened")
	}
	return nil, nil, nil, nil
}

func TestPreselectsTheServo2040(t *testing.T) {
	ports := []PortInfo{
		{Name: "/dev/ttyS0"},
		{Name: "/dev/ttyUSB0", IsUSB: true, VID: "10C4", PID: "EA60", Product: "CP2102"},
		servo,
	}
	if i := Preselect(ports); i != 2 {
		t.Fatalf("preselected %d", i)
	}
	if i := Preselect(ports[:2]); i != -1 {
		t.Fatalf("preselected %d without a Servo 2040", i)
	}
	if !strings.Contains(servo.Label(), "2e8a:000a Servo 2040") {
		t.Fatalf("label %q", servo.Label())
	}
}

func TestInterleavedTextAndFrames(t *testing.T) {
	_, _, rec, p := start(t, nil)
	var stream []byte
	stream = append(stream, "boot ok\r\n"...)
	stream = append(stream, telemetry(1)...)
	stream = append(stream, "source: usb\n"...)
	stream = append(stream, telemetry(2)...)
	stream = append(stream, "tail"...)
	stream = append(stream, '\n')
	p.send(stream)
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.frames) == 2 && len(r.lines) == 3 }) {
		t.Fatalf("frames %d lines %d", len(rec.frames), len(rec.lines))
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	want := []string{"boot ok", "source: usb", "tail"}
	for i, l := range rec.lines {
		if l.Text != want[i] || l.Stray != 0 {
			t.Errorf("line %d: %+v", i, l)
		}
	}
	if got := strings.Join(rec.events, ","); got != "line,frame,line,frame,line" {
		t.Errorf("order %s", got)
	}
	if rec.stats.Valid != 2 || rec.stats.Discarded != 0 {
		t.Errorf("stats %+v", rec.stats)
	}
}

func TestStrayStartMarkerIsShownInTheText(t *testing.T) {
	_, _, rec, p := start(t, nil)
	stream := append([]byte("volt\xA5age\n"), telemetry(5)...)
	p.send(stream)
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.frames) == 1 && len(r.lines) == 1 }) {
		t.Fatal("frame or line missing")
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	if l := rec.lines[0]; l.Text != `volt\xA5age` || l.Stray != 1 {
		t.Errorf("line %+v", l)
	}
	if rec.stats.Stray != 1 || rec.stats.Discarded != 1 || rec.stats.Valid != 1 {
		t.Errorf("stats %+v", rec.stats)
	}
}

// A line carries the counts as of its own end, including the candidate its
// stray marker started.
func TestLineCarriesCurrentCounts(t *testing.T) {
	_, _, rec, p := start(t, nil)
	p.send([]byte("\xA5x\n"))
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.lines) == 1 }) {
		t.Fatal("no line")
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	if rec.stats.Discarded != 1 || rec.stats.Stray != 1 {
		t.Fatalf("stats %+v", rec.stats)
	}
}

func TestPartialReads(t *testing.T) {
	_, _, rec, p := start(t, nil)
	var stream []byte
	stream = append(stream, "a line split "...)
	stream = append(stream, telemetry(1)...)
	stream = append(stream, protocol.EncodeCalibrationReport(protocol.CalibrationReport{LastRequestID: 9, Result: 0}, 2)...)
	stream = append(stream, "in pieces\n"...)
	// One byte, then chunks of 7, with pauses shorter than the read timeout
	// between some of them.
	p.send(stream[:1])
	for i := 1; i < len(stream); i += 7 {
		end := min(i+7, len(stream))
		p.send(stream[i:end])
		if i%3 == 0 {
			time.Sleep(time.Millisecond)
		}
	}
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.frames) == 2 && len(r.lines) == 1 }) {
		t.Fatalf("frames %d lines %d", len(rec.frames), len(rec.lines))
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	if rec.lines[0].Text != "a line split in pieces" {
		t.Errorf("line %q", rec.lines[0].Text)
	}
	if rec.frames[1].Type != protocol.TypeCalibrationReport || rec.frames[1].CalibrationReport.LastRequestID != 9 {
		t.Errorf("report %+v", rec.frames[1])
	}
}

// A frame cut short (bytes lost on the way) followed by silence and then a
// complete frame: the cut frame is abandoned when the line goes quiet, and
// the next frame is decoded.
func TestTruncatedFrameThenQuietLine(t *testing.T) {
	_, _, rec, p := start(t, nil)
	p.send(telemetry(1)[:20])
	time.Sleep(4 * fast.ReadTimeout)
	p.send(telemetry(2))
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.frames) == 1 }) {
		t.Fatal("frame after truncation not decoded")
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	if rec.frames[0].Seq != 2 || rec.stats.Discarded != 1 {
		t.Errorf("seq %d stats %+v", rec.frames[0].Seq, rec.stats)
	}
}

func TestWriterSendsCommandsThenAtMostOneSetPerPeriod(t *testing.T) {
	src := &staticSource{
		cmd:  protocol.Command{Mode: 2, Speed: 44, Flags: protocol.FlagControllerConnected},
		sets: []protocol.CalibrationSet{{RequestID: 1, Joint: 3}, {RequestID: 2, Joint: 4}},
	}
	_, _, _, p := start(t, src)
	time.Sleep(20 * fast.CommandPeriod)
	frames := p.written()
	if len(frames) < 5 {
		t.Fatalf("only %d frames written", len(frames))
	}
	if frames[0].Type != protocol.TypeCommand || frames[1].Type != protocol.TypeCalibrationSet ||
		frames[2].Type != protocol.TypeCommand || frames[3].Type != protocol.TypeCalibrationSet ||
		frames[4].Type != protocol.TypeCommand {
		t.Fatalf("order %v %v %v %v %v", frames[0].Type, frames[1].Type, frames[2].Type, frames[3].Type, frames[4].Type)
	}
	if frames[1].CalibrationSet.RequestID != 1 || frames[3].CalibrationSet.RequestID != 2 {
		t.Fatal("sets out of order")
	}
	for i, f := range frames {
		if f.Seq != uint8(i) {
			t.Fatalf("frame %d has seq %d", i, f.Seq)
		}
		if f.Type == protocol.TypeCommand && f.Command != src.cmd {
			t.Fatalf("command %+v", f.Command)
		}
	}
}

func TestDisconnectSendsAFinalFrameWithoutController(t *testing.T) {
	src := &staticSource{cmd: protocol.Command{Mode: 3, Flags: protocol.FlagControllerConnected | protocol.FlagEmergencyStop}}
	l, _, rec, p := start(t, src)
	time.Sleep(5 * fast.CommandPeriod)
	l.Disconnect()
	if !p.isClosed() {
		t.Fatal("port left open")
	}
	frames := p.written()
	last := frames[len(frames)-1]
	if last.Type != protocol.TypeCommand || last.Command.Flags != protocol.FlagEmergencyStop {
		t.Fatalf("last frame %+v", last)
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	if s := rec.states[len(rec.states)-1]; s.Status != Disconnected {
		t.Fatalf("state %+v", s)
	}
}

func TestReconnectsToTheSameDeviceAfterUnplug(t *testing.T) {
	_, sys, rec, p := start(t, nil)
	p.send(telemetry(1))
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.frames) == 1 }) {
		t.Fatal("no frame")
	}
	// Unplug: the port fails and the device disappears from the list.
	sys.setPresent(false, "")
	p.unplug()
	if !rec.waitFor(time.Second, func(r *recorder) bool {
		return len(r.states) > 0 && r.states[len(r.states)-1].Status == Waiting
	}) {
		t.Fatal("loss not reported")
	}
	time.Sleep(3 * fast.RetryInterval)
	// Replug under another name: the same serial number is found again.
	sys.setPresent(true, "/dev/ttyACM1")
	var p2 *fakePort
	select {
	case p2 = <-sys.opened:
	case <-time.After(time.Second):
		t.Fatal("not reopened")
	}
	if !rec.waitFor(time.Second, func(r *recorder) bool {
		s := r.states[len(r.states)-1]
		return s.Status == Connected && s.Port.Name == "/dev/ttyACM1"
	}) {
		t.Fatal("reconnection not reported")
	}
	p2.send(telemetry(2))
	if !rec.waitFor(time.Second, func(r *recorder) bool { return len(r.frames) == 2 }) {
		t.Fatal("no frame after reconnect")
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	// The new connection decodes afresh.
	if rec.stats.Valid != 1 {
		t.Errorf("stats %+v", rec.stats)
	}
	var sawWaiting bool
	for _, s := range rec.states {
		if s.Status == Waiting {
			sawWaiting = true
			if s.Err == nil {
				t.Error("waiting without a reason")
			}
		}
	}
	if !sawWaiting {
		t.Error("never waiting")
	}
}

func TestOpenFailureIsRetriedAndReported(t *testing.T) {
	sys := newFakeSystem(servo)
	sys.openErr = errBusyForTest
	rec := newRecorder()
	l := New(sys, rec, &staticSource{}, fast)
	l.Connect(servo)
	defer l.Close()
	if !rec.waitFor(time.Second, func(r *recorder) bool {
		return len(r.states) >= 2 && r.states[len(r.states)-1].Err == errBusyForTest
	}) {
		t.Fatal("open failure not reported")
	}
	sys.mu.Lock()
	sys.openErr = nil
	sys.mu.Unlock()
	select {
	case <-sys.opened:
	case <-time.After(time.Second):
		t.Fatal("not retried")
	}
}

var errBusyForTest = &testError{"busy"}

type testError struct{ s string }

func (e *testError) Error() string { return e.s }

func TestSameDevice(t *testing.T) {
	moved := servo
	moved.Name = "/dev/ttyACM3"
	other := servo
	other.SerialNumber = "OTHER"
	if !SameDevice(servo, moved) || SameDevice(servo, other) {
		t.Fatal("serial number matching")
	}
	plain := PortInfo{Name: "/dev/ttyS1"}
	if !SameDevice(plain, PortInfo{Name: "/dev/ttyS1"}) || SameDevice(plain, PortInfo{Name: "/dev/ttyS2"}) {
		t.Fatal("name matching")
	}
}

func TestTextAccumulator(t *testing.T) {
	var lines []Line
	acc := textAccumulator{emit: func(l Line) { lines = append(lines, l) }}
	for _, b := range []byte("tab\there\r\n\x01\n\n") {
		acc.add(b)
	}
	for i := 0; i < MaxLineLength+3; i++ {
		acc.add('x')
	}
	if len(lines) != 4 || lines[0].Text != "tab\there" || lines[1].Text != `\x01` || lines[1].Binary != 1 || lines[2].Text != "" {
		t.Fatalf("lines %+v", lines[:min(3, len(lines))])
	}
	if len(lines[3].Text) != MaxLineLength || !acc.pending() {
		t.Fatalf("long line %d", len(lines[3].Text))
	}
}
