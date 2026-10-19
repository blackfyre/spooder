package console

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"errors"
	"flag"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// goldenPath is test/golden/protocol_v3.txt, written by the C++ encoder.
var goldenPath = filepath.Join("..", "..", "..", "test", "golden", "protocol_v3.txt")

type vector struct {
	name     string
	frame    []byte
	rejected bool
}

func readVectors(t *testing.T) []vector {
	t.Helper()
	f, err := os.Open(goldenPath)
	if err != nil {
		t.Fatalf("golden vectors: %v", err)
	}
	defer f.Close()
	var out []vector
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	for sc.Scan() {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		parts := strings.Split(text, " ")
		frame, err := hex.DecodeString(parts[1])
		if err != nil {
			t.Fatal(err)
		}
		out = append(out, vector{name: parts[0], frame: frame, rejected: len(parts) == 3 && parts[2] == "rejected"})
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	return out
}

func decodeOne(t *testing.T, b []byte) protocol.Frame {
	t.Helper()
	var d protocol.Decoder
	for _, c := range b {
		d.Push(c)
	}
	f, ok := d.Next()
	if !ok {
		t.Fatalf("no frame in %x", b)
	}
	return f
}

// The one-line decode of every golden vector.
func TestFormatGoldenVectors(t *testing.T) {
	want := map[string]string{
		"command_walk":                         "command seq=0 mode=2 translateX=0 translateY=50 rotate=0 height=50 speed=44 gait=0 stride=0 flags=1 clearCounter=0 modeRequestCounter=0 gesture=0 gestureRequestCounter=0",
		"command_extremes_seq_wrap":            "command seq=255 mode=4 translateX=-128 translateY=127 rotate=-100 height=255 speed=100 gait=5 stride=1 flags=255 clearCounter=255 modeRequestCounter=255 gesture=255 gestureRequestCounter=255",
		"command_stand_gesture_wiggle":         "command seq=17 mode=0 translateX=0 translateY=0 rotate=0 height=30 speed=44 gait=0 stride=0 flags=1 clearCounter=0 modeRequestCounter=4 gesture=2 gestureRequestCounter=9",
		"command_estop_no_controller":          "command seq=0 mode=3 translateX=0 translateY=0 rotate=0 height=0 speed=0 gait=0 stride=0 flags=2 clearCounter=1 modeRequestCounter=0 gesture=0 gestureRequestCounter=0",
		"telemetry_calibrate_usb":              "telemetry #7  Calibrate  gait Tri  output on  6.01 V  1.35 A  faults none  link 1000 ok 2 bad  source USB",
		"telemetry_perform_say_hi":             "telemetry #30  Perform  gait Tri  output on  6.05 V  2.10 A  faults none  link 0 ok 0 bad  source bridge  gesture say hi",
		"telemetry_conflict":                   "telemetry #128  Sleep  gait Tri  output off  0.00 V  0.00 A  faults emergency stop  link 0 ok 0 bad  source conflict (bridge and USB)",
		"telemetry_extremes":                   "telemetry #255  mode 255  gait 255  output on  65.53 V  65.53 A  faults overcurrent, sustained current, undervoltage, brown-out, overvoltage, emergency stop, fault bit 6, fault bit 7, fault bit 8, fault bit 9, fault bit 10, fault bit 11, fault bit 12, fault bit 13, fault bit 14, fault bit 15  link 65535 ok 65535 bad  source conflict (bridge and USB)  gesture 255  tof 65535 mm (status 255)  pitch -327.68 deg roll 327.67 deg",
		"calibration_set_default_joint0":       "calibration_set seq=1 requestId=1 joint=0 offsetCentideg=9000 pulse0Us=500 pulse180Us=2500 flags=0",
		"calibration_set_reversed_joint17":     "calibration_set seq=42 requestId=200 joint=17 offsetCentideg=-2450 pulse0Us=2400 pulse180Us=600 flags=1",
		"calibration_set_extremes":             "calibration_set seq=255 requestId=255 joint=255 offsetCentideg=-32768 pulse0Us=0 pulse180Us=65535 flags=255",
		"calibration_report_defaults":          "calibration report #0  request 0  none yet  reversed 0",
		"calibration_report_clamped":           "calibration report #9  request 17  clamped  reversed 1",
		"calibration_report_rejected_seq_wrap": "calibration report #0  request 18  rejected  reversed 1",
		"calibration_report_extremes":          "calibration report #255  request 255  result 255  reversed 18",
		"pose_calibration_outputs_on":          "pose seq=3 flags=1",
		"pose_output_off_legs_unset":           "pose seq=200 flags=0",
		"pose_extremes":                        "pose seq=255 flags=255",
	}
	seen := 0
	for _, v := range readVectors(t) {
		if v.rejected {
			continue
		}
		w, ok := want[v.name]
		if !ok {
			t.Errorf("%s: no expected line", v.name)
			continue
		}
		seen++
		if got := FormatFrame(decodeOne(t, v.frame)); got != w {
			t.Errorf("%s:\n got %q\nwant %q", v.name, got, w)
		}
	}
	if seen != len(want) {
		t.Errorf("%d of %d vectors seen", seen, len(want))
	}
}

func TestFormatSensorsAndUnknownType(t *testing.T) {
	tel := protocol.Telemetry{BodyMode: 2, Gait: 2, SensorsAvailable: protocol.SensorToF, ToFMm: 123, Flags: protocol.TelemetryBridgeInControl}
	got := FormatFrame(protocol.Frame{Type: protocol.TypeTelemetry, Seq: 1, Telemetry: tel})
	want := "telemetry #1  Walk  gait Ripple  output off  0.00 V  0.00 A  faults none  link 0 ok 0 bad  source bridge  tof 123 mm (status 0)"
	if got != want {
		t.Errorf("got %q", got)
	}
	if got := FormatFrame(protocol.Frame{Type: 9, Seq: 3}); got != "type9 seq=3" {
		t.Errorf("unknown type: %q", got)
	}
}

// body strips the timestamps.
func body(out string) []string {
	var lines []string
	for _, l := range strings.Split(strings.TrimSuffix(out, "\n"), "\n") {
		if len(l) >= len(TimeFormat)+2 {
			l = l[len(TimeFormat)+2:]
		}
		lines = append(lines, l)
	}
	return lines
}

func tel(seq uint8, t protocol.Telemetry) []byte { return protocol.EncodeTelemetry(t, seq) }

var calibrating = protocol.Telemetry{BodyMode: 4, OutputsOn: true, VoltageMv: 6012, CurrentMa: 1350, ValidFrames: 1000, DiscardedFrames: 2, Flags: protocol.TelemetryUSBInControl}

// Text and frames interleaved, a frame inside a text line, everything split
// into small reads: the text arrives whole and in order, frames apart.
func TestInterleavedTextAndFramesInSmallReads(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true}, sys, nil)
	p := waitPort(t, sys)
	var stream []byte
	stream = append(stream, "boot ok\r\n"...)
	stream = append(stream, tel(1, calibrating)...)
	stream = append(stream, "supply 1.35 A"...)
	stream = append(stream, tel(2, calibrating)...) // inside a line
	stream = append(stream, " (avg 1.30 A)\r\n"...)
	stream = append(stream, "operator source: USB\n"...)
	p.sendChunks(stream, 3)
	m.out.waitFor(t, "operator source: USB")
	m.stop()
	got := body(m.out.String())
	want := []string{
		"-- connected /dev/ttyACM0 (2e8a:000a Servo 2040)",
		"boot ok",
		"> telemetry #1  Calibrate  gait Tri  output on  6.01 V  1.35 A  faults none  link 1000 ok 2 bad  source USB",
		"> telemetry #2  Calibrate  gait Tri  output on  6.01 V  1.35 A  faults none  link 1000 ok 2 bad  source USB",
		"supply 1.35 A (avg 1.30 A)",
		"operator source: USB",
		"-- disconnected",
		"-- totals: 2 telemetry, 0 calibration reports, 0 other; 0 rejected; 0 stray 0xA5",
	}
	if strings.Join(got, "\n") != strings.Join(want, "\n") {
		t.Fatalf("got:\n%s\nwant:\n%s", strings.Join(got, "\n"), strings.Join(want, "\n"))
	}
}

// Without --frames only the text and the monitor's own notes appear.
func TestTextOnlyByDefault(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{}, sys, nil)
	p := waitPort(t, sys)
	p.send(append(append([]byte("one\n"), tel(1, calibrating)...), "two\n"...))
	m.out.waitFor(t, "two")
	m.stop()
	for _, l := range body(m.out.String()) {
		if strings.HasPrefix(l, markFrame) {
			t.Errorf("frame line without --frames: %q", l)
		}
	}
	if !strings.Contains(m.out.String(), "totals: 1 telemetry") {
		t.Errorf("telemetry not counted:\n%s", m.out.String())
	}
}

func TestStrayStartMarker(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true}, sys, nil)
	p := waitPort(t, sys)
	p.send([]byte("volt\xA5age 6.0 V\r\n"))
	m.out.waitFor(t, "stray")
	m.stop()
	got := body(m.out.String())
	// The stray marker is also a rejected candidate (its version byte is 'a').
	if got[1] != "! frame candidate rejected (1 in total)" || got[2] != `volt\xA5age 6.0 V   [1 stray 0xA5]` {
		t.Errorf("lines %q", got[1:3])
	}
	if !strings.HasSuffix(m.out.String(), "1 stray 0xA5\n") {
		t.Errorf("totals:\n%s", m.out.String())
	}
}

func TestTelemetryThrottle(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true, TelemetryEvery: 10}, sys, nil)
	p := waitPort(t, sys)
	var stream []byte
	for i := 0; i < 20; i++ {
		stream = append(stream, tel(uint8(i), calibrating)...)
	}
	faulted := calibrating
	faulted.Faults = 1 << 2
	stream = append(stream, tel(20, faulted)...)
	stream = append(stream, tel(21, faulted)...)
	stream = append(stream, "end\n"...)
	p.send(stream)
	m.out.waitFor(t, "end")
	m.stop()
	var seqs []string
	for _, l := range body(m.out.String()) {
		if strings.HasPrefix(l, "> telemetry #") {
			seqs = append(seqs, strings.Fields(l)[2])
		}
	}
	// The first, the eleventh, then the fault at once; 21 is throttled.
	if got := strings.Join(seqs, " "); got != "#0 #10 #20" {
		t.Errorf("printed %s", got)
	}
	if !strings.Contains(m.out.String(), "#20  Calibrate  gait Tri  output on  6.01 V  1.35 A  faults undervoltage") {
		t.Errorf("fault not named:\n%s", m.out.String())
	}
}

func TestRejectedCandidatesAreCounted(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true}, sys, nil)
	p := waitPort(t, sys)
	bad := tel(1, calibrating)
	bad[len(bad)-1] ^= 0xFF // CRC
	stream := append(append([]byte(nil), bad...), tel(2, calibrating)...)
	stream = append(stream, "next\n"...)
	p.send(stream)
	m.out.waitFor(t, "next")
	m.stop()
	out := m.out.String()
	if !strings.Contains(out, "! frame candidate rejected (1 in total)") {
		t.Errorf("rejection not reported:\n%s", out)
	}
	if !strings.Contains(out, "> telemetry #2 ") {
		t.Errorf("valid frame after garbage lost:\n%s", out)
	}
	if !strings.Contains(out, "; 1 rejected; 1 stray 0xA5") {
		t.Errorf("totals:\n%s", out)
	}
}

// A rejected candidate followed by silence is reported once the connection
// ends.
func TestRejectionReportedWhenTheConnectionEnds(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true}, sys, nil)
	p := waitPort(t, sys)
	p.send([]byte("x\n"))
	m.out.waitFor(t, "x")
	p.send([]byte{protocol.Start, 0x01, 0x02}) // a bad version, no newline
	time.Sleep(50 * time.Millisecond)
	m.stop()
	if !strings.Contains(m.out.String(), "! frame candidate rejected (1 in total)") {
		t.Errorf("rejection not reported:\n%s", m.out.String())
	}
}

// Without --keys nothing reaches any port, across a reconnect.
func TestNeverWrites(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true}, sys, nil)
	p := waitPort(t, sys)
	p.send(append([]byte("hello\n"), tel(1, calibrating)...))
	m.out.waitFor(t, "> telemetry #1")
	time.Sleep(100 * time.Millisecond) // five link writer periods
	sys.setPresent(false)
	p.unplug()
	m.out.waitFor(t, "lost /dev/ttyACM0 (unplugged); waiting for it")
	sys.setPresent(true)
	p2 := waitPort(t, sys)
	m.out.waitFor(t, "-- connected")
	p2.send([]byte("again\n"))
	m.out.waitFor(t, "again")
	m.stop()
	for i, port := range sys.allPorts() {
		if n, b := port.written(); n != 0 || len(b) != 0 {
			t.Errorf("port %d: %d writes %q", i, n, b)
		}
	}
	// The guard itself refuses writes.
	g := &gate{}
	gp := g.attach(newFakePort())
	if n, err := gp.Write([]byte{protocol.Start}); n != 0 || !errors.Is(err, errReadOnly) {
		t.Errorf("guarded write: %d %v", n, err)
	}
}

func TestReconnectAfterUnplug(t *testing.T) {
	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{}, sys, nil)
	p := waitPort(t, sys)
	sys.setPresent(false)
	p.unplug()
	m.out.waitFor(t, "waiting for /dev/ttyACM0 (2e8a:000a Servo 2040): device not present")
	sys.setPresent(true)
	waitPort(t, sys)
	m.out.waitFor(t, "waiting for it")
	m.stop()
	got := body(m.out.String())
	want := []string{
		"-- connected /dev/ttyACM0 (2e8a:000a Servo 2040)",
		"-- lost /dev/ttyACM0 (unplugged); waiting for it",
		"-- waiting for /dev/ttyACM0 (2e8a:000a Servo 2040): device not present",
		"-- connected /dev/ttyACM0 (2e8a:000a Servo 2040)",
		"-- disconnected",
	}
	if strings.Join(got[:5], "\n") != strings.Join(want, "\n") {
		t.Fatalf("got:\n%s", strings.Join(got, "\n"))
	}
}

// With no Servo 2040 present the monitor waits, then picks it up.
func TestWaitsForAServo2040(t *testing.T) {
	sys := newFakeSystem(servo, false)
	m := startMonitor(t, Config{}, sys, nil)
	m.out.waitFor(t, "-- waiting for a Servo 2040 (USB 2e8a:000a)")
	sys.setPresent(true)
	waitPort(t, sys)
	m.out.waitFor(t, "-- connected /dev/ttyACM0")
	m.stop()
	if n := strings.Count(m.out.String(), "waiting for a Servo 2040"); n != 1 {
		t.Errorf("waiting printed %d times", n)
	}
}

// --port names a path enumeration does not list.
func TestPortFlagUsesAnUnlistedPath(t *testing.T) {
	pty := link.PortInfo{Name: "/tmp/spooder-robot"}
	sys := newFakeSystem(pty, false)
	s := &system{base: sys, path: pty.Name, exists: func(string) bool { return true }, gate: &gate{}}
	ports, err := s.List()
	if err != nil || len(ports) != 2 || ports[1].Name != pty.Name {
		t.Fatalf("ports %+v %v", ports, err)
	}
	s.exists = func(string) bool { return false }
	if ports, _ := s.List(); len(ports) != 1 {
		t.Fatalf("absent path listed: %+v", ports)
	}
}

func TestKeysForwardPrintableASCIIOnly(t *testing.T) {
	sys := newFakeSystem(servo, true)
	keys := newKeyInput()
	m := startMonitor(t, Config{Keys: true}, sys, keys)
	p := waitPort(t, sys)
	m.out.waitFor(t, "-- connected")
	keys.ch <- []byte("s")
	keys.ch <- []byte{0x01, protocol.Start, 'w', '\r', 0x7F, 0xFF, '+'}
	deadline := time.Now().Add(2 * time.Second)
	for {
		if _, b := p.written(); string(b) == "sw+" {
			break
		}
		if time.Now().After(deadline) {
			_, b := p.written()
			t.Fatalf("written %q", b)
		}
		time.Sleep(2 * time.Millisecond)
	}
	m.stop()
	if !strings.Contains(m.out.String(), "-- forwarding typed keys") {
		t.Errorf("no key note:\n%s", m.out.String())
	}
}

func TestKeysWhileDisconnectedAreDropped(t *testing.T) {
	sys := newFakeSystem(servo, false)
	keys := newKeyInput()
	m := startMonitor(t, Config{Keys: true}, sys, keys)
	m.out.waitFor(t, "waiting for a Servo 2040")
	keys.ch <- []byte("z")
	m.out.waitFor(t, `-- not connected: dropped "z"`)
	m.stop()
}

func TestPrintable(t *testing.T) {
	in := []byte{0x00, 0x1F, ' ', 'a', '~', 0x7F, 0xA5, '\n', '\t'}
	if got := Printable(in); string(got) != " a~" {
		t.Errorf("%q", got)
	}
	if in[2] != ' ' {
		t.Error("input changed")
	}
}

func TestFlags(t *testing.T) {
	var help bytes.Buffer
	if _, err := ParseFlags([]string{"--help"}, &help); !errors.Is(err, flag.ErrHelp) {
		t.Fatalf("help: %v", err)
	}
	for _, s := range []string{"never sends a frame", "never sends an emergency stop", "--keys", "2e8a:000a", "-telemetry-every"} {
		if !strings.Contains(help.String(), s) {
			t.Errorf("help lacks %q:\n%s", s, help.String())
		}
	}
	c, err := ParseFlags([]string{"--frames", "--telemetry-every", "5", "--port", "/dev/pts/3", "--keys"}, &bytes.Buffer{})
	if err != nil || c != (Config{Port: "/dev/pts/3", Frames: true, TelemetryEvery: 5, Keys: true}) {
		t.Fatalf("%+v %v", c, err)
	}
	if c, err := ParseFlags(nil, &bytes.Buffer{}); err != nil || c.TelemetryEvery != 1 || c.Frames || c.Keys {
		t.Fatalf("defaults %+v %v", c, err)
	}
	if _, err := ParseFlags([]string{"--telemetry-every", "0"}, &bytes.Buffer{}); err == nil {
		t.Error("zero accepted")
	}
	if _, err := ParseFlags([]string{"extra"}, &bytes.Buffer{}); err == nil {
		t.Error("argument accepted")
	}
}

// transcriptPath is the sample session: the golden vectors between text
// lines, as the monitor prints them with --frames. Regenerate with
// SPOODER_WRITE_TRANSCRIPT=1 go test ./internal/console.
var transcriptPath = filepath.Join("testdata", "transcript.txt")

func TestTranscript(t *testing.T) {
	var stream []byte
	text := func(s string) { stream = append(stream, s+"\r\n"...) }
	text("spooder: Servo 2040, link build")
	text("supply 0.00 A (avg 0.00 A), 0.00 V; output off; mode 0; faults 0x00")
	for i, v := range readVectors(t) {
		stream = append(stream, v.frame...)
		if v.rejected {
			text("") // the corrupted frame's bytes end up as one text line
		}
		switch i {
		case 2:
			text("operator source: USB")
		case 5:
			text("supply 1.35 A (avg 1.30 A), 6.01 V; output on; mode 4; faults 0x00")
		case 8:
			text("fault latched: undervo\xA5ltage (0.00 A, 0.00 V)")
		}
	}
	text("movement stopped: waiting for the fault to clear and a new request")

	sys := newFakeSystem(servo, true)
	m := startMonitor(t, Config{Frames: true}, sys, nil)
	p := waitPort(t, sys)
	p.sendChunks(stream, 17)
	m.out.waitFor(t, "movement stopped")
	m.stop()
	got := m.out.String()
	if os.Getenv("SPOODER_WRITE_TRANSCRIPT") != "" {
		if err := os.WriteFile(transcriptPath, []byte(got), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	want, err := os.ReadFile(transcriptPath)
	if err != nil {
		t.Fatal(err)
	}
	if got != string(want) {
		t.Errorf("transcript differs from %s:\n%s", transcriptPath, got)
	}
}
