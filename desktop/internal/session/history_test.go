package session

import (
	"bytes"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

func diagnosticsFrame(d protocol.Diagnostics) protocol.Frame {
	return protocol.Frame{Type: protocol.TypeDiagnostics, Diagnostics: d}
}

func poseFrame(servoCentideg int16) protocol.Frame {
	var p protocol.Pose
	p.Flags = protocol.PoseOutputsOn
	for i := range p.Joints {
		p.Joints[i] = protocol.PoseJoint{ServoCentideg: servoCentideg, PulseUs: 1500}
	}
	return protocol.Frame{Type: protocol.TypePose, Pose: p}
}

// Telemetry, diagnostics and poses are kept per connection, ten minutes
// back, and a new connection starts empty (spec "New connection").
func TestHistoryKeepsTenMinutesPerConnection(t *testing.T) {
	s, c := connected(t)
	for i := 0; i < HistorySamples+100; i++ {
		s.Frame(telemetryFrame(protocol.Telemetry{VoltageMv: uint16(6000 + i%10)}), link.Stats{Valid: uint32(i)})
		s.Frame(diagnosticsFrame(protocol.Diagnostics{MotionSharePermille: 1000}), link.Stats{})
		c.advance(100 * time.Millisecond)
	}
	h, version := s.HistorySince(time.Time{})
	if len(h.Telemetry) != HistorySamples || len(h.Diagnostics) != HistorySamples {
		t.Fatalf("kept %d telemetry, %d diagnostics", len(h.Telemetry), len(h.Diagnostics))
	}
	if span := h.Telemetry[len(h.Telemetry)-1].At.Sub(h.Telemetry[0].At); span > HistoryLength {
		t.Fatalf("span %v", span)
	}
	// A window query returns only its samples.
	recent, _ := s.HistorySince(c.now().Add(-30 * time.Second))
	if n := len(recent.Telemetry); n < 299 || n > 301 {
		t.Fatalf("30 s window: %d samples", n)
	}
	s.Frame(telemetryFrame(protocol.Telemetry{}), link.Stats{})
	if s.HistoryVersion() == version {
		t.Fatal("version unchanged by a new sample")
	}
	s.LinkState(link.State{Status: link.Disconnected})
	s.LinkState(link.State{Status: link.Connected, Port: device})
	if h, _ := s.HistorySince(time.Time{}); len(h.Telemetry)+len(h.Diagnostics)+len(h.Poses) != 0 || !h.Start.Equal(c.now()) {
		t.Fatalf("not cleared on a new connection: %+v", h.Start)
	}
}

// Counter rates across a 16-bit wrap.
func TestCountRateAcrossAWrap(t *testing.T) {
	if r := CountRate(65530, 4, 100*time.Millisecond); r != 100 {
		t.Fatalf("rate %v", r)
	}
	if !math.IsNaN(CountRate(1, 2, 0)) {
		t.Fatal("rate with no interval")
	}
}

// Joint speed: the fastest servo angle change per joint kind between poses.
func TestJointSpeedFromConsecutivePoses(t *testing.T) {
	s, c := connected(t)
	s.Frame(poseFrame(9000), link.Stats{})
	c.advance(100 * time.Millisecond)
	f := poseFrame(9000)
	f.Pose.Joints[4].ServoCentideg = 9000 + 2000 // leg 2 femur: 20 deg in 100 ms
	f.Pose.Joints[2].ServoCentideg = 9000 - 500  // leg 1 tibia: 5 deg
	s.Frame(f, link.Stats{})
	h, _ := s.HistorySince(time.Time{})
	if !math.IsNaN(h.Poses[0].FastestDeg[Femur]) {
		t.Fatal("speed for the first pose")
	}
	got := h.Poses[1].FastestDeg
	if math.Abs(got[Coxa]) > 1e-9 || math.Abs(got[Femur]-200) > 1e-6 || math.Abs(got[Tibia]-50) > 1e-6 {
		t.Fatalf("speeds %v", got)
	}
	if h.Poses[1].ServoDeg[4] != 110 {
		t.Fatalf("servo angle %v", h.Poses[1].ServoDeg[4])
	}
}

// Spec "Save": a header and one row per telemetry sample, written whole.
func TestSaveCSV(t *testing.T) {
	s, c := connected(t)
	start := c.now()
	for i := 0; i < 600; i++ {
		s.Frame(telemetryFrame(protocol.Telemetry{VoltageMv: 6000, CurrentMa: 1500, ValidFrames: uint16(5 * i)}), link.Stats{Valid: uint32(10 * i)})
		if i == 10 {
			s.Frame(diagnosticsFrame(protocol.Diagnostics{MotionSharePermille: 412, TickMaxUs: 6480, TripCurrentMa: 9000}), link.Stats{})
		}
		c.advance(100 * time.Millisecond)
	}
	h, _ := s.HistorySince(time.Time{})
	dir := t.TempDir()
	path, err := h.SaveCSV(dir)
	if err != nil {
		t.Fatal(err)
	}
	if want := filepath.Join(dir, "spooder-"+start.Format("20060102-150405")+".csv"); path != want {
		t.Fatalf("path %s, want %s", path, want)
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(string(data)), "\n")
	if len(lines) != 601 || !strings.HasPrefix(lines[0], "time_s,body_mode,gait,gesture,source,bridge_link_up,bridge_gamepad,outputs_on,faults,voltage_v,current_a,") {
		t.Fatalf("%d lines, header %q", len(lines), lines[0])
	}
	// Row 2 (the second sample): 50 robot link frames/s, 100 host frames/s,
	// and no diagnostics yet; row 12 has them.
	cells := strings.Split(lines[2], ",")
	if cells[0] != "0.100" || cells[9] != "6.000" || cells[10] != "1.500" || cells[11] != "50.0" || cells[15] != "100.0" || cells[18] != "" {
		t.Fatalf("row 2: %v", cells)
	}
	if cells = strings.Split(lines[12], ","); cells[18] != "41.2" || cells[20] != "6.480" || cells[22] != "9.000" {
		t.Fatalf("row 12: %v", cells)
	}
	if entries, _ := os.ReadDir(dir); len(entries) != 1 {
		t.Fatalf("left %d files", len(entries))
	}
	var b bytes.Buffer
	if err := (History{}).WriteCSV(&b); err != nil || strings.Count(b.String(), "\n") != 1 {
		t.Fatalf("empty history: %q %v", b.String(), err)
	}
}

// A kind that stops arriving ages out with the others.
func TestStoppedStreamsAgeOut(t *testing.T) {
	s, c := connected(t)
	s.Frame(diagnosticsFrame(protocol.Diagnostics{MotionSharePermille: 500}), link.Stats{})
	s.Frame(poseFrame(9000), link.Stats{})
	for i := 0; i < 6100; i++ { // telemetry alone, for over ten minutes
		c.advance(100 * time.Millisecond)
		s.Frame(telemetryFrame(protocol.Telemetry{}), link.Stats{})
	}
	h, _ := s.HistorySince(time.Time{})
	if len(h.Diagnostics) != 0 || len(h.Poses) != 0 {
		t.Fatalf("kept %d diagnostics and %d poses past ten minutes", len(h.Diagnostics), len(h.Poses))
	}
}

// The downloads directory falls back when the configured one is missing.
func TestDownloadsDirFallsBack(t *testing.T) {
	home := t.TempDir()
	t.Setenv("HOME", home)
	configured := t.TempDir()
	t.Setenv("XDG_DOWNLOAD_DIR", configured)
	if d, _ := DownloadsDir(); d != configured {
		t.Fatalf("configured: %s", d)
	}
	t.Setenv("XDG_DOWNLOAD_DIR", filepath.Join(home, "gone"))
	if d, _ := DownloadsDir(); d != home {
		t.Fatalf("missing configured, no Downloads: %s", d)
	}
	if err := os.Mkdir(filepath.Join(home, "Downloads"), 0o755); err != nil {
		t.Fatal(err)
	}
	if d, _ := DownloadsDir(); d != filepath.Join(home, "Downloads") {
		t.Fatalf("missing configured, with Downloads: %s", d)
	}
}

// The download directory from user-dirs.dirs, with no environment variable.
func TestDownloadsDirFromUserDirs(t *testing.T) {
	home := t.TempDir()
	t.Setenv("HOME", home)
	t.Setenv("XDG_DOWNLOAD_DIR", "")
	t.Setenv("XDG_CONFIG_HOME", "")
	if err := os.MkdirAll(filepath.Join(home, ".config"), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(filepath.Join(home, "Letöltések"), 0o755); err != nil {
		t.Fatal(err)
	}
	dirs := "# written by xdg-user-dirs-update\nXDG_DESKTOP_DIR=\"$HOME/Asztal\"\nXDG_DOWNLOAD_DIR=\"$HOME/Letöltések\"\n"
	if err := os.WriteFile(filepath.Join(home, ".config", "user-dirs.dirs"), []byte(dirs), 0o644); err != nil {
		t.Fatal(err)
	}
	if d, _ := DownloadsDir(); d != filepath.Join(home, "Letöltések") {
		t.Fatalf("from user-dirs.dirs: %s", d)
	}
}

// The CSV names the gesture playing, the source and the bridge link.
func TestCSVGestureAndSource(t *testing.T) {
	s, _ := connected(t)
	s.Frame(telemetryFrame(protocol.Telemetry{BodyMode: uint8(BodyPerform), Gesture: protocol.GestureSayHi,
		Flags: protocol.TelemetrySourceConflict | protocol.TelemetryBridgeLinkUp}), link.Stats{})
	h, _ := s.HistorySince(time.Time{})
	var b bytes.Buffer
	if err := h.WriteCSV(&b); err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(b.String()), "\n")
	header, cells := strings.Split(lines[0], ","), strings.Split(lines[1], ",")
	got := map[string]string{}
	for i, name := range header {
		got[name] = cells[i]
	}
	if got["body_mode"] != "Perform" || got["gesture"] != GestureName(protocol.GestureSayHi) || got["source"] != "conflict" ||
		got["bridge_link_up"] != "1" || got["bridge_gamepad"] != "0" {
		t.Fatalf("row %v", got)
	}
}
