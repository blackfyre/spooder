package ui

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"gioui.org/io/key"
	"go.bug.st/serial"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/export"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

func TestSelectionMoves(t *testing.T) {
	s := selection{}
	if s = s.move(key.NameLeftArrow); s.joint != 17 {
		t.Fatalf("left from 0: %d", s.joint)
	}
	if s = s.move(key.NameRightArrow); s.joint != 0 {
		t.Fatalf("right from 17: %d", s.joint)
	}
	if s = s.move(key.NameUpArrow); s.field != calib.FieldDirection {
		t.Fatalf("up from offset: %d", s.field)
	}
	if s = s.move(key.NameDownArrow); s.field != calib.FieldOffset {
		t.Fatalf("down from direction: %d", s.field)
	}
}

func TestNudgeKeys(t *testing.T) {
	for _, tc := range []struct {
		name   key.Name
		mods   key.Modifiers
		steps  int
		coarse bool
	}{
		{"=", 0, 1, false},
		{"+", key.ModShift, 1, true}, // Shift+= on US layouts
		{"+", 0, 1, false},           // + unshifted
		{"*", key.ModShift, 1, true}, // Shift and + where + is unshifted
		{"-", 0, -1, false},
		{"_", key.ModShift, -1, true},
		{"-", key.ModShift, -1, true},
	} {
		field, steps, coarse, ok := nudgeKey(tc.name, tc.mods)
		if !ok || field != nil || steps != tc.steps || coarse != tc.coarse {
			t.Errorf("%q %v: %v %d %v %v", tc.name, tc.mods, field, steps, coarse, ok)
		}
	}
	if field, _, _, ok := nudgeKey("R", 0); !ok || field == nil || *field != calib.FieldDirection {
		t.Fatal("R does not flip the direction")
	}
	if _, _, _, ok := nudgeKey("Q", 0); ok {
		t.Fatal("Q nudges")
	}
}

func TestBridgeText(t *testing.T) {
	for _, tc := range []struct {
		tel           protocol.Telemetry
		state, counts string
	}{
		// Down still shows the counts: a stalled or failing link.
		{protocol.Telemetry{BridgeValidFrames: 9, BridgeDiscardedFrames: 4}, "down", "9 ok · 4 bad"},
		{protocol.Telemetry{Flags: protocol.TelemetryBridgeLinkUp, BridgeValidFrames: 12}, "up · no pad", "12 ok · 0 bad"},
		{protocol.Telemetry{Flags: protocol.TelemetryBridgeLinkUp | protocol.TelemetryBridgeGamepad, BridgeValidFrames: 3, BridgeDiscardedFrames: 1}, "up · pad", "3 ok · 1 bad"},
	} {
		if state, counts := bridgeText(tc.tel); state != tc.state || counts != tc.counts {
			t.Errorf("%q %q, want %q %q", state, counts, tc.state, tc.counts)
		}
	}
}

func TestFormatting(t *testing.T) {
	j := calib.Joint{OffsetCentideg: -1850, Pulse0Us: 510, Pulse180Us: 2500, Reversed: true}
	for f, want := range map[calib.Field]string{
		calib.FieldOffset: "-18.50°", calib.FieldPulse0: "510 µs", calib.FieldPulse180: "2500 µs", calib.FieldDirection: "reversed",
	} {
		if got := fieldText(1, j, f); got != want {
			t.Errorf("field %d: %q", f, got)
		}
	}
	// Coxa offsets show from the servo centre (stored minus 90°).
	for stored, want := range map[int16]string{9300: "+3.00°", 9000: "0.00°", 8550: "-4.50°", 0: "-90.00°"} {
		if got := fieldText(3, calib.Joint{OffsetCentideg: stored}, calib.FieldOffset); got != want {
			t.Errorf("coxa %d: %q", stored, got)
		}
	}
	if fieldLabel(0, calib.FieldOffset) != "offset from centre" || fieldLabel(1, calib.FieldOffset) != "offset" {
		t.Fatal("offset labels")
	}
	if voltsText(5480) != "5.48 V" || ampsText(65535) != "65.53 A" || ampsText(120) != "0.12 A" {
		t.Fatal("units")
	}
	if ageText(1500*time.Millisecond) != "1.5 s" {
		t.Fatal(ageText(1500 * time.Millisecond))
	}
}

func TestConnectionTextNamesThePermissionFix(t *testing.T) {
	p := link.PortInfo{Name: "/dev/ttyACM0"}
	got := connectionText(link.State{Status: link.Waiting, Port: p, Err: &serial.PortError{}})
	if strings.Contains(got, "dialout") {
		t.Fatal("generic port error named as a permission problem")
	}
	got = connectionText(link.State{Status: link.Waiting, Port: p, Err: errors.Join(link.ErrLost, os.ErrPermission)})
	if !strings.Contains(got, "dialout") {
		t.Fatalf("permission hint missing: %q", got)
	}
	if got := connectionText(link.State{Status: link.Connected, Port: p}); got != "connected /dev/ttyACM0" {
		t.Fatal(got)
	}
}

// checkoutDir makes a minimal spooder checkout.
func checkoutDir(t *testing.T) string {
	t.Helper()
	dir := t.TempDir()
	for _, p := range []string{export.DefaultsPath, filepath.Join("openspec", "project.md")} {
		full := filepath.Join(dir, p)
		if err := os.MkdirAll(filepath.Dir(full), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(full, []byte("old\n"), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	return dir
}

func TestCheckoutCandidates(t *testing.T) {
	root := checkoutDir(t)
	exe := filepath.Join(root, "build", "desktop", "spooder-calibrator")
	got := CheckoutCandidates("", root, root+"/", exe)
	if len(got) != 1 || got[0] != root {
		t.Fatalf("candidates %v", got)
	}
	other := t.TempDir()
	got = CheckoutCandidates(other, "", root, "")
	if len(got) != 2 || got[0] != other {
		t.Fatalf("candidates %v", got)
	}
	m := exportModel{candidates: got}
	m.preferred()
	if m.dir() != root {
		t.Fatalf("preferred %q", m.dir())
	}
}

func table() [calib.Joints]calib.Joint {
	var tb [calib.Joints]calib.Joint
	for i := range tb {
		tb[i] = calib.Joint{OffsetCentideg: 9000, Pulse0Us: 500, Pulse180Us: 2500}
	}
	return tb
}

func TestExportModel(t *testing.T) {
	notCheckout := t.TempDir()
	root := checkoutDir(t)
	var saved string
	m := exportModel{candidates: []string{notCheckout, root}, save: func(d string) error { saved = d; return nil }}

	// Spec "Not a checkout": refused, and why.
	m.prepare(table())
	if m.plan != nil || !m.failed || !strings.Contains(m.status, "calibration_defaults.cpp") {
		t.Fatalf("not a checkout: %+v", m)
	}
	m.write(table()) // nothing to write
	if entries, _ := os.ReadDir(notCheckout); len(entries) != 0 {
		t.Fatal("wrote into a non-checkout")
	}

	m.choose(1)
	tb := table()
	m.prepare(tb)
	if m.plan == nil || !m.canWrite(tb) {
		t.Fatalf("plan %+v", m)
	}
	// The table changes after the difference was shown: no write.
	changed := tb
	changed[3].Pulse0Us = 510
	if !m.stale(changed) || m.canWrite(changed) {
		t.Fatal("stale plan writable")
	}
	m.write(changed)
	if b, _ := os.ReadFile(filepath.Join(root, export.DefaultsPath)); string(b) != "old\n" {
		t.Fatal("stale plan written")
	}
	// Confirmed with the table it was made for: written and remembered.
	m.write(tb)
	if b, _ := os.ReadFile(filepath.Join(root, export.DefaultsPath)); string(b) != export.Format(tb) {
		t.Fatal("not written")
	}
	if saved != root || m.plan != nil || m.failed {
		t.Fatalf("after write: saved %q %+v", saved, m)
	}
}

// snapshotAfter is a session's snapshot once it has received @p tel.
func snapshotAfter(tel protocol.Telemetry) session.Snapshot {
	s := session.New(func() time.Time { return clock }, nil)
	s.Frame(protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: tel}, link.Stats{})
	return s.Snapshot()
}

func bannerTitles(bs []bannerSpec) []string {
	var out []string
	for _, b := range bs {
		out = append(out, b.title)
	}
	return out
}

// Spec "Robot in simulation": the banner and the output as simulated with the
// flag; neither without it.
func TestSimulationBannerAndOutput(t *testing.T) {
	sim := protocol.Telemetry{BodyMode: uint8(session.BodyWalk), OutputsOn: true, VoltageMv: 6000,
		Flags: protocol.TelemetrySimulated | protocol.TelemetryUSBInControl}
	snap := snapshotAfter(sim)
	bs := bannerSpecs(snap, "")
	if len(bs) != 1 || bs[0].title != simulationTitle || bs[0].kind != bannerWarning || !strings.Contains(bs[0].body, "No servo moves") {
		t.Fatalf("banners %+v", bs)
	}
	if got := outputText(snap.Telemetry); got != "on (sim)" {
		t.Errorf("output %q", got)
	}
	// Output off in simulation is plainly off.
	off := sim
	off.OutputsOn = false
	if got := outputText(off); got != "off" {
		t.Errorf("output off %q", got)
	}
	// Beside the other banners, in order.
	snap.EmergencyStop = true
	if got := bannerTitles(bannerSpecs(snap, "busy")); strings.Join(got, "|") != "Emergency stop|Bench simulation|Not done" {
		t.Errorf("order %v", got)
	}

	plain := sim
	plain.Flags = protocol.TelemetryUSBInControl
	snap = snapshotAfter(plain)
	if bs := bannerSpecs(snap, ""); len(bs) != 0 {
		t.Errorf("banners without the flag %+v", bs)
	}
	if got := outputText(snap.Telemetry); got != "on" {
		t.Errorf("output without the flag %q", got)
	}
	// No telemetry yet: no banner.
	var none session.Snapshot
	none.Telemetry.Flags = protocol.TelemetrySimulated
	if bs := bannerSpecs(none, ""); len(bs) != 0 {
		t.Errorf("banners without telemetry %+v", bs)
	}
}

// Spec "Supply appears in simulation": the fault named, not by bit number.
func TestSimulationSupplyFaultIsNamed(t *testing.T) {
	snap := snapshotAfter(protocol.Telemetry{Flags: protocol.TelemetrySimulated, Faults: session.FaultSimulationSupply})
	if got := faultsText(snap.Faults); got != "supply in simulation" {
		t.Errorf("faults %q", got)
	}
}
