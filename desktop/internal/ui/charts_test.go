package ui

import (
	"gioui.org/io/pointer"

	"gioui.org/io/key"

	"math"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// feedHistory sends seconds of robot frames at 10 Hz, advancing the test
// clock: standing, then walking with a supply dip under load, the budget
// slowing the motion, and a short e-stop fault.
func feedHistory(s *session.Session, seconds int) {
	var robotOK, bridgeOK, backstop uint16
	var host uint32
	n := seconds * 10
	for i := 0; i < n; i++ {
		clock = clock.Add(100 * time.Millisecond)
		walking := i > n/4
		phase := float64(i) / 10
		t := protocol.Telemetry{BodyMode: 1, OutputsOn: true, VoltageMv: 6050, CurrentMa: 400, Gait: 0}
		d := protocol.Diagnostics{MotionSharePermille: 1000, TickMaxUs: 1900, TickMeanUs: 1650, AverageCurrentMa: 400,
			TripCurrentMa: 9000, SustainedCurrentMa: 6000, MinMv: 5200, MaxMv: 6800, BrownoutMv: 4800,
			BudgetDegPerS: 257, RatedDegPerS: 429}
		if walking {
			t.BodyMode = 2
			load := 2.2 + 1.4*math.Max(0, math.Sin(phase*4))
			t.CurrentMa = uint16(load * 1000)
			t.VoltageMv = uint16(6050 - load*180)
			d.AverageCurrentMa = 2600
			d.MotionSharePermille = uint16(420 + 300*math.Abs(math.Sin(phase*2)))
			d.TickMaxUs = uint16(6000 + 2400*math.Abs(math.Sin(phase*3)))
			d.TickMeanUs = 5200
		}
		if i > n*3/4 && i < n*3/4+15 {
			t.Faults = session.FaultEmergencyStop
			t.OutputsOn = false
			t.BodyMode = 3
		}
		if i%37 == 0 && walking {
			backstop++
		}
		robotOK += 5
		bridgeOK += 50
		host += 30
		t.ValidFrames, t.BridgeValidFrames = robotOK, bridgeOK
		d.BackstopTicks = backstop
		st := link.Stats{Valid: host, Discarded: uint32(i / 100)}
		s.Frame(protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: t}, st)
		s.Frame(protocol.Frame{Type: protocol.TypeDiagnostics, Diagnostics: d}, st)
		var p protocol.Pose
		p.Flags = protocol.PoseOutputsOn
		for j := range p.Joints {
			a := 9000.0
			if walking {
				a += 2500 * math.Sin(phase*3+float64(j))
			}
			p.Joints[j] = protocol.PoseJoint{ServoCentideg: int16(a), CommandedCentideg: int16(a), PulseUs: 1500}
		}
		s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: p}, st)
	}
}

// The model derives every chart from the history.
func TestChartsModel(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 30)
	h, _ := s.HistorySince(clock.Add(-2 * time.Minute))
	m := buildCharts(h, clock, 2*time.Minute, 1)
	if len(m.charts) != chartCount {
		t.Fatalf("%d charts", len(m.charts))
	}
	for _, c := range m.charts {
		if c.missing != "" {
			t.Errorf("%s: %s", c.title, c.missing)
		}
	}
	if got := m.charts[1].refs; len(got) != 2 || got[0].v != 9 || got[1].v != 6 {
		t.Fatalf("current limits %v", got)
	}
	if got := m.charts[4].refs; len(got) != 2 || got[1].v != 257 {
		t.Fatalf("speed references %v", got)
	}
	// 5 robot frames per 100 ms: 50 per second.
	if v := m.charts[2].series[0].pts[10].v; math.Abs(v-50) > 1e-6 {
		t.Fatalf("robot link rate %v", v)
	}
	modes := []string{}
	for _, s := range m.timeline.modes {
		modes = append(modes, s.label)
	}
	if got := strings.Join(modes, ","); got != "Stand,Walk,Sleep,Walk" {
		t.Fatalf("timeline modes %s", got)
	}
	faults := 0
	for _, f := range m.timeline.faults {
		if f.on {
			faults++
		}
	}
	if faults != 1 {
		t.Fatalf("fault spans %d", faults)
	}
	frame(w, router)
}

// Spec "Charts": a robot without diagnostics or poses says so.
func TestChartsSayWhatIsMissing(t *testing.T) {
	c := newTestClockHistory()
	m := buildCharts(c, clock, 30*time.Second, 0)
	for i, want := range []string{"", "", "", "", "no pose frames from this robot", "no pose frames from this robot",
		"no diagnostics from this robot", "no diagnostics from this robot", "no diagnostics from this robot"} {
		if m.charts[i].missing != want {
			t.Errorf("chart %d (%s): %q, want %q", i, m.charts[i].title, m.charts[i].missing, want)
		}
	}
}

func newTestClockHistory() session.History {
	s := session.New(func() time.Time { return clock }, nil)
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	for i := 0; i < 20; i++ {
		clock = clock.Add(100 * time.Millisecond)
		s.Frame(protocol.Frame{Type: protocol.TypeTelemetry, Telemetry: protocol.Telemetry{VoltageMv: 6000}}, link.Stats{})
	}
	h, _ := s.HistorySince(time.Time{})
	return h
}

// Spec "Window" and "Hover": the selector sets the span; hovering a plot
// sets the moment on every chart.
func TestChartsWindowAndHover(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 60)
	w.tab = tabCharts
	frame(w, router)
	if w.charts.model.window != 120 {
		t.Fatalf("default window %v", w.charts.model.window)
	}
	click(w, router, &w.charts.windows[0])
	if w.charts.model.window != 30 {
		t.Fatalf("30 s window %v", w.charts.model.window)
	}
	// Point at the middle of the first chart's plot (in its own
	// coordinates, as pointer events arrive).
	x0, width := w.charts.plotX[0][0], w.charts.plotX[0][1]
	if width <= 0 {
		t.Fatal("no plot laid out")
	}
	w.hoverAt(0, float32(x0+width/2))
	frame(w, router)
	if !w.charts.hoverOn || math.Abs(w.charts.hoverT+15) > 0.5 {
		t.Fatalf("hover on %v at %v s, want -15", w.charts.hoverOn, w.charts.hoverT)
	}
	// Every chart reads its value there: the voltage at -15 s.
	v := valueAt(w.charts.model.charts[0].series[0].pts, w.charts.hoverT)
	if math.IsNaN(v) || v < 5 || v > 7 {
		t.Fatalf("voltage at the hover %v", v)
	}
	// The timeline's own plot, scrolled into view, maps the same way.
	w.charts.list.Position.First = chartCount - 1
	frame(w, router)
	w.hoverAt(chartCount, float32(w.charts.plotX[chartCount][0]))
	if w.charts.hoverT != -30 {
		t.Fatalf("timeline left edge %v", w.charts.hoverT)
	}
}

// Spec "Save": the CSV button writes the history and names the file, in
// the background.
func TestChartsSaveCSV(t *testing.T) {
	dir := t.TempDir()
	t.Setenv("XDG_DOWNLOAD_DIR", dir)
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 10)
	w.tab = tabCharts
	frame(w, router)
	click(w, router, &w.charts.save)
	for i := 0; i < 200 && w.charts.saving; i++ {
		time.Sleep(5 * time.Millisecond)
		frame(w, router)
	}
	if !strings.HasPrefix(w.notice, "History saved to "+dir) {
		t.Fatalf("notice %q", w.notice)
	}
}

// A save that blocks never holds up driving or the e-stop.
func TestBlockedSaveDoesNotBlockTheKeys(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := standingInControl(t, w, router)
	feedHistory(s, 2)
	release := make(chan struct{})
	w.charts.saver = func(session.History) (string, error) { <-release; return "/tmp/x.csv", nil }
	w.tab = tabCharts
	hold(w, router, "W")
	frame(w, router)
	click(w, router, &w.charts.save)
	if !w.charts.saving {
		t.Fatal("not saving")
	}
	letGo(w, router, "W")
	press(w, router, key.NameSpace, 0) // the e-stop
	c := s.Command()
	if c.TranslateY != 0 || c.Flags&protocol.FlagEmergencyStop == 0 {
		t.Fatalf("keys held up by the save: %+v", c)
	}
	close(release)
	for i := 0; i < 200 && w.charts.saving; i++ {
		time.Sleep(5 * time.Millisecond)
		frame(w, router)
	}
	if w.notice != "History saved to /tmp/x.csv" {
		t.Fatalf("notice %q", w.notice)
	}
}

// Decimation keeps gaps: valid, missing, valid within one column stays
// broken.
func TestDecimateKeepsGaps(t *testing.T) {
	var pts []point
	for i := 0; i < 1000; i++ {
		v := float64(i % 7)
		if i == 500 {
			v = math.NaN()
		}
		pts = append(pts, point{-100 + float64(i)/10, v})
	}
	d := decimate(pts, -100, 0, 100)
	gaps := 0
	for _, p := range d {
		if math.IsNaN(p.v) {
			gaps++
		}
	}
	if gaps != 1 || len(d) >= len(pts) {
		t.Fatalf("%d gaps in %d points", gaps, len(d))
	}
}

// With no frames arriving, the charts still move on with time.
func TestChartsMoveOnWithoutFrames(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 10)
	w.tab = tabCharts
	frame(w, router)
	last := w.charts.model.charts[0].series[0].pts
	before := last[len(last)-1].t
	clock = clock.Add(5 * time.Second)
	frame(w, router)
	last = w.charts.model.charts[0].series[0].pts
	n := len(last) - 1
	for n > 0 && math.IsNaN(last[n].v) {
		n-- // the gap marker after the last sample
	}
	if after := last[n].t; math.Abs(after-(before-5)) > 0.1 {
		t.Fatalf("latest sample at %v s, then %v s", before, after)
	}
}

// After the frames stop, the state and the values are unknown, not the last
// ones stretched to now; lines and segments break at gaps.
func TestChartsShowGapsAsUnknown(t *testing.T) {
	w, _, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 20)                  // ends walking
	clock = clock.Add(10 * time.Second) // then silence
	h, _ := s.HistorySince(clock.Add(-2 * time.Minute))
	m := buildCharts(h, clock, 2*time.Minute, 0)
	if v := valueAt(m.charts[0].series[0].pts, -1); !math.IsNaN(v) {
		t.Fatalf("voltage 1 s ago %v, want unknown", v)
	}
	if v := valueAt(m.charts[0].series[0].pts, -10.05); math.IsNaN(v) {
		t.Fatal("voltage at the last sample unknown")
	}
	if got := segmentAt(m.timeline.modes, -1); got != "unknown" {
		t.Fatalf("mode 1 s ago %q", got)
	}
	if got := segmentAt(m.timeline.modes, -10.05); got != "Walk" {
		t.Fatalf("mode at the last sample %q", got)
	}
	// The e-stop span is far shorter than a label needs; the readout names it.
	var fault segment
	for _, f := range m.timeline.faults {
		if f.on {
			fault = f
		}
	}
	if got := segmentAt(m.timeline.faults, (fault.t0+fault.t1)/2); got != "emergency stop" {
		t.Fatalf("fault at its span %q", got)
	}
}

// Moving between plots keeps the hover whichever order Enter and Leave
// come in; leaving the hovered plot ends it.
func TestHoverAcrossPlots(t *testing.T) {
	w, router, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 10)
	w.tab = tabCharts
	frame(w, router)
	x := float32(w.charts.plotX[0][0] + 10)
	for _, order := range [][2]int{{1, 0}, {0, 1}} {
		from, to := order[0], order[1]
		w.hoverEvent(from, pointer.Enter, x)
		w.hoverEvent(to, pointer.Enter, x)   // the new plot first...
		w.hoverEvent(from, pointer.Leave, x) // ...then the old one's Leave
		if !w.charts.hoverOn || w.charts.hoverAt != to {
			t.Fatalf("%d to %d: hover lost", from, to)
		}
		w.hoverEvent(to, pointer.Leave, x)
		if w.charts.hoverOn {
			t.Fatalf("%d to %d: hover kept after leaving", from, to)
		}
	}
}

// Decimation keeps each column's last point, so a flat line ends at the
// newest sample, and a spike and its return stay in time order.
func TestDecimateKeepsTheNewestAndTheOrder(t *testing.T) {
	var flat []point
	for i := 0; i < 6000; i++ {
		flat = append(flat, point{-600 + float64(i)/10, 6})
	}
	d := decimate(flat, -600, 0, 500)
	if last := d[len(d)-1]; last.t != flat[len(flat)-1].t {
		t.Fatalf("flat line ends at %v, want %v", last.t, flat[len(flat)-1].t)
	}
	var spike []point
	for i := 0; i < 6000; i++ {
		v := 1.0
		if i == 3000 {
			v = 9
		}
		spike = append(spike, point{-600 + float64(i)/10, v})
	}
	d = decimate(spike, -600, 0, 500)
	peak := false
	for i := 1; i < len(d); i++ {
		if d[i].t < d[i-1].t {
			t.Fatalf("out of order at %d: %v then %v", i, d[i-1].t, d[i].t)
		}
		if d[i].v == 9 {
			peak = true
		}
	}
	if !peak || d[len(d)-1].v != 1 {
		t.Fatalf("peak kept %v, ends at %v", peak, d[len(d)-1].v)
	}
}

// tenHz returns points every 0.1 s ending at end (≤ 0): first, then rest.
func tenHz(from, end float64, first, rest float64) []point {
	var pts []point
	for t := from; t <= end+1e-9; t += 0.1 {
		v := rest
		if len(pts) == 0 {
			v = first
		}
		pts = append(pts, point{t, v})
	}
	return pts
}

// newestAtNow shifts pts so that the newest is at t = 0.
func newestAtNow(pts []point) []point {
	out := make([]point, len(pts))
	for i, p := range pts {
		out[i] = point{p.t - pts[len(pts)-1].t, p.v}
	}
	return out
}

// The spec's scenarios, the same numbers as the bridge's test_peak_hold.
func TestHeldExtreme(t *testing.T) {
	near := func(got, want float64) bool { return math.Abs(got-want) < 0.01 }
	// Stand surge: 4.3 A once, then 0.4 A; seen at 2 s, 4 s and 6 s after.
	for _, c := range []struct{ after, want float64 }{{2, 4.3}, {4, 2.3}, {6, 0.4}} {
		got, ok := heldExtreme(newestAtNow(tenHz(-10, -10+c.after, 4.3, 0.4)), true, currentFallAps)
		if !ok || !near(got, c.want) {
			t.Errorf("%v s after the surge: held %v (%v), want %v", c.after, got, ok, c.want)
		}
	}
	// Supply sag: 5.5 V, then 6.0 V; held 3 s, at 6.0 within a further second.
	for _, c := range []struct{ after, want float64 }{{2.8, 5.5}, {4.1, 6.0}} {
		got, ok := heldExtreme(newestAtNow(tenHz(-10, -10+c.after, 5.5, 6.0)), false, voltageRiseVps)
		if !ok || !near(got, c.want) {
			t.Errorf("%v s after the sag: held %v (%v), want %v", c.after, got, ok, c.want)
		}
	}
	// A reading rising into the falling peak takes it over and holds anew,
	// as on the bridge: falling, the peak is 2.1 A when 2.2 A arrives 4.1 s
	// after the surge; 2 s later the new hold still stands.
	rising := append(tenHz(-10, -6.0, 4.3, 0.4), point{-5.9, 2.2})
	if got, ok := heldExtreme(newestAtNow(rising), true, currentFallAps); !ok || !near(got, 2.2) {
		t.Errorf("rising reading: held %v, want 2.2", got)
	}
	later := append(rising, tenHz(-5.8, -3.9, 0.4, 0.4)...)
	if got, _ := heldExtreme(newestAtNow(later), true, currentFallAps); !near(got, 2.2) {
		t.Errorf("2 s after the new hold: held %v, want 2.2", got)
	}

	// No telemetry, or none for longer than a gap: nothing held.
	if _, ok := heldExtreme(nil, true, currentFallAps); ok {
		t.Error("held with no samples")
	}
	if _, ok := heldExtreme([]point{{-2, 4.3}, {-1.9, 0.4}}, true, currentFallAps); ok {
		t.Error("held although the newest sample is older than a gap")
	}
	// A gap restarts the hold: the old surge is forgotten.
	gapped := []point{{-1.5, 9}, {-0.3, 0.5}, {-0.2, 0.5}, {-0.1, 0.5}, {0, 0.5}}
	if got, ok := heldExtreme(gapped, true, currentFallAps); !ok || !near(got, 0.5) {
		t.Errorf("after a gap: held %v, want 0.5", got)
	}
}

func TestChartsShowHeldExtremes(t *testing.T) {
	w, _, _ := testWindow(t, false)
	s := w.env.Session
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	feedHistory(s, 30)
	h, _ := s.HistorySince(clock.Add(-2 * time.Minute))
	m := buildCharts(h, clock, 2*time.Minute, 1)
	byTitle := map[string]chartSpec{}
	for _, c := range m.charts {
		byTitle[c.title] = c
	}
	amps, volts := byTitle["Current"], byTitle["Supply"]
	if amps.held == nil || !strings.HasPrefix(amps.held.name, "peak ") {
		t.Fatalf("current chart held line: %+v", amps.held)
	}
	if volts.held == nil || !strings.HasPrefix(volts.held.name, "low ") {
		t.Fatalf("supply chart held line: %+v", volts.held)
	}
	last := func(c chartSpec) float64 { p := c.series[0].pts; return p[len(p)-1].v }
	if amps.held.v < last(amps) || volts.held.v > last(volts) {
		t.Errorf("held beyond the wrong side: peak %v vs %v, low %v vs %v", amps.held.v, last(amps), volts.held.v, last(volts))
	}
}
