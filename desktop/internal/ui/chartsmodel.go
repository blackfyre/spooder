package ui

import (
	"fmt"
	"math"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// chartWindows are the charts' time windows; the history keeps the longest.
var chartWindows = [...]struct {
	d     time.Duration
	label string
}{
	{30 * time.Second, "30 s"},
	{2 * time.Minute, "2 min"},
	{10 * time.Minute, "10 min"},
}

const defaultChartWindow = 1 // 2 min

// sampleGap is how long a sample stands for: the robot sends every kind at
// 10 Hz, so a longer silence (over three missed frames) is a gap, where
// values are unknown and lines break.
const sampleGap = 0.35

// withGaps inserts a NaN point wherever consecutive points are more than
// sampleGap apart, and after the last one when it is older than that, so
// lines break and readings are unknown there.
func withGaps(pts []point) []point {
	if len(pts) == 0 {
		return pts
	}
	out := make([]point, 0, len(pts)+4)
	for i, p := range pts {
		if i > 0 && p.t-pts[i-1].t > sampleGap {
			out = append(out, point{pts[i-1].t + sampleGap, math.NaN()})
		}
		out = append(out, p)
	}
	if last := pts[len(pts)-1]; last.t < -sampleGap {
		out = append(out, point{last.t + sampleGap, math.NaN()})
	}
	return out
}

// point is one sample: seconds relative to now (≤ 0), and its value.
type point struct{ t, v float64 }

// series is one line on a chart, in a categorical slot (0, 1 or 2).
type series struct {
	name string
	slot int
	pts  []point
}

// refLine is a named horizontal reference: a limit or a period.
type refLine struct {
	name string
	v    float64
}

// heldLine is a held extreme on its series' chart: the recent highest
// current or lowest voltage.
type heldLine struct {
	name string
	v    float64
	slot int
}

// The held extremes' rule, as on the bridge (core/status/peak_hold): a
// reading at or beyond the held value takes it at once; it holds holdSeconds
// from when it was last reached, then moves towards the reading at a set rate.
const (
	holdSeconds    = 3.0
	currentFallAps = 2.0 // A per second
	voltageRiseVps = 0.5 // V per second
)

// heldExtreme runs the hold over pts (oldest first) up to now (t = 0), for
// the highest (max) or lowest value. A gap between samples restarts it, and
// with no sample for longer than a gap nothing is held.
func heldExtreme(pts []point, max bool, rate float64) (float64, bool) {
	var held, reached, last float64
	present := false
	towards := func(t, v float64) {
		from := math.Max(last, reached+holdSeconds)
		last = t
		if t <= from {
			return
		}
		if step := rate * (t - from); max {
			held = math.Max(held-step, v)
		} else {
			held = math.Min(held+step, v)
		}
	}
	for i, p := range pts {
		if math.IsNaN(p.v) || (i > 0 && p.t-pts[i-1].t > sampleGap) {
			present = false
			if math.IsNaN(p.v) {
				continue
			}
		}
		if present {
			// Until this sample the previous reading stood, as the bridge
			// sees it between telemetry frames.
			towards(p.t, pts[i-1].v)
		}
		beyond := (max && p.v >= held) || (!max && p.v <= held)
		if !present || beyond {
			held, reached, last, present = p.v, p.t, p.t, true
		}
	}
	if !present || pts[len(pts)-1].t < -sampleGap {
		return 0, false
	}
	towards(0, pts[len(pts)-1].v)
	return held, true
}

// chartSpec is one chart: a title with its unit, the series on one value
// axis, the reference lines, and why there is nothing to draw (empty when
// there is).
type chartSpec struct {
	title, unit string
	decimals    int
	series      []series
	refs        []refLine
	held        *heldLine // the held extreme, if any
	missing     string
	minRange    float64 // the value axis spans at least this much
	floorZero   bool    // the value axis starts at zero
}

// segment is a span of the timeline: from t0 to t1 (seconds, ≤ 0), with a
// label.
type segment struct {
	t0, t1 float64
	label  string
	on     bool // for the output and fault rows: a condition that holds
}

// timelineSpec is the mode, output and fault band.
type timelineSpec struct {
	modes, output, faults []segment
	missing               string
}

// chartsModel is everything the charts tab draws, for one window.
type chartsModel struct {
	window   float64 // seconds shown
	charts   []chartSpec
	timeline timelineSpec
	leg      int
}

// rel is t relative to now, in seconds.
func rel(t, now time.Time) float64 { return t.Sub(now).Seconds() }

// buildCharts derives the charts from the history (oldest first) for the
// window ending at now, for the selected leg.
func buildCharts(h session.History, now time.Time, window time.Duration, leg int) chartsModel {
	m := chartsModel{window: window.Seconds(), leg: leg}
	tel, diag, poses := h.Telemetry, h.Diagnostics, h.Poses
	noTelemetry := "no telemetry yet"
	noDiagnostics := "no diagnostics from this robot"
	noPoses := "no pose frames from this robot"

	var latest *protocol.Diagnostics
	if len(diag) > 0 {
		latest = &diag[len(diag)-1].D
	}

	// Supply voltage.
	volts := chartSpec{title: "Supply", unit: "V", decimals: 2, minRange: 0.5}
	var v []point
	for _, s := range tel {
		v = append(v, point{rel(s.At, now), float64(s.T.VoltageMv) / 1000})
	}
	volts.series = []series{{name: "voltage", slot: 0, pts: v}}
	if low, ok := heldExtreme(v, false, voltageRiseVps); ok {
		volts.held = &heldLine{name: fmt.Sprintf("low %.2f V", low), v: low, slot: 0}
	}
	if latest != nil {
		volts.refs = []refLine{
			{"max", float64(latest.MaxMv) / 1000}, {"min", float64(latest.MinMv) / 1000}, {"brown-out", float64(latest.BrownoutMv) / 1000},
		}
	}
	if len(tel) == 0 {
		volts.missing = noTelemetry
	}

	// Current, and the protection's average.
	amps := chartSpec{title: "Current", unit: "A", decimals: 2, minRange: 1, floorZero: true}
	var a, avg []point
	for _, s := range tel {
		a = append(a, point{rel(s.At, now), float64(s.T.CurrentMa) / 1000})
	}
	for _, s := range diag {
		avg = append(avg, point{rel(s.At, now), float64(s.D.AverageCurrentMa) / 1000})
	}
	amps.series = []series{{name: "current", slot: 0, pts: a}}
	if peak, ok := heldExtreme(a, true, currentFallAps); ok {
		amps.held = &heldLine{name: fmt.Sprintf("peak %.2f A", peak), v: peak, slot: 0}
	}
	if len(diag) > 0 {
		amps.series = append(amps.series, series{name: "average", slot: 1, pts: avg})
		amps.refs = []refLine{{"trip", float64(latest.TripCurrentMa) / 1000}, {"sustained", float64(latest.SustainedCurrentMa) / 1000}}
	}
	if len(tel) == 0 {
		amps.missing = noTelemetry
	}

	// Frame rates on the three links.
	accepted := chartSpec{title: "Frames accepted", unit: "/s", decimals: 0, minRange: 10, floorZero: true}
	rejected := chartSpec{title: "Frames rejected", unit: "/s", decimals: 1, minRange: 1, floorZero: true}
	var okR, okB, okH, badR, badB, badH []point
	for i := 1; i < len(tel); i++ {
		p, s := tel[i-1], tel[i]
		dt := s.At.Sub(p.At)
		t := rel(s.At, now)
		okR = append(okR, point{t, session.CountRate(p.T.ValidFrames, s.T.ValidFrames, dt)})
		okB = append(okB, point{t, session.CountRate(p.T.BridgeValidFrames, s.T.BridgeValidFrames, dt)})
		okH = append(okH, point{t, session.HostRate(p.Host.Valid, s.Host.Valid, dt)})
		badR = append(badR, point{t, session.CountRate(p.T.DiscardedFrames, s.T.DiscardedFrames, dt)})
		badB = append(badB, point{t, session.CountRate(p.T.BridgeDiscardedFrames, s.T.BridgeDiscardedFrames, dt)})
		badH = append(badH, point{t, session.HostRate(p.Host.Discarded, s.Host.Discarded, dt)})
	}
	accepted.series = []series{{"robot link", 0, okR}, {"bridge link", 1, okB}, {"host link", 2, okH}}
	rejected.series = []series{{"robot link", 0, badR}, {"bridge link", 1, badB}, {"host link", 2, badH}}
	if len(tel) < 2 {
		accepted.missing, rejected.missing = noTelemetry, noTelemetry
	}

	// Joint speeds and the selected leg's servo angles.
	speeds := chartSpec{title: "Fastest joint", unit: "°/s", decimals: 0, minRange: 50, floorZero: true}
	angles := chartSpec{title: legTitle(leg) + " servo angles", unit: "°", decimals: 1, minRange: 20}
	kinds := [session.JointKinds]string{"coxa", "femur", "tibia"}
	for k, name := range kinds {
		var sp, an []point
		for _, s := range poses {
			t := rel(s.At, now)
			sp = append(sp, point{t, s.FastestDeg[k]})
			an = append(an, point{t, s.ServoDeg[leg*session.JointKinds+k]})
		}
		speeds.series = append(speeds.series, series{name, k, sp})
		angles.series = append(angles.series, series{name, k, an})
	}
	if latest != nil {
		speeds.refs = []refLine{{"rated", float64(latest.RatedDegPerS)}, {"budget", float64(latest.BudgetDegPerS)}}
	}
	angles.refs = []refLine{{"180°", 180}, {"0°", 0}}
	if len(poses) == 0 {
		speeds.missing, angles.missing = noPoses, noPoses
	}

	// The robot's internals.
	share := chartSpec{title: "Motion share (budget)", unit: "%", decimals: 0, minRange: 20}
	loop := chartSpec{title: "Control tick", unit: "ms", decimals: 2, minRange: 2, floorZero: true}
	hits := chartSpec{title: "Backstop hits", unit: "/s", decimals: 1, minRange: 1, floorZero: true}
	var sh, tmax, tmean, bh []point
	for i, s := range diag {
		t := rel(s.At, now)
		sh = append(sh, point{t, float64(s.D.MotionSharePermille) / 10})
		tmax = append(tmax, point{t, float64(s.D.TickMaxUs) / 1000})
		tmean = append(tmean, point{t, float64(s.D.TickMeanUs) / 1000})
		if i > 0 {
			bh = append(bh, point{t, session.CountRate(diag[i-1].D.BackstopTicks, s.D.BackstopTicks, s.At.Sub(diag[i-1].At))})
		}
	}
	share.series = []series{{"share", 0, sh}}
	share.refs = []refLine{{"full rate", 100}}
	loop.series = []series{{"longest", 0, tmax}, {"mean", 1, tmean}}
	loop.refs = []refLine{{"period", 20}}
	hits.series = []series{{"hits", 0, bh}}
	if len(diag) == 0 {
		share.missing, loop.missing, hits.missing = noDiagnostics, noDiagnostics, noDiagnostics
	}

	m.charts = []chartSpec{volts, amps, accepted, rejected, speeds, angles, share, loop, hits}
	for i := range m.charts {
		for j := range m.charts[i].series {
			m.charts[i].series[j].pts = withGaps(m.charts[i].series[j].pts)
		}
	}
	m.timeline = buildTimeline(tel, now)
	if len(tel) == 0 {
		m.timeline.missing = noTelemetry
	}
	return m
}

// buildTimeline turns the telemetry into spans of equal mode, output and
// faults.
func buildTimeline(tel []session.TelemetrySample, now time.Time) timelineSpec {
	var tl timelineSpec
	add := func(list []segment, t0, t1 float64, label string, on bool) []segment {
		if n := len(list); n > 0 && list[n-1].label == label && list[n-1].on == on && list[n-1].t1 >= t0-1e-9 {
			list[n-1].t1 = t1
			return list
		}
		return append(list, segment{t0, t1, label, on})
	}
	for i, s := range tel {
		// A sample holds until the next, or for sampleGap at most: after a
		// silence the state is unknown.
		t0 := rel(s.At, now)
		t1 := math.Min(t0+sampleGap, 0)
		if i+1 < len(tel) {
			t1 = math.Min(rel(tel[i+1].At, now), t0+sampleGap)
		}
		tl.modes = add(tl.modes, t0, t1, bodyText(session.Snapshot{Body: session.BodyMode(s.T.BodyMode), Gesture: s.T.Gesture}), true)
		out := "output off"
		if s.T.OutputsOn {
			out = "output on"
		}
		tl.output = add(tl.output, t0, t1, out, s.T.OutputsOn)
		faults := "no fault"
		if names := session.FaultNames(s.T.Faults); len(names) > 0 {
			faults = joinNames(names)
		}
		tl.faults = add(tl.faults, t0, t1, faults, s.T.Faults != 0)
	}
	return tl
}

func joinNames(names []string) string {
	out := names[0]
	for _, n := range names[1:] {
		out += ", " + n
	}
	return out
}

// valueAt is the value of the last point at or before t, NaN when there is
// none, or it is older than a sample gap (the points carry NaN markers at
// gaps, so a reading inside a gap is NaN too).
func valueAt(pts []point, t float64) float64 {
	lo, hi := 0, len(pts)
	for lo < hi {
		mid := (lo + hi) / 2
		if pts[mid].t <= t {
			lo = mid + 1
		} else {
			hi = mid
		}
	}
	if lo == 0 || t-pts[lo-1].t > sampleGap {
		return math.NaN()
	}
	return pts[lo-1].v
}

// segmentAt is the label of the segment holding t, "unknown" when none does.
func segmentAt(segs []segment, t float64) string {
	for _, s := range segs {
		if s.t0 <= t && t <= s.t1 {
			return s.label
		}
	}
	return "unknown"
}

// valueRange is the value axis: the data's and the references' extent,
// widened to minRange and padded a little, with "nice" gridline steps.
func (c chartSpec) valueRange(t0 float64) (lo, hi, step float64) {
	lo, hi = math.Inf(1), math.Inf(-1)
	for _, s := range c.series {
		for _, p := range s.pts {
			if p.t >= t0 && !math.IsNaN(p.v) {
				lo, hi = math.Min(lo, p.v), math.Max(hi, p.v)
			}
		}
	}
	for _, r := range c.refs {
		lo, hi = math.Min(lo, r.v), math.Max(hi, r.v)
	}
	if c.held != nil {
		lo, hi = math.Min(lo, c.held.v), math.Max(hi, c.held.v)
	}
	if math.IsInf(lo, 0) {
		lo, hi = 0, c.minRange
	}
	if c.floorZero {
		lo = math.Min(lo, 0)
	}
	if span := hi - lo; span < c.minRange {
		mid := (hi + lo) / 2
		lo, hi = mid-c.minRange/2, mid+c.minRange/2
		if c.floorZero && lo < 0 {
			lo, hi = 0, c.minRange
		}
	}
	pad := (hi - lo) * 0.08
	if !(c.floorZero && lo == 0) {
		lo -= pad
	}
	hi += pad
	step = niceStep((hi - lo) / 3)
	return lo, hi, step
}

// niceStep rounds a gridline interval to 1, 2 or 5 times a power of ten.
func niceStep(raw float64) float64 {
	if raw <= 0 || math.IsNaN(raw) {
		return 1
	}
	p := math.Pow(10, math.Floor(math.Log10(raw)))
	for _, m := range []float64{1, 2, 5, 10} {
		if raw <= m*p {
			return m * p
		}
	}
	return 10 * p
}

// decimate keeps, per pixel column of the plot, the first, lowest, highest
// and last points, in time order, so spikes survive, the line ends at the
// newest sample, and a long window stays cheap to draw. Gaps (NaN) are kept.
func decimate(pts []point, t0, t1 float64, columns int) []point {
	if columns <= 0 || len(pts) <= 4*columns {
		return pts
	}
	out := make([]point, 0, 4*columns)
	col := -1
	var first, lo, hi, last point
	var loI, hiI, firstI, lastI int
	flush := func() {
		if col < 0 {
			return
		}
		// The four in time order, each once.
		idx := []int{firstI, loI, hiI, lastI}
		ps := map[int]point{firstI: first, loI: lo, hiI: hi, lastI: last}
		for i := 1; i < len(idx); i++ {
			for j := i; j > 0 && idx[j] < idx[j-1]; j-- {
				idx[j], idx[j-1] = idx[j-1], idx[j]
			}
		}
		for i, k := range idx {
			if i == 0 || k != idx[i-1] {
				out = append(out, ps[k])
			}
		}
	}
	for i, p := range pts {
		if math.IsNaN(p.v) {
			// A gap stays a gap: close the column and keep the marker.
			flush()
			col = -1
			out = append(out, p)
			continue
		}
		c := int((p.t - t0) / (t1 - t0) * float64(columns))
		if c != col {
			flush()
			col = c
			first, lo, hi, last = p, p, p, p
			firstI, loI, hiI, lastI = i, i, i, i
			continue
		}
		if p.v < lo.v {
			lo, loI = p, i
		}
		if p.v > hi.v {
			hi, hiI = p, i
		}
		last, lastI = p, i
	}
	flush()
	return out
}
