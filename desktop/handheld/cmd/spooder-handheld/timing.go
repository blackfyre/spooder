package main

import (
	"fmt"
	"io"
	"time"
)

// drawTimingEnv turns on the draw-time report (design D8 of
// add-handheld-gesture-overlay).
const drawTimingEnv = "SPOODER_DRAW_TIMING"

// drawTiming reports, once a second, the longest scr.draw of that second and
// the longest interval between successive SetControls calls: the complete
// sampling gap, the poll's delay plus a draw. The input is read on the
// drawing thread, so a slow draw delays the controls the sender's 20 ms tick
// sends. A nil *drawTiming records nothing.
type drawTiming struct {
	out              io.Writer
	since            time.Time // the reported second's start
	lastSet          time.Time
	maxDraw, maxGap  time.Duration
	draws, samplings int
}

// newDrawTiming returns a report to out when the environment's
// SPOODER_DRAW_TIMING is 1, else nil.
func newDrawTiming(getenv func(string) string, out io.Writer) *drawTiming {
	if getenv(drawTimingEnv) != "1" {
		return nil
	}
	return &drawTiming{out: out}
}

// controlsSet records a SetControls call at now, and reports the second
// once it is over.
func (t *drawTiming) controlsSet(now time.Time) {
	if t == nil {
		return
	}
	if t.since.IsZero() {
		t.since = now
	}
	if !t.lastSet.IsZero() {
		t.maxGap = max(t.maxGap, now.Sub(t.lastSet))
		t.samplings++
	}
	t.lastSet = now
	if now.Sub(t.since) >= time.Second {
		fmt.Fprintf(t.out, "spooder-handheld: draw timing: %d draws, longest %v; %d samplings, longest gap %v\n",
			t.draws, t.maxDraw.Round(10*time.Microsecond), t.samplings, t.maxGap.Round(10*time.Microsecond))
		t.since, t.maxDraw, t.maxGap, t.draws, t.samplings = now, 0, 0, 0, 0
	}
}

// drew records one draw that took d.
func (t *drawTiming) drew(d time.Duration) {
	if t == nil {
		return
	}
	t.maxDraw = max(t.maxDraw, d)
	t.draws++
}
