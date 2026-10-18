package console

import (
	"errors"
	"fmt"
	"io"
	"strings"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// TimeFormat is the timestamp at the start of every line.
const TimeFormat = "15:04:05.000"

// Printer is the link handler that writes the console's lines. Its methods
// may be called from several goroutines.
type Printer struct {
	mu     sync.Mutex
	w      io.Writer
	now    func() time.Time
	frames bool
	every  int

	// Telemetry throttle: frames since the last one printed, and its key.
	sinceTelemetry int
	lastKey        telemetryKey
	printedAny     bool

	// The link's counts on the current connection, as last seen.
	seenDiscarded, seenStray uint32
	lastStatus               string

	totals Totals
}

// Totals are the counts over the whole run.
type Totals struct {
	Telemetry, Reports, Other int
	Rejected, Stray           uint32
}

// NewPrinter writes to w. With frames, decoded frames are printed too, and
// every telemetry frame in every (at least 1) unless it changed.
func NewPrinter(w io.Writer, now func() time.Time, frames bool, every int) *Printer {
	if now == nil {
		now = time.Now
	}
	return &Printer{w: w, now: now, frames: frames, every: max(every, 1)}
}

// println writes one line with its timestamp; p.mu is held.
func (p *Printer) println(s string) {
	fmt.Fprintf(p.w, "%s  %s\n", p.now().Format(TimeFormat), s)
}

// Status prints a note of the monitor's own.
func (p *Printer) Status(s string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.println(markStatus + s)
}

// counts takes in the link's counts, reporting new rejections; p.mu is held.
func (p *Printer) counts(s link.Stats) {
	if s.Discarded > p.seenDiscarded {
		n := s.Discarded - p.seenDiscarded
		p.totals.Rejected += n
		if p.frames {
			what := "frame candidate rejected"
			if n > 1 {
				what = fmt.Sprintf("%d frame candidates rejected", n)
			}
			p.println(fmt.Sprintf("%s%s (%d in total)", markReject, what, p.totals.Rejected))
		}
	}
	if s.Stray > p.seenStray {
		p.totals.Stray += s.Stray - p.seenStray
	}
	p.seenDiscarded, p.seenStray = s.Discarded, s.Stray
}

// LinkState prints each change of the connection, once.
func (p *Printer) LinkState(s link.State) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if s.Status == link.Connected {
		// A new connection's decoder counts from zero.
		p.seenDiscarded, p.seenStray = 0, 0
	}
	text := statusText(s)
	if text == p.lastStatus {
		return
	}
	p.lastStatus = text
	p.println(markStatus + text)
}

func statusText(s link.State) string {
	switch s.Status {
	case link.Connected:
		return "connected " + s.Port.Label()
	case link.Waiting:
		switch {
		case s.Err != nil && link.IsPermission(s.Err):
			return "no access to " + s.Port.Name + ": add yourself to the dialout group, then log in again"
		case s.Err != nil && link.IsBusy(s.Err):
			return s.Port.Name + " is in use by another program"
		case errors.Is(s.Err, link.ErrLost):
			return "lost " + s.Port.Name + " (" + cause(s.Err) + "); waiting for it"
		case s.Err != nil:
			return "waiting for " + s.Port.Label() + ": " + cause(s.Err)
		}
		return "waiting for " + s.Port.Label()
	}
	return "disconnected"
}

// cause is an error's text without the link's ErrLost wrapper, on one line.
func cause(err error) string {
	var parts []string
	if j, ok := err.(interface{ Unwrap() []error }); ok {
		for _, e := range j.Unwrap() {
			if e != link.ErrLost {
				parts = append(parts, e.Error())
			}
		}
	}
	if len(parts) == 0 {
		parts = []string{err.Error()}
	}
	return strings.ReplaceAll(strings.Join(parts, ": "), "\n", ": ")
}

// Line prints a text line.
func (p *Printer) Line(l link.Line, s link.Stats) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.counts(s)
	text := l.Text
	if l.Stray > 0 {
		text += fmt.Sprintf("   [%d stray 0xA5]", l.Stray)
	}
	p.println(text)
}

// Frame prints a frame line, if frames are shown and the throttle lets it.
func (p *Printer) Frame(f protocol.Frame, s link.Stats) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.counts(s)
	switch f.Type {
	case protocol.TypeTelemetry:
		p.totals.Telemetry++
		key := keyOf(f.Telemetry)
		p.sinceTelemetry++
		if p.printedAny && key == p.lastKey && p.sinceTelemetry < p.every {
			return
		}
		p.printedAny, p.lastKey, p.sinceTelemetry = true, key, 0
	case protocol.TypeCalibrationReport:
		p.totals.Reports++
	default:
		p.totals.Other++
	}
	if p.frames {
		p.println(markFrame + FormatFrame(f))
	}
}

// Totals returns the counts so far.
func (p *Printer) Totals() Totals {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.totals
}

// PrintTotals prints the counts over the run.
func (p *Printer) PrintTotals() {
	p.mu.Lock()
	defer p.mu.Unlock()
	t := p.totals
	p.println(fmt.Sprintf("%stotals: %d telemetry, %d calibration reports, %d other; %d rejected; %d stray 0xA5",
		markStatus, t.Telemetry, t.Reports, t.Other, t.Rejected, t.Stray))
}
