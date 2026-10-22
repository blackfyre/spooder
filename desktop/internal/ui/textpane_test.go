package ui

import (
	"fmt"
	"image"
	"image/color"
	"strings"
	"testing"

	"gioui.org/f32"
	"gioui.org/io/input"
	"gioui.org/io/key"
	"gioui.org/io/pointer"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/text"
	"gioui.org/unit"
	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/ui/fonts"
)

// paneRig lays out a text pane alone, at the origin, against an input
// router, so pointer events can be placed on its lines.
type paneRig struct {
	t      *testing.T
	p      *textPane
	th     *material.Theme
	router *input.Router
	lines  []paneLine
	next   uint64
}

const rigWidth, rigHeight = 600, 200

func newPaneRig(t *testing.T, n int) *paneRig {
	t.Helper()
	faces, err := fonts.Collection()
	if err != nil {
		t.Fatal(err)
	}
	th := material.NewTheme()
	th.Shaper = text.NewShaper(text.NoSystemFonts(), text.WithCollection(faces))
	r := &paneRig{t: t, p: newTextPane(), th: th, router: new(input.Router)}
	r.add(n, 0)
	// Two frames: the list's scroll range comes from the frame before.
	r.frame()
	r.frame()
	return r
}

// add appends n lines, dropping the oldest beyond max (0: no limit), as
// the session does.
func (r *paneRig) add(n, max int) {
	for range n {
		r.lines = append(r.lines, paneLine{seq: r.next, text: fmt.Sprintf("line %04d some text", r.next), c: color.NRGBA{A: 0xFF}})
		r.next++
	}
	if max > 0 && len(r.lines) > max {
		r.lines = r.lines[len(r.lines)-max:]
	}
}

func (r *paneRig) gtx(ops *op.Ops) layout.Context {
	return layout.Context{Ops: ops, Metric: unit.Metric{PxPerDp: 1, PxPerSp: 1},
		Constraints: layout.Exact(image.Pt(rigWidth, rigHeight)), Now: nextFrameTime(), Source: r.router.Source()}
}

func (r *paneRig) source() paneLines {
	lines := r.lines
	return paneLines{n: len(lines), line: func(i int) paneLine { return lines[i] }}
}

func (r *paneRig) frame() {
	var ops op.Ops
	r.p.Layout(r.gtx(&ops), r.th, color.NRGBA{R: 0xFF, A: 0xFF}, r.source())
	r.router.Frame(&ops)
}

// at is the point on the row in view showing line seq, just inside the
// glyph of rune col.
func (r *paneRig) at(seq uint64, col int) f32.Point {
	r.t.Helper()
	for _, row := range r.p.rows {
		if row.seq != seq {
			continue
		}
		var ops op.Ops
		cs := r.p.shape(r.gtx(&ops), r.th.Shaper, r.lines[seq-r.lines[0].seq].text)
		return f32.Pt(float32(cs[col].x0.Floor()+1), float32(row.y0+2))
	}
	r.t.Fatalf("line %d not in view (%d to %d)", seq, r.p.rows[0].seq, r.p.rows[len(r.p.rows)-1].seq)
	return f32.Point{}
}

func (r *paneRig) pointer(kind pointer.Kind, pt f32.Point, mods key.Modifiers) {
	r.router.Queue(pointer.Event{Kind: kind, Source: pointer.Mouse, Buttons: pointer.ButtonPrimary, Position: pt, Modifiers: mods})
	r.frame()
}

func (r *paneRig) drag(from, to f32.Point) {
	r.pointer(pointer.Press, from, 0)
	r.pointer(pointer.Move, to, 0)
	r.router.Queue(pointer.Event{Kind: pointer.Release, Source: pointer.Mouse, Position: to})
	r.frame()
}

func (r *paneRig) scroll(dy float32) {
	r.router.Queue(pointer.Event{Kind: pointer.Scroll, Source: pointer.Mouse, Position: f32.Pt(10, 10), Scroll: f32.Pt(0, dy)})
	r.frame()
}

func (r *paneRig) firstInView() uint64 { return r.p.rows[0].seq }
func (r *paneRig) lastInView() uint64  { return r.p.rows[len(r.p.rows)-1].seq }

// Spec "Copy across lines".
func TestTextPaneSelectsAcrossLines(t *testing.T) {
	r := newPaneRig(t, 100)
	if r.lastInView() != 99 {
		t.Fatalf("not at the end: last line in view %d", r.lastInView())
	}
	a, b := r.lastInView()-4, r.lastInView()-1
	r.drag(r.at(a, 5), r.at(b, 9))
	want := r.lines[a].text[5:] + "\n" + r.lines[a+1].text + "\n" + r.lines[a+2].text + "\n" + r.lines[b].text[:9]
	if got := r.p.selectedText(r.source()); got != want {
		t.Fatalf("copied %q, want %q", got, want)
	}
	// Upwards, the same.
	r.drag(r.at(b, 9), r.at(a, 5))
	if got := r.p.selectedText(r.source()); got != want {
		t.Fatalf("copied upwards %q, want %q", got, want)
	}
	// Shift and a click extends from the anchor.
	r.pointer(pointer.Press, r.at(b-1, 2), key.ModShift)
	r.pointer(pointer.Release, r.at(b-1, 2), key.ModShift)
	if got, want := r.p.selectedText(r.source()), r.lines[b-1].text[2:]+"\n"+r.lines[b].text[:9]; got != want {
		t.Fatalf("extended %q, want %q", got, want)
	}
	// A click selects nothing.
	r.drag(r.at(a, 3), r.at(a, 3))
	if got := r.p.selectedText(r.source()); got != "" {
		t.Fatalf("a click selected %q", got)
	}
}

// Spec "Selecting at the bottom": the selection holds the view and itself
// until the user scrolls back to the end.
func TestTextPaneSelectionHoldsTheView(t *testing.T) {
	r := newPaneRig(t, 100)
	last := r.lastInView()
	r.drag(r.at(last-1, 0), r.at(last, 4))
	first := r.firstInView()
	want := r.p.selectedText(r.source())
	for range 5 {
		r.add(3, 0)
		r.frame()
	}
	if r.firstInView() != first || r.p.selectedText(r.source()) != want {
		t.Fatalf("view moved from %d to %d, or the selection changed: %q", first, r.firstInView(), r.p.selectedText(r.source()))
	}
	r.scroll(10000)
	r.add(1, 0)
	r.frame()
	if r.lastInView() != r.next-1 {
		t.Fatalf("not following after scrolling to the end: last in view %d of %d", r.lastInView(), r.next-1)
	}
	if r.p.selectedText(r.source()) != want {
		t.Fatal("selection lost on resuming")
	}
	// A click at the end lets it follow again, from a fresh hold.
	r.drag(r.at(r.lastInView(), 1), r.at(r.lastInView(), 3))
	r.drag(r.at(r.lastInView(), 1), r.at(r.lastInView(), 1))
	r.add(2, 0)
	r.frame()
	if r.lastInView() != r.next-1 {
		t.Fatalf("not following after a click at the end: last in view %d of %d", r.lastInView(), r.next-1)
	}
}

// Spec "Reading back": scrolled up in a full pane, the lines in view stay
// while the oldest are dropped; at the end again, it follows.
func TestTextPaneKeepsItsPlaceWhenLinesDrop(t *testing.T) {
	const max = 120
	r := newPaneRig(t, max)
	r.scroll(-300)
	first := r.firstInView()
	if r.lastInView() == r.next-1 {
		t.Fatalf("still at the end after scrolling up: %+v rows %+v", r.p.list.Position, r.p.rows)
	}
	for range 10 {
		r.add(2, max)
		r.frame()
	}
	if r.firstInView() != first {
		t.Fatalf("view moved from line %d to %d", first, r.firstInView())
	}
	r.scroll(10000)
	r.add(2, max)
	r.frame()
	if r.lastInView() != r.next-1 {
		t.Fatalf("not following: last in view %d of %d", r.lastInView(), r.next-1)
	}
	// While following, the end stays in view as lines are dropped.
	for range 5 {
		r.add(3, max)
		r.frame()
	}
	if r.lastInView() != r.next-1 {
		t.Fatalf("lost the end: last in view %d of %d", r.lastInView(), r.next-1)
	}
}

// A selection whose lines are dropped keeps only what remains; the copy
// never reaches lines that are gone.
func TestTextPaneSelectionClipsToKeptLines(t *testing.T) {
	const max = 30
	r := newPaneRig(t, max)
	a := r.firstInView() + 1
	r.drag(r.at(a, 0), r.at(r.lastInView(), 4))
	r.add(int(a-r.lines[0].seq)+2, max)
	r.frame()
	got := r.p.selectedText(r.source())
	if !strings.HasPrefix(got, r.lines[0].text+"\n") || strings.Count(got, "\n") != int(r.p.caret.line-r.lines[0].seq) {
		t.Fatalf("clipped selection %q", got)
	}
}

// Selection painting shapes only the lines in view: the pane's layout of a
// long, wholly selected pane stays as cheap as without the selection.
func TestTextPaneShapesOnlyTheLinesInView(t *testing.T) {
	r := newPaneRig(t, 2000)
	r.p.anchor, r.p.caret, r.p.anchored = textPos{line: 0}, textPos{line: 1999, col: 3}, true
	var made int
	src := r.source()
	line := src.line
	src.line = func(i int) paneLine { made++; return line(i) }
	var ops op.Ops
	r.p.Layout(r.gtx(&ops), r.th, color.NRGBA{A: 0xFF}, src)
	if made > 2*len(r.p.rows)+4 {
		t.Fatalf("%d lines made for %d in view", made, len(r.p.rows))
	}
}
