package ui

import (
	"image"
	"image/color"
	"strings"
	"unicode/utf8"

	"gioui.org/f32"
	"gioui.org/io/event"
	"gioui.org/io/key"
	"gioui.org/io/pointer"
	"gioui.org/layout"
	"gioui.org/op/clip"
	"gioui.org/op/paint"
	"gioui.org/text"
	"gioui.org/unit"
	"gioui.org/widget"
	"gioui.org/widget/material"
	"golang.org/x/image/math/fixed"
)

// paneTextSize is the text pane lines' size, in JetBrains Mono.
const paneTextSize unit.Sp = 12

// paneLine is one line of a text pane, as shown and copied.
type paneLine struct {
	seq  uint64 // the session's line number (session.LogLine.Seq)
	text string
	c    color.NRGBA
}

// paneLines are a pane's lines, oldest first, made on demand: only the
// lines in view, and the selected ones on a copy, are formatted.
type paneLines struct {
	n    int
	line func(i int) paneLine
}

// textPos is a place in a text pane: a line by its number, and a rune
// offset into its text.
type textPos struct {
	line uint64
	col  int
}

func (a textPos) before(b textPos) bool {
	return a.line < b.line || a.line == b.line && a.col < b.col
}

// paneRow is a line as laid out in the last frame, in the list's
// coordinates.
type paneRow struct {
	seq    uint64
	y0, y1 int
}

// textPane is a read-only list of lines (design D1–D3): the pointer
// selects within and across lines, Shift and a click extends the
// selection, and the pane follows new lines only while its end is in view
// and no selection holds it. Only the lines in view are laid out; the
// copied text is assembled on request.
type textPane struct {
	list           widget.List
	anchor, caret  textPos
	anchored       bool // a press placed the anchor
	dragging       bool
	held           bool // the selection holds the view: no following
	atEnd          bool // the view was at the end in the last frame
	base           uint64
	haveBase       bool
	width          int       // the lines' width in the last frame
	rows           []paneRow // the lines in view in the last frame
	laid           []laidLine
	shaped         []glyphCluster
	selectionRects []image.Rectangle
}

type laidLine struct{ index, height int }

func newTextPane() *textPane {
	p := &textPane{}
	p.list.Axis = layout.Vertical
	p.list.ScrollToEnd = true
	return p
}

// selection returns the selection in order; ok is false when it is empty.
func (p *textPane) selection() (start, end textPos, ok bool) {
	start, end = p.anchor, p.caret
	if end.before(start) {
		start, end = end, start
	}
	return start, end, p.anchored && start != end
}

// selectedText is the selection's text: the selected part of each line,
// joined by newlines.
func (p *textPane) selectedText(lines paneLines) string {
	start, end, ok := p.selection()
	if !ok || lines.n == 0 {
		return ""
	}
	base := lines.line(0).seq
	var b strings.Builder
	for seq := max(start.line, base); seq <= end.line && seq-base < uint64(lines.n); seq++ {
		r := []rune(lines.line(int(seq - base)).text)
		from, to := 0, len(r)
		if seq == start.line {
			from = min(start.col, len(r))
		}
		if seq == end.line {
			to = min(end.col, len(r))
		}
		if seq != max(start.line, base) {
			b.WriteByte('\n')
		}
		if from < to {
			b.WriteString(string(r[from:to]))
		}
	}
	return b.String()
}

// sync follows the oldest lines being dropped (design D2): the view moves
// back by as many lines, so it shows the same ones, and the selection
// keeps only the lines still present.
func (p *textPane) sync(lines paneLines) {
	if lines.n == 0 {
		p.haveBase, p.anchored, p.held = false, false, false
		return
	}
	base := lines.line(0).seq
	if p.haveBase && base > p.base {
		dropped := base - p.base
		pos := &p.list.Position
		if uint64(pos.First) >= dropped {
			pos.First -= int(dropped)
		} else {
			pos.First, pos.Offset = 0, 0
		}
	}
	p.base, p.haveBase = base, true
	for _, q := range []*textPos{&p.anchor, &p.caret} {
		if q.line < base {
			*q = textPos{line: base}
		}
	}
	if _, _, ok := p.selection(); !ok && !p.dragging {
		p.held = false
	}
}

// update handles the pointer: a press places the caret (and the anchor,
// unless Shift extends), a drag moves the caret.
func (p *textPane) update(gtx layout.Context, lt *text.Shaper, lines paneLines) {
	for {
		ev, ok := gtx.Event(pointer.Filter{Target: p, Kinds: pointer.Press | pointer.Drag | pointer.Release | pointer.Cancel})
		if !ok {
			return
		}
		e, ok := ev.(pointer.Event)
		if !ok {
			continue
		}
		switch e.Kind {
		case pointer.Press:
			// Only the primary button, and not on the scrollbar.
			if e.Buttons != pointer.ButtonPrimary || int(e.Position.X) >= p.width {
				continue
			}
			pos, ok := p.hit(gtx, lt, lines, e.Position)
			if !ok {
				continue
			}
			if !p.anchored || e.Modifiers&key.ModShift == 0 {
				p.anchor = pos
			}
			p.caret, p.anchored, p.dragging, p.held = pos, true, true, true
		case pointer.Drag:
			if !p.dragging {
				continue
			}
			if pos, ok := p.hit(gtx, lt, lines, e.Position); ok {
				p.caret = pos
			}
		case pointer.Release, pointer.Cancel:
			if !p.dragging {
				continue
			}
			p.dragging = false
			if _, _, ok := p.selection(); !ok {
				p.held = false // a click: following as the view allows
			}
		}
	}
}

// hit maps a point in the list to a place in the text, from the last
// frame's rows; only the line under the point is shaped.
func (p *textPane) hit(gtx layout.Context, lt *text.Shaper, lines paneLines, pt f32.Point) (textPos, bool) {
	if len(p.rows) == 0 || lines.n == 0 {
		return textPos{}, false
	}
	y := int(pt.Y)
	row := p.rows[0]
	if y < row.y0 {
		return textPos{line: max(row.seq, lines.line(0).seq)}, true
	}
	for _, r := range p.rows {
		if y >= r.y0 {
			row = r
		}
	}
	base := lines.line(0).seq
	if row.seq < base || row.seq-base >= uint64(lines.n) {
		return textPos{}, false
	}
	s := lines.line(int(row.seq - base)).text
	if y >= row.y1 {
		return textPos{line: row.seq, col: utf8.RuneCountInString(s)}, true
	}
	return textPos{line: row.seq, col: p.column(gtx, lt, s, pt.X, y-row.y0)}, true
}

// glyphCluster is one cluster of a shaped line: its runes and its box.
type glyphCluster struct {
	col, runes int
	x0, x1     fixed.Int26_6
	top, bot   int
}

// shape lays out s as the pane's labels do and returns its clusters, in
// a slice reused from frame to frame.
func (p *textPane) shape(gtx layout.Context, lt *text.Shaper, s string) []glyphCluster {
	// The same parameters as widget.Label's, for valueLabel at
	// paneTextSize with no line limit.
	lt.LayoutString(text.Parameters{
		Font:     monoFont,
		PxPerEm:  fixed.I(gtx.Sp(paneTextSize)),
		MaxWidth: p.width,
		MinWidth: 0,
		Locale:   gtx.Locale,
	}, s)
	out := p.shaped[:0]
	col, open := 0, false
	var cur glyphCluster
	for g, ok := lt.NextGlyph(); ok; g, ok = lt.NextGlyph() {
		top, bot := int(g.Y)-g.Ascent.Ceil(), int(g.Y)+g.Descent.Ceil()
		if !open {
			cur = glyphCluster{col: col, x0: g.X, x1: g.X + g.Advance, top: top, bot: bot}
			open = true
		} else {
			cur.x0, cur.x1 = min(cur.x0, g.X), max(cur.x1, g.X+g.Advance)
		}
		if g.Flags&text.FlagClusterBreak != 0 {
			cur.runes = int(g.Runes)
			out = append(out, cur)
			col += cur.runes
			open = false
		}
	}
	p.shaped = out
	return out
}

// column is the rune offset nearest to (x, y) in the line s, y from the
// line's top.
func (p *textPane) column(gtx layout.Context, lt *text.Shaper, s string, x float32, y int) int {
	cs := p.shape(gtx, lt, s)
	if len(cs) == 0 {
		return 0
	}
	// The wrapped row at y: the first whose bottom is below it, else the
	// last.
	rowTop := cs[len(cs)-1].top
	for _, c := range cs {
		if y < c.bot {
			rowTop = c.top
			break
		}
	}
	fx := fixed.Int26_6(x * 64)
	end := 0
	for _, c := range cs {
		if c.top != rowTop {
			continue
		}
		if fx < (c.x0+c.x1)/2 {
			return c.col
		}
		end = c.col + c.runes
	}
	return end
}

// paintSelection fills the selected part of line s (runes from to to,
// to past the end for the whole line), merged per wrapped row.
func (p *textPane) paintSelection(gtx layout.Context, lt *text.Shaper, s string, from, to int, c color.NRGBA) {
	rects := p.selectionRects[:0]
	for _, g := range p.shape(gtx, lt, s) {
		if g.runes == 0 || g.col < from || g.col+g.runes > to {
			continue
		}
		r := image.Rect(g.x0.Floor(), g.top, g.x1.Ceil(), g.bot)
		if n := len(rects); n > 0 && rects[n-1].Min.Y == r.Min.Y {
			rects[n-1] = rects[n-1].Union(r)
			continue
		}
		rects = append(rects, r)
	}
	for _, r := range rects {
		paint.FillShape(gtx.Ops, c, clip.Rect(r).Op())
	}
	p.selectionRects = rects
}

// Layout draws the lines, oldest first, in a list that fills the
// constraints.
func (p *textPane) Layout(gtx layout.Context, th *material.Theme, selection color.NRGBA, lines paneLines) layout.Dimensions {
	p.sync(lines)
	p.update(gtx, th.Shaper, lines)
	if p.held {
		p.list.Position.BeforeEnd = true // stay: no jump to the end
	}
	// The handler is the list's ancestor, so the wheel and the scrollbar
	// keep working beneath it.
	defer clip.Rect{Max: gtx.Constraints.Max}.Push(gtx.Ops).Pop()
	event.Op(gtx.Ops, p)

	start, end, selected := p.selection()
	p.laid = p.laid[:0]
	dims := material.List(th, &p.list).Layout(gtx, lines.n, func(gtx layout.Context, i int) layout.Dimensions {
		l := lines.line(i)
		p.width = gtx.Constraints.Max.X
		if selected && l.seq >= start.line && l.seq <= end.line {
			from, to := 0, utf8.RuneCountInString(l.text)
			if l.seq == start.line {
				from = start.col
			}
			if l.seq == end.line {
				to = end.col
			}
			p.paintSelection(gtx, th.Shaper, l.text, from, to, selection)
		}
		lbl := valueLabel(th, paneTextSize, l.c, l.text)
		lbl.MaxLines = 0
		d := lbl.Layout(gtx)
		area := clip.Rect{Max: d.Size}.Push(gtx.Ops)
		pointer.CursorText.Add(gtx.Ops)
		area.Pop()
		p.laid = append(p.laid, laidLine{index: i, height: d.Size.Y})
		return d
	})
	p.place(lines, dims.Size.Y)

	// The hold ends when the view reaches the end from elsewhere: a
	// scroll, the scrollbar or a resize. New lines only move the end away.
	atEnd := !p.list.Position.BeforeEnd
	if p.held && !p.dragging && atEnd && !p.atEnd {
		p.held = false
	}
	p.atEnd = atEnd
	return dims
}

// place records where the lines in view are, for hit testing.
func (p *textPane) place(lines paneLines, height int) {
	p.rows = p.rows[:0]
	if lines.n == 0 {
		return
	}
	base := lines.line(0).seq // the lines are numbered without gaps
	pos := p.list.Position
	y := -pos.Offset
	for i := pos.First; i < lines.n && y < height; i++ {
		h, ok := 0, false
		for _, l := range p.laid {
			if l.index == i {
				h, ok = l.height, true
				break
			}
		}
		if !ok {
			break
		}
		p.rows = append(p.rows, paneRow{seq: base + uint64(i), y0: y, y1: y + h})
		y += h
	}
}
