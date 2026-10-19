package ui

import (
	"image"
	"image/color"
	"strings"

	"gioui.org/font"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/op/clip"
	"gioui.org/op/paint"
	"gioui.org/text"
	"gioui.org/unit"
	"gioui.org/widget"
	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/ui/fonts"
)

// Type sizes.
const (
	sizeLabel unit.Sp = 11 // mono labels and buttons
	sizeValue unit.Sp = 15 // mono values
	sizeBig   unit.Sp = 22 // telemetry values
	sizeText  unit.Sp = 14 // Archivo text
	sizeTitle unit.Sp = 18
	// trackingEm is the label letter-spacing, as on the site (.06–.08em).
	trackingEm = 0.08
)

var (
	monoFont = font.Font{Typeface: fonts.Mono}
	textFont = font.Font{Typeface: fonts.Text, Weight: font.SemiBold}
)

// textLabel is Archivo text.
func textLabel(th *material.Theme, size unit.Sp, c color.NRGBA, s string) material.LabelStyle {
	l := material.Label(th, size, s)
	l.Font = textFont
	l.Color = c
	return l
}

// valueLabel is a JetBrains Mono value, as written (numbers keep their
// case).
func valueLabel(th *material.Theme, size unit.Sp, c color.NRGBA, s string) material.LabelStyle {
	l := material.Label(th, size, s)
	l.Font = monoFont
	l.Color = c
	l.MaxLines = 1
	return l
}

// tracked lays out an uppercase JetBrains Mono label with wide
// letter-spacing, one glyph at a time (Gio's text layout has no tracking).
func tracked(th *material.Theme, size unit.Sp, c color.NRGBA, s string) layout.Widget {
	s = strings.ToUpper(s)
	return func(gtx layout.Context) layout.Dimensions {
		gap := int(float32(gtx.Sp(size)) * trackingEm)
		x, h, base := 0, 0, 0
		gtx.Constraints.Min = image.Point{}
		for i, r := range s {
			if i > 0 {
				x += gap
			}
			trans := op.Offset(image.Pt(x, 0)).Push(gtx.Ops)
			l := valueLabel(th, size, c, string(r))
			d := l.Layout(gtx)
			trans.Pop()
			x += d.Size.X
			h = max(h, d.Size.Y)
			base = max(base, d.Baseline)
		}
		return layout.Dimensions{Size: image.Pt(x, h), Baseline: base}
	}
}

// fill paints a rounded rectangle.
func fill(gtx layout.Context, r image.Rectangle, radius int, c color.NRGBA) {
	paint.FillShape(gtx.Ops, c, clip.UniformRRect(r, radius).Op(gtx.Ops))
}

// outline strokes a rounded rectangle.
func outline(gtx layout.Context, r image.Rectangle, radius int, width int, c color.NRGBA) {
	rr := clip.UniformRRect(r, radius)
	paint.FillShape(gtx.Ops, c, clip.Stroke{Path: rr.Path(gtx.Ops), Width: float32(width)}.Op())
}

// panel lays content out on a rounded surface.
func panel(gtx layout.Context, bg, border color.NRGBA, inset unit.Dp, content layout.Widget) layout.Dimensions {
	m := op.Record(gtx.Ops)
	dims := layout.UniformInset(inset).Layout(gtx, content)
	call := m.Stop()
	r := image.Rectangle{Max: dims.Size}
	fill(gtx, r, gtx.Dp(8), bg)
	if border.A != 0 {
		outline(gtx, r, gtx.Dp(8), gtx.Dp(1), border)
	}
	call.Add(gtx.Ops)
	return dims
}

// dot is a small filled circle, for status.
func dot(c color.NRGBA, size unit.Dp) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		d := gtx.Dp(size)
		r := image.Rectangle{Max: image.Pt(d, d)}
		paint.FillShape(gtx.Ops, c, clip.Ellipse(r).Op(gtx.Ops))
		return layout.Dimensions{Size: r.Max, Baseline: d}
	}
}

// buttonKind selects a button's colours.
type buttonKind int

const (
	kindNeutral buttonKind = iota
	kindPrimary            // the accent
	kindQuiet              // small nudges and toggles
	kindActive             // a toggle that is on
)

// buttonColours are a button's face, text and shadow.
func (p Palette) buttonColours(k buttonKind) (face, fg, shadow color.NRGBA) {
	switch k {
	case kindPrimary, kindActive:
		return p.Accent, p.OnAccent, p.AccentPressed
	case kindQuiet:
		return p.QuietBg, p.QuietFg, p.QuietShadow
	}
	return p.ButtonBg, p.ButtonFg, p.ButtonShadow
}

// Tactile button geometry (design D7): 7 px radius, flat fill, and a 2 px
// darker bottom shadow that flattens on press.
const (
	buttonRadius unit.Dp = 7
	buttonShadow unit.Dp = 2
)

// tactileButton is the Blackfyre button: a flat face on a darker 2 dp
// shadow; while pressed the face moves down onto it. A disabled button has
// no shadow and muted colours, and takes no clicks.
type tactileButton struct {
	click   *widget.Clickable
	kind    buttonKind
	label   string
	size    unit.Sp
	enabled bool
	minW    unit.Dp
	padX    unit.Dp
	padY    unit.Dp
	pal     *Palette
	th      *material.Theme
}

func (w *window) button(c *widget.Clickable, kind buttonKind, label string, enabled bool) tactileButton {
	return tactileButton{click: c, kind: kind, label: label, size: sizeLabel, enabled: enabled, padX: 12, padY: 7, pal: &w.pal, th: w.th}
}

func (b tactileButton) width(dp unit.Dp) tactileButton { b.minW = dp; return b }

// small makes a compact button, for the nudges.
func (b tactileButton) small() tactileButton {
	b.size, b.padX, b.padY = 10, 5, 5
	return b
}

func (b tactileButton) Layout(gtx layout.Context) layout.Dimensions {
	face, fg, shadow := b.pal.buttonColours(b.kind)
	if !b.enabled {
		face, fg, shadow = b.pal.SurfaceAlt, b.pal.Muted, color.NRGBA{}
		gtx = gtx.Disabled()
	}
	drop := gtx.Dp(buttonShadow)
	return b.click.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
		pressed := b.enabled && b.click.Pressed()
		m := op.Record(gtx.Ops)
		inner := gtx
		inner.Constraints.Min = image.Pt(gtx.Dp(b.minW), 0)
		dims := layout.Inset{Top: b.padY, Bottom: b.padY, Left: b.padX, Right: b.padX}.Layout(inner, func(gtx layout.Context) layout.Dimensions {
			return layout.Center.Layout(gtx, tracked(b.th, b.size, fg, b.label))
		})
		call := m.Stop()
		r := image.Rectangle{Max: dims.Size}
		radius := gtx.Dp(buttonRadius)
		offset := 0
		if pressed {
			offset = drop
		} else if shadow.A != 0 {
			fill(gtx, r.Add(image.Pt(0, drop)), radius, shadow)
		}
		t := op.Offset(image.Pt(0, offset)).Push(gtx.Ops)
		fill(gtx, r, radius, face)
		call.Add(gtx.Ops)
		t.Pop()
		return layout.Dimensions{Size: image.Pt(dims.Size.X, dims.Size.Y+drop), Baseline: dims.Baseline}
	})
}

// sideBySide lays out widgets in a row with a gap.
func sideBySide(gtx layout.Context, gap unit.Dp, ws ...layout.Widget) layout.Dimensions {
	children := make([]layout.FlexChild, 0, 2*len(ws))
	for i, w := range ws {
		if i > 0 {
			children = append(children, layout.Rigid(layout.Spacer{Width: gap}.Layout))
		}
		children = append(children, layout.Rigid(w))
	}
	return layout.Flex{Alignment: layout.Middle}.Layout(gtx, children...)
}

// stacked lays out widgets in a column with a gap.
func stacked(gtx layout.Context, gap unit.Dp, ws ...layout.Widget) layout.Dimensions {
	children := make([]layout.FlexChild, 0, 2*len(ws))
	for i, w := range ws {
		if i > 0 {
			children = append(children, layout.Rigid(layout.Spacer{Height: gap}.Layout))
		}
		children = append(children, layout.Rigid(w))
	}
	return layout.Flex{Axis: layout.Vertical}.Layout(gtx, children...)
}

// wrapped lets a label take as many lines as it needs.
func wrapped(l material.LabelStyle) layout.Widget {
	l.MaxLines = 0
	return l.Layout
}

// oneLine keeps a label to one line, truncating.
func oneLine(l material.LabelStyle) material.LabelStyle {
	l.MaxLines = 1
	l.Truncator = "…"
	l.Alignment = text.Start
	return l
}

// flow lays widgets out left to right, each at its natural size, starting
// a new line when the next one does not fit; lines are centred vertically.
func flow(gtx layout.Context, gapX, gapY unit.Dp, ws ...layout.Widget) layout.Dimensions {
	maxW := gtx.Constraints.Max.X
	gx, gy := gtx.Dp(gapX), gtx.Dp(gapY)
	type item struct {
		call op.CallOp
		size image.Point
	}
	cgtx := gtx
	cgtx.Constraints.Min = image.Point{}
	items := make([]item, len(ws))
	for i, w := range ws {
		m := op.Record(gtx.Ops)
		d := w(cgtx)
		items[i] = item{m.Stop(), d.Size}
	}
	width, y := 0, 0
	for start := 0; start < len(items); {
		end, x, h := start, 0, 0
		for end < len(items) {
			next := x + items[end].size.X
			if end > start {
				next += gx
			}
			if end > start && next > maxW {
				break
			}
			x = next
			if items[end].size.Y > h {
				h = items[end].size.Y
			}
			end++
		}
		x = 0
		for i := start; i < end; i++ {
			if i > start {
				x += gx
			}
			t := op.Offset(image.Pt(x, y+(h-items[i].size.Y)/2)).Push(gtx.Ops)
			items[i].call.Add(gtx.Ops)
			t.Pop()
			x += items[i].size.X
		}
		if x > width {
			width = x
		}
		y += h
		if end < len(items) {
			y += gy
		}
		start = end
	}
	return layout.Dimensions{Size: image.Pt(width, y)}
}
