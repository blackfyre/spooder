package main

import (
	"fmt"
	"strings"
	"time"

	"github.com/veandco/go-sdl2/sdl"
	"github.com/veandco/go-sdl2/ttf"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/internal/ui/fonts/files"
)

// The drive screen's layout is for 1280×720 (the TrimUI Smart Pro) and
// scales with the window's width. Two columns: the robot on the left, with
// the leave banner or a notice below it; what the pak requests and the
// link's quality on the right; the controls' help along the bottom.
const (
	layoutWidth  = 1280
	margin       = 32
	columnGap    = 40
	titleSize    = 28
	labelSize    = 22
	valueSize    = 24
	titleGap     = 8  // below a column title, above its rule
	ruleHeight   = 2  // the rule under a column title
	ruleGap      = 10 // below the rule
	rowGap       = 4  // between rows
	blockGap     = 16 // below a block of rows
	valueAtShare = 0.42
)

var (
	colBackground = sdl.Color{R: 0x12, G: 0x13, B: 0x16, A: 0xff}
	colText       = sdl.Color{R: 0xe8, G: 0xe6, B: 0xe1, A: 0xff}
	colDim        = sdl.Color{R: 0x8a, G: 0x8f, B: 0x98, A: 0xff}
	colWarn       = sdl.Color{R: 0xe0, G: 0xa0, B: 0x30, A: 0xff}
	colDanger     = sdl.Color{R: 0xe0, G: 0x50, B: 0x50, A: 0xff}
	colRule       = sdl.Color{R: 0x2c, G: 0x2f, B: 0x35, A: 0xff}
)

const helpText = "Menu leave   Select+Start e-stop   Select+Y 1 s clear fault   L2 gestures"

// fontSet is the screen's three faces: Archivo SemiBold for titles and
// labels, JetBrains Mono for values (the Blackfyre typefaces; gabagool keeps
// its own fonts internal).
type fontSet struct {
	title, label, mono *ttf.Font
}

func openFont(data []byte, size int) (*ttf.Font, error) {
	rw, err := sdl.RWFromMem(data)
	if err != nil {
		return nil, fmt.Errorf("font: %w", err)
	}
	f, err := ttf.OpenFontRW(rw, 1, size)
	if err != nil {
		return nil, fmt.Errorf("font: %w", err)
	}
	return f, nil
}

// loadFonts opens the faces at the layout's sizes times scale. ttf must be
// initialised (gabagool.Init does it).
func loadFonts(scale float64) (*fontSet, error) {
	px := func(v int) int { return int(float64(v)*scale + 0.5) }
	f := &fontSet{}
	var err error
	if f.title, err = openFont(files.ArchivoSemiBold, px(titleSize)); err != nil {
		return nil, err
	}
	if f.label, err = openFont(files.ArchivoSemiBold, px(labelSize)); err != nil {
		f.close()
		return nil, err
	}
	if f.mono, err = openFont(files.JetBrainsMonoRegular, px(valueSize)); err != nil {
		f.close()
		return nil, err
	}
	return f, nil
}

func (f *fontSet) close() {
	for _, ft := range []*ttf.Font{f.title, f.label, f.mono} {
		if ft != nil {
			ft.Close()
		}
	}
}

// Rows per block. The layout test checks the blocks fit the screen with
// these counts.
const (
	robotRowCount     = 6
	requestedRowCount = 6
	linkRowCount      = 6
)

// layout is the screen's geometry for one window size and font set.
type layout struct {
	left, right, colW int32
	valueOff          int32 // a value's x offset in its column
	top               int32
	titleH, rowH      int32
	labelH            int32
	gap               func(int) int32
	footerY           int32 // the help line's top
	bannerY           int32 // the leave banner's or notice's top, left column
	linkY             int32 // the link block's top, right column
	rightBottom       int32 // the right column's bottom
}

func newLayout(w, h int32, f *fontSet) layout {
	scale := float64(w) / layoutWidth
	gap := func(v int) int32 { return int32(float64(v)*scale + 0.5) }
	l := layout{gap: gap}
	l.left = gap(margin)
	l.colW = (w - 2*gap(margin) - gap(columnGap)) / 2
	l.right = l.left + l.colW + gap(columnGap)
	l.valueOff = int32(float64(l.colW) * valueAtShare)
	l.top = gap(margin)
	l.titleH = int32(f.title.Height())
	l.labelH = int32(f.label.Height())
	l.rowH = int32(f.mono.Height()) + gap(rowGap)
	l.footerY = h - gap(margin) - l.labelH
	l.bannerY = l.blockBottom(l.top, robotRowCount)
	l.linkY = l.blockBottom(l.top, requestedRowCount)
	l.rightBottom = l.blockBottom(l.linkY, linkRowCount)
	return l
}

// blockHeader is a block's title, rule and gaps.
func (l layout) blockHeader() int32 {
	return l.titleH + l.gap(titleGap) + l.gap(ruleHeight) + l.gap(ruleGap)
}

// blockBottom is the y below a block of n rows starting at y.
func (l layout) blockBottom(y int32, n int) int32 {
	return y + l.blockHeader() + int32(n)*l.rowH + l.gap(blockGap)
}

// screen draws the drive screen (design D4) on gabagool's renderer.
type screen struct {
	win window
	r   *sdl.Renderer
	f   *fontSet
	l   layout
}

// window is what the screen needs of gabagool's window, whose type lives in
// gabagool's internal package.
type window interface {
	GetWidth() int32
	GetHeight() int32
	Present()
}

func newScreen(win window, r *sdl.Renderer) (*screen, error) {
	f, err := loadFonts(float64(win.GetWidth()) / layoutWidth)
	if err != nil {
		return nil, err
	}
	return &screen{win: win, r: r, f: f, l: newLayout(win.GetWidth(), win.GetHeight(), f)}, nil
}

func (s *screen) close() { s.f.close() }

// fit shortens str with an ellipsis until it is at most width wide in f.
func fit(f *ttf.Font, str string, width int32) string {
	if w, _, err := f.SizeUTF8(str); err != nil || int32(w) <= width {
		return str
	}
	r := []rune(str)
	for len(r) > 0 {
		r = r[:len(r)-1]
		cut := strings.TrimRight(string(r), " ,") + "…"
		if w, _, err := f.SizeUTF8(cut); err == nil && int32(w) <= width {
			return cut
		}
	}
	return ""
}

// text draws str at x, y, shortened to width.
func (s *screen) text(f *ttf.Font, str string, x, y, width int32, c sdl.Color) {
	str = fit(f, str, width)
	if str == "" {
		return
	}
	surf, err := f.RenderUTF8Blended(str, c)
	if err != nil {
		return
	}
	defer surf.Free()
	tex, err := s.r.CreateTextureFromSurface(surf)
	if err != nil {
		return
	}
	defer tex.Destroy()
	_ = s.r.Copy(tex, nil, &sdl.Rect{X: x, Y: y, W: surf.W, H: surf.H})
}

// wrapped draws str wrapped to width at x, y, at most up to maxY.
func (s *screen) wrapped(f *ttf.Font, str string, x, y, width, maxY int32, c sdl.Color) {
	if str == "" || maxY <= y {
		return
	}
	surf, err := f.RenderUTF8BlendedWrapped(str, c, int(width))
	if err != nil {
		return
	}
	defer surf.Free()
	tex, err := s.r.CreateTextureFromSurface(surf)
	if err != nil {
		return
	}
	defer tex.Destroy()
	h := min(surf.H, maxY-y)
	_ = s.r.Copy(tex, &sdl.Rect{W: surf.W, H: h}, &sdl.Rect{X: x, Y: y, W: surf.W, H: h})
}

// row is one label and value of a block.
type row struct {
	label, value string
	colour       sdl.Color
}

// block draws a titled block of rows at x, y.
func (s *screen) block(title string, titleColour sdl.Color, rows []row, x, y int32) {
	l := s.l
	s.text(s.f.title, title, x, y, l.colW, titleColour)
	y += l.titleH + l.gap(titleGap)
	_ = s.r.SetDrawColor(colRule.R, colRule.G, colRule.B, colRule.A)
	_ = s.r.FillRect(&sdl.Rect{X: x, Y: y, W: l.colW, H: l.gap(ruleHeight)})
	y += l.gap(ruleHeight) + l.gap(ruleGap)
	labelDy := (int32(s.f.mono.Height()) - l.labelH) / 2
	for _, rw := range rows {
		s.text(s.f.label, rw.label, x, y+labelDy, l.valueOff-l.gap(8), colDim)
		s.text(s.f.mono, rw.value, x+l.valueOff, y, l.colW-l.valueOff, rw.colour)
		y += l.rowH
	}
}

func onOff(on bool) string {
	if on {
		return "on"
	}
	return "off"
}

func (s *screen) draw(sc app.Screen) {
	l := s.l
	_ = s.r.SetDrawColor(colBackground.R, colBackground.G, colBackground.B, colBackground.A)
	_ = s.r.Clear()

	s.block(robotTitle(sc.Robot), robotColour(sc.Robot), robotRows(sc.Robot), l.left, l.top)
	s.block("REQUESTED", colText, requestedRows(sc.Requested), l.right, l.top)
	s.block("LINK", colText, linkRows(sc.Link), l.right, l.linkY)

	switch sc.State {
	case app.Leaving:
		s.wrapped(s.f.title, fmt.Sprintf("Leaving (%s): waiting for the robot to sleep", sc.Leave), l.left, l.bannerY, l.colW, l.footerY, colWarn)
	case app.Acting:
		s.wrapped(s.f.title, fmt.Sprintf("Leaving (%s)", sc.Leave), l.left, l.bannerY, l.colW, l.footerY, colWarn)
	default:
		s.wrapped(s.f.label, sc.Notice, l.left, l.bannerY, l.colW, l.footerY, colWarn)
	}
	s.text(s.f.label, helpText, l.left, l.footerY, 2*l.colW+l.gap(columnGap), colDim)
	s.win.Present()
}

func robotTitle(rb app.Robot) string {
	if rb.Stale {
		return "ROBOT (stale)"
	}
	return "ROBOT"
}

func robotColour(rb app.Robot) sdl.Color {
	if rb.Stale {
		return colWarn
	}
	return colText
}

func robotRows(rb app.Robot) []row {
	if !rb.Have {
		return []row{{"Telemetry", "none yet", colDim}}
	}
	val := colText
	if rb.Stale {
		val = colDim
	}
	faults, fc := "none", val
	if len(rb.Faults) > 0 {
		faults, fc = strings.Join(rb.Faults, ", "), colDanger
	}
	gesture := rb.Gesture
	if gesture == "" {
		gesture = "none"
	}
	return []row{
		{"Body", rb.Body, val},
		{"Gait", rb.Gait, val},
		{"Output", onOff(rb.OutputsOn), val},
		{"Faults", faults, fc},
		{"Gesture", gesture, val},
		{"Supply", fmt.Sprintf("%.2f V %.2f A", float64(rb.VoltageMv)/1000, float64(rb.CurrentMa)/1000), val},
	}
}

func requestedRows(rq app.Requested) []row {
	estop, ec := "off", colText
	if rq.EStop {
		estop, ec = "STOP", colDanger
	}
	return []row{
		{"Mode", rq.Mode, colText},
		{"Height, speed", fmt.Sprintf("%d, %d", rq.Height, rq.Speed), colText},
		{"Gait", rq.Gait, colText},
		{"Stride", rq.Stride, colText},
		{"Sticks", rq.StickMode, colText},
		{"E-stop", estop, ec},
	}
}

func linkRows(l app.LinkView) []row {
	signal := "unknown"
	if l.SignalKnown {
		signal = fmt.Sprintf("%d dBm", l.SignalDBm)
	}
	since, sc := "no frame yet", colWarn
	if l.HaveFrame {
		since, sc = l.SinceLast.Round(10*time.Millisecond).String(), colText
		if l.SinceLast >= app.StaleAfter {
			sc = colWarn
		}
	}
	count := func(n int) sdl.Color {
		if n > 0 {
			return colWarn
		}
		return colText
	}
	return []row{
		{"Signal", signal, colText},
		{"Tel/pose/diag /s", fmt.Sprintf("%d / %d / %d", l.Telemetry, l.Pose, l.Diagnostics), colText},
		{"Invalid", fmt.Sprint(l.Invalid), count(l.Invalid)},
		{"Lost / late", fmt.Sprintf("%d / %d", l.Lost, l.Late), count(l.Lost + l.Late)},
		{"Last frame", since, sc},
		{"Sent (errors)", fmt.Sprintf("%d (%d)", l.Sent, l.SendErrors), count(l.SendErrors)},
	}
}
