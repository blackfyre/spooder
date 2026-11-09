package main

import (
	"fmt"
	"strings"
	"time"

	"github.com/veandco/go-sdl2/gfx"
	"github.com/veandco/go-sdl2/sdl"
	"github.com/veandco/go-sdl2/ttf"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/handheld/internal/supplyband"
	"github.com/blackfyre/spooder/desktop/internal/ui/fonts/files"
)

// The drive screen in the Blackfyre look, dark as the design system's
// Applications section builds it: the window on bf-ink, panels on
// bf-ink-raised with a black outline, wells on bf-ink-code, text in
// bf-surface, labels in uppercase tracked JetBrains Mono in bf-ink-subtle,
// one orange. The layout is for 1280×720 (the TrimUI Smart Pro) and scales
// with the window's width: a header (the title and the status chips), the
// robot's panel on the left, what the pak requests and the link's quality
// on the right, and the controls as keycaps along the bottom (the leave
// banner in their place while leaving). Select + X shows the driving view in
// the panels' place (driveview.go). Holding L2 opens the gesture overlay
// over either page (gestures.go).
const (
	layoutWidth = 1280
	margin      = 28
	panelGap    = 20
	panelPad    = 20
	panelRadius = 16 // cards inside the window: bf-radius-2xl
	wellRadius  = 10 // bf-radius-lg
	chipHeight  = 38
	keyHeight   = 40
	keyRadius   = 7 // the keyboards' corners
	keyEdge     = 3 // bf-shadow-raised-dark's edge
	headerH     = 56
	labelGap    = 6  // between a cell's label and its value
	kickerGap   = 14 // below a panel's kicker
	cellGap     = 14 // between rows of cells
	dotRadius   = 6

	heroSize  = 76 // the body mode
	titleSize = 40 // "Spooder."
	valueSize = 28 // values in Archivo
	bigMono   = 44 // the supply's figures
	monoSize  = 28 // numbers
	microSize = 17 // labels, kickers, chips and keycaps
	hintSize  = 13 // the keys named beside a requested value's label

	// Tracking, in em: labels and keycaps keep the label spacing (0.06em);
	// kickers 0.1em, micro labels 0.12em.
	trackLabel  = 0.06
	trackKicker = 0.10
	trackMicro  = 0.12
)

// The Blackfyre tokens the screen draws with (desktop/internal/ui/theme.go
// has the same values from the design system).
var (
	colWindow    = hex(0x1B1B19) // bf-ink
	colPanel     = hex(0x2C2C29) // bf-ink-raised
	colWell      = hex(0x121211) // bf-ink-code
	colChip      = hex(0x242421) // bf-ink-chip
	colKey       = hex(0x383834) // bf-ink-card-hover
	colKeyEdge   = hex(0x0E0E0D) // bf-ink-deep
	colOutline   = hex(0x000000) // bf-shadow-outline-dark
	colText      = hex(0xF3F2EE) // bf-surface
	colMuted     = hex(0xA19E96) // bf-ink-subtle
	colAccent    = hex(0xE2551D) // bf-accent
	colAccentTxt = hex(0xFF8A4C) // bf-accent-on-dark
	colOnAccent  = hex(0x1B1B19) // bf-on-accent
	colOk        = hex(0x7FD88A) // bf-ok
	colWarn      = hex(0xE0AF68) // bf-warn: fill, and text on dark
	colOnStatus  = hex(0x1B1B19) // bf-ink, on status fills
)

func hex(v uint32) sdl.Color {
	return sdl.Color{R: uint8(v >> 16), G: uint8(v >> 8), B: uint8(v), A: 0xff}
}

// key is a keycap's label and the control that presses it (none for Menu,
// which never reaches the mapping).
type key struct {
	label string
	b     padmap.Buttons
}

// The controls along the bottom: keycaps, then what they do. clear marks the
// fault clear, whose hold fills its text; onView is what the page toggle
// does while the driving view shows.
var helpKeys = []struct {
	keys   []key
	does   string
	clear  bool
	onView string
}{
	{[]key{{"MENU", 0}}, "LEAVE", false, ""},
	{[]key{{"SELECT", padmap.HandSelect}, {"START", padmap.HandStart}}, "E-STOP", false, ""},
	{[]key{{"SELECT", padmap.HandSelect}, {"Y", padmap.HandY}}, "HOLD: CLEAR FAULT", true, ""},
	{[]key{{"SELECT", padmap.HandSelect}, {"X", padmap.HandX}}, "VIEW", false, "PANELS"},
	{[]key{{"L2", padmap.HandL2}}, "GESTURES", false, ""},
}

// fontSet is the screen's faces: Archivo SemiBold for what is read,
// JetBrains Mono for labels and numbers (the Blackfyre typefaces; gabagool
// keeps its own fonts internal).
type fontSet struct {
	hero, title, value     *ttf.Font // Archivo
	big, mono, micro, hint *ttf.Font // JetBrains Mono
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
	for _, o := range []struct {
		dst  **ttf.Font
		data []byte
		size int
	}{
		{&f.hero, files.ArchivoSemiBold, heroSize},
		{&f.title, files.ArchivoSemiBold, titleSize},
		{&f.value, files.ArchivoSemiBold, valueSize},
		{&f.big, files.JetBrainsMonoRegular, bigMono},
		{&f.mono, files.JetBrainsMonoRegular, monoSize},
		{&f.micro, files.JetBrainsMonoRegular, microSize},
		{&f.hint, files.JetBrainsMonoRegular, hintSize},
	} {
		ft, err := openFont(o.data, px(o.size))
		if err != nil {
			f.close()
			return nil, err
		}
		*o.dst = ft
	}
	return f, nil
}

func (f *fontSet) close() {
	for _, ft := range []*ttf.Font{f.hero, f.title, f.value, f.big, f.mono, f.micro, f.hint} {
		if ft != nil {
			ft.Close()
		}
	}
}

// tracking is em in pixels for f's size.
func tracking(f *ttf.Font, em float64) int32 {
	return int32(float64(f.Height())*em*0.8 + 0.5) // a line is about 1.25em
}

// trackedWidth is s's width in f with em tracking between its characters.
func trackedWidth(f *ttf.Font, s string, em float64) int32 {
	var w int32
	n := 0
	for _, r := range s {
		if m, err := f.GlyphMetrics(r); err == nil {
			w += int32(m.Advance)
		}
		n++
	}
	if n > 1 {
		w += int32(n-1) * tracking(f, em)
	}
	return w
}

// textWidth is s's width in f, untracked.
func textWidth(f *ttf.Font, s string) int32 {
	w, _, err := f.SizeUTF8(s)
	if err != nil {
		return 0
	}
	return int32(w)
}

// fit shortens str with an ellipsis until it is at most width wide in f.
func fit(f *ttf.Font, str string, width int32) string {
	if textWidth(f, str) <= width {
		return str
	}
	r := []rune(str)
	for len(r) > 0 {
		r = r[:len(r)-1]
		cut := strings.TrimRight(string(r), " ,") + "…"
		if textWidth(f, cut) <= width {
			return cut
		}
	}
	return ""
}

// layout is the screen's geometry for one window size and font set.
type layout struct {
	scale                 float64
	w, h                  int32
	left, colW, right     int32 // the two columns
	headerY, bodyY        int32
	footerY               int32 // the keycaps' top
	bodyBottom            int32
	requestedY, requested int32 // the right column's panels: top and height
	linkY, link           int32
	microH, valueH        int32
}

func newLayout(w, h int32, f *fontSet) layout {
	l := layout{scale: float64(w) / layoutWidth, w: w, h: h}
	l.left = l.gap(margin)
	l.colW = (w - 2*l.gap(margin) - l.gap(panelGap)) / 2
	l.right = l.left + l.colW + l.gap(panelGap)
	l.headerY = l.gap(margin) - l.gap(4)
	l.bodyY = l.headerY + l.gap(headerH) + l.gap(16)
	l.footerY = h - l.gap(margin) - l.gap(keyHeight) - l.gap(keyEdge)
	l.bodyBottom = l.footerY - l.gap(panelGap)
	l.microH = int32(f.micro.Height())
	l.valueH = int32(f.value.Height())
	// The requested panel takes its two rows of three cells; the link panel
	// (three rows of two) the rest.
	l.requestedY, l.requested = l.bodyY, l.gridH(requestedRows)
	l.linkY = l.bodyY + l.requested + l.gap(panelGap)
	l.link = l.bodyBottom - l.linkY
	return l
}

func (l layout) gap(v int) int32 { return int32(float64(v)*l.scale + 0.5) }

// cellH is one label-over-value cell's height.
func (l layout) cellH() int32 { return l.microH + l.gap(labelGap) + l.valueH }

// gridH is the height a panel needs for its kicker and rows of cells.
func (l layout) gridH(rows int) int32 {
	return 2*l.gap(panelPad) + l.microH + l.gap(kickerGap) + int32(rows)*l.cellH() + int32(rows-1)*l.gap(cellGap)
}

// cell is one labelled value, with the keys that change it named beside
// its label.
type cell struct {
	label, value string
	colour       sdl.Color
	mono         bool // a number: JetBrains Mono
	keys         []key
}

// texCache keeps rendered text between frames; draw drops what a frame did
// not use.
type texKey struct {
	f *ttf.Font
	s string
	c sdl.Color
}

type texEntry struct {
	tex  *sdl.Texture
	w, h int32
	used bool
}

// screen draws the drive screen (design D4) on gabagool's renderer.
type screen struct {
	win   window
	r     *sdl.Renderer
	f     *fontSet
	l     layout
	cache map[texKey]*texEntry
	// now is the screen's clock (design D5 of add-handheld-gesture-overlay):
	// the wall clock in main, a fixed or stepped one in tests; nil is the
	// wall clock.
	now func() time.Time
	ov  overlayState // the gesture overlay (gestures.go)
}

// clock is the screen's time now.
func (s *screen) clock() time.Time {
	if s.now == nil {
		return time.Now()
	}
	return s.now()
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
	return &screen{win: win, r: r, f: f, l: newLayout(win.GetWidth(), win.GetHeight(), f), now: time.Now}, nil
}

func (s *screen) close() {
	for k, e := range s.cache {
		_ = e.tex.Destroy()
		delete(s.cache, k)
	}
	s.f.close()
}

// texture returns str rendered in f and c, from the cache when it can.
func (s *screen) texture(f *ttf.Font, str string, c sdl.Color) *texEntry {
	if s.cache == nil {
		s.cache = map[texKey]*texEntry{}
	}
	k := texKey{f, str, c}
	if e, ok := s.cache[k]; ok {
		e.used = true
		return e
	}
	surf, err := f.RenderUTF8Blended(str, c)
	if err != nil {
		return nil
	}
	defer surf.Free()
	tex, err := s.r.CreateTextureFromSurface(surf)
	if err != nil {
		return nil
	}
	e := &texEntry{tex: tex, w: surf.W, h: surf.H, used: true}
	s.cache[k] = e
	return e
}

// endFrame drops the textures this frame did not use.
func (s *screen) endFrame() {
	for k, e := range s.cache {
		if !e.used {
			_ = e.tex.Destroy()
			delete(s.cache, k)
			continue
		}
		e.used = false
	}
}

// text draws str at x, y, shortened to width, and returns its width.
func (s *screen) text(f *ttf.Font, str string, x, y, width int32, c sdl.Color) int32 {
	str = fit(f, str, width)
	if str == "" {
		return 0
	}
	e := s.texture(f, str, c)
	if e == nil {
		return 0
	}
	_ = s.r.Copy(e.tex, nil, &sdl.Rect{X: x, Y: y, W: e.w, H: e.h})
	return e.w
}

// tracked draws str one character at a time with em tracking and returns
// its width. Labels are short and set within their cells, so nothing is
// shortened.
func (s *screen) tracked(f *ttf.Font, str string, x, y int32, c sdl.Color, em float64) int32 {
	x0, t := x, tracking(f, em)
	for i, r := range str {
		if i > 0 {
			x += t
		}
		m, err := f.GlyphMetrics(r)
		if err != nil {
			continue
		}
		if r != ' ' {
			if e := s.texture(f, string(r), c); e != nil {
				_ = s.r.Copy(e.tex, nil, &sdl.Rect{X: x, Y: y, W: e.w, H: e.h})
			}
		}
		x += int32(m.Advance)
	}
	return x - x0
}

func (s *screen) fill(x, y, w, h, rad int32, c sdl.Color) {
	if w <= 0 || h <= 0 {
		return
	}
	if rad <= 0 {
		_ = s.r.SetDrawColor(c.R, c.G, c.B, c.A)
		_ = s.r.FillRect(&sdl.Rect{X: x, Y: y, W: w, H: h})
		return
	}
	gfx.RoundedBoxColor(s.r, x, y, x+w-1, y+h-1, min(rad, h/2, w/2), c)
}

// panel draws a card: a black outline, the raised face, and its kicker
// ("01 · ROBOT"); it returns the y below the kicker.
func (s *screen) panel(x, y, w, h int32, number, title string, titleColour sdl.Color) int32 {
	l := s.l
	s.fill(x-1, y-1, w+2, h+2, l.gap(panelRadius)+1, colOutline)
	s.fill(x, y, w, h, l.gap(panelRadius), colPanel)
	kx, ky := x+l.gap(panelPad), y+l.gap(panelPad)
	kx += s.tracked(s.f.micro, number, kx, ky, colAccentTxt, trackKicker)
	kx += s.tracked(s.f.micro, " · ", kx, ky, colMuted, trackKicker)
	s.tracked(s.f.micro, title, kx, ky, titleColour, trackKicker)
	return ky + l.microH + l.gap(kickerGap)
}

// The panels' grids: rows and columns of cells.
const (
	requestedRows, requestedCols = 2, 3
	linkRows, linkCols           = 3, 2
)

// cellW is a cell's width in a grid of cols across width.
func (l layout) cellW(width int32, cols int) int32 {
	return (width - int32(cols-1)*l.gap(cellGap)) / int32(cols)
}

// cells draws rows of cols label-over-value cells across width from x, y;
// a hint key beside a label presses while held has its control (hintPressed).
func (s *screen) cells(cs []cell, cols int, x, y, width int32, held padmap.Buttons) {
	l := s.l
	colW := l.cellW(width, cols)
	for i, c := range cs {
		cx := x + int32(i%cols)*(colW+l.gap(cellGap))
		cy := y + int32(i/cols)*(l.cellH()+l.gap(cellGap))
		kx := cx + s.tracked(s.f.micro, c.label, cx, cy, colMuted, trackMicro) + l.gap(10)
		for _, k := range c.keys {
			kx += s.hintKey(kx, cy, k.label, hintPressed(k.b, held)) + l.gap(4)
		}
		vy := cy + l.microH + l.gap(labelGap)
		if c.mono {
			// Mono sits on the same baseline as Archivo's value line.
			vy += (l.valueH - int32(s.f.mono.Height())) / 2
			s.text(s.f.mono, c.value, cx, vy, colW, c.colour)
		} else {
			s.text(s.f.value, c.value, cx, vy, colW, c.colour)
		}
	}
}

// chip draws a pill at x, y (its height chipHeight) with an optional dot
// and tracked micro text; it returns its width.
func (s *screen) chip(x, y int32, text string, face, fg sdl.Color, dot *sdl.Color) int32 {
	l := s.l
	h := l.gap(chipHeight)
	pad := l.gap(14)
	tw := trackedWidth(s.f.micro, text, trackLabel)
	w := 2*pad + tw
	if dot != nil {
		w += 2*l.gap(dotRadius) + l.gap(10)
	}
	s.fill(x, y, w, h, h/2, face)
	tx := x + pad
	if dot != nil {
		r := l.gap(dotRadius)
		gfx.FilledCircleColor(s.r, tx+r, y+h/2, r, *dot)
		gfx.AACircleColor(s.r, tx+r, y+h/2, r, *dot)
		tx += 2*r + l.gap(10)
	}
	s.tracked(s.f.micro, text, tx, y+(h-l.microH)/2, fg, trackLabel)
	return w
}

// chipWidth is chip's width without drawing it.
func (s *screen) chipWidth(text string, dot bool) int32 {
	l := s.l
	w := 2*l.gap(14) + trackedWidth(s.f.micro, text, trackLabel)
	if dot {
		w += 2*l.gap(dotRadius) + l.gap(10)
	}
	return w
}

// keycapWidth is a footer keycap's width.
func (s *screen) keycapWidth(label string) int32 {
	l := s.l
	return max(l.gap(keyHeight), 2*l.gap(12)+trackedWidth(s.f.micro, label, trackLabel))
}

// keycap draws a raised key at x, y and returns its width. A pressed key
// sits down on its edge, as the system's controls do on :active, with its
// label in the orange text tone.
func (s *screen) keycap(x, y int32, label string, pressed bool) int32 {
	l := s.l
	h := l.gap(keyHeight)
	w := s.keycapWidth(label)
	fg := colText
	if pressed {
		y += l.gap(keyEdge)
		fg = colAccentTxt
	} else {
		s.fill(x, y+l.gap(keyEdge), w, h, l.gap(keyRadius), colKeyEdge)
	}
	s.fill(x, y, w, h, l.gap(keyRadius), colKey)
	tw := trackedWidth(s.f.micro, label, trackLabel)
	s.tracked(s.f.micro, label, x+(w-tw)/2, y+(h-l.microH)/2, fg, trackLabel)
	return w
}

// hintKeyWidth is a small key's width beside a label.
func (s *screen) hintKeyWidth(label string) int32 {
	l := s.l
	return max(l.microH, 2*l.gap(6)+trackedWidth(s.f.hint, label, trackLabel))
}

// hintEdge is a hint key's edge, below its face at rest.
const hintEdge = 2

// hintKey draws a small key, as high as a label, at x, y and returns its
// width. At rest it stands on a thin edge; pressed, it sits down on it with
// its label in the orange text tone, as a footer keycap does.
func (s *screen) hintKey(x, y int32, label string, pressed bool) int32 {
	l := s.l
	w := s.hintKeyWidth(label)
	fg := colText
	if pressed {
		y += l.gap(hintEdge)
		fg = colAccentTxt
	} else {
		s.fill(x, y+l.gap(hintEdge), w, l.microH, l.gap(4), colKeyEdge)
	}
	s.fill(x, y, w, l.microH, l.gap(4), colKey)
	tw := trackedWidth(s.f.hint, label, trackLabel)
	s.tracked(s.f.hint, label, x+(w-tw)/2, y+(l.microH-int32(s.f.hint.Height()))/2, fg, trackLabel)
	return w
}

// hintPressed is whether a requested value's hint key for control b shows
// as pressed with held held (design D2 of handheld-observations): while b is
// held and its press reaches that value, so not Start, X, Y or B while
// Select is held (the e-stop, the page, the fault clear and the sound), nor
// A, B, X or Y while L2 is held (the gesture layer).
func hintPressed(b, held padmap.Buttons) bool {
	switch {
	case b == 0 || held&b == 0:
		return false
	case held&padmap.HandSelect != 0 && b&(padmap.HandStart|padmap.HandX|padmap.HandY|padmap.HandB) != 0:
		return false
	case held&padmap.HandL2 != 0 && b&(padmap.HandA|padmap.HandB|padmap.HandX|padmap.HandY) != 0:
		return false
	}
	return true
}

func (s *screen) draw(sc app.Screen) {
	l := s.l
	now := s.clock()
	s.ov.update(sc, now)
	s.fill(0, 0, l.w, l.h, 0, colWindow)
	s.header(sc)
	if sc.DriveView {
		s.driveView(sc)
	} else {
		s.robot(sc.Robot, robotNotice(sc))
		s.requested(sc.Requested, sc.Held)
		s.link(sc.Link)
	}
	if sc.State == app.Leaving || sc.State == app.Acting {
		s.leaveBanner(sc)
	} else {
		s.footer(sc.Held, sc.Clearing, sc.DriveView)
	}
	if s.ov.open {
		s.overlay(sc, now)
	}
	s.endFrame()
	s.win.Present()
}

// header: "Spooder." with the orange full stop, then the chips on the right:
// the link's state, the signal, and the emergency stop (orange when set).
func (s *screen) header(sc app.Screen) {
	l := s.l
	y := l.headerY
	tx := l.left
	tx += s.text(s.f.title, "Spooder", tx, y+(l.gap(headerH)-int32(s.f.title.Height()))/2, l.colW, colText)
	s.text(s.f.title, ".", tx, y+(l.gap(headerH)-int32(s.f.title.Height()))/2, l.colW, colAccent)

	cy := y + (l.gap(headerH)-l.gap(chipHeight))/2
	type chipSpec struct {
		text     string
		face, fg sdl.Color
		dot      *sdl.Color
	}
	estop := chipSpec{"E-STOP OFF", colChip, colMuted, nil}
	if sc.Requested.EStop {
		estop = chipSpec{"E-STOP ON", colAccent, colOnAccent, nil}
	}
	ok, warn, muted := colOk, colWarn, colMuted
	linkChip := chipSpec{"NO FRAMES", colChip, colText, &muted}
	if sc.Link.HaveFrame {
		if sc.Link.SinceLast >= app.StaleAfter {
			linkChip = chipSpec{"LINK STALE", colChip, colText, &warn}
		} else {
			linkChip = chipSpec{"LINK", colChip, colText, &ok}
		}
	}
	if sc.Link.OffNetwork {
		// Off the network the run started on: nothing is sent
		// (handheld-network-check), whatever still arrives.
		alarm := colAccent
		linkChip = chipSpec{"OFF NETWORK", colChip, colText, &alarm}
	}
	signal := "SIGNAL —"
	if sc.Link.SignalKnown {
		signal = fmt.Sprintf("%d DBM", sc.Link.SignalDBm)
	}
	sigDot := signalTone(sc.Link.SignalKnown, sc.Link.SignalDBm)
	chips := []chipSpec{linkChip, {signal, colChip, colText, &sigDot}, estop}
	if !sc.SoundOn {
		// Select + B switched the sound feedback off.
		chips = append([]chipSpec{{"SOUND OFF", colChip, colMuted, nil}}, chips...)
	}
	if sc.Robot.Have && sc.Robot.Simulated {
		// Bench simulation, on both pages: no servo moves.
		chips = append([]chipSpec{{"SIM", colWarn, colOnStatus, nil}}, chips...)
	}
	x := l.w - l.gap(margin)
	for i := len(chips) - 1; i >= 0; i-- {
		c := chips[i]
		x -= s.chipWidth(c.text, c.dot != nil)
		s.chip(x, cy, c.text, c.face, c.fg, c.dot)
		x -= l.gap(10)
	}
}

// The signal levels the header's signal dot changes tone at, in dBm: from
// signalGood up the link is strong, from signalWeak up usable, below it weak.
const (
	signalGood = -67
	signalWeak = -80
)

// signalTone is the tone of the header's signal dot: ok when strong, the
// warning tone when usable, orange when weak, muted when unknown.
func signalTone(known bool, dbm int) sdl.Color {
	switch {
	case !known:
		return colMuted
	case dbm >= signalGood:
		return colOk
	case dbm >= signalWeak:
		return colWarn
	default:
		return colAccent
	}
}

// robot: the body mode large, output on or off, the supply in a well, gait
// and gesture, and the faults as chips; a notice at the panel's foot.
func (s *screen) robot(rb app.Robot, notice string) {
	l := s.l
	x, w := l.left, l.colW
	h := l.bodyBottom - l.bodyY
	title, tc := "ROBOT", colMuted
	if rb.Stale {
		title, tc = "ROBOT · STALE", colWarn
	}
	y := s.panel(x, l.bodyY, w, h, "01", title, tc)
	ix, iw := x+l.gap(panelPad), w-2*l.gap(panelPad)
	val := colText
	if rb.Stale {
		val = colMuted
	}

	// The body mode, a display title: an orange full stop, and the output
	// chip beside it.
	body := "No telemetry"
	if rb.Have {
		body = rb.Body
	}
	heroY := y - l.gap(8)
	bw := s.text(s.f.hero, body, ix, heroY, iw, val)
	if rb.Have {
		bw += s.text(s.f.hero, ".", ix+bw, heroY, iw-bw, colAccent)
		out, dot := "OUTPUT OFF", colMuted
		if rb.OutputsOn {
			out, dot = "OUTPUT ON", colOk
			if rb.Simulated {
				out = "OUTPUT SIM"
			}
		}
		if rb.Stale {
			dot = colMuted
		}
		cw := s.chipWidth(out, true)
		s.chip(ix+iw-cw, heroY+(int32(s.f.hero.Height())-l.gap(chipHeight))/2, out, colChip, colText, &dot)
	}
	y = heroY + int32(s.f.hero.Height()) + l.gap(10)

	// The supply in a well.
	wellH := l.gap(18) + l.microH + l.gap(labelGap) + int32(s.f.big.Height()) + l.gap(18)
	s.fill(ix, y, iw, wellH, l.gap(wellRadius), colWell)
	volts, amps := "—", "—"
	if rb.Have {
		volts = fmt.Sprintf("%.2f V", float64(rb.VoltageMv)/1000)
		amps = fmt.Sprintf("%.2f A", float64(rb.CurrentMa)/1000)
	}
	half := iw / 2
	vc, ac := supplyColours(rb, val)
	for i, v := range []struct {
		label, value string
		c            sdl.Color
	}{{"SUPPLY", volts, vc}, {"CURRENT", amps, ac}} {
		vx := ix + l.gap(20) + int32(i)*half
		s.tracked(s.f.micro, v.label, vx, y+l.gap(18), colMuted, trackMicro)
		s.text(s.f.big, v.value, vx, y+l.gap(18)+l.microH+l.gap(labelGap), half-l.gap(24), v.c)
	}
	y += wellH + l.gap(22)

	s.cells(robotCells(rb, val), robotCols, ix, y, iw, 0)
	y += l.cellH() + l.gap(cellGap)

	// Faults: chips that wrap; the emergency stop in orange, others in the
	// warning tone, always with their names.
	s.tracked(s.f.micro, "FAULTS", ix, y, colMuted, trackMicro)
	y += l.microH + l.gap(labelGap) + l.gap(2)
	bottom := l.bodyBottom - l.gap(panelPad)
	if notice != "" {
		bottom -= l.microH + l.gap(12)
	}
	s.faultChips(rb, ix, y, iw, bottom, val)
	if notice != "" {
		ny := l.bodyBottom - l.gap(panelPad) - l.microH
		nx := ix + s.tracked(s.f.micro, "NOTICE  ", ix, ny, colWarn, trackMicro)
		s.text(s.f.micro, notice, nx, ny, ix+iw-nx, colWarn)
	}
}

// robotNotice is the robot panel's notice: off the network the run started
// on, the network to rejoin (handheld-network-check), in place of any other.
func robotNotice(sc app.Screen) string {
	if sc.Link.OffNetwork {
		return "REJOIN " + sc.Link.Network + " · NOTHING SENT"
	}
	return sc.Notice
}

// bandColour is a supply value's colour for its band (design D3 of
// handheld-observations): the orange at or past a fault level (the
// Blackfyre palette's alarm colour: it has no red), the warning tone in the
// warning band, ok when ok, and normal without a band.
func bandColour(b supplyband.Band, normal sdl.Color) sdl.Color {
	switch b {
	case supplyband.Fault:
		return colAccentTxt
	case supplyband.Warning:
		return colWarn
	case supplyband.Ok:
		return colOk
	}
	return normal
}

// supplyColours are the supply voltage's and current's colours: by band
// against the robot's reported limits from fresh telemetry, val (the value
// tone, muted while stale) otherwise.
func supplyColours(rb app.Robot, val sdl.Color) (volts, amps sdl.Color) {
	if !rb.Have || rb.Stale {
		return val, val
	}
	return bandColour(supplyband.Voltage(rb.SupplyLimits, rb.VoltageMv), val),
		bandColour(supplyband.Current(rb.SupplyLimits, rb.CurrentMa), val)
}

// faultChips draws the faults from x, y as chips that wrap within width,
// down to bottom: the emergency stop in orange, others in the warning tone,
// always with their names.
func (s *screen) faultChips(rb app.Robot, x, y, width, bottom int32, val sdl.Color) {
	l := s.l
	switch {
	case !rb.Have:
		s.text(s.f.value, "—", x, y, width, colMuted)
	case len(rb.Faults) == 0:
		s.text(s.f.value, "None", x, y, width, val)
	default:
		cx := x
		for _, name := range rb.Faults {
			text := strings.ToUpper(name)
			face := colWarn
			if strings.Contains(name, "emergency") {
				face = colAccent
			}
			cw := s.chipWidth(text, false)
			if cx > x && cx+cw > x+width {
				cx, y = x, y+l.gap(chipHeight)+l.gap(8)
			}
			if y+l.gap(chipHeight) > bottom {
				break
			}
			s.chip(cx, y, text, face, colOnStatus, nil)
			cx += cw + l.gap(8)
		}
	}
}

// requested: what the pak requests, each value's keys pressed while held.
func (s *screen) requested(rq app.Requested, held padmap.Buttons) {
	l := s.l
	y := s.panel(l.right, l.requestedY, l.colW, l.requested, "02", "REQUESTED", colMuted)
	s.cells(requestedCells(rq), requestedCols, l.right+l.gap(panelPad), y, l.colW-2*l.gap(panelPad), held)
}

func (s *screen) link(lv app.LinkView) {
	l := s.l
	y := s.panel(l.right, l.linkY, l.colW, l.link, "03", "LINK", colMuted)
	s.cells(linkCells(lv), linkCols, l.right+l.gap(panelPad), y, l.colW-2*l.gap(panelPad), 0)
}

// footer: the controls as keycaps, each pressed while its control is held;
// the fault clear's text fills with the orange as its hold runs (clearing,
// 0 to 1); the page toggle names the page it goes to.
func (s *screen) footer(held padmap.Buttons, clearing float64, driveView bool) {
	l := s.l
	x, y := l.left, l.footerY
	ty := y + (l.gap(keyHeight)-l.microH)/2
	for i, h := range helpKeys {
		if i > 0 {
			x += l.gap(28)
		}
		for j, k := range h.keys {
			if j > 0 {
				x += l.gap(6) + s.tracked(s.f.micro, "+", x+l.gap(6), ty, colMuted, trackLabel) + l.gap(6)
			}
			x += s.keycap(x, y, k.label, k.b != 0 && held&k.b != 0)
		}
		x += l.gap(10)
		fg := colMuted
		if h.clear && clearing > 0 {
			fg = colText
		}
		does := h.does
		if driveView && h.onView != "" {
			does = h.onView
		}
		tw := s.tracked(s.f.micro, does, x, ty, fg, trackLabel)
		if h.clear && clearing > 0 {
			bar := l.gap(3)
			s.fill(x, ty+l.microH+l.gap(4), tw, bar, 0, colChip)
			s.fill(x, ty+l.microH+l.gap(4), int32(float64(tw)*clearing+0.5), bar, 0, colAccent)
		}
		x += tw
	}
}

// leaveBanner: an orange band with bf-on-accent text in the footer's place.
func (s *screen) leaveBanner(sc app.Screen) {
	l := s.l
	h := l.gap(keyHeight) + l.gap(keyEdge)
	y := l.footerY
	s.fill(l.left, y, l.w-2*l.gap(margin), h, l.gap(wellRadius), colAccent)
	text := fmt.Sprintf("LEAVING (%s) · WAITING FOR THE ROBOT TO SLEEP", strings.ToUpper(sc.Leave.String()))
	if sc.State == app.Acting {
		text = fmt.Sprintf("LEAVING (%s)", strings.ToUpper(sc.Leave.String()))
	}
	s.tracked(s.f.micro, text, l.left+l.gap(20), y+(h-l.microH)/2, colOnAccent, trackLabel)
}

// robotCols is the robot panel's cells across.
const robotCols = 3

// robotCells is the robot panel's gait, gesture and source in control. A
// source other than the bridge shows in the warning tone: the robot takes a
// fault clear only from the source in control.
func robotCells(rb app.Robot, val sdl.Color) []cell {
	gesture, gait, source, sc := "None", "—", "—", val
	if rb.Have {
		gait = rb.Gait
		if rb.Gesture != "" {
			gesture = rb.Gesture
		}
		source = sourceName(rb.Source)
		if rb.Source != "bridge" && !rb.Stale {
			sc = colWarn
		}
	}
	return []cell{{"GAIT", gait, val, false, nil}, {"GESTURE", gesture, val, false, nil}, {"SOURCE", source, sc, false, nil}}
}

func sourceName(s string) string {
	switch s {
	case "bridge":
		return "Bridge"
	case "USB":
		return "USB"
	case "conflict":
		return "Conflict"
	}
	return "None"
}

// requestedCells names, beside each label, the keys that change it (the
// mapping, internal/padmap): Y sleeps and wakes, Start walks and stands.
func requestedCells(rq app.Requested) []cell {
	return []cell{
		{"MODE", rq.Mode, colText, false, []key{{"Y", padmap.HandY}, {"START", padmap.HandStart}}},
		{"GAIT", rq.Gait, colText, false, []key{{"L1", padmap.HandL1}, {"R1", padmap.HandR1}}},
		{"STRIDE", rq.Stride, colText, false, []key{{"X", padmap.HandX}}},
		{"HEIGHT", fmt.Sprint(rq.Height), colText, true, []key{{"↑", padmap.HandUp}, {"↓", padmap.HandDown}}},
		{"SPEED", fmt.Sprint(rq.Speed), colText, true, []key{{"←", padmap.HandLeft}, {"→", padmap.HandRight}}},
		{"STICKS", rq.StickMode, colText, false, []key{{"B", padmap.HandB}}},
	}
}

func linkCells(lv app.LinkView) []cell {
	signal := "Unknown"
	sigMono := false
	if lv.SignalKnown {
		signal, sigMono = fmt.Sprintf("%d dBm", lv.SignalDBm), true
	}
	since, sc := "No frame yet", colWarn
	sinceMono := false
	if lv.HaveFrame {
		since, sc, sinceMono = lv.SinceLast.Round(10*time.Millisecond).String(), colText, true
		if lv.SinceLast >= app.StaleAfter {
			sc = colWarn
		}
	}
	count := func(n int) sdl.Color {
		if n > 0 {
			return colWarn
		}
		return colText
	}
	return []cell{
		{"SIGNAL", signal, colText, sigMono, nil},
		{"LAST FRAME", since, sc, sinceMono, nil},
		{"TEL · POSE · DIAG /S", fmt.Sprintf("%d · %d · %d", lv.Telemetry, lv.Pose, lv.Diagnostics), colText, true, nil},
		{"INVALID", fmt.Sprint(lv.Invalid), count(lv.Invalid), true, nil},
		{"LOST · LATE", fmt.Sprintf("%d · %d", lv.Lost, lv.Late), count(lv.Lost + lv.Late), true, nil},
		{"SENT (ERRORS)", fmt.Sprintf("%d (%d)", lv.Sent, lv.SendErrors), count(lv.SendErrors), true, nil},
	}
}
