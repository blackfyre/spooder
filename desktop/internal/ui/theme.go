package ui

import (
	"image/color"
	"math"

	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
)

// Palette holds the semantic colours the UI draws with, from the Blackfyre
// design system's tokens (bf-*). The structure, NewPalette and the contrast
// helpers follow bopen's internal/ui/theme.go.
type Palette struct {
	Bg         color.NRGBA // window background (paper in light mode)
	Well       color.NRGBA // recessed areas: logs, the diff
	Surface    color.NRGBA // panels and cards
	SurfaceAlt color.NRGBA // selection and hover
	Fg         color.NRGBA // primary text (ink)
	Muted      color.NRGBA // secondary text, values not current
	Border     color.NRGBA // card outlines and rules

	Accent        color.NRGBA // the orange: e-stop, primary actions, selection
	TextSelection color.NRGBA // selected text in the log and frames panes: the accent over Well
	AccentPressed color.NRGBA // the accent's raised edge
	AccentText    color.NRGBA // the accent as text on Bg or Surface
	OnAccent      color.NRGBA // text on Accent

	// Neutral buttons (ink in light mode) and quiet buttons (nudges, the
	// design system's light secondary button): face, text, raised edge.
	ButtonBg, ButtonFg, ButtonShadow color.NRGBA
	QuietBg, QuietFg, QuietShadow    color.NRGBA

	// Chart series: a categorical set of three, validated (dataviz
	// validator, all pairs) on Surface in each mode. Not the accent orange,
	// which marks the e-stop and active controls.
	Series [3]color.NRGBA

	// Status fills, drawn with OnStatus text or as dots.
	Green, Yellow, Blue color.NRGBA
	OnStatus            color.NRGBA

	dark bool
}

func rgb(v uint32) color.NRGBA {
	return color.NRGBA{R: uint8(v >> 16), G: uint8(v >> 8), B: uint8(v), A: 0xFF}
}

// The Blackfyre design system's colour tokens. It has one (paper) theme;
// the dark palette is built from its dark-panel tokens, as its
// Applications section describes.
var (
	bfPaper         = rgb(0xE6E4DF)
	bfSurface       = rgb(0xF3F2EE)
	bfSurfaceSunk   = rgb(0xDEDBD4)
	bfLine          = rgb(0xD3D0C9)
	bfLineStrong    = rgb(0xBFBBB3)
	bfInk           = rgb(0x1B1B19)
	bfInkRaised     = rgb(0x2C2C29)
	bfInkCardHover  = rgb(0x383834)
	bfInkDeep       = rgb(0x0E0E0D)
	bfInkCode       = rgb(0x121211)
	bfInkMuted      = rgb(0x5E5C56)
	bfInkSubtle     = rgb(0xA19E96)
	bfAccent        = rgb(0xE2551D)
	bfAccentEdge    = rgb(0xA93A0F)
	bfAccentText    = rgb(0xA3390C)
	bfAccentOnDark  = rgb(0xFF8A4C)
	bfOk            = rgb(0x7FD88A)
	bfOutlineOnDark = rgb(0x000000) // bf-shadow-outline-dark

	// The system's app tokens (its Applications section).
	bfInkChip     = rgb(0x242421)
	bfWarn        = rgb(0xE0AF68)
	bfInfo        = rgb(0x7AA2F7)
	bfSeries1     = rgb(0x2A78D6)
	bfSeries2     = rgb(0xD55181)
	bfSeries3     = rgb(0x008300)
	bfSeries1Dark = rgb(0x3987E5)
	bfSeries3Dark = rgb(0x1A9A1A)
)

// Text on the accent is ink, not bf-white, as the system's Applications
// section sets it: white reaches only 3.78:1 on the orange, ink 4.56:1.
var (
	lightPalette = Palette{
		Bg: bfPaper, Well: bfSurfaceSunk, Surface: bfSurface, SurfaceAlt: bfSurfaceSunk,
		Fg: bfInk, Muted: bfInkMuted, Border: bfLine,
		Accent: bfAccent, AccentPressed: bfAccentEdge, AccentText: bfAccentText, OnAccent: bfInk,
		ButtonBg: bfInk, ButtonFg: bfSurface, ButtonShadow: bfInkDeep,
		QuietBg: bfPaper, QuietFg: bfInk, QuietShadow: bfLineStrong,
		Green: bfOk, Yellow: bfWarn, Blue: bfInfo, OnStatus: bfInk,
		Series: [3]color.NRGBA{bfSeries1, bfSeries2, bfSeries3},
	}
	darkPalette = Palette{
		Bg: bfInk, Well: bfInkCode, Surface: bfInkRaised, SurfaceAlt: bfInkCardHover,
		Fg: bfSurface, Muted: bfInkSubtle, Border: bfOutlineOnDark,
		Accent: bfAccent, AccentPressed: bfAccentEdge, AccentText: bfAccentOnDark, OnAccent: bfInk,
		ButtonBg: bfInkCardHover, ButtonFg: bfSurface, ButtonShadow: bfInkDeep,
		QuietBg: bfInkChip, QuietFg: bfSurface, QuietShadow: bfInkDeep,
		Green: bfOk, Yellow: bfWarn, Blue: bfInfo, OnStatus: bfInk,
		Series: [3]color.NRGBA{bfSeries1Dark, bfSeries2, bfSeries3Dark},
		dark:   true,
	}
)

// The text selection's accent opacity over the well, in each mode: as much
// as keeps the text legible on it.
const (
	selectionAlphaLight = 0x40
	selectionAlphaDark  = 0x59
)

func init() {
	lightPalette.TextSelection = over(lightPalette.Accent, selectionAlphaLight, lightPalette.Well)
	darkPalette.TextSelection = over(darkPalette.Accent, selectionAlphaDark, darkPalette.Well)
}

// over is top at the given opacity (0–255) over an opaque bottom, opaque.
func over(top color.NRGBA, alpha uint8, bottom color.NRGBA) color.NRGBA {
	mix := func(t, b uint8) uint8 {
		return uint8((int(t)*int(alpha) + int(b)*(255-int(alpha)) + 127) / 255)
	}
	return color.NRGBA{R: mix(top.R, bottom.R), G: mix(top.G, bottom.G), B: mix(top.B, bottom.B), A: 0xFF}
}

// NewPalette returns the Blackfyre palette for the host appearance: light
// (paper) or dark. The system accent colour is not used; the accent is
// Blackfyre's orange.
func NewPalette(s appearance.Settings) Palette {
	if s.Dark {
		return darkPalette
	}
	return lightPalette
}

// apply sets the stock widget theme from the palette.
func (p Palette) apply(th *material.Theme) {
	th.Palette = material.Palette{Bg: p.Surface, Fg: p.Fg, ContrastBg: p.Accent, ContrastFg: p.OnAccent}
}

// luminance is the WCAG relative luminance of an opaque colour.
func luminance(c color.NRGBA) float64 {
	ch := func(v uint8) float64 {
		s := float64(v) / 255
		if s <= 0.03928 {
			return s / 12.92
		}
		return math.Pow((s+0.055)/1.055, 2.4)
	}
	return 0.2126*ch(c.R) + 0.7152*ch(c.G) + 0.0722*ch(c.B)
}

// contrast is the WCAG contrast ratio between two opaque colours.
func contrast(a, b color.NRGBA) float64 {
	la, lb := luminance(a), luminance(b)
	if la < lb {
		la, lb = lb, la
	}
	return (la + 0.05) / (lb + 0.05)
}

// faded is c at the given opacity (0–255).
func faded(c color.NRGBA, alpha uint8) color.NRGBA {
	c.A = alpha
	return c
}
