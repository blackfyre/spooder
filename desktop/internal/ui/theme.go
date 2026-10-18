package ui

import (
	"image/color"
	"math"

	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
)

// Palette holds the semantic colours the UI draws with: the Blackfyre
// tokens of blackfyre.ninja (design D7). The structure, NewPalette and the
// contrast helpers follow bopen's internal/ui/theme.go.
type Palette struct {
	Bg         color.NRGBA // window background (paper in light mode)
	Well       color.NRGBA // recessed areas: logs, the diff
	Surface    color.NRGBA // panels and cards
	SurfaceAlt color.NRGBA // selection and hover
	Fg         color.NRGBA // primary text (ink)
	Muted      color.NRGBA // secondary text, values not current
	Border     color.NRGBA // card outlines and rules

	Accent        color.NRGBA // the orange: e-stop, primary actions, selection
	AccentPressed color.NRGBA // the accent's shadow and pressed fill
	AccentText    color.NRGBA // the accent as text on Bg or Surface
	OnAccent      color.NRGBA // text on Accent

	// Neutral buttons (ink in light mode) and quiet buttons (nudges).
	ButtonBg, ButtonFg, ButtonShadow color.NRGBA
	QuietBg, QuietFg, QuietShadow    color.NRGBA

	// Status fills, drawn with OnStatus text or as dots.
	Green, Yellow, Blue color.NRGBA
	OnStatus            color.NRGBA
}

func rgb(v uint32) color.NRGBA {
	return color.NRGBA{R: uint8(v >> 16), G: uint8(v >> 8), B: uint8(v), A: 0xFF}
}

// Blackfyre tokens (design D7; blackfyre.ninja styles.css and its Zed
// theme for the dark muted, accent-text and surface shades).
var (
	ink   = rgb(0x1B1B19)
	paper = rgb(0xE6E4DF)

	lightPalette = Palette{
		Bg: paper, Well: rgb(0xDEDBD4), Surface: rgb(0xF3F2EE), SurfaceAlt: rgb(0xDEDBD4),
		Fg: ink, Muted: rgb(0x5E5C56), Border: rgb(0xD3D0C9),
		Accent: rgb(0xE2551D), AccentPressed: rgb(0xA93A0F), AccentText: rgb(0xA3390C), OnAccent: ink,
		ButtonBg: ink, ButtonFg: rgb(0xF3F2EE), ButtonShadow: rgb(0x000000),
		QuietBg: rgb(0xDEDBD4), QuietFg: ink, QuietShadow: rgb(0xC4C0B8),
		Green: rgb(0x9ECE6A), Yellow: rgb(0xE0AF68), Blue: rgb(0x7AA2F7), OnStatus: ink,
	}
	darkPalette = Palette{
		Bg: ink, Well: rgb(0x121211), Surface: rgb(0x2C2C29), SurfaceAlt: rgb(0x3A3935),
		Fg: rgb(0xF3F2EE), Muted: rgb(0xA19E96), Border: rgb(0x3A3935),
		Accent: rgb(0xE2551D), AccentPressed: rgb(0xA93A0F), AccentText: rgb(0xFF8A4C), OnAccent: ink,
		ButtonBg: rgb(0x3A3935), ButtonFg: rgb(0xF3F2EE), ButtonShadow: rgb(0x121211),
		QuietBg: rgb(0x242421), QuietFg: rgb(0xF3F2EE), QuietShadow: rgb(0x121211),
		Green: rgb(0x9ECE6A), Yellow: rgb(0xE0AF68), Blue: rgb(0x7AA2F7), OnStatus: ink,
	}
)

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
