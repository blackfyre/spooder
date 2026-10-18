// Package appearance reads the host system's colour mode, accent colour and
// contrast preference.
//
// Copied from bopen (github.com/blackfyre/bopen, internal/appearance) with
// its behaviour unchanged: the XDG desktop portal on Linux, the registry on
// Windows. MIT licence, Copyright (c) 2026 The bopen authors; see
// desktop/LICENSE.bopen. The calibrator uses only the colour mode.
package appearance

import (
	"image/color"
	"math"
)

// Settings is the host appearance.
type Settings struct {
	Dark         bool
	HighContrast bool
	// Accent is valid only when HasAccent is set.
	Accent    color.NRGBA
	HasAccent bool
}

// Portal colour-scheme values (org.freedesktop.appearance color-scheme).
const (
	schemeNoPreference uint32 = 0
	schemeDark         uint32 = 1
	schemeLight        uint32 = 2
)

// parseScheme interprets a portal color-scheme value; ok is false for a
// value of the wrong type.
func parseScheme(v any) (dark, ok bool) {
	n, ok := v.(uint32)
	if !ok {
		return false, false
	}
	return n == schemeDark, true
}

// parseContrast interprets a portal contrast value: 1 is high contrast.
func parseContrast(v any) (high, ok bool) {
	n, ok := v.(uint32)
	if !ok {
		return false, false
	}
	return n == 1, true
}

// parseAccent interprets a portal accent-color value, an (r, g, b) triple
// of float64 in the range 0–1. Values out of range are rejected.
func parseAccent(v any) (color.NRGBA, bool) {
	var rgb []float64
	switch t := v.(type) {
	case []any:
		for _, c := range t {
			f, ok := c.(float64)
			if !ok {
				return color.NRGBA{}, false
			}
			rgb = append(rgb, f)
		}
	case []float64:
		rgb = t
	default:
		return color.NRGBA{}, false
	}
	if len(rgb) != 3 {
		return color.NRGBA{}, false
	}
	var out [3]uint8
	for i, c := range rgb {
		if math.IsNaN(c) || c < 0 || c > 1 {
			return color.NRGBA{}, false
		}
		out[i] = uint8(math.Round(c * 255))
	}
	return color.NRGBA{R: out[0], G: out[1], B: out[2], A: 0xFF}, true
}

// accentFromABGR decodes the Windows DWM AccentColor DWORD (0xAABBGGRR).
func accentFromABGR(v uint64) color.NRGBA {
	return color.NRGBA{R: uint8(v), G: uint8(v >> 8), B: uint8(v >> 16), A: 0xFF}
}
