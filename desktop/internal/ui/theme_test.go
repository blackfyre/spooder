package ui

import (
	"image/color"
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
)

// Spec "System in dark mode": the Blackfyre dark palette; otherwise paper.
func TestPaletteFollowsTheSystem(t *testing.T) {
	if p := NewPalette(appearance.Settings{Dark: true}); p.Bg != rgb(0x1B1B19) || p.Well != rgb(0x121211) || p.Surface != rgb(0x2C2C29) || p.Fg != rgb(0xF3F2EE) || p.SurfaceAlt != rgb(0x383834) {
		t.Fatalf("dark %+v", p)
	}
	if p := NewPalette(appearance.Settings{}); p.Bg != rgb(0xE6E4DF) || p.Fg != rgb(0x1B1B19) || p.Muted != rgb(0x5E5C56) {
		t.Fatalf("light %+v", p)
	}
	for _, dark := range []bool{false, true} {
		p := NewPalette(appearance.Settings{Dark: dark, Accent: color.NRGBA{B: 255, A: 255}, HasAccent: true})
		if p.Accent != rgb(0xE2551D) || p.AccentPressed != rgb(0xA93A0F) {
			t.Fatalf("dark=%v: accent %v", dark, p.Accent)
		}
		if p.Green != rgb(0x7FD88A) || p.Yellow != rgb(0xE0AF68) || p.Blue != rgb(0x7AA2F7) {
			t.Fatalf("dark=%v: status colours", dark)
		}
	}
	if p := NewPalette(appearance.Settings{}); p.AccentText != rgb(0xA3390C) {
		t.Fatal("light accent text")
	}
}

// Every text and background pair the UI draws is legible in both palettes.
func TestPaletteLegibility(t *testing.T) {
	for _, dark := range []bool{false, true} {
		p := NewPalette(appearance.Settings{Dark: dark})
		check := func(what string, fg, bg color.NRGBA, min float64) {
			t.Helper()
			if r := contrast(fg, bg); r < min {
				t.Errorf("dark=%v: %s %.2f < %.1f", dark, what, r, min)
			}
		}
		for name, bg := range map[string]color.NRGBA{"bg": p.Bg, "surface": p.Surface, "well": p.Well} {
			check("text on "+name, p.Fg, bg, 4.5)
			check("muted on "+name, p.Muted, bg, 4.5)
			check("accent text on "+name, p.AccentText, bg, 4.5)
		}
		check("text on selection", p.Fg, p.SurfaceAlt, 4.5)
		check("text on text selection", p.Fg, p.TextSelection, 4.5)
		// Stray-byte lines are an accent mark; selected, they stay readable.
		check("accent text on text selection", p.AccentText, p.TextSelection, 3)
		check("text selection on well", p.TextSelection, p.Well, 1.3)
		check("text on accent", p.OnAccent, p.Accent, 4.5)
		for _, k := range []buttonKind{kindNeutral, kindPrimary, kindQuiet, kindActive} {
			face, fg, _ := p.buttonColours(k)
			check("button text", fg, face, 4.5)
		}
		check("disabled button", p.Muted, p.SurfaceAlt, 3)
		for name, c := range map[string]color.NRGBA{"green": p.Green, "yellow": p.Yellow, "blue": p.Blue} {
			check("text on "+name, p.OnStatus, c, 4.5)
		}
		check("accent outline on surface", p.Accent, p.Surface, 3)
	}
}
