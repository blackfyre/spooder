package appearance

import (
	"image/color"
	"math"
	"testing"
)

func TestParseScheme(t *testing.T) {
	for _, tc := range []struct {
		v        any
		dark, ok bool
	}{
		{uint32(1), true, true},
		{uint32(2), false, true},
		{uint32(0), false, true},
		{"dark", false, false},
	} {
		dark, ok := parseScheme(tc.v)
		if dark != tc.dark || ok != tc.ok {
			t.Errorf("%v: got %v %v", tc.v, dark, ok)
		}
	}
}

func TestParseContrast(t *testing.T) {
	if high, ok := parseContrast(uint32(1)); !high || !ok {
		t.Fatal("contrast 1 not high")
	}
	if high, _ := parseContrast(uint32(0)); high {
		t.Fatal("contrast 0 high")
	}
}

func TestParseAccent(t *testing.T) {
	got, ok := parseAccent([]any{0.478431, 0.635294, 0.968628})
	if !ok || got != (color.NRGBA{R: 122, G: 162, B: 247, A: 255}) {
		t.Fatalf("got %v %v", got, ok)
	}
	for _, bad := range []any{
		[]any{1.2, 0.5, 0.5},
		[]any{-1.0, 0.5, 0.5},
		[]any{math.NaN(), 0.5, 0.5},
		[]any{0.5, 0.5},
		[]any{"a", 0.5, 0.5},
		uint32(3),
	} {
		if _, ok := parseAccent(bad); ok {
			t.Errorf("%v accepted", bad)
		}
	}
}

func TestAccentFromABGR(t *testing.T) {
	// Windows stores 0xAABBGGRR: this is R=0x10, G=0x7C, B=0xD7 (Windows blue).
	if got := accentFromABGR(0xFFD77C10); got != (color.NRGBA{R: 0x10, G: 0x7C, B: 0xD7, A: 0xFF}) {
		t.Fatalf("got %v", got)
	}
}
