package appearance

import (
	"image/color"
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/winreg"
)

func TestFromRegistry(t *testing.T) {
	reg := winreg.NewFake()
	if s := fromRegistry(reg); s.Dark || s.HasAccent {
		t.Fatalf("empty registry: %+v", s)
	}
	reg.SetInteger(winreg.CurrentUser, personalizeKey, "AppsUseLightTheme", 0)
	reg.SetInteger(winreg.CurrentUser, dwmKey, "AccentColor", 0xFFD77C10)
	s := fromRegistry(reg)
	if !s.Dark || !s.HasAccent || s.Accent != (color.NRGBA{R: 0x10, G: 0x7C, B: 0xD7, A: 0xFF}) {
		t.Fatalf("got %+v", s)
	}
	reg.SetInteger(winreg.CurrentUser, personalizeKey, "AppsUseLightTheme", 1)
	if s := fromRegistry(reg); s.Dark {
		t.Fatalf("light mode read as dark: %+v", s)
	}
}
