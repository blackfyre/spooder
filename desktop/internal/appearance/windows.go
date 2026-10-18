package appearance

import (
	"github.com/blackfyre/spooder/desktop/internal/winreg"
)

const (
	personalizeKey = `Software\Microsoft\Windows\CurrentVersion\Themes\Personalize`
	dwmKey         = `Software\Microsoft\Windows\DWM`
)

// fromRegistry reads the Windows colour mode and accent colour. A missing
// AppsUseLightTheme means light mode.
func fromRegistry(reg winreg.Registry) Settings {
	var s Settings
	if v, err := reg.Integer(winreg.CurrentUser, personalizeKey, "AppsUseLightTheme"); err == nil {
		s.Dark = v == 0
	}
	if v, err := reg.Integer(winreg.CurrentUser, dwmKey, "AccentColor"); err == nil {
		s.Accent, s.HasAccent = accentFromABGR(v), true
	}
	return s
}
