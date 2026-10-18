package appearance

import (
	"context"
	"unsafe"

	"golang.org/x/sys/windows"

	"github.com/blackfyre/spooder/desktop/internal/winreg"
)

// Read returns the Windows appearance.
func Read() Settings {
	s := fromRegistry(winreg.System{})
	s.HighContrast = highContrast()
	return s
}

// Watch does nothing on Windows; the window re-reads the appearance when it
// regains focus.
func Watch(ctx context.Context, changed func(Settings)) {}

const (
	spiGetHighContrast = 0x0042
	hcfHighContrastOn  = 0x00000001
)

type highContrastInfo struct {
	size        uint32
	flags       uint32
	defaultName *uint16
}

func highContrast() bool {
	info := highContrastInfo{size: uint32(unsafe.Sizeof(highContrastInfo{}))}
	proc := windows.NewLazySystemDLL("user32.dll").NewProc("SystemParametersInfoW")
	ok, _, _ := proc.Call(spiGetHighContrast, uintptr(info.size), uintptr(unsafe.Pointer(&info)), 0)
	return ok != 0 && info.flags&hcfHighContrastOn != 0
}
