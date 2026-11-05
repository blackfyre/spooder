// Package fonts embeds the Blackfyre typefaces: Archivo SemiBold for text
// and JetBrains Mono Regular for labels, buttons and values. Both are under
// the SIL Open Font License 1.1, whose texts are embedded with them
// (package files: Archivo-OFL.txt, JetBrainsMono-OFL.txt) and shipped in the
// binary.
package fonts

import (
	"fmt"

	"gioui.org/font"
	"gioui.org/font/opentype"

	"github.com/blackfyre/spooder/desktop/internal/ui/fonts/files"
)

// Typeface names.
const (
	Text font.Typeface = "Archivo"
	Mono font.Typeface = "JetBrains Mono"
)

var (
	archivo       = files.ArchivoSemiBold
	jetBrainsMono = files.JetBrainsMonoRegular

	// ArchivoLicence and JetBrainsMonoLicence are the fonts' OFL texts.
	ArchivoLicence       = files.ArchivoLicence
	JetBrainsMonoLicence = files.JetBrainsMonoLicence
)

// Collection returns both faces for a text shaper.
func Collection() ([]font.FontFace, error) {
	text, err := opentype.Parse(archivo)
	if err != nil {
		return nil, fmt.Errorf("parse Archivo: %w", err)
	}
	mono, err := opentype.Parse(jetBrainsMono)
	if err != nil {
		return nil, fmt.Errorf("parse JetBrains Mono: %w", err)
	}
	return []font.FontFace{
		{Font: font.Font{Typeface: Text, Weight: font.SemiBold}, Face: text},
		{Font: font.Font{Typeface: Mono, Weight: font.Normal}, Face: mono},
	}, nil
}
