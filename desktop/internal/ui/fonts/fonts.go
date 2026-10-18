// Package fonts embeds the Blackfyre typefaces: Archivo SemiBold for text
// and JetBrains Mono Regular for labels, buttons and values. Both are under
// the SIL Open Font License 1.1, whose texts are embedded next to them
// (Archivo-OFL.txt, JetBrainsMono-OFL.txt) and shipped in the binary.
package fonts

import (
	_ "embed"
	"fmt"

	"gioui.org/font"
	"gioui.org/font/opentype"
)

// Typeface names.
const (
	Text font.Typeface = "Archivo"
	Mono font.Typeface = "JetBrains Mono"
)

var (
	//go:embed Archivo-SemiBold.ttf
	archivo []byte
	//go:embed JetBrainsMono-Regular.ttf
	jetBrainsMono []byte

	// ArchivoLicence and JetBrainsMonoLicence are the fonts' OFL texts.
	//go:embed Archivo-OFL.txt
	ArchivoLicence string
	//go:embed JetBrainsMono-OFL.txt
	JetBrainsMonoLicence string
)

// Collection returns both faces for a text shaper.
func Collection() ([]font.FontFace, error) {
	text, err := opentype.Parse(archivo)
	if err != nil {
		return nil, fmt.Errorf("Archivo: %w", err)
	}
	mono, err := opentype.Parse(jetBrainsMono)
	if err != nil {
		return nil, fmt.Errorf("JetBrains Mono: %w", err)
	}
	return []font.FontFace{
		{Font: font.Font{Typeface: Text, Weight: font.SemiBold}, Face: text},
		{Font: font.Font{Typeface: Mono, Weight: font.Normal}, Face: mono},
	}, nil
}
