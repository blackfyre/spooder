// Package files embeds the Blackfyre typefaces' font files, Archivo SemiBold
// and JetBrains Mono Regular, with their SIL Open Font License 1.1 texts. It
// imports nothing beyond embed, so both the calibrator (through package fonts,
// for Gio) and the handheld (desktop/handheld, for SDL_ttf) can use it.
package files

import _ "embed"

var (
	//go:embed Archivo-SemiBold.ttf
	ArchivoSemiBold []byte
	//go:embed JetBrainsMono-Regular.ttf
	JetBrainsMonoRegular []byte

	// ArchivoLicence and JetBrainsMonoLicence are the fonts' OFL texts.
	//go:embed Archivo-OFL.txt
	ArchivoLicence string
	//go:embed JetBrainsMono-OFL.txt
	JetBrainsMonoLicence string
)
