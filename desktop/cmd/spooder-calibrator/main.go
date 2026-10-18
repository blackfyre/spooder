// Command spooder-calibrator is the desktop front-end for bench calibration
// and diagnostics of the Spooder robot on its USB port: it watches the
// robot, takes control on request for calibration, and writes the
// calibrated defaults into a spooder checkout.
package main

import (
	"flag"
	"os"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
	"github.com/blackfyre/spooder/desktop/internal/export"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/session"
	"github.com/blackfyre/spooder/desktop/internal/ui"
)

func main() {
	checkout := flag.String("checkout", "", "spooder checkout to export the calibration defaults into")
	flag.Parse()

	configDir, _ := os.UserConfigDir()
	cwd, _ := os.Getwd()
	exe, _ := os.Executable()
	remembered := ""
	if configDir != "" {
		remembered = export.LoadLastDir(configDir)
	}

	redraw := new(ui.Redraw)
	s := session.New(nil, redraw.Invalidate)
	sys := link.Serial{}
	l := link.New(sys, s, s, link.Options{})
	ui.Run(ui.Env{
		Session:    s,
		Redraw:     redraw,
		Link:       l,
		Ports:      sys.List,
		ConfigDir:  configDir,
		Checkouts:  ui.CheckoutCandidates(*checkout, remembered, cwd, exe),
		Appearance: appearance.Read(),
	})
}
