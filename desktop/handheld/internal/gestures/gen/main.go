// Command gen writes table_gen.go from test/golden/gestures.txt (or the copy
// SPOODER_GESTURES_GOLDEN names). Run it with go generate ./internal/gestures
// in desktop/handheld, after the golden file is regenerated.
package main

import (
	"fmt"
	"os"

	"github.com/blackfyre/spooder/desktop/handheld/internal/gestures"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "gen:", err)
		os.Exit(1)
	}
}

func run() error {
	f, err := os.Open(gestures.GoldenPath())
	if err != nil {
		return err
	}
	defer f.Close()
	gs, err := gestures.Parse(f)
	if err != nil {
		return err
	}
	src, err := gestures.Format(gs)
	if err != nil {
		return err
	}
	return os.WriteFile("table_gen.go", src, 0o644)
}
