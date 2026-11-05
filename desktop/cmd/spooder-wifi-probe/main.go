// Command spooder-wifi-probe stands in for a WiFi handheld in the bridge's
// WiFi drive: command frames at 50 Hz over UDP, and a per-second summary of
// the robot frames relayed back; see --help.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"

	"github.com/blackfyre/spooder/desktop/internal/wifiprobe"
)

func main() {
	os.Exit(run())
}

func run() int {
	cfg, err := wifiprobe.ParseFlags(os.Args[1:], os.Stderr)
	if errors.Is(err, flag.ErrHelp) {
		return 0
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-wifi-probe:", err)
		return 2
	}
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	if _, err := wifiprobe.Run(ctx, cfg, os.Stdout); err != nil {
		fmt.Fprintln(os.Stderr, "spooder-wifi-probe:", err)
		return 1
	}
	return 0
}
