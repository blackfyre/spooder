// Command spooder-monitor is the text console for the Spooder robot's USB
// port: the robot's debug text with timestamps and, with --frames, a decode
// of each link frame. It only observes; see --help.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"

	"github.com/blackfyre/spooder/desktop/internal/console"
	"github.com/blackfyre/spooder/desktop/internal/link"
)

func main() {
	os.Exit(run())
}

func run() int {
	cfg, err := console.ParseFlags(os.Args[1:], os.Stderr)
	if errors.Is(err, flag.ErrHelp) {
		return 0
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-monitor:", err)
		return 2
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	if cfg.Keys {
		if restore, err := console.KeyMode(int(os.Stdin.Fd())); err == nil {
			defer restore()
		}
	}

	err = console.Run(ctx, cfg, console.Env{
		System: link.Serial{},
		Out:    os.Stdout,
		Keys:   os.Stdin,
	})
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-monitor:", err)
		return 1
	}
	return 0
}
