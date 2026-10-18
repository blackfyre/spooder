// Package console is spooder-monitor, the text console for the robot's USB
// port: it prints the robot's debug text with timestamps and, on request, a
// one-line decode of each link frame. It shares the desktop calibrator's
// stream splitting and decoding, but it never writes a frame: only typed
// keys, with --keys, ever reach the port (openspec change
// add-robot-console-monitor).
package console

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"strconv"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
)

// Config is the command line.
type Config struct {
	Port           string // "" picks the first Servo 2040
	Frames         bool
	TelemetryEvery int
	Keys           bool
}

const usageText = `Usage: spooder-monitor [flags]

A text console for the Spooder robot's USB port. It prints the robot's debug
text, one timestamped line each, and keeps the binary link frames out of it.
With --frames it also prints a one-line decode of every frame.

It only observes: it never sends a frame, so it never claims control, never
reports a controller and never sends an emergency stop. Without --keys it
writes nothing to the port at all. With --keys it forwards the printable
ASCII typed on standard input, for the keyboard commands of a --serial build.

Without --port it uses the first Servo 2040 (USB ID 2e8a:000a), waiting for
one to appear. It reconnects after an unplug; Ctrl+C exits.

Flags:
`

// ParseFlags reads the command line. It returns flag.ErrHelp after printing
// the help for -h or --help.
func ParseFlags(args []string, out io.Writer) (Config, error) {
	var c Config
	fs := flag.NewFlagSet("spooder-monitor", flag.ContinueOnError)
	fs.SetOutput(out)
	fs.StringVar(&c.Port, "port", "", "serial port `path` to use (a USB port, a pseudo-terminal or a symlink to one)")
	fs.BoolVar(&c.Frames, "frames", false, "also print a one-line decode of each frame")
	fs.IntVar(&c.TelemetryEvery, "telemetry-every", 1, "with --frames, print every `N`th telemetry frame (and any that changes mode, gait, output, faults or source)")
	fs.BoolVar(&c.Keys, "keys", false, "forward typed printable ASCII to the robot (for a --serial build)")
	fs.Usage = func() {
		fmt.Fprint(out, usageText)
		fs.PrintDefaults()
	}
	if err := fs.Parse(args); err != nil {
		return c, err
	}
	if fs.NArg() > 0 {
		fs.Usage()
		return c, fmt.Errorf("unexpected argument %q", fs.Arg(0))
	}
	if c.TelemetryEvery < 1 {
		return c, errors.New("--telemetry-every must be at least 1, not " + strconv.Itoa(c.TelemetryEvery))
	}
	return c, nil
}

// Env is what the monitor runs against.
type Env struct {
	System  link.System // the computer's ports (SerialSystem)
	Exists  func(path string) bool
	Out     io.Writer
	Keys    io.Reader // standard input, read only with --keys
	Now     func() time.Time
	Options link.Options // timing, for tests
}

// Run monitors until ctx is cancelled, then closes the port and prints the
// totals.
func Run(ctx context.Context, c Config, env Env) error {
	if env.Exists == nil {
		env.Exists = pathExists
	}
	opts := env.Options
	if opts.RetryInterval == 0 {
		opts.RetryInterval = link.RetryInterval
	}
	p := NewPrinter(env.Out, env.Now, c.Frames, c.TelemetryEvery)
	g := &gate{}
	sys := &system{base: env.System, path: c.Port, exists: env.Exists, gate: g}
	l := link.New(sys, p, nil, opts)
	defer func() {
		l.Close()
		p.PrintTotals()
	}()

	if c.Keys && env.Keys != nil {
		p.Status("forwarding typed keys (printable ASCII) to the robot")
		go forwardKeys(env.Keys, g, p)
	}

	device := link.PortInfo{Name: c.Port}
	if c.Port == "" {
		var ok bool
		if device, ok = findServo(ctx, sys, p, opts.RetryInterval); !ok {
			return nil
		}
	}
	l.Connect(device)
	<-ctx.Done()
	return nil
}

// findServo waits for a Servo 2040 to appear.
func findServo(ctx context.Context, sys link.System, p *Printer, retry time.Duration) (link.PortInfo, bool) {
	said := ""
	for {
		ports, err := sys.List()
		if i := link.Preselect(ports); i >= 0 {
			return ports[i], true
		}
		msg := "waiting for a Servo 2040 (USB " + link.Servo2040VID + ":" + link.Servo2040PID + ")"
		if err != nil {
			msg = "cannot list the serial ports: " + err.Error()
		}
		if msg != said {
			p.Status(msg)
			said = msg
		}
		select {
		case <-ctx.Done():
			return link.PortInfo{}, false
		case <-time.After(retry):
		}
	}
}

// forwardKeys sends what is typed to the open port, printable ASCII only,
// until the input ends.
func forwardKeys(in io.Reader, g *gate, p *Printer) {
	buf := make([]byte, 64)
	for {
		n, err := in.Read(buf)
		if keys := Printable(buf[:n]); len(keys) > 0 {
			if e := g.send(keys); errors.Is(e, errNotConnected) {
				p.Status(fmt.Sprintf("not connected: dropped %q", keys))
			} else if e != nil {
				p.Status(fmt.Sprintf("could not send %q: %v", keys, e))
			}
		}
		if err != nil {
			return
		}
	}
}
