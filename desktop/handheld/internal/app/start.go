package app

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/udplink"
	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
)

// Config is the command line.
type Config struct {
	// Device: running on the TrimUI under NextUI, so the WiFi steps and the
	// address check run and the bridge's fixed address is used.
	Device bool
	// Addr is the bridge's address without Device (desktop Linux, the
	// virtual bridge).
	Addr string
}

const usageText = `Usage: spooder-handheld [flags]

Drives Spooder through the bridge's WiFi drive. On the TrimUI the pak's
launch.sh runs it with --device; on desktop Linux give --addr instead, such as
the virtual robot's --wifi-bridge port.

Flags:
`

// ParseFlags reads the command line. It returns flag.ErrHelp after printing
// the help for -h or --help.
func ParseFlags(args []string, out io.Writer) (Config, error) {
	var c Config
	fs := flag.NewFlagSet("spooder-handheld", flag.ContinueOnError)
	fs.SetOutput(out)
	fs.BoolVar(&c.Device, "device", false, "run on the handheld: check wlan0, manage WiFi power saving, drive "+udplink.DefaultAddr)
	fs.StringVar(&c.Addr, "addr", udplink.DefaultAddr, "the bridge's `address:port` (not with --device)")
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
	addrSet := false
	fs.Visit(func(f *flag.Flag) { addrSet = addrSet || f.Name == "addr" })
	if c.Device && addrSet {
		return c, errors.New("--addr cannot be used with --device: on the handheld the bridge is " + udplink.DefaultAddr)
	}
	if c.Device {
		c.Addr = udplink.DefaultAddr
	}
	return c, nil
}

// System is what Start needs of the system, injectable for the tests.
type System struct {
	Runner   wlan.Runner
	Addrs    wlan.Addrs
	ReadFile func(string) ([]byte, error)
}

// OS is the running system.
func OS() System {
	return System{Runner: wlan.ExecRunner{}, Addrs: wlan.InterfaceAddrs, ReadFile: os.ReadFile}
}

// Session is one started run: the link and the app driving through it.
type Session struct {
	App  *App
	Link *udplink.Link
	// Notice is a start-up problem worth showing that did not stop the
	// start, or "".
	Notice string
	power  *wlan.PowerSave
}

// Start prepares a run. With Device it checks wlan0's network first, by
// its name and then its address (a *wlan.NotJoinedError when it is not on
// a bridge's WiFi drive network: show its message and call Start again to
// retry), keeping the network for the run's check while it drives, then reads WiFi power saving and
// turns it off; without Device it does neither. Then it makes the link's
// socket. Nothing is sent until Run.
func Start(ctx context.Context, cfg Config, sys System) (*Session, error) {
	s := &Session{}
	var power PowerSave
	var readFile func(string) ([]byte, error)
	var network string
	if cfg.Device {
		n, err := wlan.CheckNetwork(ctx, sys.Runner, sys.Addrs)
		if err != nil {
			return nil, err
		}
		network = n
		// A setting that cannot be read is left alone: driving with power
		// saving on is slower to react, not unsafe.
		p, err := wlan.ReadPowerSave(ctx, sys.Runner)
		if err != nil {
			s.Notice = err.Error()
		} else {
			s.power, power = p, p
			if err := p.Off(ctx); err != nil {
				s.Notice = err.Error()
			}
		}
		readFile = sys.ReadFile
	}
	link, err := udplink.Dial(cfg.Addr)
	if err != nil {
		s.Close()
		return nil, err
	}
	s.Link = link
	s.App = New(link, sys.Runner, power, readFile)
	s.App.network = network
	s.App.SampleWLAN(ctx)
	if s.Notice != "" {
		s.App.setNotice("%s", s.Notice)
	}
	return s, nil
}

// Run sends until ctx ends or a leave ends the run, sampling wlan0's
// network and signal level every SignalPeriod meanwhile.
func (s *Session) Run(ctx context.Context) error {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	sampled := make(chan struct{})
	go func() {
		defer close(sampled)
		t := time.NewTicker(SignalPeriod)
		defer t.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-t.C:
				s.App.SampleWLAN(ctx)
			}
		}
	}()
	err := s.Link.Run(ctx, s.App.Step)
	cancel()
	<-sampled
	return err
}

// Close closes the link and restores the WiFi power-saving setting read at
// the start. It is safe to call more than once.
func (s *Session) Close() error {
	var errs []error
	if s.Link != nil {
		errs = append(errs, s.Link.Close())
		s.Link = nil
	}
	if s.power != nil {
		errs = append(errs, s.power.Restore(context.Background()))
	}
	return errors.Join(errs...)
}
