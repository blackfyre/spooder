// Package wifiprobe is spooder-wifi-probe, a stand-in for a WiFi handheld
// in the bridge's WiFi drive (openspec change add-wifi-drive, D7): it sends
// command frames at 50 Hz to the bridge's UDP port, one frame per datagram,
// and summarises the robot frames the bridge relays back, once a second.
package wifiprobe

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"net"
	"strings"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

const (
	// DefaultAddr is the bridge's access point address and WiFi drive port
	// (handheld_controller::PORT).
	DefaultAddr = "192.168.4.1:52040"
	// Period is the command rate of a handheld, 50 Hz.
	Period = 20 * time.Millisecond
	// StartWait is the quiet time before the first frame: a handheld waits
	// out the bridge's liveness timeout (150 ms) after it starts, so a
	// restart never continues the previous run's session (design D4).
	StartWait = 150 * time.Millisecond
	// The command fields a handheld sends at rest: no translation or
	// rotation, a gamepad's default height and speed.
	commandHeight = 0
	commandSpeed  = 44
)

// Config is the command line. Every "after" is measured from the probe's
// start; a counter changes only in a frame after the first one, which the
// bridge takes as the session's baseline.
type Config struct {
	Addr            string
	Mode            string        // "" requests no mode: the frames ask for Sleep
	ModeAfter       time.Duration // when the one mode request is made
	EStop           bool          // send the emergency-stop flag from the start
	EStopClearAfter time.Duration // clear it then; 0 keeps it set
	ClearAfter      time.Duration // one fault clear then; 0 sends none
	PauseAfter      time.Duration // stop sending then, for PauseFor
	PauseFor        time.Duration // 0 never pauses
	Flood           int           // extra datagrams per second, copies of the latest frame
	Duration        time.Duration // run this long; 0 runs until interrupted

	summaryEvery time.Duration // 1 s; shorter in tests
}

var modes = map[string]session.RequestedMode{
	"stand":     session.RequestStand,
	"walk":      session.RequestWalk,
	"sleep":     session.RequestSleep,
	"calibrate": session.RequestCalibrate,
}

const usageText = `Usage: spooder-wifi-probe [flags]

Drives the robot through the bridge's WiFi drive like a handheld would, for
the checks on the boards: join the laptop to the bridge's spooder-drive-XXXX
network first. It sends one command frame per datagram at 50 Hz, reporting a
connected controller, and prints once a second the frames sent, the robot
frames received per type, the datagrams that are not exactly one robot frame
(errors, with the command frames among them), and the latest telemetry's body
mode and faults.

The frames request Sleep until --mode is requested, once, at --mode-after.
Every time is measured from the probe's start; the first frame goes out after
150 ms. Ctrl+C exits.

Flags:
`

// ParseFlags reads the command line. It returns flag.ErrHelp after printing
// the help for -h or --help.
func ParseFlags(args []string, out io.Writer) (Config, error) {
	c := Config{summaryEvery: time.Second}
	fs := flag.NewFlagSet("spooder-wifi-probe", flag.ContinueOnError)
	fs.SetOutput(out)
	fs.StringVar(&c.Addr, "addr", DefaultAddr, "the bridge's `address:port`")
	fs.StringVar(&c.Mode, "mode", "", "the `mode` to request once: stand, walk, sleep or calibrate")
	fs.DurationVar(&c.ModeAfter, "mode-after", 300*time.Millisecond, "when to make the mode request")
	fs.BoolVar(&c.EStop, "estop", false, "send the emergency-stop flag from the start")
	fs.DurationVar(&c.EStopClearAfter, "estop-clear-after", 0, "when to clear the emergency-stop flag (0 keeps it)")
	fs.DurationVar(&c.ClearAfter, "clear-after", 0, "when to send one fault clear (0 sends none)")
	fs.DurationVar(&c.PauseAfter, "pause-after", 0, "when to stop sending, for --pause-for")
	fs.DurationVar(&c.PauseFor, "pause-for", 0, "how long to stop sending, to let the bridge's liveness lapse (0 never pauses)")
	fs.IntVar(&c.Flood, "flood", 0, "extra datagrams `per second`, copies of the latest frame, for the rate-limit check")
	fs.DurationVar(&c.Duration, "duration", 0, "how long to run (0 runs until interrupted)")
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
	if _, ok := modes[c.Mode]; c.Mode != "" && !ok {
		return c, fmt.Errorf("--mode must be stand, walk, sleep or calibrate, not %q", c.Mode)
	}
	for name, d := range map[string]time.Duration{
		"--mode-after": c.ModeAfter, "--estop-clear-after": c.EStopClearAfter, "--clear-after": c.ClearAfter,
		"--pause-after": c.PauseAfter, "--pause-for": c.PauseFor, "--duration": c.Duration,
	} {
		if d < 0 {
			return c, fmt.Errorf("%s must not be negative", name)
		}
	}
	if c.Flood < 0 {
		return c, errors.New("--flood must not be negative")
	}
	return c, nil
}

// Counts are the probe's datagram counts.
type Counts struct {
	Sent       int // command frames at 50 Hz
	Flood      int // the extra copies
	FloodShort int // copies owed but not sent, for want of time in their period
	SendErrors int
	Received   [protocol.TypeDiagnostics + 1]int // robot frames, by type
	Errors     int                               // datagrams that are not exactly one robot frame
	Commands   int                               // command frames among the errors
}

func (c *Counts) add(o Counts) {
	c.Sent += o.Sent
	c.Flood += o.Flood
	c.FloodShort += o.FloodShort
	c.SendErrors += o.SendErrors
	for i := range c.Received {
		c.Received[i] += o.Received[i]
	}
	c.Errors += o.Errors
	c.Commands += o.Commands
}

// Result is what a run saw in total.
type Result struct {
	Counts
	Telemetry     protocol.Telemetry // the latest
	HaveTelemetry bool
}

// isRobotFrame tells the frame types a robot sends.
func isRobotFrame(t protocol.FrameType) bool {
	switch t {
	case protocol.TypeTelemetry, protocol.TypeCalibrationReport, protocol.TypePose, protocol.TypeDiagnostics:
		return true
	}
	return false
}

// classify decodes a datagram that must hold exactly one valid frame and
// nothing else. It returns the frame and whether it is one.
func classify(b []byte) (protocol.Frame, bool) {
	stray := 0
	d := protocol.Decoder{Discard: func(byte) { stray++ }}
	for _, x := range b {
		d.Push(x)
	}
	f, ok := d.Next()
	if !ok || stray != 0 || d.Pending() != 0 || d.Discarded() != 0 {
		return protocol.Frame{}, false
	}
	return f, true
}

// probe is one run's state.
type probe struct {
	cfg  Config
	conn *net.UDPConn
	out  io.Writer

	mu        sync.Mutex
	window    Counts // since the last summary
	total     Result
	telemetry protocol.Telemetry
	haveTel   bool

	// The sender's state, owned by the send loop.
	seq           uint8
	frames        int // command frames sent (not counting floods)
	modeCounter   uint8
	modeRequested bool
	mode          uint8
	clearCounter  uint8
	cleared       bool
	floodAcc      int64 // datagrams x nanoseconds owed to the flood
}

// command is the frame for elapsed time t since the start.
func (p *probe) command(t time.Duration) protocol.Command {
	if p.frames > 0 {
		if !p.modeRequested && p.cfg.Mode != "" && t >= p.cfg.ModeAfter {
			p.modeRequested = true
			p.modeCounter++
			p.mode = uint8(modes[p.cfg.Mode])
		}
		if !p.cleared && p.cfg.ClearAfter > 0 && t >= p.cfg.ClearAfter {
			p.cleared = true
			p.clearCounter++
		}
	}
	c := protocol.Command{
		Mode:               p.mode,
		Height:             commandHeight,
		Speed:              commandSpeed,
		Flags:              protocol.FlagControllerConnected,
		ClearCounter:       p.clearCounter,
		ModeRequestCounter: p.modeCounter,
	}
	if p.cfg.EStop && (p.cfg.EStopClearAfter == 0 || t < p.cfg.EStopClearAfter) {
		c.Flags |= protocol.FlagEmergencyStop
	}
	return c
}

func (p *probe) paused(t time.Duration) bool {
	return p.cfg.PauseFor > 0 && t >= p.cfg.PauseAfter && t < p.cfg.PauseAfter+p.cfg.PauseFor
}

// floodBudget is the share of a period the flood copies may take, so a
// flood the computer cannot send never delays the next frame, a timed
// change or the summary.
const floodBudget = Period / 2

// send sends one period's frame and its flood copies.
func (p *probe) send(t time.Duration) {
	deadline := time.Now().Add(floodBudget)
	if p.paused(t) {
		return
	}
	frame := protocol.EncodeCommand(p.command(t), p.seq)
	p.seq++
	p.frames++
	var c Counts
	if _, err := p.conn.Write(frame); err != nil {
		c.SendErrors++
	} else {
		c.Sent++
	}
	p.floodAcc += int64(p.cfg.Flood) * int64(Period)
	for ; p.floodAcc >= int64(time.Second); p.floodAcc -= int64(time.Second) {
		if time.Now().After(deadline) {
			c.FloodShort += int(p.floodAcc / int64(time.Second))
			p.floodAcc %= int64(time.Second)
			break
		}
		if _, err := p.conn.Write(frame); err != nil {
			c.SendErrors++
		} else {
			c.Flood++
		}
	}
	p.count(c)
}

func (p *probe) count(c Counts) {
	p.mu.Lock()
	p.window.add(c)
	p.mu.Unlock()
}

// receive reads datagrams until the socket closes.
func (p *probe) receive() {
	buf := make([]byte, 2048)
	for {
		n, err := p.conn.Read(buf)
		if errors.Is(err, net.ErrClosed) {
			return
		}
		if err != nil {
			continue // an ICMP error from an earlier send: the bridge is not there yet
		}
		p.datagram(buf[:n])
	}
}

func (p *probe) datagram(b []byte) {
	f, ok := classify(b)
	p.mu.Lock()
	defer p.mu.Unlock()
	switch {
	case ok && isRobotFrame(f.Type):
		p.window.Received[f.Type]++
		if f.Type == protocol.TypeTelemetry {
			p.telemetry, p.haveTel = f.Telemetry, true
		}
	case ok && f.Type == protocol.TypeCommand:
		p.window.Errors++
		p.window.Commands++
	default:
		p.window.Errors++
	}
}

// summary prints and resets the window's counts.
func (p *probe) summary(t time.Duration) {
	p.mu.Lock()
	w := p.window
	p.total.add(w)
	p.window = Counts{}
	tel, have := p.telemetry, p.haveTel
	p.mu.Unlock()
	fmt.Fprintf(p.out, "%6.1fs %s | %s\n", t.Seconds(), formatCounts(w), formatTelemetry(tel, have))
}

func formatCounts(c Counts) string {
	var b strings.Builder
	fmt.Fprintf(&b, "sent %d flood %d", c.Sent, c.Flood)
	if c.FloodShort > 0 {
		fmt.Fprintf(&b, " flood short %d", c.FloodShort)
	}
	if c.SendErrors > 0 {
		fmt.Fprintf(&b, " send errors %d", c.SendErrors)
	}
	b.WriteString(" | recv")
	for _, t := range []protocol.FrameType{protocol.TypeTelemetry, protocol.TypeCalibrationReport, protocol.TypePose, protocol.TypeDiagnostics} {
		fmt.Fprintf(&b, " %s %d", t.Name(), c.Received[t])
	}
	fmt.Fprintf(&b, " | errors %d (command %d)", c.Errors, c.Commands)
	return b.String()
}

func formatTelemetry(t protocol.Telemetry, have bool) string {
	if !have {
		return "no telemetry"
	}
	faults := "none"
	if names := session.FaultNames(t.Faults); len(names) > 0 {
		faults = strings.Join(names, ", ")
	}
	outputs := "off"
	if t.OutputsOn {
		outputs = "on"
	}
	return fmt.Sprintf("body %s, outputs %s, faults %s", session.BodyMode(t.BodyMode), outputs, faults)
}

// Run sends until ctx ends or the configured duration has passed, then
// prints the totals and returns them.
func Run(ctx context.Context, cfg Config, out io.Writer) (Result, error) {
	if cfg.summaryEvery <= 0 {
		cfg.summaryEvery = time.Second
	}
	raddr, err := net.ResolveUDPAddr("udp4", cfg.Addr)
	if err != nil {
		return Result{}, err
	}
	// A connected socket: the bridge relays to the address and port the
	// frames come from, and the kernel drops datagrams from anyone else.
	conn, err := net.DialUDP("udp4", nil, raddr)
	if err != nil {
		return Result{}, err
	}
	p := &probe{cfg: cfg, conn: conn, out: out, mode: uint8(session.RequestSleep)}
	fmt.Fprintf(out, "spooder-wifi-probe: %s to %s\n", conn.LocalAddr(), raddr)

	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		p.receive()
	}()

	start := time.Now()
	if cfg.Duration > 0 {
		var cancel context.CancelFunc
		ctx, cancel = context.WithDeadline(ctx, start.Add(cfg.Duration))
		defer cancel()
	}
	summaries := time.NewTicker(cfg.summaryEvery)
	defer summaries.Stop()
	var ticks <-chan time.Time
	first := time.NewTimer(StartWait)
	defer first.Stop()
	var ticker *time.Ticker
loop:
	for {
		select {
		case <-ctx.Done():
			break loop
		case <-first.C:
			ticker = time.NewTicker(Period)
			defer ticker.Stop()
			ticks = ticker.C
			p.send(time.Since(start))
		case <-ticks:
			p.send(time.Since(start))
		case <-summaries.C:
			p.summary(time.Since(start))
		}
	}
	// Let the last relayed frames arrive before closing.
	time.Sleep(Period)
	conn.Close()
	wg.Wait()
	p.summary(time.Since(start))

	p.mu.Lock()
	res := p.total
	res.Telemetry, res.HaveTelemetry = p.telemetry, p.haveTel
	p.mu.Unlock()
	fmt.Fprintf(out, "total: %s\n", formatCounts(res.Counts))
	return res, nil
}
