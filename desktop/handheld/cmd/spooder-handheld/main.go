// Command spooder-handheld drives Spooder from a TrimUI Smart Pro through the
// bridge's WiFi drive (openspec change add-handheld-controller). It is the
// only package that imports gabagool or go-sdl2: it starts gabagool, reads
// the controls from SDL's game controller, draws the start screen and the
// drive screen and hands the Power button to the app. The rest is
// internal/app, which the tests and the e2e drive without SDL.
//
// On the handheld the pak's launch.sh runs it with --device; on desktop Linux
// it takes --addr, such as the virtual robot's --wifi-bridge port.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/BrandonKowalski/gabagool/v2/pkg/gabagool"
	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
)

// SDL wants its video and event calls on the thread that initialised it.
func init() { runtime.LockOSThread() }

const (
	// pollPeriod is how often the controls are read: well inside the
	// sender's 20 ms tick, so a tick never sends controls older than one
	// poll.
	pollPeriod = 5 * time.Millisecond
	// drawPeriod is the drive screen's redraw (20 Hz, design D4).
	drawPeriod = 50 * time.Millisecond
	// signalGrace is how long a SIGINT or SIGTERM may take to end the run
	// through its context before the process restores WiFi power saving
	// and exits by itself.
	signalGrace = time.Second
)

func main() { os.Exit(run()) }

func run() int {
	cfg, err := app.ParseFlags(os.Args[1:], os.Stderr)
	if errors.Is(err, flag.ErrHelp) {
		return 0
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-handheld:", err)
		return 2
	}

	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM, syscall.SIGHUP)
	defer stop()

	var current atomic.Pointer[app.App]
	var session sessionHolder
	go func() {
		<-ctx.Done()
		time.Sleep(signalGrace)
		session.close()
		os.Exit(1)
	}()

	opts := gabagool.Options{WindowTitle: "Spooder", IsNextUI: cfg.Device, LogPath: logPath()}
	if cfg.Device {
		opts.PowerButtonHook = func(long bool) { powerButton(&current, long) }
	}
	gabagool.Init(opts)
	defer gabagool.Close()

	// One screen for the whole run (design D1 of handheld-start-screen):
	// the start screen, then the drive screen.
	win := gabagool.GetWindow()
	scr, err := newScreen(win, win.Renderer)
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-handheld:", err)
		return 1
	}
	defer scr.close()

	s, ok := join(ctx, cfg, scr)
	if !ok {
		return 1
	}
	session.set(s)
	defer session.close()
	current.Store(s.App)

	runErr := make(chan error, 1)
	go func() { runErr <- s.Run(ctx) }()
	drive(s.App, runErr, scr)
	// drive returns once the run has ended; Run's error was taken there.
	if ctx.Err() != nil {
		return 1
	}
	return 0
}

// logPath is gabagool's log: in NextUI's log directory on the device,
// else in the temporary directory (gabagool's default is logs/ under the
// current directory).
func logPath() string {
	if dir := os.Getenv("LOGS_PATH"); dir != "" {
		return filepath.Join(dir, "Spooder.gabagool.txt")
	}
	return filepath.Join(os.TempDir(), "spooder-handheld-gabagool.log")
}

// sessionHolder lets the signal backstop close the session the main
// goroutine opened.
type sessionHolder struct {
	mu   sync.Mutex
	s    *app.Session
	once sync.Once
}

func (h *sessionHolder) set(s *app.Session) {
	h.mu.Lock()
	h.s = s
	h.mu.Unlock()
}

// close closes the session once: the link, and the WiFi power-saving
// setting restored.
func (h *sessionHolder) close() {
	h.once.Do(func() {
		h.mu.Lock()
		s := h.s
		h.mu.Unlock()
		if s != nil {
			if err := s.Close(); err != nil {
				fmt.Fprintln(os.Stderr, "spooder-handheld:", err)
			}
		}
	})
}

// powerButton is gabagool's power-button hook (the patched gabagool,
// design D5). While the app drives, the press starts a leave, which runs
// the suspend script or the power-off command once Sleep is confirmed.
// Before driving starts (the join message) there is nothing to protect, so
// the press acts at once, as unpatched gabagool would.
func powerButton(current *atomic.Pointer[app.App], long bool) {
	act, command := app.ActionSuspend, app.SuspendScript
	if long {
		act, command = app.ActionPowerOff, app.PowerOffCommand
	}
	if a := current.Load(); a != nil {
		a.Press(act, time.Now())
		return
	}
	if err := exec.Command(command).Run(); err != nil {
		fmt.Fprintf(os.Stderr, "spooder-handheld: %s: %v\n", command, err)
	}
}

// join starts the run on the start screen (handheld-start-screen): the
// check under way, then the join message (or any other start-up error) with
// retry and quit, until the start succeeds or the user quits.
func join(ctx context.Context, cfg app.Config, scr *screen) (*app.Session, bool) {
	pad := newPad()
	defer pad.close()
	for {
		scr.start(checkingPage(cfg.Device, cfg.Addr), 0)
		s, err := app.Start(ctx, cfg, app.OS())
		if err == nil {
			return s, true
		}
		if ctx.Err() != nil {
			return nil, false
		}
		if !failed(ctx, scr, pad, failedPage(err)) {
			return nil, false
		}
	}
}

// failed shows a failed start until the user retries (true) or quits, or
// the run's context ends (false).
func failed(ctx context.Context, scr *screen, pad *pad, page startPage) bool {
	var lastDraw time.Time
	for ctx.Err() == nil {
		for ev := sdl.PollEvent(); ev != nil; ev = sdl.PollEvent() {
			pad.event(ev) // reopens the controller when one is added or removed
			switch startEvent(ev) {
			case startRetry:
				return true
			case startQuit:
				return false
			}
		}
		if now := time.Now(); now.Sub(lastDraw) >= drawPeriod {
			lastDraw = now
			scr.start(page, pad.controls().Buttons)
		}
		sdl.Delay(uint32(pollPeriod / time.Millisecond))
	}
	return false
}

// drive is the main thread's loop while the app drives: it polls SDL for
// the controls and the Menu button, and redraws the drive screen, until the
// run ends.
func drive(a *app.App, runErr <-chan error, scr *screen) {
	pad := newPad()
	defer pad.close()

	snd := startSound(a)
	defer snd.close()

	timing := newDrawTiming(os.Getenv, os.Stderr)
	var lastDraw time.Time
	for {
		select {
		case err := <-runErr:
			if err != nil && !errors.Is(err, context.Canceled) {
				fmt.Fprintln(os.Stderr, "spooder-handheld:", err)
			}
			return
		default:
		}
		for ev := sdl.PollEvent(); ev != nil; ev = sdl.PollEvent() {
			if pad.event(ev) {
				a.Press(app.ActionExit, time.Now())
			}
		}
		a.SetControls(pad.controls())
		timing.controlsSet(time.Now())
		if now := time.Now(); now.Sub(lastDraw) >= drawPeriod {
			lastDraw = now
			sc := a.Screen(now)
			snd.update(sc, now)
			scr.draw(sc)
			timing.drew(time.Since(now))
		}
		sdl.Delay(uint32(pollPeriod / time.Millisecond))
	}
}
