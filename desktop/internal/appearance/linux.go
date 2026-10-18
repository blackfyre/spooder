//go:build linux

package appearance

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/godbus/dbus/v5"
)

const (
	portalDest    = "org.freedesktop.portal.Desktop"
	portalPath    = dbus.ObjectPath("/org/freedesktop/portal/desktop")
	settingsIface = "org.freedesktop.portal.Settings"
	namespace     = "org.freedesktop.appearance"
	// readTimeout bounds each portal query; keys are read concurrently.
	readTimeout = 200 * time.Millisecond
)

// Read returns the host appearance from the XDG desktop portal, or the
// defaults (light, no accent) when the portal is unavailable.
func Read() Settings {
	conn, err := sessionBus(os.Getenv)
	if err != nil {
		return Settings{}
	}
	defer conn.Close()
	return readFrom(conn)
}

// Watch calls changed with the new appearance whenever the portal reports
// a change in the appearance namespace, until ctx ends.
func Watch(ctx context.Context, changed func(Settings)) {
	conn, err := sessionBus(os.Getenv)
	if err != nil {
		return
	}
	go func() {
		<-ctx.Done()
		conn.Close()
	}()
	watchOn(ctx, conn, changed)
}

// sessionAddress returns the address of an already running session bus.
// Unlike dbus.SessionBus it never falls back to dbus-launch, which would
// start a new bus daemon on systems without one.
func sessionAddress(getenv func(string) string) (string, bool) {
	if addr := getenv("DBUS_SESSION_BUS_ADDRESS"); addr != "" {
		return addr, true
	}
	if dir := getenv("XDG_RUNTIME_DIR"); dir != "" {
		path := filepath.Join(dir, "bus")
		if fi, err := os.Stat(path); err == nil && fi.Mode()&os.ModeSocket != 0 {
			return "unix:path=" + path, true
		}
	}
	return "", false
}

func sessionBus(getenv func(string) string) (*dbus.Conn, error) {
	addr, ok := sessionAddress(getenv)
	if !ok {
		return nil, errors.New("no session bus")
	}
	return dbus.Connect(addr)
}

func readFrom(conn *dbus.Conn) Settings {
	obj := conn.Object(portalDest, portalPath)
	keys := []string{"color-scheme", "accent-color", "contrast"}
	values := make([]any, len(keys))
	var wg sync.WaitGroup
	for i, key := range keys {
		wg.Add(1)
		go func() {
			defer wg.Done()
			values[i] = readKey(obj, key)
		}()
	}
	wg.Wait()

	var s Settings
	if dark, ok := parseScheme(values[0]); ok {
		s.Dark = dark
	}
	if accent, ok := parseAccent(values[1]); ok {
		s.Accent, s.HasAccent = accent, true
	}
	if high, ok := parseContrast(values[2]); ok {
		s.HighContrast = high
	}
	return s
}

// readKey reads one appearance key, trying ReadOne (portal version 2) and
// then the deprecated Read, which wraps the value in a second variant.
func readKey(obj dbus.BusObject, key string) any {
	ctx, cancel := context.WithTimeout(context.Background(), readTimeout)
	defer cancel()
	var v dbus.Variant
	if err := obj.CallWithContext(ctx, settingsIface+".ReadOne", 0, namespace, key).Store(&v); err == nil {
		return v.Value()
	}
	if err := obj.CallWithContext(ctx, settingsIface+".Read", 0, namespace, key).Store(&v); err != nil {
		return nil
	}
	if inner, ok := v.Value().(dbus.Variant); ok {
		return inner.Value()
	}
	return v.Value()
}

func watchOn(ctx context.Context, conn *dbus.Conn, changed func(Settings)) {
	opts := []dbus.MatchOption{
		dbus.WithMatchInterface(settingsIface),
		dbus.WithMatchMember("SettingChanged"),
		dbus.WithMatchArg(0, namespace),
	}
	if err := conn.AddMatchSignal(opts...); err != nil {
		return
	}
	signals := make(chan *dbus.Signal, 8)
	conn.Signal(signals)
	go func() {
		defer func() {
			conn.RemoveSignal(signals)
			_ = conn.RemoveMatchSignal(opts...)
		}()
		for {
			select {
			case <-ctx.Done():
				return
			case sig := <-signals:
				if sig == nil || sig.Name != settingsIface+".SettingChanged" ||
					len(sig.Body) < 1 || sig.Body[0] != namespace {
					continue
				}
				changed(readFrom(conn))
			}
		}
	}()
}
