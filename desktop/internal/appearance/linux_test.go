//go:build linux

package appearance

import (
	"bufio"
	"context"
	"image/color"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/godbus/dbus/v5"
)

// fakePortal serves org.freedesktop.portal.Settings for the appearance keys.
type fakePortal struct {
	mu     sync.Mutex
	values map[string]dbus.Variant
	// legacy makes ReadOne fail, as on portals older than version 2.
	legacy bool
}

func (p *fakePortal) ReadOne(ns, key string) (dbus.Variant, *dbus.Error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.legacy {
		return dbus.Variant{}, dbus.NewError("org.freedesktop.DBus.Error.UnknownMethod", nil)
	}
	v, ok := p.values[key]
	if ns != namespace || !ok {
		return dbus.Variant{}, dbus.NewError("org.freedesktop.portal.Error.NotFound", nil)
	}
	return v, nil
}

func (p *fakePortal) Read(ns, key string) (dbus.Variant, *dbus.Error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	v, ok := p.values[key]
	if ns != namespace || !ok {
		return dbus.Variant{}, dbus.NewError("org.freedesktop.portal.Error.NotFound", nil)
	}
	return dbus.MakeVariant(v), nil
}

// privateBus starts a dbus-daemon for the test and returns its address.
func privateBus(t *testing.T) string {
	t.Helper()
	// A minimal configuration without service directories, so nothing
	// (such as the real desktop portal) is activated on this bus.
	config := filepath.Join(t.TempDir(), "bus.conf")
	if err := os.WriteFile(config, []byte(`<!DOCTYPE busconfig PUBLIC "-//freedesktop//DTD D-Bus Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <type>session</type>
  <listen>unix:dir=`+t.TempDir()+`</listen>
  <policy context="default">
    <allow send_destination="*" eavesdrop="true"/>
    <allow eavesdrop="true"/>
    <allow own="*"/>
  </policy>
</busconfig>`), 0o644); err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command("dbus-daemon", "--config-file="+config, "--nofork", "--print-address")
	out, err := cmd.StdoutPipe()
	if err != nil {
		t.Fatal(err)
	}
	if err := cmd.Start(); err != nil {
		t.Skipf("dbus-daemon unavailable: %v", err)
	}
	t.Cleanup(func() { cmd.Process.Kill(); cmd.Wait() })
	addr, err := bufio.NewReader(out).ReadString('\n')
	if err != nil {
		t.Fatal(err)
	}
	return strings.TrimSpace(addr)
}

func connect(t *testing.T, addr string) *dbus.Conn {
	t.Helper()
	conn, err := dbus.Connect(addr)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { conn.Close() })
	return conn
}

func servePortal(t *testing.T, addr string, p *fakePortal) *dbus.Conn {
	conn := connect(t, addr)
	if err := conn.Export(p, portalPath, settingsIface); err != nil {
		t.Fatal(err)
	}
	if reply, err := conn.RequestName(portalDest, dbus.NameFlagDoNotQueue); err != nil || reply != dbus.RequestNameReplyPrimaryOwner {
		t.Fatalf("request name: %v %v", reply, err)
	}
	return conn
}

func cosmicValues() map[string]dbus.Variant {
	return map[string]dbus.Variant{
		"color-scheme": dbus.MakeVariant(uint32(1)),
		// The portal sends the accent as a (ddd) struct.
		"accent-color": dbus.MakeVariant(struct{ R, G, B float64 }{0.478431, 0.635294, 0.968628}),
		"contrast":     dbus.MakeVariant(uint32(0)),
	}
}

func TestReadFromPortal(t *testing.T) {
	addr := privateBus(t)
	servePortal(t, addr, &fakePortal{values: cosmicValues()})
	got := readFrom(connect(t, addr))
	want := Settings{Dark: true, Accent: color.NRGBA{R: 122, G: 162, B: 247, A: 255}, HasAccent: true}
	if got != want {
		t.Fatalf("got %+v, want %+v", got, want)
	}
}

func TestReadFromLegacyPortal(t *testing.T) {
	addr := privateBus(t)
	servePortal(t, addr, &fakePortal{values: cosmicValues(), legacy: true})
	if got := readFrom(connect(t, addr)); !got.Dark || !got.HasAccent {
		t.Fatalf("got %+v", got)
	}
}

func TestReadWithoutPortalIsFastDefault(t *testing.T) {
	addr := privateBus(t)
	start := time.Now()
	got := readFrom(connect(t, addr))
	if got != (Settings{}) || time.Since(start) > 250*time.Millisecond {
		t.Fatalf("got %+v after %v", got, time.Since(start))
	}
}

func TestWatchReportsChanges(t *testing.T) {
	addr := privateBus(t)
	portal := &fakePortal{values: cosmicValues()}
	server := servePortal(t, addr, portal)
	client := connect(t, addr)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	changes := make(chan Settings, 4)
	watchOn(ctx, client, func(s Settings) { changes <- s })
	time.Sleep(100 * time.Millisecond) // let the match rule register

	portal.mu.Lock()
	portal.values["color-scheme"] = dbus.MakeVariant(uint32(2))
	portal.values["contrast"] = dbus.MakeVariant(uint32(1))
	portal.mu.Unlock()
	if err := server.Emit(portalPath, settingsIface+".SettingChanged", namespace, "color-scheme", dbus.MakeVariant(uint32(2))); err != nil {
		t.Fatal(err)
	}
	select {
	case s := <-changes:
		if s.Dark || !s.HighContrast {
			t.Fatalf("got %+v", s)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("no change reported")
	}
}

func TestNoSessionBusMeansDefaultsWithoutDbusLaunch(t *testing.T) {
	env := map[string]string{"XDG_RUNTIME_DIR": t.TempDir()}
	if _, ok := sessionAddress(func(k string) string { return env[k] }); ok {
		t.Fatal("found a bus where there is none")
	}
	// With PATH pointing nowhere, a dbus-launch fallback would fail loudly;
	// Read must return the defaults quickly instead.
	t.Setenv("DBUS_SESSION_BUS_ADDRESS", "")
	t.Setenv("XDG_RUNTIME_DIR", t.TempDir())
	t.Setenv("PATH", t.TempDir())
	start := time.Now()
	if got := Read(); got != (Settings{}) || time.Since(start) > 250*time.Millisecond {
		t.Fatalf("got %+v after %v", got, time.Since(start))
	}
}

func TestSessionAddressFromRuntimeDir(t *testing.T) {
	addr := privateBus(t)
	path := strings.TrimPrefix(strings.Split(addr, ",")[0], "unix:path=")
	if !strings.HasPrefix(addr, "unix:path=") {
		t.Skipf("bus address %q is not a path", addr)
	}
	dir := t.TempDir()
	if err := os.Symlink(path, filepath.Join(dir, "bus")); err != nil {
		t.Fatal(err)
	}
	got, ok := sessionAddress(func(k string) string {
		if k == "XDG_RUNTIME_DIR" {
			return dir
		}
		return ""
	})
	if !ok || got != "unix:path="+filepath.Join(dir, "bus") {
		t.Fatalf("got %q %v", got, ok)
	}
}
