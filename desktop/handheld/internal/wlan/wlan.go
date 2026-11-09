// Package wlan is the handheld's WiFi housekeeping on the device
// (openspec changes add-handheld-controller, designs D6 and D7, and
// handheld-network-check): the check that wlan0 is on the bridge's WiFi
// drive network, by its name and then its address with one lease renewal,
// the join message, the power-saving setting read, turned off and restored,
// and the network and signal level while running. Every command runs through a Runner, so the host tests
// cover each branch with a fake one.
package wlan

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"net"
	"os/exec"
	"regexp"
	"strconv"
	"strings"
)

// Interface is the device's WiFi interface.
const Interface = "wlan0"

// BridgeNet is the bridge access point's network.
var BridgeNet = net.IPNet{IP: net.IPv4(192, 168, 4, 0).To4(), Mask: net.CIDRMask(24, 32)}

// driveNetwork is the name the bridge's WiFi drive gives its access point:
// "spooder-drive-" and the last four hex digits of its MAC, lower case
// (core/setup/access_point's driveSsid). The setup network, "spooder-" and
// the same digits, does not match.
var driveNetwork = regexp.MustCompile(`^spooder-drive-[0-9a-f]{4}$`)

// IsDriveNetwork tells whether ssid names a bridge's WiFi drive network.
func IsDriveNetwork(ssid string) bool { return driveNetwork.MatchString(ssid) }

// WirelessPath is the kernel's wireless statistics.
const WirelessPath = "/proc/net/wireless"

// Runner runs a command to completion and returns its standard output.
type Runner interface {
	Run(ctx context.Context, name string, args ...string) ([]byte, error)
}

// ExecRunner runs commands as processes; ctx's end kills one still
// running.
type ExecRunner struct{}

// Run runs name with args and waits for it.
func (ExecRunner) Run(ctx context.Context, name string, args ...string) ([]byte, error) {
	return exec.CommandContext(ctx, name, args...).Output()
}

// Addrs lists an interface's IP addresses.
type Addrs func(iface string) ([]net.IP, error)

// InterfaceAddrs is the Addrs of the running system.
func InterfaceAddrs(iface string) ([]net.IP, error) {
	ifi, err := net.InterfaceByName(iface)
	if err != nil {
		return nil, err
	}
	as, err := ifi.Addrs()
	if err != nil {
		return nil, err
	}
	var ips []net.IP
	for _, a := range as {
		if n, ok := a.(*net.IPNet); ok {
			ips = append(ips, n.IP)
		}
	}
	return ips, nil
}

// onBridge tells whether one of the interface's IPv4 addresses is in
// BridgeNet. An interface that cannot be read holds none.
func onBridge(addrs Addrs) bool {
	ips, err := addrs(Interface)
	if err != nil {
		return false
	}
	for _, ip := range ips {
		if v4 := ip.To4(); v4 != nil && BridgeNet.Contains(v4) {
			return true
		}
	}
	return false
}

// NotJoinedError is the address check's failure; its message is the join
// message shown to the user.
type NotJoinedError struct {
	// Network is the SSID wlan0 is on, or "" when it is on none.
	Network string
}

func (e *NotJoinedError) Error() string {
	var b strings.Builder
	b.WriteString("Join the spooder-drive-XXXX network in NextUI's WiFi settings, " +
		"with the password shown on the bridge's link page.")
	if e.Network != "" {
		fmt.Fprintf(&b, " %s is on %q now.", Interface, e.Network)
	} else {
		fmt.Fprintf(&b, " %s is on no network now.", Interface)
	}
	return b.String()
}

// RenewArgs is the one lease renewal: udhcpc in the foreground, quitting
// once it has a lease or after five tries.
var RenewArgs = []string{"-i", Interface, "-n", "-q", "-t", "5"}

// CheckNetwork passes when wlan0 is on a bridge's WiFi drive network (by
// its name, IsDriveNetwork) and holds an IPv4 address on the bridge's
// network, and returns that network's name. On another network, or none,
// it returns a *NotJoinedError naming it without renewing the lease: a
// renewal cannot change the network. On a drive network without the
// address it renews the lease once with udhcpc and checks again; still
// none, it returns a *NotJoinedError.
func CheckNetwork(ctx context.Context, run Runner, addrs Addrs) (string, error) {
	ssid := CurrentNetwork(ctx, run)
	if !IsDriveNetwork(ssid) {
		return "", &NotJoinedError{Network: ssid}
	}
	if onBridge(addrs) {
		return ssid, nil
	}
	// A failed renewal is not an error of its own: the second check says
	// whether the address is there.
	_, _ = run.Run(ctx, "udhcpc", RenewArgs...)
	if onBridge(addrs) {
		return ssid, nil
	}
	return "", &NotJoinedError{Network: ssid}
}

// CurrentNetwork is the SSID wlan0 is on (`iw dev wlan0 link`), or "".
func CurrentNetwork(ctx context.Context, run Runner) string {
	out, err := run.Run(ctx, "iw", "dev", Interface, "link")
	if err != nil {
		return ""
	}
	return ParseLinkSSID(out)
}

// ParseLinkSSID reads the SSID from the output of `iw dev wlan0 link`, ""
// when it names none (Not connected.).
func ParseLinkSSID(b []byte) string {
	for _, line := range strings.Split(string(b), "\n") {
		if ssid, ok := strings.CutPrefix(strings.TrimSpace(line), "SSID:"); ok {
			return strings.TrimSpace(ssid)
		}
	}
	return ""
}

// PowerSave is wlan0's power-saving setting as read at the start, for
// turning it off and restoring it.
type PowerSave struct {
	run   Runner
	saved string // "on" or "off"
}

// ReadPowerSave reads the setting (`iw dev wlan0 get power_save`, which
// prints "Power save: on" or "... off").
func ReadPowerSave(ctx context.Context, run Runner) (*PowerSave, error) {
	out, err := run.Run(ctx, "iw", "dev", Interface, "get", "power_save")
	if err != nil {
		return nil, fmt.Errorf("reading %s power save: %w", Interface, err)
	}
	_, v, ok := strings.Cut(string(out), ":")
	v = strings.TrimSpace(v)
	if !ok || (v != "on" && v != "off") {
		return nil, fmt.Errorf("reading %s power save: unexpected %q", Interface, bytes.TrimSpace(out))
	}
	return &PowerSave{run: run, saved: v}, nil
}

// Saved is the setting read at the start.
func (p *PowerSave) Saved() string { return p.saved }

func (p *PowerSave) set(ctx context.Context, v string) error {
	if _, err := p.run.Run(ctx, "iw", "dev", Interface, "set", "power_save", v); err != nil {
		return fmt.Errorf("setting %s power save %s: %w", Interface, v, err)
	}
	return nil
}

// Off turns power saving off.
func (p *PowerSave) Off(ctx context.Context) error { return p.set(ctx, "off") }

// Restore sets the setting read at the start again. Restoring twice is
// harmless.
func (p *PowerSave) Restore(ctx context.Context) error { return p.set(ctx, p.saved) }

// ErrNoSignal is returned when the signal level is unknown.
var ErrNoSignal = errors.New("signal level unknown")

// ParseSignal reads wlan0's signal level in dBm from the contents of
// /proc/net/wireless: the level column of its line, as in
//
//	wlan0: 0000   70.  -40.  -256        0      0      0      0      0        0
func ParseSignal(b []byte) (int, error) {
	for _, line := range strings.Split(string(b), "\n") {
		rest, ok := strings.CutPrefix(strings.TrimSpace(line), Interface+":")
		if !ok {
			continue
		}
		fields := strings.Fields(rest)
		if len(fields) < 3 {
			return 0, ErrNoSignal
		}
		level, err := strconv.ParseFloat(strings.TrimSuffix(fields[2], "."), 64)
		if err != nil {
			return 0, ErrNoSignal
		}
		return int(level), nil
	}
	return 0, ErrNoSignal
}

// ParseLinkSignal reads the signal level in dBm from the output of
// `iw dev wlan0 link`, as in
//
//	Connected to 02:00:00:00:00:00 (on wlan0)
//		SSID: spooder-drive-1A2B
//		signal: -40 dBm
//
// Not connected (no signal line) reads as ErrNoSignal.
func ParseLinkSignal(b []byte) (int, error) {
	for _, line := range strings.Split(string(b), "\n") {
		rest, ok := strings.CutPrefix(strings.TrimSpace(line), "signal:")
		if !ok {
			continue
		}
		fields := strings.Fields(rest)
		if len(fields) == 0 {
			return 0, ErrNoSignal
		}
		level, err := strconv.Atoi(fields[0])
		if err != nil {
			return 0, ErrNoSignal
		}
		return level, nil
	}
	return 0, ErrNoSignal
}

// Sample is one reading of wlan0 while running: the network it is on and
// its signal level.
type Sample struct {
	// NetworkKnown: the network could be read; Network is its SSID, "" for
	// none.
	NetworkKnown bool
	Network      string
	// SignalKnown: SignalDBm holds the signal level.
	SignalKnown bool
	SignalDBm   int
}

// Read reads wlan0's network and signal level with one `iw dev wlan0 link`
// through run (openspec change handheld-network-check, D3); the level comes
// from the kernel's wireless statistics through readFile first, where they
// have a wlan0 line (cfg80211 drivers without wireless extensions, as on
// the TrimUI Smart Pro, have no file at all). An iw that fails leaves the
// network unknown.
func Read(ctx context.Context, run Runner, readFile func(string) ([]byte, error)) Sample {
	var s Sample
	if b, err := readFile(WirelessPath); err == nil {
		if dbm, err := ParseSignal(b); err == nil {
			s.SignalKnown, s.SignalDBm = true, dbm
		}
	}
	out, err := run.Run(ctx, "iw", "dev", Interface, "link")
	if err != nil {
		return s
	}
	s.NetworkKnown, s.Network = true, ParseLinkSSID(out)
	if !s.SignalKnown {
		if dbm, err := ParseLinkSignal(out); err == nil {
			s.SignalKnown, s.SignalDBm = true, dbm
		}
	}
	return s
}
