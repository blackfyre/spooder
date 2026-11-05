package pak

import (
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"testing"
	"time"
)

// stubIW is iw on PATH: it keeps wlan0's power-saving setting in $IW_STATE
// and logs every set to $IW_LOG.
const stubIW = `#!/bin/sh
case "$*" in
"dev wlan0 get power_save") echo "Power save: $(cat "$IW_STATE")" ;;
"dev wlan0 set power_save "*) echo "$5" > "$IW_STATE"; echo "set $5" >> "$IW_LOG" ;;
*) exit 1 ;;
esac
`

// pakDir makes a pak with launch.sh and a stub binary, and a bin directory
// with the stub iw, power saving on.
func pakDir(t *testing.T, binary string) (dir string, env []string) {
	t.Helper()
	dir = t.TempDir()
	script, err := os.ReadFile("launch.sh")
	if err != nil {
		t.Fatal(err)
	}
	bin := filepath.Join(dir, "bin")
	pak := filepath.Join(dir, "Spooder.pak")
	for _, d := range []string{bin, pak} {
		if err := os.Mkdir(d, 0o755); err != nil {
			t.Fatal(err)
		}
	}
	write := func(path, body string) {
		if err := os.WriteFile(path, []byte(body), 0o755); err != nil {
			t.Fatal(err)
		}
	}
	write(filepath.Join(pak, "launch.sh"), string(script))
	write(filepath.Join(pak, "spooder-handheld"), binary)
	write(filepath.Join(bin, "iw"), stubIW)
	write(filepath.Join(dir, "state"), "on\n")
	env = append(os.Environ(),
		"PATH="+bin+":"+os.Getenv("PATH"),
		"IW_STATE="+filepath.Join(dir, "state"),
		"IW_LOG="+filepath.Join(dir, "iw.log"),
		"MARK="+filepath.Join(dir, "running"),
		"ARGS="+filepath.Join(dir, "args"),
		"GRAND="+filepath.Join(dir, "grandchild"),
		"LOGS_PATH=",
	)
	return dir, env
}

func read(t *testing.T, path string) string {
	t.Helper()
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	return strings.TrimSpace(string(b))
}

// waitFor waits until path exists.
func waitFor(t *testing.T, path string) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for {
		if _, err := os.Stat(path); err == nil {
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("%s did not appear", path)
		}
		time.Sleep(10 * time.Millisecond)
	}
}

func TestLaunchRestoresAfterBinaryFails(t *testing.T) {
	dir, env := pakDir(t, `#!/bin/sh
echo "$@" > "$ARGS"
iw dev wlan0 set power_save off
exit 1
`)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = env
	err := cmd.Run()
	if ee, ok := err.(*exec.ExitError); !ok || ee.ExitCode() != 1 {
		t.Fatalf("launch.sh: %v, want exit status 1", err)
	}
	if got := read(t, filepath.Join(dir, "args")); got != "--device" {
		t.Errorf("binary run with %q, want --device", got)
	}
	if got := read(t, filepath.Join(dir, "state")); got != "on" {
		t.Errorf("power save %q after the binary failed, want on", got)
	}
	if got := read(t, filepath.Join(dir, "iw.log")); got != "set off\nset on" {
		t.Errorf("iw sets %q, want off then on", got)
	}
}

// The binary turns power saving off, marks itself running and waits; it
// is then killed with SIGTERM, directly or through the script.
const waitingBinary = `#!/bin/sh
iw dev wlan0 set power_save off
sleep 30 &
echo $! > "$GRAND"
echo $$ > "$MARK"
exec sleep 30
`

func TestLaunchRestoresAfterBinaryKilled(t *testing.T) {
	dir, env := pakDir(t, waitingBinary)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = env
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	waitFor(t, filepath.Join(dir, "running"))
	pid, err := strconv.Atoi(read(t, filepath.Join(dir, "running")))
	if err != nil {
		t.Fatal(err)
	}
	if err := syscall.Kill(pid, syscall.SIGTERM); err != nil {
		t.Fatal(err)
	}
	if err := cmd.Wait(); err == nil {
		t.Error("launch.sh exited 0 after the binary was killed")
	}
	if got := read(t, filepath.Join(dir, "state")); got != "on" {
		t.Errorf("power save %q after the binary was killed, want on", got)
	}
}

func TestLaunchRestoresAndStopsBinaryOnSIGTERM(t *testing.T) {
	dir, env := pakDir(t, waitingBinary)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = env
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	waitFor(t, filepath.Join(dir, "running"))
	pid, err := strconv.Atoi(read(t, filepath.Join(dir, "running")))
	if err != nil {
		t.Fatal(err)
	}
	if err := cmd.Process.Signal(syscall.SIGTERM); err != nil {
		t.Fatal(err)
	}
	done := make(chan error, 1)
	go func() { done <- cmd.Wait() }()
	select {
	case err := <-done:
		if ee, ok := err.(*exec.ExitError); !ok || ee.ExitCode() != 143 {
			t.Errorf("launch.sh: %v, want exit status 143", err)
		}
	case <-time.After(5 * time.Second):
		_ = cmd.Process.Kill()
		t.Fatal("launch.sh did not exit on SIGTERM")
	}
	if got := read(t, filepath.Join(dir, "state")); got != "on" {
		t.Errorf("power save %q after SIGTERM, want on", got)
	}
	if err := syscall.Kill(pid, 0); err == nil {
		_ = syscall.Kill(pid, syscall.SIGKILL)
		t.Error("the binary is still running after launch.sh's SIGTERM")
	}
}

func TestLaunchLeavesUnreadableSettingAlone(t *testing.T) {
	dir, env := pakDir(t, `#!/bin/sh
exit 0
`)
	if err := os.WriteFile(filepath.Join(dir, "state"), []byte("unknown\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = env
	if err := cmd.Run(); err != nil {
		t.Fatalf("launch.sh: %v", err)
	}
	if _, err := os.Stat(filepath.Join(dir, "iw.log")); err == nil {
		t.Errorf("iw set %q with an unreadable setting, want nothing", read(t, filepath.Join(dir, "iw.log")))
	}
}

// alive tells whether the process with the pid in path still runs.
func alive(t *testing.T, path string) bool {
	t.Helper()
	pid, err := strconv.Atoi(read(t, path))
	if err != nil {
		t.Fatal(err)
	}
	// A zombie no one reaps would still answer signal 0; the test's
	// processes are reaped by launch.sh or reparented to init, which reaps
	// them, so a short wait settles it.
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if syscall.Kill(pid, 0) != nil {
			return false
		}
		time.Sleep(20 * time.Millisecond)
	}
	_ = syscall.Kill(pid, syscall.SIGKILL)
	return true
}

// A binary that crashes while a process it started still runs: launch.sh
// stops that process too.
func TestLaunchStopsWhatACrashedBinaryStarted(t *testing.T) {
	dir, env := pakDir(t, `#!/bin/sh
iw dev wlan0 set power_save off
sleep 30 &
echo $! > "$GRAND"
kill -SEGV $$
`)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = env
	if err := cmd.Run(); err == nil {
		t.Error("launch.sh exited 0 after the binary crashed")
	}
	if got := read(t, filepath.Join(dir, "state")); got != "on" {
		t.Errorf("power save %q after the crash, want on", got)
	}
	if alive(t, filepath.Join(dir, "grandchild")) {
		t.Error("the crashed binary's child still runs after launch.sh exited")
	}
}

func TestLaunchStopsBinaryChildrenOnSIGTERM(t *testing.T) {
	dir, env := pakDir(t, waitingBinary)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = env
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	waitFor(t, filepath.Join(dir, "running"))
	if err := cmd.Process.Signal(syscall.SIGTERM); err != nil {
		t.Fatal(err)
	}
	_ = cmd.Wait()
	if alive(t, filepath.Join(dir, "grandchild")) {
		t.Error("the binary's child still runs after launch.sh's SIGTERM")
	}
}

// A binary that ignores SIGTERM is killed after the grace period, and the
// setting is restored.
func TestLaunchKillsBinaryIgnoringSIGTERM(t *testing.T) {
	dir, env := pakDir(t, `#!/bin/sh
trap '' TERM
iw dev wlan0 set power_save off
echo $$ > "$MARK"
while :; do sleep 1; done
`)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = append(env, "STOP_GRACE=0.3")
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	waitFor(t, filepath.Join(dir, "running"))
	if err := cmd.Process.Signal(syscall.SIGTERM); err != nil {
		t.Fatal(err)
	}
	done := make(chan error, 1)
	go func() { done <- cmd.Wait() }()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		_ = cmd.Process.Kill()
		t.Fatal("launch.sh did not exit with a binary ignoring SIGTERM")
	}
	if got := read(t, filepath.Join(dir, "state")); got != "on" {
		t.Errorf("power save %q, want on", got)
	}
	if alive(t, filepath.Join(dir, "running")) {
		t.Error("the binary ignoring SIGTERM still runs")
	}
}

// A binary that exits on SIGTERM ends the script at once, the watchdog
// included: a caller reading the script's output sees its end well before
// the grace period.
func TestLaunchEndsPromptlyOnSIGTERM(t *testing.T) {
	dir, env := pakDir(t, waitingBinary)
	cmd := exec.Command("sh", filepath.Join(dir, "Spooder.pak", "launch.sh"))
	cmd.Env = append(env, "STOP_GRACE=5")
	out, err := cmd.StdoutPipe()
	if err != nil {
		t.Fatal(err)
	}
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	waitFor(t, filepath.Join(dir, "running"))
	start := time.Now()
	if err := cmd.Process.Signal(syscall.SIGTERM); err != nil {
		t.Fatal(err)
	}
	eof := make(chan struct{})
	go func() {
		buf := make([]byte, 256)
		for {
			if _, err := out.Read(buf); err != nil {
				close(eof)
				return
			}
		}
	}()
	select {
	case <-eof:
	case <-time.After(4 * time.Second):
		_ = cmd.Process.Kill()
		t.Fatal("the script's output stayed open after SIGTERM")
	}
	_ = cmd.Wait()
	if d := time.Since(start); d > 2*time.Second {
		t.Errorf("output closed %v after SIGTERM, want well under the 5 s grace", d)
	}
}
