package scripts

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// stubReadelf prints a controlled header and version needs.
func stubReadelf(machine, glibc string) string {
	return `#!/bin/sh
case "$1" in
-h) printf '%s\n' "ELF Header:" "  Class:                             ELF64" "  Machine:                           ` + machine + `" ;;
-V) printf '%s\n' "Version needs section '.gnu.version_r' contains 1 entry:" \
     " Addr: 0x0000000000000000  Offset: 0x000400  Link: 6 (.dynstr)" \
     "  000000: Version: 1  File: libc.so.6  Cnt: 2" \
     "  0x0010:   Name: GLIBC_2.17  Flags: none  Version: 3" \
     "  0x0020:   Name: ` + glibc + `  Flags: none  Version: 2" ;;
*) exit 2 ;;
esac
`
}

func runCheckGlibc(t *testing.T, machine, glibc string) (string, error) {
	t.Helper()
	bin := t.TempDir()
	if err := os.WriteFile(filepath.Join(bin, "readelf"), []byte(stubReadelf(machine, glibc)), 0o755); err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command("sh", "check-glibc.sh", "spooder-handheld")
	cmd.Env = append(os.Environ(), "PATH="+bin+":"+os.Getenv("PATH"))
	out, err := cmd.CombinedOutput()
	return string(out), err
}

func TestCheckGlibc(t *testing.T) {
	for _, c := range []struct {
		machine, glibc string
		pass           bool
	}{
		{"AArch64", "GLIBC_2.31", true},
		{"AArch64", "GLIBC_2.34", false},
		// 2.4 < 2.31: the versions compare as numbers, not as text.
		{"AArch64", "GLIBC_2.4", true},
		{"Advanced Micro Devices X86-64", "GLIBC_2.17", false},
	} {
		out, err := runCheckGlibc(t, c.machine, c.glibc)
		if (err == nil) != c.pass {
			t.Errorf("%s with %s: err %v, want pass %v\n%s", c.machine, c.glibc, err, c.pass, out)
		}
	}
}

func TestCheckGoVersion(t *testing.T) {
	dir := t.TempDir()
	write := func(name, body string) string {
		p := filepath.Join(dir, name)
		if err := os.WriteFile(p, []byte(body), 0o644); err != nil {
			t.Fatal(err)
		}
		return p
	}
	mise := write("mise.toml", "[tools]\ngo = \"1.27.1\"\n")
	for _, c := range []struct {
		arg  string
		pass bool
	}{
		{"ARG GO_VERSION=1.27.1", true},
		{"ARG GO_VERSION=1.27.0", false},
		{"ARG GO_VERSIONS=1.27.1", false},
	} {
		cf := write("Containerfile", "FROM x\n"+c.arg+"\n")
		out, err := exec.Command("sh", "check-go-version.sh", cf, mise).CombinedOutput()
		if (err == nil) != c.pass {
			t.Errorf("%q: err %v, want pass %v\n%s", c.arg, err, c.pass, out)
		}
	}
}

// The repository's own Containerfile and mise.toml agree.
func TestRepositoryGoVersionsAgree(t *testing.T) {
	out, err := exec.Command("sh", "check-go-version.sh", "../build/Containerfile", "../../../mise.toml").CombinedOutput()
	if err != nil || !strings.Contains(string(out), "Go ") {
		t.Fatalf("check-go-version: %v\n%s", err, out)
	}
}
