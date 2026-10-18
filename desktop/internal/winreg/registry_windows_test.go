package winreg

import (
	"errors"
	"os"
	"testing"
)

func integration(t *testing.T) {
	if os.Getenv("BOPEN_WINDOWS_INTEGRATION") != "1" {
		t.Skip("set BOPEN_WINDOWS_INTEGRATION=1 to write to the real registry")
	}
}

func TestSystemRoundTrip(t *testing.T) {
	integration(t)
	const key = `Software\bopen-test\round-trip`
	reg := System{}
	t.Cleanup(func() { reg.DeleteTree(CurrentUser, `Software\bopen-test`) })
	if err := reg.SetString(CurrentUser, key+`\child`, "", "default"); err != nil {
		t.Fatal(err)
	}
	if err := reg.SetString(CurrentUser, key, "named", "value"); err != nil {
		t.Fatal(err)
	}
	if v, err := reg.String(CurrentUser, key, "named"); err != nil || v != "value" {
		t.Fatalf("String = %q, %v", v, err)
	}
	if subs, err := reg.SubKeys(CurrentUser, key); err != nil || len(subs) != 1 || subs[0] != "child" {
		t.Fatalf("SubKeys = %v, %v", subs, err)
	}
	if err := reg.DeleteValue(CurrentUser, key, "named"); err != nil {
		t.Fatal(err)
	}
	if _, err := reg.String(CurrentUser, key, "named"); !errors.Is(err, ErrNotExist) {
		t.Fatalf("deleted value: %v", err)
	}
	if err := reg.DeleteTree(CurrentUser, key); err != nil {
		t.Fatal(err)
	}
	if _, err := reg.SubKeys(CurrentUser, key); !errors.Is(err, ErrNotExist) {
		t.Fatalf("deleted tree: %v", err)
	}
}

func TestSystemReadsRealValues(t *testing.T) {
	integration(t)
	// Every Windows installation has these.
	if _, err := (System{}).SubKeys(LocalMachine, `SOFTWARE\Microsoft`); err != nil {
		t.Fatal(err)
	}
	if _, err := (System{}).String(LocalMachine, `SOFTWARE\Microsoft\Windows NT\CurrentVersion`, "ProductName"); err != nil {
		t.Fatal(err)
	}
}
