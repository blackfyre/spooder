package link

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

type listOnly struct {
	ports []PortInfo
	err   error
}

func (l listOnly) List() ([]PortInfo, error) { return l.ports, l.err }
func (l listOnly) Open(string) (Port, error) { return nil, errors.New("not here") }

func TestWithPortListsAnExistingPath(t *testing.T) {
	path := filepath.Join(t.TempDir(), "usb")
	sys := WithPort(listOnly{ports: []PortInfo{servo}}, path)

	ports, err := sys.List()
	if err != nil || len(ports) != 1 {
		t.Fatalf("absent path listed: %v %v", ports, err)
	}
	if err := os.WriteFile(path, nil, 0o600); err != nil {
		t.Fatal(err)
	}
	ports, _ = sys.List()
	if len(ports) != 2 || ports[1] != (PortInfo{Name: path}) {
		t.Fatalf("got %v", ports)
	}
	// Reconnection follows the path: the same device by name.
	if !SameDevice(PortInfo{Name: path}, ports[1]) {
		t.Fatal("not the same device")
	}
}

func TestWithPortDoesNotDuplicateAnEnumeratedPort(t *testing.T) {
	sys := WithPort(listOnly{ports: []PortInfo{servo}}, servo.Name)
	ports, _ := sys.List()
	if len(ports) != 1 || ports[0] != servo {
		t.Fatalf("got %v", ports)
	}
}

func TestWithPortSurvivesAnEnumerationFailure(t *testing.T) {
	path := filepath.Join(t.TempDir(), "usb")
	if err := os.WriteFile(path, nil, 0o600); err != nil {
		t.Fatal(err)
	}
	ports, err := WithPort(listOnly{err: errors.New("no sysfs")}, path).List()
	if err != nil || len(ports) != 1 || ports[0].Name != path {
		t.Fatalf("got %v %v", ports, err)
	}
}

func TestWithoutPortIsUnchanged(t *testing.T) {
	sys := listOnly{ports: []PortInfo{servo}}
	if _, ok := WithPort(sys, "").(listOnly); !ok {
		t.Fatal("wrapped")
	}
}
