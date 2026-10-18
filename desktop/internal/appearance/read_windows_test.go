package appearance

import "testing"

// TestReadRealSystem runs the real Windows readers; it must not fail or
// panic whatever the runner's settings are.
func TestReadRealSystem(t *testing.T) {
	s := Read()
	t.Logf("dark=%v high-contrast=%v accent=%v (%v)", s.Dark, s.HighContrast, s.Accent, s.HasAccent)
}
