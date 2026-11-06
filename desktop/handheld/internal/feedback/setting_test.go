package feedback

import "testing"

// The sound is on by default; off is kept as a file, and on removes it.
func TestSettingKeptBetweenRuns(t *testing.T) {
	s := SettingIn(t.TempDir())
	if !s.On() {
		t.Fatal("sound off without a setting")
	}
	if err := s.Save(false); err != nil {
		t.Fatal(err)
	}
	if SettingIn(s.File[:len(s.File)-len(SettingFile)-1]).On() {
		t.Fatal("sound on after saving off")
	}
	for range 2 { // removing twice is harmless
		if err := s.Save(true); err != nil {
			t.Fatal(err)
		}
	}
	if !s.On() {
		t.Fatal("sound off after saving on")
	}
}
