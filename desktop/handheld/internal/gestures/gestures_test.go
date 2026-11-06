package gestures

import (
	"bytes"
	"os"
	"strings"
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// TestTableMatchesGolden fails when table_gen.go is not exactly what the
// generator writes from the golden file: a regenerated fixture without a
// regenerated table, or a hand edit of the table.
func TestTableMatchesGolden(t *testing.T) {
	f, err := os.Open(GoldenPath())
	if err != nil {
		t.Fatalf("golden gestures: %v", err)
	}
	defer f.Close()
	gs, err := Parse(f)
	if err != nil {
		t.Fatalf("golden gestures: %v", err)
	}
	want, err := Format(gs)
	if err != nil {
		t.Fatal(err)
	}
	got, err := os.ReadFile("table_gen.go")
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got, want) {
		t.Fatal("table_gen.go differs from the golden gestures: run go generate ./internal/gestures in desktop/handheld")
	}
}

func TestTableScripts(t *testing.T) {
	if len(Table) != protocol.GestureCount-1 {
		t.Fatalf("%d gestures, want %d", len(Table), protocol.GestureCount-1)
	}
	scripted := map[uint8]bool{protocol.GestureSayHi: true, protocol.GestureWiggle: true}
	for i, g := range Table {
		if g.ID != uint8(i+1) || g.StepMs != StepMs {
			t.Errorf("Table[%d]: id %d step %d", i, g.ID, g.StepMs)
		}
		if g.Scripted != scripted[g.ID] {
			t.Errorf("gesture %d (%s): scripted %t", g.ID, g.Name, g.Scripted)
		}
		// A script's preview has more than its starting pose; an unscripted
		// one has none.
		if g.Scripted && len(g.Frames) < 2 || !g.Scripted && len(g.Frames) != 0 {
			t.Errorf("gesture %d (%s): %d frames", g.ID, g.Name, len(g.Frames))
		}
	}
}

const angles = " 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90"

func validText() string {
	return "# comment\n\ngesture 1 say_hi scripted\npose 1 0" + angles + "\npose 1 50" + angles +
		"\ngesture 2 wiggle none\ngesture 3 dance none\ngesture 4 attack none\ngesture 5 ball none\n"
}

func TestParse(t *testing.T) {
	gs, err := Parse(strings.NewReader(validText()))
	if err != nil {
		t.Fatal(err)
	}
	if len(gs) != 5 || !gs[0].Scripted || len(gs[0].Frames) != 2 || gs[0].Frames[1][17] != 90 ||
		gs[1].Scripted || gs[4].Name != "ball" {
		t.Errorf("Parse = %+v", gs)
	}
}

func TestParseRefuses(t *testing.T) {
	v := validText()
	cases := map[string]string{
		"scripted without poses":      strings.Replace(v, "gesture 2 wiggle none", "gesture 2 wiggle scripted", 1),
		"last scripted without poses": strings.Replace(v, "gesture 5 ball none", "gesture 5 ball scripted", 1),
		"pose for none":               strings.Replace(v, "gesture 3 dance none", "gesture 3 dance none\npose 3 0"+angles, 1),
		"pose before gesture":         "pose 1 0" + angles + "\n" + v,
		"wrong pose id":               strings.Replace(v, "pose 1 50", "pose 2 50", 1),
		"time gap":                    strings.Replace(v, "pose 1 50", "pose 1 100", 1),
		"first time not zero":         strings.Replace(v, "pose 1 0", "pose 1 50", 1),
		"ids out of order":            strings.Replace(v, "gesture 3 dance", "gesture 4 dance", 1),
		"missing gesture":             strings.Replace(v, "gesture 5 ball none\n", "", 1),
		"extra gesture":               v + "gesture 6 extra none\n",
		"short pose":                  strings.Replace(v, "pose 1 50"+angles, "pose 1 50 90", 1),
		"angle not a number":          strings.Replace(v, "pose 1 50 90", "pose 1 50 x", 1),
		"angle NaN":                   strings.Replace(v, "pose 1 50 90", "pose 1 50 NaN", 1),
		"angle out of range":          strings.Replace(v, "pose 1 50 90", "pose 1 50 180.5", 1),
		"angle below range":           strings.Replace(v, "pose 1 50 90", "pose 1 50 -180.5", 1),
		"bad availability":            strings.Replace(v, "dance none", "dance maybe", 1),
		"bad name":                    strings.Replace(v, "dance", "Dance", 1),
		"unknown record":              v + "frame 1\n",
		"double space":                strings.Replace(v, "gesture 3 dance", "gesture  3 dance", 1),
	}
	for name, text := range cases {
		if _, err := Parse(strings.NewReader(text)); err == nil {
			t.Errorf("%s: Parse accepted it", name)
		}
	}
}
