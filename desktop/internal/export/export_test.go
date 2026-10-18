package export

import (
	"bufio"
	"encoding/hex"
	"errors"
	"io/fs"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

var repoRoot = filepath.Join("..", "..", "..")

// reportedDefaults decodes the golden report of the compiled defaults
// (calibration_report_defaults in test/golden/protocol_v2.txt, written by
// the C++ encoder from joint_calibration::DEFAULTS).
func reportedDefaults(t *testing.T) [calib.Joints]calib.Joint {
	t.Helper()
	f, err := os.Open(filepath.Join(repoRoot, "test", "golden", "protocol_v2.txt"))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	for sc.Scan() {
		parts := strings.Split(sc.Text(), " ")
		if parts[0] != "calibration_report_defaults" {
			continue
		}
		raw, err := hex.DecodeString(parts[1])
		if err != nil {
			t.Fatal(err)
		}
		var d protocol.Decoder
		for _, b := range raw {
			d.Push(b)
		}
		frame, ok := d.Next()
		if !ok || frame.Type != protocol.TypeCalibrationReport {
			t.Fatal("golden defaults report does not decode")
		}
		var table [calib.Joints]calib.Joint
		for i, j := range frame.CalibrationReport.Joints {
			table[i] = calib.FromWire(j)
		}
		return table
	}
	t.Fatal("no calibration_report_defaults vector")
	return [calib.Joints]calib.Joint{}
}

// The export of the reported defaults is the golden defaults file, byte
// for byte (design D6).
func TestGoldenDefaults(t *testing.T) {
	got := Format(reportedDefaults(t))
	for _, path := range []string{
		filepath.Join(repoRoot, "test", "golden", "calibration_defaults.cpp"),
	} {
		want, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		if got != string(want) {
			t.Errorf("export differs from %s:\n%s", path, render(Diff(string(want), got)))
		}
	}
}

func render(d []DiffLine) string {
	var b strings.Builder
	for _, l := range d {
		if l.Kind != Same {
			b.WriteByte(byte(l.Kind))
			b.WriteString(l.Text)
			b.WriteByte('\n')
		}
	}
	return b.String()
}

func TestFormatValues(t *testing.T) {
	var table [calib.Joints]calib.Joint
	table[0] = calib.Joint{OffsetCentideg: -50, Pulse0Us: 2450, Pulse180Us: 610, Reversed: true}
	table[17] = calib.Joint{OffsetCentideg: 12345, Pulse0Us: 400, Pulse180Us: 2600}
	table[1] = calib.Joint{OffsetCentideg: -32768}
	text := Format(table)
	for _, want := range []string{
		"  {-0.50, 2450, 610, true}, // leg 1 coxa\n",
		"  {-327.68, 0, 0, false}, // leg 1 femur\n",
		"  {0.00, 0, 0, false}, // leg 1 tibia\n",
		"  {123.45, 400, 2600, false}, // leg 6 tibia\n",
	} {
		if !strings.Contains(text, want) {
			t.Errorf("missing %q", want)
		}
	}
}

// checkout makes a minimal spooder checkout with some other files.
func checkout(t *testing.T) string {
	t.Helper()
	dir := t.TempDir()
	for path, content := range map[string]string{
		DefaultsPath: "old defaults\n",
		filepath.Join("core", "movement", "body.cpp"): "body\n",
		filepath.Join("openspec", "project.md"):       "spec\n",
		"CLAUDE.md":                                   "notes\n",
	} {
		full := filepath.Join(dir, path)
		if err := os.MkdirAll(filepath.Dir(full), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(full, []byte(content), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	return dir
}

type entry struct {
	mode    fs.FileMode
	content string
	mtime   time.Time
}

// tree lists every path under dir with its mode, content and time.
func tree(t *testing.T, dir string) map[string]entry {
	t.Helper()
	out := map[string]entry{}
	err := filepath.WalkDir(dir, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		fi, err := d.Info()
		if err != nil {
			return err
		}
		e := entry{mode: fi.Mode()}
		if !d.IsDir() {
			b, err := os.ReadFile(path)
			if err != nil {
				return err
			}
			e.content = string(b)
			e.mtime = fi.ModTime()
		}
		rel, _ := filepath.Rel(dir, path)
		out[rel] = e
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
	return out
}

// Spec "Write the defaults": after the shown difference is confirmed, the
// defaults file holds the table and nothing else in the checkout changes.
func TestWriteTouchesOnlyTheDefaults(t *testing.T) {
	dir := checkout(t)
	if err := os.Chmod(filepath.Join(dir, DefaultsPath), 0o640); err != nil {
		t.Fatal(err)
	}
	before := tree(t, dir)
	table := reportedDefaults(t)
	table[4].Pulse0Us = 2450
	plan, err := Prepare(dir, table)
	if err != nil {
		t.Fatal(err)
	}
	if !plan.Changed || plan.Old != "old defaults\n" {
		t.Fatalf("plan %+v", plan)
	}
	if removed, added := Changes(plan.Diff); removed != 1 || added != strings.Count(plan.New, "\n") {
		t.Fatalf("diff -%d +%d", removed, added)
	}
	// Preparing writes nothing.
	if after := tree(t, dir); !equalTrees(before, after, "") {
		t.Fatal("prepare changed the checkout")
	}
	if err := plan.Write(); err != nil {
		t.Fatal(err)
	}
	after := tree(t, dir)
	if !equalTrees(before, after, DefaultsPath) {
		t.Fatalf("other paths changed:\nbefore %v\nafter  %v", keys(before), keys(after))
	}
	got := after[DefaultsPath]
	if got.content != Format(table) || got.mode != before[DefaultsPath].mode {
		t.Fatalf("defaults %q mode %v", got.content[:40], got.mode)
	}
}

func equalTrees(a, b map[string]entry, except string) bool {
	if len(a) != len(b) {
		return false
	}
	for k, v := range a {
		w, ok := b[k]
		if !ok {
			return false
		}
		if k == except {
			continue
		}
		if k == filepath.Dir(except) || k == filepath.Dir(filepath.Dir(except)) || k == "." {
			// Directory times change with the rename; compare modes only.
			if v.mode != w.mode {
				return false
			}
			continue
		}
		if v != w {
			return false
		}
	}
	return true
}

func keys(m map[string]entry) []string {
	var out []string
	for k := range m {
		out = append(out, k)
	}
	return out
}

// Spec "Not a checkout": the app refuses and says why.
func TestNotACheckout(t *testing.T) {
	empty := t.TempDir()
	_, err := Prepare(empty, reportedDefaults(t))
	if !errors.Is(err, ErrNotCheckout) || !strings.Contains(err.Error(), "core/movement/calibration_defaults.cpp") {
		t.Fatalf("empty dir: %v", err)
	}
	noSpec := checkout(t)
	if err := os.RemoveAll(filepath.Join(noSpec, "openspec")); err != nil {
		t.Fatal(err)
	}
	if err := Check(noSpec); !errors.Is(err, ErrNotCheckout) || !strings.Contains(err.Error(), "openspec") {
		t.Fatalf("no openspec: %v", err)
	}
	if err := Check(""); !errors.Is(err, ErrNotCheckout) {
		t.Fatalf("no dir: %v", err)
	}
	linked := checkout(t)
	target := filepath.Join(t.TempDir(), "elsewhere.cpp")
	if err := os.WriteFile(target, []byte("x"), 0o644); err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(linked, DefaultsPath)
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(target, path); err != nil {
		t.Skip("no symlinks:", err)
	}
	if err := Check(linked); !errors.Is(err, ErrNotCheckout) {
		t.Fatalf("symlinked defaults: %v", err)
	}
	if empty := tree(t, empty); len(empty) != 1 {
		t.Fatal("refusal wrote something")
	}
}

// Review finding: a symlinked core or core/movement would let the write
// land outside the chosen checkout.
func TestSymlinkedDirectoriesAreRefused(t *testing.T) {
	for _, linked := range []string{"core", filepath.Join("core", "movement")} {
		dir := checkout(t)
		outside := t.TempDir()
		if err := os.MkdirAll(filepath.Join(outside, "movement"), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(outside, "movement", "calibration_defaults.cpp"), []byte("x\n"), 0o644); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(outside, "calibration_defaults.cpp"), []byte("x\n"), 0o644); err != nil {
			t.Fatal(err)
		}
		if err := os.RemoveAll(filepath.Join(dir, linked)); err != nil {
			t.Fatal(err)
		}
		if err := os.MkdirAll(filepath.Dir(filepath.Join(dir, linked)), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.Symlink(outside, filepath.Join(dir, linked)); err != nil {
			t.Skip("no symlinks:", err)
		}
		if _, err := Prepare(dir, reportedDefaults(t)); !errors.Is(err, ErrNotCheckout) {
			t.Errorf("%s linked: %v", linked, err)
		}
	}
}

func TestWriteRefusesAFileChangedSinceTheDiff(t *testing.T) {
	dir := checkout(t)
	plan, err := Prepare(dir, reportedDefaults(t))
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(dir, DefaultsPath)
	if err := os.WriteFile(path, []byte("edited meanwhile\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := plan.Write(); !errors.Is(err, ErrChanged) {
		t.Fatalf("write: %v", err)
	}
	if b, _ := os.ReadFile(path); string(b) != "edited meanwhile\n" {
		t.Fatal("file overwritten")
	}
	assertNoTemporaries(t, dir)
}

func TestWriteFailureLeavesTheFileAndNoTemporary(t *testing.T) {
	if os.Getuid() == 0 {
		t.Skip("root ignores directory permissions")
	}
	dir := checkout(t)
	plan, err := Prepare(dir, reportedDefaults(t))
	if err != nil {
		t.Fatal(err)
	}
	movement := filepath.Join(dir, "core", "movement")
	if err := os.Chmod(movement, 0o555); err != nil {
		t.Fatal(err)
	}
	defer os.Chmod(movement, 0o755)
	if err := plan.Write(); err == nil {
		t.Fatal("write into a read-only directory succeeded")
	}
	if b, _ := os.ReadFile(filepath.Join(dir, DefaultsPath)); string(b) != "old defaults\n" {
		t.Fatal("file changed")
	}
	assertNoTemporaries(t, dir)
}

func assertNoTemporaries(t *testing.T, dir string) {
	t.Helper()
	entries, err := os.ReadDir(filepath.Join(dir, "core", "movement"))
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range entries {
		if strings.HasSuffix(e.Name(), ".tmp") {
			t.Fatalf("temporary left: %s", e.Name())
		}
	}
}

func TestUnchangedExportWritesNothing(t *testing.T) {
	dir := checkout(t)
	table := reportedDefaults(t)
	path := filepath.Join(dir, DefaultsPath)
	if err := os.WriteFile(path, []byte(Format(table)), 0o644); err != nil {
		t.Fatal(err)
	}
	before := tree(t, dir)
	plan, err := Prepare(dir, table)
	if err != nil {
		t.Fatal(err)
	}
	if plan.Changed {
		t.Fatal("changed")
	}
	if removed, added := Changes(plan.Diff); removed+added != 0 {
		t.Fatal("diff not empty")
	}
	if err := plan.Write(); err != nil {
		t.Fatal(err)
	}
	if !equalTrees(before, tree(t, dir), "") {
		t.Fatal("checkout changed")
	}
}

func TestDiff(t *testing.T) {
	d := Diff("a\nb\nc\n", "a\nB\nc\nd\n")
	want := []DiffLine{{Same, "a"}, {Added, "B"}, {Removed, "b"}, {Same, "c"}, {Added, "d"}}
	if len(d) != len(want) {
		t.Fatalf("diff %+v", d)
	}
	for i := range d {
		if d[i] != want[i] {
			t.Fatalf("line %d: %+v, want %+v", i, d[i], want[i])
		}
	}
	if d := Diff("x", "x\n"); len(d) != 2 {
		t.Fatalf("missing final newline not shown: %+v", d)
	}
	if d := Diff("", "a\n"); len(d) != 1 || d[0].Kind != Added {
		t.Fatalf("from empty: %+v", d)
	}
}

func TestLastDir(t *testing.T) {
	cfg := t.TempDir()
	if LoadLastDir(cfg) != "" {
		t.Fatal("remembered nothing yet")
	}
	if err := SaveLastDir(cfg, "/home/me/spooder"); err != nil {
		t.Fatal(err)
	}
	if got := LoadLastDir(cfg); got != "/home/me/spooder" {
		t.Fatalf("got %q", got)
	}
}
