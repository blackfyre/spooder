// Package export writes a reported calibration table as the complete text
// of core/movement/calibration_defaults.cpp (the fixed format of
// add-live-joint-calibration D6), and writes it into a spooder checkout
// safely: only into a directory that is a checkout, only after the
// difference from the file on disk has been shown, and atomically, touching
// that one file only (design D6).
package export

import (
	"errors"
	"fmt"
	"io/fs"
	"os"
	"path/filepath"
	"strings"

	"github.com/blackfyre/spooder/desktop/internal/calib"
)

// DefaultsPath is the defaults file, relative to a checkout's root.
var DefaultsPath = filepath.Join("core", "movement", "calibration_defaults.cpp")

// header is the file's fixed beginning, up to the table.
const header = `// Spooder joint calibration defaults, in a fixed format: an export from a
// calibration front-end replaces this whole file. Do not reformat it.
//
// One line per joint, in servo output order (leg 1 coxa, femur, tibia, then
// leg 2 ...):
//   {offset in degrees (two decimals), pulse at 0 degrees in us,
//    pulse at 180 degrees in us, reversed},
// test/golden/calibration_defaults.cpp must equal this file.
#include "joint_calibration.h"

namespace joint_calibration
{

const joint_calibration::Joint DEFAULTS[18] = {
`

const footer = `};

} // namespace joint_calibration
`

// degrees prints centidegrees with two decimals, from the integer, so no
// rounding is involved.
func degrees(centideg int16) string {
	v := int(centideg)
	sign := ""
	if v < 0 {
		sign, v = "-", -v
	}
	return fmt.Sprintf("%s%d.%02d", sign, v/100, v%100)
}

// Format returns the complete text of calibration_defaults.cpp for table.
func Format(table [calib.Joints]calib.Joint) string {
	var b strings.Builder
	b.WriteString(header)
	for i, j := range table {
		fmt.Fprintf(&b, "  {%s, %d, %d, %t}, // leg %d %s\n",
			degrees(j.OffsetCentideg), j.Pulse0Us, j.Pulse180Us, j.Reversed, calib.Leg(i), calib.Name(i))
	}
	b.WriteString(footer)
	return b.String()
}

// ErrNotCheckout is returned for a directory that is not a spooder
// checkout; the wrapped message says what is missing.
var ErrNotCheckout = errors.New("not a spooder checkout")

// Check verifies that dir is a spooder checkout: it holds DefaultsPath as a
// regular file and an openspec directory. Neither the file nor the
// directories on the way to it may be symbolic links, so the write cannot
// land outside the checkout (and a rename would replace a link, not its
// target).
func Check(dir string) error {
	if dir == "" {
		return fmt.Errorf("%w: no directory chosen", ErrNotCheckout)
	}
	sub := dir
	for _, part := range []string{"core", "movement"} {
		sub = filepath.Join(sub, part)
		fi, err := os.Lstat(sub)
		if errors.Is(err, fs.ErrNotExist) {
			return fmt.Errorf("%w: %s has no %s", ErrNotCheckout, dir, filepath.ToSlash(DefaultsPath))
		}
		if err != nil {
			return fmt.Errorf("%w: %v", ErrNotCheckout, err)
		}
		if !fi.IsDir() || fi.Mode()&fs.ModeSymlink != 0 {
			rel, _ := filepath.Rel(dir, sub)
			return fmt.Errorf("%w: %s is not a plain directory", ErrNotCheckout, filepath.ToSlash(rel))
		}
	}
	fi, err := os.Lstat(filepath.Join(dir, DefaultsPath))
	switch {
	case errors.Is(err, fs.ErrNotExist):
		return fmt.Errorf("%w: %s has no %s", ErrNotCheckout, dir, filepath.ToSlash(DefaultsPath))
	case err != nil:
		return fmt.Errorf("%w: %v", ErrNotCheckout, err)
	case !fi.Mode().IsRegular():
		return fmt.Errorf("%w: %s is not a regular file", ErrNotCheckout, filepath.ToSlash(DefaultsPath))
	}
	fi, err = os.Stat(filepath.Join(dir, "openspec"))
	if err != nil || !fi.IsDir() {
		return fmt.Errorf("%w: %s has no openspec directory", ErrNotCheckout, dir)
	}
	return nil
}

// Plan is an export prepared for confirmation.
type Plan struct {
	Dir     string
	Path    string // the defaults file
	Old     string // the file as read when the plan was made
	New     string
	Diff    []DiffLine
	Changed bool
}

// Prepare checks dir and computes the difference between its defaults file
// and the export of table.
func Prepare(dir string, table [calib.Joints]calib.Joint) (Plan, error) {
	if err := Check(dir); err != nil {
		return Plan{}, err
	}
	path := filepath.Join(dir, DefaultsPath)
	old, err := os.ReadFile(path)
	if err != nil {
		return Plan{}, err
	}
	p := Plan{Dir: dir, Path: path, Old: string(old), New: Format(table)}
	p.Diff = Diff(p.Old, p.New)
	p.Changed = p.Old != p.New
	return p, nil
}

// ErrChanged means the file changed after the plan's difference was shown.
var ErrChanged = errors.New("the file changed since the difference was shown; export again")

// Write replaces the defaults file with the plan's text, after checking
// again that dir is a checkout and that the file still holds what the shown
// difference was made from. The new text goes to a temporary file in the
// same directory, which is synced and renamed over the original, so the
// file is never left partly written and nothing else in the checkout
// changes.
func (p Plan) Write() error {
	if err := Check(p.Dir); err != nil {
		return err
	}
	fi, err := os.Stat(p.Path)
	if err != nil {
		return err
	}
	current, err := os.ReadFile(p.Path)
	if err != nil {
		return err
	}
	if string(current) != p.Old {
		return ErrChanged
	}
	if !p.Changed {
		return nil
	}
	tmp, err := os.CreateTemp(filepath.Dir(p.Path), ".calibration_defaults.cpp.*.tmp")
	if err != nil {
		return err
	}
	done := false
	defer func() {
		if !done {
			tmp.Close()
			os.Remove(tmp.Name())
		}
	}()
	if _, err := tmp.WriteString(p.New); err != nil {
		return err
	}
	if err := tmp.Chmod(fi.Mode().Perm()); err != nil {
		return err
	}
	if err := tmp.Sync(); err != nil {
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	if err := os.Rename(tmp.Name(), p.Path); err != nil {
		return err
	}
	done = true
	syncDir(filepath.Dir(p.Path))
	return nil
}

// syncDir makes the rename durable where the platform supports it.
func syncDir(dir string) {
	if d, err := os.Open(dir); err == nil {
		_ = d.Sync()
		d.Close()
	}
}
