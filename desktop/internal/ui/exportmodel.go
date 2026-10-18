package ui

import (
	"errors"
	"path/filepath"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/export"
)

// CheckoutCandidates lists the directories the operator can export into,
// without duplicates and in order of preference: the one named on the
// command line, the one remembered from the last export, the working
// directory (`mise run desktop-run` runs in the checkout), and the checkout
// the binary was built in (build/desktop/spooder-calibrator). There is no
// free text field, so the Space key stays the e-stop everywhere.
func CheckoutCandidates(flag, remembered, cwd, executable string) []string {
	var out []string
	seen := map[string]bool{}
	add := func(dir string) {
		if dir == "" {
			return
		}
		if abs, err := filepath.Abs(dir); err == nil {
			dir = abs
		}
		dir = filepath.Clean(dir)
		if !seen[dir] {
			seen[dir] = true
			out = append(out, dir)
		}
	}
	add(flag)
	add(remembered)
	add(cwd)
	if executable != "" {
		add(filepath.Dir(filepath.Dir(filepath.Dir(executable))))
	}
	return out
}

// exportModel is the export pane's state: the chosen checkout, the
// prepared plan whose difference is shown, and the outcome.
type exportModel struct {
	candidates []string
	chosen     int
	plan       *export.Plan
	status     string // the last outcome, for the pane
	failed     bool
	// save remembers a checkout after a successful write.
	save func(dir string) error
}

// preferred picks the first candidate that is a checkout.
func (m *exportModel) preferred() {
	for i, dir := range m.candidates {
		if export.Check(dir) == nil {
			m.chosen = i
			return
		}
	}
}

func (m *exportModel) dir() string {
	if m.chosen < 0 || m.chosen >= len(m.candidates) {
		return ""
	}
	return m.candidates[m.chosen]
}

// choose selects a checkout and drops a plan made for another.
func (m *exportModel) choose(i int) {
	if i != m.chosen {
		m.chosen = i
		m.plan = nil
		m.status, m.failed = "", false
	}
}

// prepare makes the plan for table and shows its difference, or says why
// the directory is refused.
func (m *exportModel) prepare(table [calib.Joints]calib.Joint) {
	plan, err := export.Prepare(m.dir(), table)
	if err != nil {
		m.plan = nil
		m.status, m.failed = "Refused: "+err.Error(), true
		return
	}
	m.plan = &plan
	if plan.Changed {
		m.status, m.failed = "Review the difference, then write.", false
	} else {
		m.status, m.failed = "The checkout already holds this table.", false
	}
}

// stale reports whether the shown plan no longer matches table, so it
// must not be written.
func (m *exportModel) stale(table [calib.Joints]calib.Joint) bool {
	return m.plan != nil && m.plan.New != export.Format(table)
}

// canWrite reports whether the shown difference may be confirmed.
func (m *exportModel) canWrite(table [calib.Joints]calib.Joint) bool {
	return m.plan != nil && m.plan.Changed && !m.stale(table)
}

// write confirms the shown plan.
func (m *exportModel) write(table [calib.Joints]calib.Joint) {
	if !m.canWrite(table) {
		return
	}
	err := m.plan.Write()
	switch {
	case errors.Is(err, export.ErrChanged):
		m.status, m.failed = "Not written: "+err.Error(), true
	case err != nil:
		m.status, m.failed = "Not written: "+err.Error(), true
	default:
		m.status, m.failed = "Wrote "+m.plan.Path+". Rebuild and flash to make it the robot's defaults.", false
		if m.save != nil {
			_ = m.save(m.plan.Dir)
		}
	}
	m.plan = nil
}

// cancel drops the shown plan.
func (m *exportModel) cancel() {
	m.plan = nil
	m.status, m.failed = "", false
}
