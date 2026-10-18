package export

import "strings"

// DiffKind marks a line of a difference.
type DiffKind byte

const (
	Same    DiffKind = ' '
	Removed DiffKind = '-'
	Added   DiffKind = '+'
)

// DiffLine is one line of a difference.
type DiffLine struct {
	Kind DiffKind
	Text string
}

func splitLines(s string) []string {
	if s == "" {
		return nil
	}
	lines := strings.SplitAfter(s, "\n")
	if lines[len(lines)-1] == "" {
		lines = lines[:len(lines)-1]
	}
	for i, l := range lines {
		lines[i] = strings.TrimSuffix(l, "\n")
	}
	return lines
}

// Diff is the line difference from old to new (longest common
// subsequence), every line of both texts in order. A missing final newline
// shows as a change of the last line.
func Diff(old, new string) []DiffLine {
	a, b := splitLines(old), splitLines(new)
	if strings.HasSuffix(old, "\n") != strings.HasSuffix(new, "\n") {
		// Make the difference visible rather than silent.
		if len(a) > 0 && !strings.HasSuffix(old, "\n") {
			a[len(a)-1] += " (no newline at end of file)"
		}
		if len(b) > 0 && !strings.HasSuffix(new, "\n") {
			b[len(b)-1] += " (no newline at end of file)"
		}
	}
	// lcs[i][j] is the common length of a[i:] and b[j:].
	lcs := make([][]int, len(a)+1)
	for i := range lcs {
		lcs[i] = make([]int, len(b)+1)
	}
	for i := len(a) - 1; i >= 0; i-- {
		for j := len(b) - 1; j >= 0; j-- {
			if a[i] == b[j] {
				lcs[i][j] = lcs[i+1][j+1] + 1
			} else {
				lcs[i][j] = max(lcs[i+1][j], lcs[i][j+1])
			}
		}
	}
	var out []DiffLine
	i, j := 0, 0
	for i < len(a) || j < len(b) {
		switch {
		case i < len(a) && j < len(b) && a[i] == b[j]:
			out = append(out, DiffLine{Same, a[i]})
			i++
			j++
		case j < len(b) && (i == len(a) || lcs[i][j+1] >= lcs[i+1][j]):
			out = append(out, DiffLine{Added, b[j]})
			j++
		default:
			out = append(out, DiffLine{Removed, a[i]})
			i++
		}
	}
	return out
}

// Changes counts the removed and added lines.
func Changes(d []DiffLine) (removed, added int) {
	for _, l := range d {
		switch l.Kind {
		case Removed:
			removed++
		case Added:
			added++
		}
	}
	return removed, added
}
