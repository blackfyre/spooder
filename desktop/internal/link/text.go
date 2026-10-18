package link

import (
	"fmt"
	"strings"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// MaxLineLength is the longest text line kept as one; longer runs are split.
const MaxLineLength = 1024

// Line is one line of the robot's debug text.
type Line struct {
	Text string
	// Stray counts 0xA5 bytes in the line that did not start a valid frame.
	// The robot's text must be printable ASCII, so any is a firmware bug (or
	// a frame corrupted on the way); they are shown as \xA5.
	Stray int
	// Binary counts other bytes outside printable ASCII, tab and CR,
	// shown as \xNN.
	Binary int
}

// textAccumulator collects the bytes the decoder discards into lines.
type textAccumulator struct {
	b      strings.Builder
	stray  int
	binary int
	n      int // bytes in the line, as received
	emit   func(Line)
}

func (t *textAccumulator) add(c byte) {
	switch {
	case c == '\n':
		t.flush()
		return
	case c == '\r':
		// line endings are CRLF or LF
	case c == '\t' || (c >= 0x20 && c < 0x7F):
		t.b.WriteByte(c)
	default:
		if c == protocol.Start {
			t.stray++
		} else {
			t.binary++
		}
		fmt.Fprintf(&t.b, `\x%02X`, c)
	}
	t.n++
	if t.n >= MaxLineLength {
		t.flush()
	}
}

// flush emits the current line, even without a newline.
func (t *textAccumulator) flush() {
	l :=Line{Text: t.b.String(), Stray: t.stray, Binary: t.binary}
	t.b.Reset()
	t.stray, t.binary, t.n = 0, 0, 0
	t.emit(l)
}

// pending reports whether a partial line is held.
func (t *textAccumulator) pending() bool { return t.n > 0 }
