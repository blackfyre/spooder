package link

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// Fuzz tests of the split of the stream into frames and text lines
// (openspec link-fuzzing, "Fuzzed text and frame splitting"). They call
// (*Link).read directly on an in-memory port, so the end-of-stream drain and
// line flushing are the real ones (design D4). The seed corpus runs in every
// `go test`; `mise run fuzz` adds random inputs.

// goldenPath is test/golden/protocol_v3.txt, the seed source (design D2).
var goldenPath = filepath.Join("..", "..", "..", "test", "golden", "protocol_v3.txt")

// goldenFrame is one line of the golden file: its frame bytes, and whether
// the decoder accepts it.
type goldenFrame struct {
	frame    []byte
	accepted bool
}

// readGoldenFrames reads the frame column of the golden file. The protocol
// package has a fuller reader, but test helpers cannot be shared across
// packages.
func readGoldenFrames(tb testing.TB) []goldenFrame {
	tb.Helper()
	f, err := os.Open(goldenPath)
	if err != nil {
		tb.Fatalf("golden vectors: %v", err)
	}
	defer f.Close()
	var out []goldenFrame
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	for line := 1; sc.Scan(); line++ {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		parts := strings.Split(text, " ")
		if len(parts) < 3 {
			tb.Fatalf("golden line %d: %q", line, text)
		}
		frame, err := hex.DecodeString(parts[1])
		if err != nil {
			tb.Fatalf("golden line %d: %v", line, err)
		}
		out = append(out, goldenFrame{frame: frame, accepted: parts[2] != "rejected"})
	}
	if err := sc.Err(); err != nil {
		tb.Fatal(err)
	}
	if len(out) == 0 {
		tb.Fatal("no golden vectors")
	}
	return out
}

// acceptedGoldenFrames are the golden frames the decoder accepts.
func acceptedGoldenFrames(tb testing.TB) [][]byte {
	tb.Helper()
	var out [][]byte
	for _, g := range readGoldenFrames(tb) {
		if g.accepted {
			out = append(out, g.frame)
		}
	}
	if len(out) == 0 {
		tb.Fatal("no accepted golden frames")
	}
	return out
}

// memPort is an in-memory port: it returns the stream in chunks whose sizes
// come from the stream itself, never an empty read (that would mean a quiet
// line, which a bare decoder has no equivalent for), then io.EOF.
type memPort struct {
	data []byte
	pos  int
}

func (p *memPort) Read(b []byte) (int, error) {
	if p.pos >= len(p.data) {
		return 0, io.EOF
	}
	size := 1 + int(p.data[p.pos])%67
	n := copy(b[:min(size, len(b))], p.data[p.pos:])
	p.pos += n
	return n, nil
}

func (p *memPort) Write([]byte) (int, error)          { return 0, errors.New("memPort is read-only") }
func (p *memPort) Close() error                       { return nil }
func (p *memPort) SetReadTimeout(time.Duration) error { return nil }

// splitEvent is one call to the handler: a frame or a line.
type splitEvent struct {
	isFrame bool
	frame   protocol.Frame
	line    Line
}

func (e splitEvent) String() string {
	if e.isFrame {
		return fmt.Sprintf("frame %v", protocol.Fields(e.frame))
	}
	return fmt.Sprintf("line %+v", e.line)
}

// splitRecorder records the handler's calls without synchronising: read
// runs on the test's goroutine.
type splitRecorder struct {
	events []splitEvent
}

func (r *splitRecorder) LinkState(State) {}
func (r *splitRecorder) Frame(f protocol.Frame, _ Stats) {
	r.events = append(r.events, splitEvent{isFrame: true, frame: f})
}
func (r *splitRecorder) Line(l Line, _ Stats) {
	r.events = append(r.events, splitEvent{line: l})
}

func (r *splitRecorder) frames() []protocol.Frame {
	var out []protocol.Frame
	for _, e := range r.events {
		if e.isFrame {
			out = append(out, e.frame)
		}
	}
	return out
}

func (r *splitRecorder) lines() []Line {
	var out []Line
	for _, e := range r.events {
		if !e.isFrame {
			out = append(out, e.line)
		}
	}
	return out
}

// split reads data through (*Link).read and returns what the handler got.
func split(t *testing.T, data []byte) *splitRecorder {
	t.Helper()
	rec := &splitRecorder{}
	l := New(nil, rec, nil, Options{})
	if err := l.read(&memPort{data: data}); !errors.Is(err, io.EOF) {
		t.Fatalf("read ended with %v, want io.EOF", err)
	}
	return rec
}

// bareDecode feeds data to a decoder alone, one byte at a time with Next
// after each, then drains it as at the end of a stream. It returns the
// frames and the discarded bytes.
func bareDecode(data []byte) ([]protocol.Frame, []byte) {
	var frames []protocol.Frame
	var discarded []byte
	d := protocol.Decoder{Discard: func(b byte) { discarded = append(discarded, b) }}
	next := func() {
		for {
			f, ok := d.Next()
			if !ok {
				return
			}
			frames = append(frames, f)
		}
	}
	for _, b := range data {
		d.Push(b)
		next()
	}
	for d.Pending() > 0 {
		d.Idle()
		next()
	}
	return frames, discarded
}

// modelLine is a line of the reference model with the number of received
// bytes it holds.
type modelLine struct {
	line     Line
	received int
}

// modelLines restates the documented line rules, independently of
// textAccumulator: a line ends at '\n' (not counted) or once it holds
// MaxLineLength received bytes; '\r' counts but renders as nothing; tab and
// printable ASCII render as themselves; any other byte renders as \xNN and
// counts as Stray (0xA5) or Binary; a partial line is flushed at the end.
func modelLines(discarded []byte) []modelLine {
	var out []modelLine
	var cur modelLine
	var text strings.Builder
	emit := func() {
		cur.line.Text = text.String()
		out = append(out, cur)
		cur = modelLine{}
		text.Reset()
	}
	for _, c := range discarded {
		if c == '\n' {
			emit()
			continue
		}
		cur.received++
		switch {
		case c == '\r':
		case c == '\t' || (c >= ' ' && c <= '~'):
			text.WriteByte(c)
		default:
			fmt.Fprintf(&text, `\x%02X`, c)
			if c == 0xA5 {
				cur.line.Stray++
			} else {
				cur.line.Binary++
			}
		}
		if cur.received == MaxLineLength {
			emit()
		}
	}
	if cur.received > 0 {
		emit()
	}
	return out
}

// checkSplitArbitrary checks the properties of design D4 for an arbitrary
// stream: the frames equal a bare decoder's, and the lines equal the
// model's, one for one and in order.
func checkSplitArbitrary(t *testing.T, data []byte) {
	rec := split(t, data)
	wantFrames, discarded := bareDecode(data)
	gotFrames := rec.frames()
	if len(gotFrames) != len(wantFrames) {
		t.Fatalf("%d frames, a bare decoder gives %d", len(gotFrames), len(wantFrames))
	}
	for i := range wantFrames {
		if gotFrames[i] != wantFrames[i] {
			t.Fatalf("frame %d:\n got %v\nwant %v", i, protocol.Fields(gotFrames[i]), protocol.Fields(wantFrames[i]))
		}
	}
	want := modelLines(discarded)
	got := rec.lines()
	if len(got) != len(want) {
		t.Fatalf("%d lines, the line rules give %d", len(got), len(want))
	}
	for i, m := range want {
		if m.received > MaxLineLength {
			t.Fatalf("model line %d holds %d received bytes", i, m.received)
		}
		if got[i] != m.line {
			t.Fatalf("line %d:\n got %+v\nwant %+v", i, got[i], m.line)
		}
		if len(got[i].Text) > 4*m.received {
			t.Fatalf("line %d renders %d received bytes in %d characters", i, m.received, len(got[i].Text))
		}
	}
}

// repeat returns n copies of c.
func repeat(c byte, n int) []byte { return bytes.Repeat([]byte{c}, n) }

// splitSeeds are the golden frames, the extra seeds of design D2 and its
// line-boundary seeds.
func splitSeeds(tb testing.TB) [][]byte {
	var seeds [][]byte
	for _, g := range readGoldenFrames(tb) {
		seeds = append(seeds, g.frame)
	}
	accepted := acceptedGoldenFrames(tb)
	a, b := accepted[0], accepted[len(accepted)-1]
	cat := func(parts ...[]byte) []byte { return bytes.Join(parts, nil) }
	seeds = append(seeds,
		cat(a, b), // two frames back to back
		cat([]byte("ok\r\n\x00\xa5\x03"), a, []byte("\xa5\xff tail\n")), // garbage before and after
		a[:len(a)-3],                          // truncated frame
		[]byte{protocol.Start},                // lone start marker
		cat(repeat('y', MaxLineLength+10), a), // a long line ending in a frame
	)
	for _, c := range []byte{'x', '\r', 0x01} {
		for _, n := range []int{MaxLineLength - 1, MaxLineLength, MaxLineLength + 1} {
			seeds = append(seeds, repeat(c, n))
		}
	}
	return seeds
}

func FuzzSplitArbitrary(f *testing.F) {
	for _, s := range splitSeeds(f) {
		f.Add(s)
	}
	f.Fuzz(checkSplitArbitrary)
}

// A run of one less than, exactly, and one more than MaxLineLength bytes
// is cut after exactly MaxLineLength received bytes, carriage returns
// included, and the rest starts the next line ("Line length boundary").
func TestSplitLineLengthBoundary(t *testing.T) {
	for _, c := range []struct {
		name   string
		b      byte
		render string
		binary int
	}{
		{"printable", 'x', "x", 0},
		{"carriage returns", '\r', "", 0},
		{"non-printable", 0x01, `\x01`, 1},
	} {
		for _, n := range []int{MaxLineLength - 1, MaxLineLength, MaxLineLength + 1} {
			line := func(count int) Line {
				return Line{Text: strings.Repeat(c.render, count), Binary: c.binary * count}
			}
			want := []Line{line(min(n, MaxLineLength))}
			if n > MaxLineLength {
				want = append(want, line(n-MaxLineLength))
			}
			rec := split(t, repeat(c.b, n))
			got := rec.lines()
			if len(rec.frames()) != 0 || len(got) != len(want) {
				t.Fatalf("%s × %d: %d frames, %d lines; want 0 frames, %d lines", c.name, n, len(rec.frames()), len(got), len(want))
			}
			for i := range want {
				if got[i] != want[i] {
					t.Fatalf("%s × %d line %d:\n got %d characters %+v\nwant %d characters %+v", c.name, n, i, len(got[i].Text), got[i], len(want[i].Text), want[i])
				}
			}
		}
	}
}

// Interleaved recipes (design D4). The input is read as items: a byte with
// its low bit clear picks an accepted golden frame by the next byte; one
// with it set makes a text line from the next three bytes (a 16-bit length
// and a seed for its characters). An item cut short by the end of the input
// is left out.
const (
	recipeFrame = 0
	recipeText  = 1
	// maxTextBytes keeps every written line, its '\n' included, shorter than
	// MaxLineLength received bytes, so the split never cuts it.
	maxTextBytes = MaxLineLength - 2
)

// textLine is length printable ASCII characters (never protocol.Start)
// derived from seed.
func textLine(length int, seed byte) string {
	var b strings.Builder
	for i := 0; i < length; i++ {
		b.WriteByte(byte(' ' + (int(seed)+7*i)%('~'-' '+1)))
	}
	return b.String()
}

// buildInterleaved turns a recipe into the stream and the events the split
// must give for it.
func buildInterleaved(t *testing.T, recipe []byte, accepted [][]byte) ([]byte, []splitEvent) {
	var stream []byte
	var want []splitEvent
	for i := 0; i < len(recipe); {
		switch recipe[i] & 1 {
		case recipeFrame:
			if i+2 > len(recipe) {
				return stream, want
			}
			frame := accepted[int(recipe[i+1])%len(accepted)]
			i += 2
			frames, discarded := bareDecode(frame)
			if len(frames) != 1 || len(discarded) != 0 {
				t.Fatalf("golden frame % x: %d frames, %d bytes discarded", frame, len(frames), len(discarded))
			}
			stream = append(stream, frame...)
			want = append(want, splitEvent{isFrame: true, frame: frames[0]})
		case recipeText:
			if i+4 > len(recipe) {
				return stream, want
			}
			length := (int(recipe[i+1])<<8 | int(recipe[i+2])) % (maxTextBytes + 1)
			text := textLine(length, recipe[i+3])
			i += 4
			stream = append(stream, text...)
			stream = append(stream, '\n')
			want = append(want, splitEvent{line: Line{Text: text}})
		}
	}
	return stream, want
}

// checkSplitInterleaved checks that text lines and golden frames written
// in order come back in that order: the frames exactly, the lines one for
// one.
func checkSplitInterleaved(t *testing.T, recipe []byte, accepted [][]byte) {
	stream, want := buildInterleaved(t, recipe, accepted)
	got := split(t, stream).events
	if len(got) != len(want) {
		t.Fatalf("%d events, wrote %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("event %d:\n got %v\nwant %v", i, got[i], want[i])
		}
	}
}

// interleavedSeeds are recipes for each accepted golden frame alone, and
// mixes of text and frames, including the longest text line kept whole.
func interleavedSeeds(accepted [][]byte) [][]byte {
	var seeds [][]byte
	for i := range accepted {
		seeds = append(seeds, []byte{recipeFrame, byte(i)})
	}
	text := func(length int, seed byte) []byte {
		return []byte{recipeText, byte(length >> 8), byte(length), seed}
	}
	cat := func(parts ...[]byte) []byte { return bytes.Join(parts, nil) }
	frame := func(i int) []byte { return []byte{recipeFrame, byte(i)} }
	last := len(accepted) - 1
	seeds = append(seeds,
		cat(text(5, 'a'), frame(0), text(0, 0), frame(last), text(40, 'Z')),
		cat(frame(0), frame(last), text(maxTextBytes, '!'), frame(1)),
		cat(text(maxTextBytes, 0), text(maxTextBytes, 1), frame(2)),
	)
	return seeds
}

func FuzzSplitInterleaved(f *testing.F) {
	accepted := acceptedGoldenFrames(f)
	for _, s := range interleavedSeeds(accepted) {
		f.Add(s)
	}
	f.Fuzz(func(t *testing.T, recipe []byte) {
		checkSplitInterleaved(t, recipe, accepted)
	})
}
