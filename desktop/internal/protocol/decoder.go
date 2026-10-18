package protocol

// Decoder is the incremental decoder of core/protocol (protocol::Decoder),
// with the same rules. Feed every received byte with Push, and after each
// one call Next until it returns false: the buffer holds a single frame of
// at most MaxFrameSize bytes, so bytes pushed beyond that without Next give
// up the oldest candidate. A rejected candidate (bad version, type, length
// or CRC) is counted, and decoding resumes at the next start marker after
// its first byte, so a valid frame following garbage is never lost. An
// incomplete candidate is kept until its CRC can be checked, unless the line
// goes quiet: call Idle when no byte has arrived for longer than a frame
// takes to send, so a false start cannot hide a complete frame behind it.
//
// Discard, when set, receives every byte that is not part of a decoded
// frame, in stream order: bytes between frames and the start marker of each
// rejected candidate. A host sharing the stream with text uses it to recover
// the text.
type Decoder struct {
	Discard func(b byte)

	buf       [MaxFrameSize]byte
	n         int
	valid     uint32
	discarded uint32
}

type candidate int

const (
	candidateInvalid candidate = iota
	candidateIncomplete
	candidateValid
)

// Valid is the number of frames decoded.
func (d *Decoder) Valid() uint32 { return d.valid }

// Discarded is the number of candidates rejected.
func (d *Decoder) Discarded() uint32 { return d.discarded }

// Pending is the number of bytes held as an undecided candidate.
func (d *Decoder) Pending() int { return d.n }

func (d *Decoder) discardByte(b byte) {
	if d.Discard != nil {
		d.Discard(b)
	}
}

// Push adds one received byte.
func (d *Decoder) Push(b byte) {
	// Next keeps the buffer shorter than a frame; if the caller skipped it,
	// make room by giving up the oldest candidate.
	if d.n == MaxFrameSize {
		d.reject()
	}
	if d.n == 0 && b != Start {
		d.discardByte(b) // between frames
		return
	}
	d.buf[d.n] = b
	d.n++
}

func (d *Decoder) drop(count int) {
	copy(d.buf[:], d.buf[count:d.n])
	d.n -= count
}

func (d *Decoder) reject() {
	d.discarded++
	d.discardByte(d.buf[0])
	d.drop(1) // resume the search just after this candidate's start marker
}

// check classifies the buffered candidate: invalid, incomplete, or a
// complete CRC-valid frame of total bytes.
func (d *Decoder) check() (c candidate, total int) {
	b := d.buf[:d.n]
	if b[0] != Start {
		return candidateInvalid, 0
	}
	if len(b) >= 2 && b[1] != Version {
		return candidateInvalid, 0
	}
	n := 0
	if len(b) >= 3 {
		n = PayloadSize(FrameType(b[2]))
	}
	if n < 0 || (len(b) >= 5 && int(b[4]) != n) {
		return candidateInvalid, 0
	}
	total = HeaderSize + n + CRCSize
	if len(b) < HeaderSize || len(b) < total {
		return candidateIncomplete, total
	}
	if CRC16(b[1:HeaderSize+n]) != get16(b[HeaderSize+n:]) {
		return candidateInvalid, 0
	}
	return candidateValid, total
}

func (d *Decoder) emit(total int) Frame {
	f := Frame{Type: FrameType(d.buf[2]), Seq: d.buf[3]}
	p := d.buf[HeaderSize:total]
	switch f.Type {
	case TypeCommand:
		f.Command = parseCommand(p)
	case TypeTelemetry:
		f.Telemetry = parseTelemetry(p)
	case TypeCalibrationSet:
		f.CalibrationSet = parseCalibrationSet(p)
	case TypeCalibrationReport:
		f.CalibrationReport = parseCalibrationReport(p)
	case TypePose:
		f.Pose = parsePose(p)
	}
	d.valid++
	d.drop(total)
	return f
}

// Idle abandons an incomplete candidate after the line has gone quiet; the
// next call to Next rescans the bytes after its start marker.
func (d *Decoder) Idle() {
	if d.n > 0 {
		if c, _ := d.check(); c == candidateIncomplete {
			d.reject()
		}
	}
}

// Next returns the next decoded frame, if one is complete.
func (d *Decoder) Next() (Frame, bool) {
	for d.n > 0 {
		c, total := d.check()
		switch c {
		case candidateValid:
			return d.emit(total), true
		case candidateInvalid:
			if d.buf[0] == Start {
				d.reject()
			} else {
				d.discardByte(d.buf[0])
				d.drop(1)
			}
			continue
		}
		// Incomplete: keep the earliest candidate until its CRC can be
		// checked, or until Idle abandons it.
		return Frame{}, false
	}
	return Frame{}, false
}
