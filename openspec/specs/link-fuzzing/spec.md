# link-fuzzing Specification

## Purpose
Fuzz testing of the host-side parsers that read the robot's USB byte stream, so malformed input cannot crash the reader, grow its memory, or mix debug text with link frames.

## Requirements

### Requirement: Fuzzed frame decoder
The host's link frame decoder SHALL have a fuzz test seeded with every frame in `test/golden/protocol_v3.txt`. For any input byte sequence, fed one byte at a time with the decoder polled after each byte and idled at the end, the test SHALL check that:
- the decoder does not panic;
- the decoder never holds more than one maximum-size frame of undecided bytes;
- every input byte is accounted for exactly once and in order, either inside an accepted frame or as a discarded byte;
- every accepted frame's own input bytes, decoded alone, give the same frame, and its re-encoding decodes to the same frame;
- an accepted frame whose input bytes are canonical (reserved and unused payload bytes zero, boolean fields 0 or 1) re-encodes to exactly those bytes.

#### Scenario: Golden frames decode unchanged
- **WHEN** the fuzz test runs a golden frame from its seed corpus
- **THEN** the frame is accepted and re-encodes to the golden bytes, and a golden line marked rejected yields no frame and only discarded bytes

#### Scenario: Non-canonical frame
- **WHEN** the fuzz test runs a frame with a valid CRC and a non-zero reserved or unused payload byte, or a boolean field other than 0 or 1
- **THEN** the frame is accepted, its re-encoding has those bytes zero and the boolean 1, and the re-encoding decodes to the same frame

#### Scenario: Arbitrary bytes
- **WHEN** the fuzz test runs an arbitrary byte sequence
- **THEN** it completes without a panic, the decoder's held bytes never exceed one maximum-size frame, and the accepted frames and discarded bytes together account for every input byte

### Requirement: Fuzzed text and frame splitting
The host's split of the USB stream into debug text lines and link frames SHALL have a fuzz test seeded from the golden frames. The test SHALL check that:
- for any input, the split does not panic, hands over the same frames as the decoder alone, and returns the text lines that the documented line rules give for the decoder's discarded bytes (a line ends at a newline or after the maximum line length of received bytes, carriage returns count but are not shown, other non-printable bytes are shown escaped), so no text line holds more than the maximum line length of received bytes and no rendered line is longer than four characters per received byte;
- for printable text interleaved with golden frames, the split returns exactly those frames in order and exactly that text, so no text byte becomes part of a frame and no frame byte appears in the text.

#### Scenario: Text interleaved with frames
- **WHEN** the fuzz test builds a stream of printable text lines (never containing the start marker) with accepted golden frames between them
- **THEN** the frames returned are the golden frames in order, and the text lines returned are the text lines written, one for one and in order

#### Scenario: Arbitrary bytes
- **WHEN** the fuzz test runs an arbitrary byte sequence through the split
- **THEN** it completes without a panic, the frames returned are those the decoder alone accepts, and the text lines returned are exactly those the line rules give for the discarded bytes, each holding at most the maximum line length of received bytes and rendered in at most four characters per received byte

#### Scenario: Line length boundary
- **WHEN** the split reads a run of one less than, exactly, and one more than the maximum line length of printable bytes, of carriage returns, or of non-printable bytes
- **THEN** a line is cut after exactly the maximum line length of received bytes, carriage returns included, and the rest starts the next line

### Requirement: Bounded fuzz runs
The repository SHALL provide one command that runs every host fuzz test with random inputs for a bounded time per test, defaulting to 60 seconds and configurable. The seed corpus and every saved failing input SHALL run as ordinary deterministic tests in the desktop test gate, so the fuzz tests run on every verification without random inputs.

#### Scenario: Bounded run
- **WHEN** a developer runs the fuzz command with a time of 10 seconds
- **THEN** each fuzz test runs for about 10 seconds and the command exits non-zero if any test finds a failing input

#### Scenario: Deterministic gate
- **WHEN** the desktop test gate runs
- **THEN** each fuzz test runs its seed corpus and saved failing inputs once, with no random inputs

#### Scenario: Found bug kept as a regression
- **WHEN** a fuzz run finds a failing input and the bug is fixed
- **THEN** the failing input is kept in the test's corpus and passes in the desktop test gate
