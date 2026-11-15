## MODIFIED Requirements

### Requirement: Fuzzed frame decoder
The host's link frame decoder SHALL have a fuzz test seeded with every frame in `test/golden/protocol_v4.txt`. For any input byte sequence, fed one byte at a time with the decoder polled after each byte and idled at the end, the test SHALL check that:
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
