package gestures

import (
	"os"
	"path/filepath"
)

// GoldenPath is test/golden/gestures.txt relative to this package's
// directory (where go generate and go test run), or the copy
// SPOODER_GESTURES_GOLDEN names, as for the C++ test.
func GoldenPath() string {
	if p := os.Getenv("SPOODER_GESTURES_GOLDEN"); p != "" {
		return p
	}
	return filepath.Join("..", "..", "..", "..", "test", "golden", "gestures.txt")
}
