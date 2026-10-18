package export

import (
	"os"
	"path/filepath"
	"strings"
)

// lastDirFile holds the last checkout chosen, under the user config
// directory.
var lastDirFile = filepath.Join("spooder-calibrator", "last-checkout")

// LoadLastDir returns the last checkout chosen, or "" if none is
// remembered.
func LoadLastDir(configDir string) string {
	b, err := os.ReadFile(filepath.Join(configDir, lastDirFile))
	if err != nil {
		return ""
	}
	return strings.TrimSpace(string(b))
}

// SaveLastDir remembers dir as the last checkout chosen.
func SaveLastDir(configDir, dir string) error {
	path := filepath.Join(configDir, lastDirFile)
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, []byte(dir+"\n"), 0o644)
}
