package feedback

import (
	"errors"
	"io/fs"
	"os"
	"path/filepath"
)

// SettingFile is the file whose presence keeps the sound off between runs,
// in the pak's directory (beside the binary).
const SettingFile = "sound-off"

// Setting is the sound toggle kept between runs: off while File exists.
type Setting struct{ File string }

// SettingIn is the setting kept in dir.
func SettingIn(dir string) Setting { return Setting{filepath.Join(dir, SettingFile)} }

// On is whether the sound is on: true unless the file exists.
func (s Setting) On() bool {
	_, err := os.Stat(s.File)
	return errors.Is(err, fs.ErrNotExist)
}

// Save keeps on: it removes the file, or creates it empty.
func (s Setting) Save(on bool) error {
	if on {
		if err := os.Remove(s.File); err != nil && !errors.Is(err, fs.ErrNotExist) {
			return err
		}
		return nil
	}
	return os.WriteFile(s.File, nil, 0o644)
}
