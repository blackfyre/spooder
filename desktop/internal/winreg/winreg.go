// Package winreg abstracts the Windows registry operations bopen needs, so
// that registry logic can be tested on any platform.
//
// Copied unchanged from bopen (github.com/blackfyre/bopen, internal/winreg),
// MIT licence, Copyright (c) 2026 The bopen authors; see desktop/LICENSE.bopen.
// The calibrator uses it only through internal/appearance on Windows.
package winreg

import (
	"errors"
	"sort"
	"strings"
)

// Hive is a registry root key.
type Hive int

const (
	CurrentUser Hive = iota
	LocalMachine
)

// ErrNotExist is returned for missing keys and values.
var ErrNotExist = errors.New("registry key or value does not exist")

// Registry is the subset of registry operations bopen uses. Paths use
// backslashes; the value name "" is the key's default value.
type Registry interface {
	SubKeys(hive Hive, path string) ([]string, error)
	String(hive Hive, path, name string) (string, error)
	Integer(hive Hive, path, name string) (uint64, error)
	SetString(hive Hive, path, name, value string) error
	DeleteValue(hive Hive, path, name string) error
	DeleteTree(hive Hive, path string) error
}

// Fake is an in-memory Registry with case-insensitive key and value names.
type Fake struct {
	keys map[Hive]map[string]map[string]string
	ints map[Hive]map[string]uint64
	// names keeps the original spelling of each key's last path element.
	names map[Hive]map[string]string
}

// NewFake returns an empty in-memory registry.
func NewFake() *Fake {
	return &Fake{keys: map[Hive]map[string]map[string]string{}, names: map[Hive]map[string]string{}}
}

func norm(s string) string { return strings.ToLower(strings.Trim(s, `\`)) }

func (f *Fake) hive(h Hive) map[string]map[string]string {
	if f.keys[h] == nil {
		f.keys[h] = map[string]map[string]string{}
	}
	return f.keys[h]
}

func (f *Fake) SubKeys(h Hive, path string) ([]string, error) {
	prefix := norm(path) + `\`
	names := map[string]string{}
	found := false
	for k := range f.hive(h) {
		if k == norm(path) {
			found = true
		}
		if rest, ok := strings.CutPrefix(k, prefix); ok {
			found = true
			first, _, _ := strings.Cut(rest, `\`)
			orig := f.names[h][prefix+first]
			if orig == "" {
				orig = first
			}
			names[first] = orig
		}
	}
	if !found {
		return nil, ErrNotExist
	}
	out := make([]string, 0, len(names))
	for _, v := range names {
		out = append(out, v)
	}
	sort.Strings(out)
	return out, nil
}

func (f *Fake) String(h Hive, path, name string) (string, error) {
	vals, ok := f.hive(h)[norm(path)]
	if !ok {
		return "", ErrNotExist
	}
	v, ok := vals[strings.ToLower(name)]
	if !ok {
		return "", ErrNotExist
	}
	return v, nil
}

func (f *Fake) SetString(h Hive, path, name, value string) error {
	// Create every ancestor so SubKeys sees intermediate keys.
	parts := strings.Split(strings.Trim(path, `\`), `\`)
	for i := range parts {
		p := strings.Join(parts[:i+1], `\`)
		if f.hive(h)[norm(p)] == nil {
			f.hive(h)[norm(p)] = map[string]string{}
		}
		if f.names[h] == nil {
			f.names[h] = map[string]string{}
		}
		f.names[h][norm(p)] = parts[i]
	}
	f.hive(h)[norm(path)][strings.ToLower(name)] = value
	return nil
}

func (f *Fake) DeleteValue(h Hive, path, name string) error {
	vals, ok := f.hive(h)[norm(path)]
	if !ok {
		return ErrNotExist
	}
	if _, ok := vals[strings.ToLower(name)]; !ok {
		return ErrNotExist
	}
	delete(vals, strings.ToLower(name))
	return nil
}

func (f *Fake) DeleteTree(h Hive, path string) error {
	p := norm(path)
	found := false
	for k := range f.hive(h) {
		if k == p || strings.HasPrefix(k, p+`\`) {
			delete(f.hive(h), k)
			found = true
		}
	}
	if !found {
		return ErrNotExist
	}
	return nil
}

// SetInteger stores a DWORD/QWORD value, creating the key.
func (f *Fake) SetInteger(h Hive, path, name string, value uint64) {
	if f.ints == nil {
		f.ints = map[Hive]map[string]uint64{}
	}
	if f.ints[h] == nil {
		f.ints[h] = map[string]uint64{}
	}
	f.ints[h][norm(path)+`\`+strings.ToLower(name)] = value
}

func (f *Fake) Integer(h Hive, path, name string) (uint64, error) {
	v, ok := f.ints[h][norm(path)+`\`+strings.ToLower(name)]
	if !ok {
		return 0, ErrNotExist
	}
	return v, nil
}

// Exists reports whether the key exists.
func (f *Fake) Exists(h Hive, path string) bool {
	_, ok := f.hive(h)[norm(path)]
	return ok
}
