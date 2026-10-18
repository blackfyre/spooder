//go:build windows

package winreg

import (
	"errors"

	"golang.org/x/sys/windows/registry"
)

// System is the real Windows registry.
type System struct{}

func root(h Hive) registry.Key {
	if h == LocalMachine {
		return registry.LOCAL_MACHINE
	}
	return registry.CURRENT_USER
}

func mapErr(err error) error {
	if errors.Is(err, registry.ErrNotExist) {
		return ErrNotExist
	}
	return err
}

func (System) SubKeys(h Hive, path string) ([]string, error) {
	k, err := registry.OpenKey(root(h), path, registry.ENUMERATE_SUB_KEYS)
	if err != nil {
		return nil, mapErr(err)
	}
	defer k.Close()
	names, err := k.ReadSubKeyNames(-1)
	return names, mapErr(err)
}

func (System) String(h Hive, path, name string) (string, error) {
	k, err := registry.OpenKey(root(h), path, registry.QUERY_VALUE)
	if err != nil {
		return "", mapErr(err)
	}
	defer k.Close()
	v, _, err := k.GetStringValue(name)
	if err != nil {
		return "", mapErr(err)
	}
	if expanded, err := registry.ExpandString(v); err == nil {
		v = expanded
	}
	return v, nil
}

func (System) Integer(h Hive, path, name string) (uint64, error) {
	k, err := registry.OpenKey(root(h), path, registry.QUERY_VALUE)
	if err != nil {
		return 0, mapErr(err)
	}
	defer k.Close()
	v, _, err := k.GetIntegerValue(name)
	return v, mapErr(err)
}

func (System) SetString(h Hive, path, name, value string) error {
	k, _, err := registry.CreateKey(root(h), path, registry.SET_VALUE)
	if err != nil {
		return err
	}
	defer k.Close()
	return k.SetStringValue(name, value)
}

func (System) DeleteValue(h Hive, path, name string) error {
	k, err := registry.OpenKey(root(h), path, registry.SET_VALUE)
	if err != nil {
		return mapErr(err)
	}
	defer k.Close()
	return mapErr(k.DeleteValue(name))
}

func (s System) DeleteTree(h Hive, path string) error {
	subs, err := s.SubKeys(h, path)
	if err != nil {
		return err
	}
	for _, sub := range subs {
		if err := s.DeleteTree(h, path+`\`+sub); err != nil && !errors.Is(err, ErrNotExist) {
			return err
		}
	}
	return mapErr(registry.DeleteKey(root(h), path))
}
