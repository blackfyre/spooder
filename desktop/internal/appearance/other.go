//go:build !linux && !windows

package appearance

import "context"

// Read returns the default appearance on platforms without a reader.
func Read() Settings { return Settings{} }

// Watch does nothing on platforms without change notifications.
func Watch(ctx context.Context, changed func(Settings)) {}
