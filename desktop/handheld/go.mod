module github.com/blackfyre/spooder/desktop/handheld

go 1.26.0

toolchain go1.27.1

require github.com/blackfyre/spooder/desktop v0.0.0

// The calibrator's module, so the handheld's packages may import
// desktop/internal/... (design D1). gabagool and go-sdl2 join in task 4.2.
replace github.com/blackfyre/spooder/desktop => ../
