# test/

Unity host tests of `core/`, one directory per area (`mise run test`, the PlatformIO native env in the root `platformio.ini`).

`mise run test` runs, in order:
1. the Python unit tests of the two scripts below (`test_check_test_sources`, `test_sanitizer_policy`), then the source-list check, `check_test_sources.py`;
2. the Unity tests (`pio test -e native`);
3. `test_sanitizers`, the sanitiser script's compiler selection against PlatformIO's own SCons (installed by step 2; the test fails, not skips, without it);
4. the bridge build's image-size check test (`firmware/bridge/scripts/test_image_size.py`).

**Source-list check.** `[env:native]`'s `build_src_filter` must be `-<*>` and then one plain `+<…>` entry per `.cpp` file under `core/`, nothing else. The check fails naming each source missing from the filter, each entry with no file under `core/` and each entry of another form (a wildcard, `..`, a second exclusion). It checks the filter PlatformIO uses, so entries from `PLATFORMIO_BUILD_SRC_FILTER` (or the old `PLATFORMIO_SRC_FILTER`), which PlatformIO appends, are checked too. A new `core/` `.cpp` is added there by hand.

**Sanitisers.** `sanitizers.py`, a `pre:` extra script of `[env:native]`, builds every test with `-fsanitize=address,undefined,float-cast-overflow` (compile and link) and `-fno-sanitize-recover=all -fno-omit-frame-pointer`, so the first finding of AddressSanitizer or UndefinedBehaviorSanitizer aborts the test program and fails the run. It first links a probe program with the compiler the native builder selects, then follows `sanitizer_policy.py`:
- the probe links: sanitised;
- it fails under CI (`CI=true`, any case) or with `SPOODER_REQUIRE_SANITIZERS=1`: the build fails, naming the missing runtimes;
- it fails otherwise: a warning naming `libasan libubsan` (Fedora packages; Debian and Ubuntu ship them with GCC), and the tests run **unsanitised**. CI always runs sanitised; install the packages to match it locally.

`pio test -e native -vvv` shows the probed compiler and the flags on every compile and link line. A sanitiser finding is fixed at its cause with a regression test, not suppressed; one in test code is fixed in the test.

`golden/` holds the golden files other implementations (the desktop calibrator) test against; `protocol_v3.txt` also seeds the desktop's link fuzz tests (`desktop/internal/protocol/fuzz_test.go`, `desktop/internal/link/fuzz_test.go`), which name it by path as `desktop/internal/protocol/golden_test.go` does, so a renamed file is renamed there too. They are fixed fixtures:
- never copy a calibration export over `calibration_defaults.cpp` there; it pins the format only;
- regenerate `protocol_v3.txt`, `leg_pose.txt` and `centre_of_gravity.txt` only for a deliberate protocol, kinematics or mass-model change (`SPOODER_WRITE_GOLDEN=1 mise run test`), and review the diff.
