# test/

Unity host tests of `core/`, one directory per area (`mise run test`, the PlatformIO native env in the root `platformio.ini`).

`golden/` holds the golden files other implementations (the desktop calibrator) test against; `protocol_v3.txt` also seeds the desktop's link fuzz tests (`desktop/internal/protocol/fuzz_test.go`, `desktop/internal/link/fuzz_test.go`), which name it by path as `desktop/internal/protocol/golden_test.go` does, so a renamed file is renamed there too. They are fixed fixtures:
- never copy a calibration export over `calibration_defaults.cpp` there; it pins the format only;
- regenerate `protocol_v3.txt`, `leg_pose.txt` and `centre_of_gravity.txt` only for a deliberate protocol, kinematics or mass-model change (`SPOODER_WRITE_GOLDEN=1 mise run test`), and review the diff.
