# host-tests Specification

## Purpose
Guarantees of the host-side test gate for `core/` (`mise run test`), so that undefined behaviour, memory errors and untested sources fail the gate instead of passing silently.

## Requirements

### Requirement: Host tests run under sanitisers

The native host tests SHALL be compiled and linked with AddressSanitizer and UndefinedBehaviorSanitizer, including the float-to-integer overflow check, with recovery disabled, so that any sanitiser finding aborts the test program and fails `mise run test`. When the compiler cannot link the sanitiser runtimes, `mise run test` SHALL fail if `CI` is `true` or `SPOODER_REQUIRE_SANITIZERS` is `1`, and otherwise SHALL print a warning naming the missing runtimes and run the tests without sanitisers.

#### Scenario: Undefined behaviour in a test

- **WHEN** a host test executes undefined behaviour, such as a signed integer overflow, on a host with the sanitiser runtimes
- **THEN** the test program aborts with the sanitiser's report and `mise run test` fails

#### Scenario: Out-of-bounds read

- **WHEN** a host test reads past the end of an array on a host with the sanitiser runtimes
- **THEN** the test program aborts with AddressSanitizer's report and `mise run test` fails

#### Scenario: Out-of-range float-to-integer conversion

- **WHEN** a host test converts a floating-point value outside the target integer type's range to that type, on a host with the sanitiser runtimes
- **THEN** the test program aborts with the sanitiser's report and `mise run test` fails

#### Scenario: Runtimes missing in CI

- **WHEN** `mise run test` runs with `CI=true` and the compiler cannot link the sanitiser runtimes
- **THEN** it fails with a message that the sanitiser runtimes are missing

#### Scenario: Runtimes missing with the strict variable

- **WHEN** `mise run test` runs outside CI with `SPOODER_REQUIRE_SANITIZERS=1` and the compiler cannot link the sanitiser runtimes
- **THEN** it fails with a message that the sanitiser runtimes are missing

#### Scenario: Runtimes missing locally

- **WHEN** `mise run test` runs outside CI, without `SPOODER_REQUIRE_SANITIZERS=1`, and the compiler cannot link the sanitiser runtimes
- **THEN** it prints a warning naming the runtimes and runs the tests without sanitisers

#### Scenario: Clean suite

- **WHEN** `mise run test` runs on `ubuntu-24.04` in CI
- **THEN** every host test runs sanitised and passes

### Requirement: Host tests compile every core source

`mise run test` SHALL check, before running the host tests, that the source filter of the native test env excludes everything with one leading `-<*>` entry and then includes, one plain `+<…>` entry each, exactly the `.cpp` files under `core/`. It SHALL fail when they differ, naming each source missing from the filter and each filter entry with no file under `core/`, and SHALL fail naming any other entry: a wildcard, a path that is not a plain relative `.cpp` path, or an exclusion other than the leading `-<*>`.

#### Scenario: New source not listed

- **WHEN** a `.cpp` file is added under `core/` without a matching filter entry
- **THEN** `mise run test` fails and names the file as missing from the filter

#### Scenario: Stale entry

- **WHEN** the filter names a `.cpp` file that no longer exists under `core/`
- **THEN** `mise run test` fails and names the entry as not in `core/`

#### Scenario: Entry of another form

- **WHEN** the filter has an entry other than the leading `-<*>` and plain `+<….cpp>` file entries, such as a wildcard or an exclusion of a listed source
- **THEN** `mise run test` fails and names the entry as invalid

#### Scenario: Lists match

- **WHEN** the filter names exactly the `.cpp` files under `core/`
- **THEN** the check passes and the host tests run
