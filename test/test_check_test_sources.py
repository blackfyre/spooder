"""Host test of check_test_sources.py: run with `mise run test`."""

import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parent / "check_test_sources.py"

SOURCES = ("movement/gait.cpp", "protocol/protocol.cpp")

FILTER_VARIABLES = ("PLATFORMIO_BUILD_SRC_FILTER", "PLATFORMIO_SRC_FILTER")


def process_env(extra=None):
    """This process's environment without PlatformIO's filter variables, plus extra."""
    env = {k: v for k, v in os.environ.items() if k not in FILTER_VARIABLES}
    env.update(extra or {})
    return env


class CheckTestSourcesTest(unittest.TestCase):
    def run_check(self, entries, sources=SOURCES, environ=None):
        """Run the script on a crafted tree; return (exit status, output)."""
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            for src in sources:
                path = root / "core" / src
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("// source\n")
            (root / "core" / "movement" / "gait.h").write_text("// header, not a source\n")
            filter_lines = "".join("    %s\n" % e for e in entries)
            (root / "platformio.ini").write_text(
                "[platformio]\nsrc_dir = core\n\n"
                "[env:native]\nplatform = native\n"
                "build_src_filter =\n" + filter_lines
            )
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(root)],
                capture_output=True,
                text=True,
                check=False,
                env=process_env(environ),
            )
        return result.returncode, result.stdout + result.stderr

    def assert_fails_naming(self, entries, line, sources=SOURCES):
        status, output = self.run_check(entries, sources)
        self.assertEqual(status, 1, output)
        self.assertIn(line, output)

    def test_matching_lists_pass(self):
        status, output = self.run_check(["-<*>", "+<movement/gait.cpp>", "+<protocol/protocol.cpp>"])
        self.assertEqual(status, 0, output)
        self.assertIn("lists every core/ source", output)

    def test_missing_source_fails(self):
        self.assert_fails_naming(
            ["-<*>", "+<movement/gait.cpp>"], "missing from build_src_filter: protocol/protocol.cpp"
        )

    def test_stale_entry_fails(self):
        self.assert_fails_naming(
            ["-<*>", "+<movement/gait.cpp>", "+<protocol/protocol.cpp>", "+<status/gone.cpp>"],
            "not in core/: status/gone.cpp",
        )

    def test_wildcard_entry_fails(self):
        self.assert_fails_naming(["-<*>", "+<movement/*.cpp>", "+<protocol/protocol.cpp>"], "invalid entry: +<movement/*.cpp>")

    def test_additional_exclusion_fails(self):
        self.assert_fails_naming(
            ["-<*>", "+<movement/gait.cpp>", "+<protocol/protocol.cpp>", "-<movement/gait.cpp>"],
            "invalid entry: -<movement/gait.cpp>",
        )

    def test_missing_leading_exclude_all_fails(self):
        self.assert_fails_naming(["+<movement/gait.cpp>", "+<protocol/protocol.cpp>"], "invalid entry: (no leading -<*>)")

    def test_exclude_all_not_first_fails(self):
        self.assert_fails_naming(["+<movement/gait.cpp>", "-<*>", "+<protocol/protocol.cpp>"], "invalid entry: -<*>")

    def test_non_plain_paths_fail(self):
        for entry in ("+<../core/movement/gait.cpp>", "+<./movement/gait.cpp>", "+<movement/gait.h>", "+<movement>"):
            with self.subTest(entry=entry):
                self.assert_fails_naming(["-<*>", "+<protocol/protocol.cpp>", entry], "invalid entry: " + entry)

    def test_duplicate_entry_fails(self):
        self.assert_fails_naming(
            ["-<*>", "+<movement/gait.cpp>", "+<protocol/protocol.cpp>", "+<movement/gait.cpp>"],
            "invalid entry: +<movement/gait.cpp>",
        )

    def test_environment_filter_is_checked(self):
        # PlatformIO appends these variables to the configured filter.
        listed = ["-<*>", "+<movement/gait.cpp>", "+<protocol/protocol.cpp>"]
        for variable in FILTER_VARIABLES:
            with self.subTest(variable=variable):
                status, output = self.run_check(listed, environ={variable: "-<*> +<movement/gait.cpp>"})
                self.assertEqual(status, 1, output)
                self.assertIn("invalid entry: -<*>", output)
                self.assertIn("invalid entry: +<movement/gait.cpp>", output)

    def test_environment_comments_are_ignored(self):
        # PlatformIO drops comment lines and ` ;` inline comments from the appended value.
        listed = ["-<*>", "+<movement/gait.cpp>", "+<protocol/protocol.cpp>"]
        for value in ("# comment", "; comment", "+<movement/gait.cpp> ; comment\n# more"):
            with self.subTest(value=value):
                status, output = self.run_check(listed, environ={"PLATFORMIO_BUILD_SRC_FILTER": value})
                if value.startswith("+"):
                    self.assertEqual(status, 1, output)  # the duplicate entry, not the comment
                    self.assertIn("invalid entry: +<movement/gait.cpp>", output)
                    self.assertNotIn("comment", output)
                else:
                    self.assertEqual(status, 0, output)

    def test_missing_option_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            pathlib.Path(tmp, "platformio.ini").write_text("[env:native]\nplatform = native\n")
            result = subprocess.run(
                [sys.executable, str(SCRIPT), tmp], capture_output=True, text=True, check=False, env=process_env()
            )
        self.assertEqual(result.returncode, 1)
        self.assertIn("has no [env:native] build_src_filter", result.stderr)


if __name__ == "__main__":
    unittest.main()
