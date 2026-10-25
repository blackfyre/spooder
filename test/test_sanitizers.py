"""Unit tests of sanitizers.probe_compiler against real SCons.

SCons comes from PlatformIO's installed tool-scons package
($PLATFORMIO_CORE_DIR, default ~/.platformio). `mise run test` runs this after
`pio test`, which installs it; a missing package fails the test rather than
skipping it.
"""

import os
import pathlib
import sys
import tempfile
import unittest


def platformio_core_dir(environ=os.environ):
    return pathlib.Path(environ.get("PLATFORMIO_CORE_DIR") or pathlib.Path.home() / ".platformio")


def find_scons(core_dir):
    """The scons-local directory of tool-scons under core_dir; raises if absent."""
    found = sorted((pathlib.Path(core_dir) / "packages" / "tool-scons").glob("scons-local-*/SCons/__init__.py"))
    if not found:
        raise FileNotFoundError(
            "PlatformIO's tool-scons package is not installed under %s (run `pio test -e native` first)" % core_dir
        )
    return found[-1].parent.parent


class FindSconsTest(unittest.TestCase):
    def test_missing_package_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileNotFoundError) as caught:
                find_scons(tmp)
            self.assertIn("tool-scons", str(caught.exception))


class SconsTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(find_scons(platformio_core_dir())))
        import SCons.Environment  # noqa: E402 - from the package found above

        cls.Environment = SCons.Environment.Environment

    def native_builder_cxx(self):
        """$CXX after the native builder's sequence, on an environment without a compiler."""
        env = self.Environment(tools=[])
        env.Tool("gcc")
        env.Tool("g++")
        return env.subst("$CXX")


class ProbeCompilerTest(SconsTestCase):
    def test_returns_the_native_builders_compiler(self):
        import sanitizers

        env = self.Environment(tools=["cc", "c++", "link"], CC="cc", CXX="c++")
        expected = self.native_builder_cxx()
        self.assertNotEqual(expected, "c++")
        self.assertEqual(sanitizers.probe_compiler(env), expected)

    def test_leaves_the_environment_unchanged(self):
        import sanitizers

        env = self.Environment(tools=["cc", "c++", "link"], CC="cc", CXX="c++")
        sanitizers.probe_compiler(env)
        self.assertEqual(env["CC"], "cc")
        self.assertEqual(env["CXX"], "c++")


class ApplyTest(SconsTestCase):
    """apply() with the probe's result stubbed."""

    def run_apply(self, probe_ok, environ):
        import sanitizers

        env = self.Environment(tools=["cc", "c++", "link"])
        exits = []
        env.Exit = exits.append
        real = sanitizers.probe_links
        sanitizers.probe_links = lambda compiler, process_env: (probe_ok, "ld: cannot find libasan.so")
        try:
            with open(os.devnull, "w") as null:
                stdout, stderr = sys.stdout, sys.stderr
                sys.stdout = sys.stderr = null
                try:
                    sanitizers.apply(env, environ)
                finally:
                    sys.stdout, sys.stderr = stdout, stderr
        finally:
            sanitizers.probe_links = real
        return env, exits

    def test_sanitise_adds_the_flags(self):
        env, exits = self.run_apply(True, {})
        self.assertEqual(exits, [])
        self.assertEqual(
            list(env["CCFLAGS"])[-3:],
            ["-fsanitize=address,undefined,float-cast-overflow", "-fno-sanitize-recover=all", "-fno-omit-frame-pointer"],
        )
        self.assertEqual(list(env["LINKFLAGS"]), ["-fsanitize=address,undefined,float-cast-overflow"])

    def test_warn_adds_nothing(self):
        env, exits = self.run_apply(False, {})
        self.assertEqual(exits, [])
        self.assertNotIn("-fsanitize=address,undefined,float-cast-overflow", list(env["CCFLAGS"]))
        self.assertEqual(list(env["LINKFLAGS"]), [])

    def test_fail_exits(self):
        for environ in ({"CI": "true"}, {"SPOODER_REQUIRE_SANITIZERS": "1"}):
            with self.subTest(environ=environ):
                _, exits = self.run_apply(False, environ)
                self.assertEqual(exits, [1])


if __name__ == "__main__":
    unittest.main()
