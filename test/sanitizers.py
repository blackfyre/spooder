"""PlatformIO `pre:` extra script of [env:native]: sanitise the host tests.

Links a one-line probe program with the sanitiser flags, using the compiler
the native platform's builder will select, then follows
sanitizer_policy.decide: add the flags, or warn and build unsanitised, or fail
the build (CI=true or SPOODER_REQUIRE_SANITIZERS=1).

The native builder runs after this script and replaces the compiler: it
deletes CC and CXX and loads SCons's gcc and g++ tools, which set them only
when absent. probe_compiler() repeats that on a clone, so the probe uses the
compiler the tests are built with. Tested by test_sanitizers.py.
"""

import os
import shlex
import shutil
import subprocess
import sys
import tempfile

SANITIZE = "-fsanitize=address,undefined,float-cast-overflow"
# Compile only: abort on the first finding, and keep ASan's traces readable.
COMPILE_ONLY = ["-fno-sanitize-recover=all", "-fno-omit-frame-pointer"]
PROBE_SOURCE = "int main() { return 0; }\n"
PROBE_TIMEOUT_S = 120


def probe_compiler(env):
    """The C++ compiler the native builder selects for env, env left unchanged."""
    clone = env.Clone()
    for key in ("CC", "CXX"):
        if key in clone:
            del clone[key]
    clone.Tool("gcc")
    clone.Tool("g++")
    return clone.subst("$CXX")


def probe_links(compiler, process_env):
    """Return (ok, detail): whether compiler links a sanitised program."""
    with tempfile.TemporaryDirectory(prefix="spooder-sanitizer-probe-") as tmp:
        source = os.path.join(tmp, "probe.cpp")
        with open(source, "w", encoding="utf-8") as fp:
            fp.write(PROBE_SOURCE)
        command = shlex.split(compiler) + [SANITIZE] + COMPILE_ONLY + [source, "-o", os.path.join(tmp, "probe")]
        try:
            result = subprocess.run(
                command, env=process_env, capture_output=True, text=True, timeout=PROBE_TIMEOUT_S, check=False
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return False, str(exc)
        return result.returncode == 0, result.stderr


def apply(env, environ):
    """Probe, decide and apply to env; exits the build on a fatal decision."""
    from sanitizer_policy import FAIL, SANITISE, decide

    compiler = probe_compiler(env)
    process_env = dict(env["ENV"])
    resolved = shutil.which(shlex.split(compiler)[0], path=process_env.get("PATH")) if compiler else None
    print("Sanitisers: probing %s (%s)" % (compiler, resolved or "not found on PATH"))
    ok, detail = probe_links(compiler, process_env)
    decision = decide(ok, environ, compiler, detail)
    if decision.action == SANITISE:
        print(decision.message)
        env.Append(CCFLAGS=[SANITIZE] + COMPILE_ONLY, LINKFLAGS=[SANITIZE])
    elif decision.action == FAIL:
        sys.stderr.write(decision.message + "\n")
        env.Exit(1)
    else:
        sys.stderr.write(decision.message + "\n")


# SCons executes extra scripts with SCons.Script's globals; an import (the
# unit test) runs none of this.
if __name__ == "SCons.Script":
    Import("env")  # noqa: F821 - SCons builtin
    sys.path.insert(0, os.path.join(env.subst("$PROJECT_DIR"), "test"))  # noqa: F821
    if not env.IsCleanTarget():  # noqa: F821
        apply(env, os.environ)  # noqa: F821
