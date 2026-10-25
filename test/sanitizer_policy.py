"""Decide whether the host tests run sanitised, warn, or fail.

The native env's extra script (sanitizers.py) links a probe program with the
sanitiser flags, then asks decide() what to do with the result: sanitise when
the probe links; otherwise fail under CI (`CI=true`) or with
`SPOODER_REQUIRE_SANITIZERS=1`, and warn and build unsanitised everywhere else.
Plain Python, importable without SCons; tested by test_sanitizer_policy.py.
"""

import collections

SANITISE = "sanitise"
WARN = "warn"
FAIL = "fail"

# The runtimes GCC links for -fsanitize=address,undefined (Fedora package
# names; on Debian and Ubuntu they come with GCC).
RUNTIMES = "libasan libubsan"
STRICT_VARIABLE = "SPOODER_REQUIRE_SANITIZERS"

Decision = collections.namedtuple("Decision", "action message")


def strict(environ):
    """True when a failed probe must fail the build: CI, or the strict variable."""
    return environ.get("CI", "").strip().lower() == "true" or environ.get(STRICT_VARIABLE, "").strip() == "1"


def decide(probe_ok, environ, compiler="the compiler", detail=""):
    """Return the Decision for a probe result and a process environment.

    compiler names the probed compiler and detail is the probe's error output,
    both only for the message.
    """
    if probe_ok:
        return Decision(SANITISE, "Sanitisers: %s links AddressSanitizer and UndefinedBehaviorSanitizer" % compiler)
    reason = "%s cannot link the sanitiser runtimes (%s)" % (compiler, RUNTIMES)
    lines = [line.strip() for line in detail.splitlines() if line.strip()]
    if lines:
        # The linker names the missing library; collect2's summary comes last.
        reason += " (%s)" % next((line for line in lines if "cannot find" in line), lines[-1])
    if strict(environ):
        why = "CI=true" if environ.get("CI", "").strip().lower() == "true" else "%s=1" % STRICT_VARIABLE
        return Decision(
            FAIL,
            "Sanitisers FAILED: the sanitiser runtimes are missing; %s. They are required with %s: "
            "install %s (Fedora; on Debian and Ubuntu they come with GCC)." % (reason, why, RUNTIMES),
        )
    return Decision(
        WARN,
        "WARNING: the sanitiser runtimes are missing; %s. The host tests run WITHOUT sanitisers. "
        "Install %s (Fedora; on Debian and Ubuntu they come with GCC), or set %s=1 to make this fatal."
        % (reason, RUNTIMES, STRICT_VARIABLE),
    )
