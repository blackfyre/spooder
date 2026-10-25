"""Unit tests of sanitizer_policy.decide (run by `mise run test`)."""

import unittest

from sanitizer_policy import FAIL, SANITISE, WARN, decide


class DecideTest(unittest.TestCase):
    def test_probe_links(self):
        for environ in ({}, {"CI": "true"}, {"SPOODER_REQUIRE_SANITIZERS": "1"}):
            with self.subTest(environ=environ):
                self.assertEqual(decide(True, environ, "g++").action, SANITISE)

    def test_missing_locally_warns(self):
        decision = decide(False, {}, "g++", "ld: cannot find /usr/lib64/libasan.so.8.0.0\ncollect2: error: ld returned 1 exit status\n")
        self.assertEqual(decision.action, WARN)
        self.assertIn("libasan libubsan", decision.message)
        self.assertIn("WITHOUT sanitisers", decision.message)
        self.assertIn("g++", decision.message)
        self.assertIn("cannot find /usr/lib64/libasan.so.8.0.0", decision.message)

    def test_missing_with_other_values_warns(self):
        for environ in ({"CI": "false"}, {"CI": ""}, {"SPOODER_REQUIRE_SANITIZERS": "0"}):
            with self.subTest(environ=environ):
                self.assertEqual(decide(False, environ).action, WARN)

    def test_missing_with_strict_variable_fails(self):
        decision = decide(False, {"SPOODER_REQUIRE_SANITIZERS": "1"}, "g++")
        self.assertEqual(decision.action, FAIL)
        self.assertIn("sanitiser runtimes are missing", decision.message)
        self.assertIn("libasan libubsan", decision.message)
        self.assertIn("SPOODER_REQUIRE_SANITIZERS=1", decision.message)

    def test_missing_in_ci_fails(self):
        decision = decide(False, {"CI": "true"}, "g++")
        self.assertEqual(decision.action, FAIL)
        self.assertIn("sanitiser runtimes are missing", decision.message)
        self.assertIn("libasan libubsan", decision.message)
        self.assertIn("CI=true", decision.message)


if __name__ == "__main__":
    unittest.main()
