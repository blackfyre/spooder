"""Fail the robot build when its image exceeds the flash or static RAM budget.

Usage: check_image_budget.py <elf>. Runs `arm-none-eabi-objdump -h` on the ELF
and prints one use line per budget, or the overflow on stderr with exit 1.
`mise run build` runs it after every build. The logic lives in image_budget.py
(host-tested by test_image_budget.py).
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import image_budget  # noqa: E402

OBJDUMP = "arm-none-eabi-objdump"


def main(argv):
    if len(argv) != 2:
        sys.stderr.write("usage: check_image_budget.py <elf>\n")
        return 2
    elf = argv[1]
    try:
        result = subprocess.run([OBJDUMP, "-h", elf], capture_output=True, text=True)
        if result.returncode != 0:
            raise image_budget.CheckError("%s -h %s failed:\n%s" % (OBJDUMP, elf, result.stderr.strip()))
        lines = image_budget.check(result.stdout)
    except (image_budget.CheckError, OSError) as exc:
        sys.stderr.write("\nRobot image budget check FAILED (%s):\n%s\n\n" % (elf, exc))
        return 1
    for line in lines:
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
