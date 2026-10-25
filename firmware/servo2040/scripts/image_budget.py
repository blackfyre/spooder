"""Check the robot image's flash and static RAM use against named budgets.

Plain Python with no toolchain imports, so the host test can run it on crafted
`objdump -h` text. check_image_budget.py runs it on the built ELF.
"""

import re

KIB = 1024

# Raising a budget is a reviewed one-line change, recorded in CHANGELOG.md.
FLASH_BUDGET = 256 * KIB
RAM_BUDGET = 64 * KIB

# The Servo 2040's RP2040: 2 MiB of XIP flash and 264 KiB of SRAM (striped
# banks plus scratch X and Y), as inclusive address ranges.
FLASH_START, FLASH_END = 0x10000000, 0x101FFFFF
SRAM_START, SRAM_END = 0x20000000, 0x20041FFF

# "  8 .data  00002604  200000c0  1002133c  000230c0  2**4", then a flags line.
_HEADER = re.compile(
    r"^\s*\d+\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+2\*\*\d+\s*$"
)


class CheckError(Exception):
    """The image cannot be measured or exceeds a budget; the message says why."""


def parse_sections(text):
    """Return [(name, size, vma, lma, flags)] from `objdump -h` output.

    Raises CheckError when no section is found, so that a changed output
    format fails loudly instead of measuring zero use.
    """
    sections = []
    lines = text.splitlines()
    for i, line in enumerate(lines):
        match = _HEADER.match(line)
        if not match:
            continue
        flags_line = lines[i + 1] if i + 1 < len(lines) else ""
        flags = frozenset(f.strip() for f in flags_line.split(",") if f.strip())
        name, size, vma, lma = match.groups()
        sections.append((name, int(size, 16), int(vma, 16), int(lma, 16), flags))
    if not sections:
        raise CheckError("no sections found in the objdump -h output")
    return sections


def flash_use(sections):
    """Bytes loaded into flash: ALLOC and LOAD sections whose load address is in flash."""
    return sum(
        size
        for _name, size, _vma, lma, flags in sections
        if {"ALLOC", "LOAD"} <= flags and FLASH_START <= lma <= FLASH_END
    )


def ram_use(sections):
    """Bytes of SRAM the linker reserves: ALLOC sections whose run address is in SRAM."""
    return sum(
        size for _name, size, vma, _lma, flags in sections if "ALLOC" in flags and SRAM_START <= vma <= SRAM_END
    )


def kib(size):
    """One decimal, except a whole number of KiB (the budgets) printed without one."""
    if size % KIB == 0:
        return "%d KiB" % (size // KIB)
    return "%.1f KiB" % (size / KIB)


def _use_kib(size):
    return "%.1f KiB" % (size / KIB)


def check(objdump_text, flash_budget=FLASH_BUDGET, ram_budget=RAM_BUDGET):
    """Return the two use lines, or raise CheckError naming every budget exceeded."""
    sections = parse_sections(objdump_text)
    lines, errors = [], []
    for label, use, budget in (("flash", flash_use(sections), flash_budget), ("RAM", ram_use(sections), ram_budget)):
        if use > budget:
            errors.append(
                "%s use %d bytes (%s) exceeds its %d-byte (%s) budget by %d bytes"
                % (label, use, _use_kib(use), budget, kib(budget), use - budget)
            )
        else:
            free = (budget - use) * 100 // budget
            lines.append("%s %s of %s budget, %d %% free" % (label, _use_kib(use), kib(budget), free))
    if errors:
        raise CheckError("\n".join(errors))
    return lines
