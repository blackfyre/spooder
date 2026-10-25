"""Host test of image_budget.py: run with `mise run test-robot-build`."""

import unittest

import image_budget

KIB = 1024
FLASH_FLAGS = "CONTENTS, ALLOC, LOAD, READONLY, CODE"


def section(index, name, size, vma, lma, flags):
    return "%3d %-13s %08x  %08x  %08x  %08x  2**2\n                  %s\n" % (index, name, size, vma, lma, 0x1000, flags)


def objdump(*sections):
    header = "\nspooder.elf:     file format elf32-littlearm\n\nSections:\nIdx Name          Size      VMA       LMA       File off  Algn\n"
    return header + "".join(section(i, *s) for i, s in enumerate(sections))


def image(text_size, bss_size, data_size=0):
    """.text in flash, optional .data (run in RAM, loaded from flash), .bss in RAM, plus debug noise."""
    rows = [(".text", text_size, 0x10000100, 0x10000100, FLASH_FLAGS)]
    if data_size:
        rows.append((".data", data_size, 0x200000C0, 0x10000100 + text_size, "CONTENTS, ALLOC, LOAD, DATA"))
    rows.append((".bss", bss_size, 0x20004000, 0x10080000, "ALLOC"))
    rows.append((".ARM.attributes", 0x28, 0, 0, "CONTENTS, READONLY"))
    rows.append((".debug_info", 0x93C46, 0, 0, "CONTENTS, READONLY, DEBUGGING, OCTETS"))
    return objdump(*rows)


class ImageBudgetTest(unittest.TestCase):
    def test_both_fit_report_use_lines(self):
        self.assertEqual(
            image_budget.check(image(145724, 42512)),
            ["flash 142.3 KiB of 256 KiB budget, 44 % free", "RAM 41.5 KiB of 64 KiB budget, 35 % free"],
        )

    def test_use_exactly_at_flash_budget_passes_with_zero_free(self):
        lines = image_budget.check(image(image_budget.FLASH_BUDGET, 1000))
        self.assertEqual(lines[0], "flash 256.0 KiB of 256 KiB budget, 0 % free")

    def test_use_exactly_at_ram_budget_passes_with_zero_free(self):
        lines = image_budget.check(image(1000, image_budget.RAM_BUDGET))
        self.assertEqual(lines[1], "RAM 64.0 KiB of 64 KiB budget, 0 % free")

    def test_flash_one_byte_over_fails(self):
        with self.assertRaises(image_budget.CheckError) as raised:
            image_budget.check(image(image_budget.FLASH_BUDGET + 1, 1000))
        message = str(raised.exception)
        self.assertIn("flash use 262145 bytes", message)
        self.assertIn("262144-byte (256 KiB) budget by 1 bytes", message)
        self.assertNotIn("RAM", message)

    def test_ram_over_while_flash_fits_fails(self):
        with self.assertRaises(image_budget.CheckError) as raised:
            image_budget.check(image(145724, image_budget.RAM_BUDGET + 100))
        message = str(raised.exception)
        self.assertIn("RAM use 65636 bytes", message)
        self.assertIn("65536-byte (64 KiB) budget by 100 bytes", message)
        self.assertNotIn("flash", message)

    def test_both_over_name_both(self):
        with self.assertRaisesRegex(image_budget.CheckError, "(?s)flash use .*\nRAM use "):
            image_budget.check(image(image_budget.FLASH_BUDGET + 1, image_budget.RAM_BUDGET + 1))

    def test_debug_and_attribute_sections_count_towards_neither(self):
        sections = image_budget.parse_sections(image(100, 200))
        self.assertEqual(image_budget.flash_use(sections), 100)
        self.assertEqual(image_budget.ram_use(sections), 200)

    def test_data_counts_towards_both(self):
        sections = image_budget.parse_sections(image(100, 200, data_size=50))
        self.assertEqual(image_budget.flash_use(sections), 150)
        self.assertEqual(image_budget.ram_use(sections), 250)

    def test_ram_reservations_without_load_do_not_count_in_flash(self):
        # .bss and .heap carry a flash LMA in pico-sdk ELFs but are not loaded.
        sections = image_budget.parse_sections(
            objdump(
                (".text", 100, 0x10000100, 0x10000100, FLASH_FLAGS),
                (".heap", 0x800, 0x20009614, 0x10023940, "ALLOC, READONLY"),
                (".stack_dummy", 0x800, 0x20041000, 0x20041000, "ALLOC, READONLY"),
            )
        )
        self.assertEqual(image_budget.flash_use(sections), 100)
        self.assertEqual(image_budget.ram_use(sections), 0x1000)

    def test_contents_outside_alloc_count_towards_neither(self):
        sections = image_budget.parse_sections(
            objdump(
                (".text", 100, 0x10000100, 0x10000100, FLASH_FLAGS),
                (".scratch_x", 0x40, 0x20040000, 0x20040000, "CONTENTS"),
            )
        )
        self.assertEqual(image_budget.flash_use(sections), 100)
        self.assertEqual(image_budget.ram_use(sections), 0)

    def test_output_with_no_sections_fails(self):
        with self.assertRaisesRegex(image_budget.CheckError, "no sections"):
            image_budget.check("\nspooder.elf:     file format elf32-littlearm\n\nSections:\n")

    def test_empty_output_fails(self):
        with self.assertRaisesRegex(image_budget.CheckError, "no sections"):
            image_budget.check("")


if __name__ == "__main__":
    unittest.main()
