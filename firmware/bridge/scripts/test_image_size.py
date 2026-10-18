"""Host test of image_size.py: run with `mise run test-bridge-build`."""

import struct
import unittest

import image_size

FACTORY_OFFSET = 0x10000
FACTORY_SIZE = 0x3E0000

CSV = """# Name, Type, SubType, Offset, Size
nvs,       data, nvs,      0x9000,   0x6000
phy_init,  data, phy,      0xf000,   0x1000
factory,   app,  factory,  0x10000,  0x3E0000
coredump,  data, coredump, 0x3F0000, 0x10000
"""


def entry(label, ptype, subtype, offset, size):
    return struct.pack("<2sBBII16sI", b"\xaa\x50", ptype, subtype, offset, size, label.encode(), 0)


def table(*entries, md5=True):
    data = b"".join(entries)
    if md5:
        data += b"\xeb\xeb" + b"\xff" * 14 + bytes(16)
    return (data + b"\xff" * 0xC00)[:0xC00]


GOOD_TABLE = table(
    entry("nvs", 1, 0x02, 0x9000, 0x6000),
    entry("phy_init", 1, 0x01, 0xF000, 0x1000),
    entry("factory", 0, 0x00, FACTORY_OFFSET, FACTORY_SIZE),
    entry("coredump", 1, 0x03, 0x3F0000, 0x10000),
)


class ImageSizeTest(unittest.TestCase):
    def check(self, image, tbl=GOOD_TABLE, csv=CSV, offset=FACTORY_OFFSET):
        return image_size.check(image, tbl, csv, offset)

    def test_fits_reports_headroom(self):
        self.assertEqual(
            self.check(1589509 + 255),
            "app 1.52 MB of 3.88 MB factory partition, 60 % free",
        )

    def test_exact_fit_passes(self):
        self.assertEqual(self.check(FACTORY_SIZE), "app 3.88 MB of 3.88 MB factory partition, 0 % free")

    def test_one_byte_too_big_fails(self):
        with self.assertRaisesRegex(image_size.CheckError, "does not fit .* 1 bytes too big"):
            self.check(FACTORY_SIZE + 1)

    def test_missing_factory_partition_fails(self):
        tbl = table(entry("nvs", 1, 0x02, 0x9000, 0x6000), entry("ota_0", 0, 0x10, 0x10000, 0x3E0000))
        with self.assertRaisesRegex(image_size.CheckError, "no factory app partition"):
            self.check(1000, tbl)

    def test_table_differing_from_csv_fails_even_if_image_fits(self):
        stock = table(
            entry("nvs", 1, 0x02, 0x9000, 0x6000),
            entry("phy_init", 1, 0x01, 0xF000, 0x1000),
            entry("factory", 0, 0x00, 0x10000, 0x100000),
        )
        with self.assertRaisesRegex(image_size.CheckError, "differs from partitions.csv"):
            self.check(1000, stock)

    def test_wrong_app_offset_fails(self):
        with self.assertRaisesRegex(image_size.CheckError, "flashed at 0x20000"):
            self.check(1000, offset=0x20000)

    def test_table_without_md5_entry_parses(self):
        tbl = table(entry("factory", 0, 0x00, FACTORY_OFFSET, FACTORY_SIZE), md5=False)
        self.assertIn("factory partition", self.check(1000, tbl))

    def test_csv_without_factory_fails(self):
        with self.assertRaisesRegex(image_size.CheckError, "partitions.csv has no factory"):
            self.check(1000, csv="nvs, data, nvs, 0x9000, 0x6000\n")

    def test_csv_sizes_with_suffixes(self):
        self.assertEqual(image_size.csv_factory("factory, app, factory, 64K, 3968K\n"), (0x10000, 0x3E0000))

    def test_bad_entry_magic_fails(self):
        with self.assertRaisesRegex(image_size.CheckError, "bad entry magic"):
            self.check(1000, b"\x00" * 64)


if __name__ == "__main__":
    unittest.main()
