"""Check that the bridge app image fits the partition it is flashed into.

Plain Python with no SCons or ESP-IDF imports, so the host test can run it.
check_image_size.py hooks it into the PlatformIO build.
"""

import struct

ENTRY = struct.Struct("<2sBBII16sI")
ENTRY_MAGIC = b"\xaa\x50"
MD5_MAGIC = b"\xeb\xeb"
TYPE_APP = 0x00
SUBTYPE_FACTORY = 0x00
MIB = 1024 * 1024


class CheckError(Exception):
    """The image or partition table would not boot; the message says why."""


def parse_table_bin(data):
    """Return [(label, type, subtype, offset, size)] from a built partition table."""
    entries = []
    for start in range(0, len(data) - ENTRY.size + 1, ENTRY.size):
        magic, ptype, subtype, offset, size, label, _flags = ENTRY.unpack_from(data, start)
        if magic == ENTRY_MAGIC:
            name = label.split(b"\0", 1)[0].decode("ascii", "replace")
            entries.append((name, ptype, subtype, offset, size))
        elif magic in (MD5_MAGIC, b"\xff\xff"):
            break
        else:
            raise CheckError("partition table: bad entry magic at byte %d" % start)
    return entries


def _parse_number(text):
    text = text.strip()
    if not text:
        raise ValueError("empty")
    scale = 1
    if text[-1] in "kK":
        scale, text = 1024, text[:-1]
    elif text[-1] in "mM":
        scale, text = MIB, text[:-1]
    return int(text, 0) * scale


def csv_factory(text):
    """Return (offset, size) of the factory app row in a partitions.csv."""
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = [f.strip() for f in line.split(",")]
        if len(fields) < 5:
            continue
        if fields[1] in ("app", "0", "0x0", "0x00") and fields[2] in ("factory", "0", "0x0", "0x00"):
            try:
                return _parse_number(fields[3]), _parse_number(fields[4])
            except ValueError:
                raise CheckError("partitions.csv: the factory row needs an explicit offset and size: %s" % line)
    raise CheckError("partitions.csv has no factory app partition")


def mb(size):
    return "%.2f MB" % (size / MIB)


def check(image_size, table_bin, csv_text, app_offset):
    """Return the headroom line, or raise CheckError listing every problem found.

    image_size: bytes in firmware.bin; table_bin: built partitions.bin;
    csv_text: the project's partitions.csv; app_offset: where upload writes the image.
    """
    factories = [e for e in parse_table_bin(table_bin) if e[1] == TYPE_APP and e[2] == SUBTYPE_FACTORY]
    if not factories:
        raise CheckError("the built partition table has no factory app partition")
    _name, _t, _s, offset, size = factories[0]

    errors = []
    csv_offset, csv_size = csv_factory(csv_text)
    if (offset, size) != (csv_offset, csv_size):
        errors.append(
            "the built partition table's factory partition (offset 0x%X, size 0x%X, %s) differs from "
            "partitions.csv (offset 0x%X, size 0x%X, %s): the build took its table from somewhere else"
            % (offset, size, mb(size), csv_offset, csv_size, mb(csv_size))
        )
    if app_offset != offset:
        errors.append("the app is flashed at 0x%X but the factory partition starts at 0x%X" % (app_offset, offset))
    if image_size > size:
        errors.append(
            "app image %d bytes (%s) does not fit the factory partition of %d bytes (%s): %d bytes too big"
            % (image_size, mb(image_size), size, mb(size), image_size - size)
        )
    if errors:
        raise CheckError("\n".join(errors))
    free = (size - image_size) * 100 // size
    return "app %s of %s factory partition, %d %% free" % (mb(image_size), mb(size), free)
