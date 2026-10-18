# PlatformIO post script: fail the build, and any upload, when the app image
# does not fit the factory partition of the built partition table, or when that
# table is not partitions.csv. The platform's checkprogsize does not catch this:
# without an ota_0 partition it keeps the board's 4 MB flash size as the limit.
# The logic lives in image_size.py (host-tested by test_image_size.py).

import os
import sys

Import("env")  # noqa: F821 (SCons)

sys.path.insert(0, os.path.join(env.subst("$PROJECT_DIR"), "scripts"))  # noqa: F821
import image_size  # noqa: E402

IMAGE = "$BUILD_DIR/${PROGNAME}.bin"
TABLE = "$BUILD_DIR/partitions.bin"


def _check(target, source, env):
    image, table = (env.subst(p) for p in (IMAGE, TABLE))
    csv_path = os.path.join(env.subst("$PROJECT_DIR"), "partitions.csv")
    app_offset = env.subst("$ESP32_APP_OFFSET")
    try:
        if not app_offset:
            raise image_size.CheckError("the platform did not set ESP32_APP_OFFSET")
        with open(table, "rb") as fp:
            table_bin = fp.read()
        with open(csv_path) as fp:
            csv_text = fp.read()
        line = image_size.check(os.path.getsize(image), table_bin, csv_text, int(app_offset, 0))
    except (image_size.CheckError, OSError) as exc:
        sys.stderr.write("\nBridge image-size check FAILED:\n%s\n\n" % exc)
        return 1
    print("Bridge image-size check: " + line)
    return 0


check = env.AlwaysBuild(  # noqa: F821
    env.Alias("checkimagesize", [IMAGE, TABLE], env.VerboseAction(_check, "Checking app image size"))  # noqa: F821
)
env.Default(check)  # noqa: F821
# Both targets that write the app image (uploadfs writes the filesystem only).
for upload_target in ("upload", "erase_upload"):
    env.Depends(upload_target, check)  # noqa: F821
