# PlatformIO pre script: writes the build identity source
# ($BUILD_DIR/build_identity_id.cpp, listed in main/CMakeLists.txt) with the
# same cmake/build_identity.cmake as the robot's and the virtual robot's
# builds. PlatformIO's ESP-IDF builder compiles the component sources itself
# and never runs a CMake custom target, so this runs on every invocation; the
# CMake script rewrites the file only when the identity changes, so an
# unchanged checkout recompiles nothing.

import os
import subprocess
import sys

Import("env")  # noqa: F821 (SCons)

project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
build_dir = env.subst("$BUILD_DIR")  # noqa: F821
repo_root = os.path.normpath(os.path.join(project_dir, "..", ".."))
script = os.path.join(repo_root, "cmake", "build_identity.cmake")

# The platform's own CMake (the one that configures ESP-IDF), else PATH's.
cmake = "cmake"
cmake_dir = env.PioPlatform().get_package_dir("tool-cmake")  # noqa: F821
if cmake_dir and os.path.isfile(os.path.join(cmake_dir, "bin", "cmake")):
    cmake = os.path.join(cmake_dir, "bin", "cmake")

os.makedirs(build_dir, exist_ok=True)
result = subprocess.run(
    [cmake, "-DREPO_ROOT=" + repo_root, "-DOUTPUT=" + os.path.join(build_dir, "build_identity_id.cpp"), "-P", script]
)
if result.returncode != 0:
    sys.stderr.write("Bridge build identity: cmake -P %s failed\n" % script)
    env.Exit(1)  # noqa: F821
