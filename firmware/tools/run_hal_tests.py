#!/usr/bin/env python3
"""Run the servo-bus HAL tests on the host.

Compiles the real ``src/hal/servo_bus.cpp`` and the real Adafruit PWM Servo
Driver library against the PCA9685 model in ``test/hal/fakes``. The library is
not vendored; point ``--library`` at a copy of it. After ``pio run`` it is in
``firmware/.pio/libdeps/esp32dev/Adafruit PWM Servo Driver Library``, which is
the default.

This exercises the driver against the data sheet's documented behaviour. It
says nothing about a physical board.

Usage
-----
    python firmware/tools/run_hal_tests.py [--library DIR]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

FIRMWARE_DIR = Path(__file__).resolve().parent.parent
HAL_TEST_DIR = FIRMWARE_DIR / "test" / "hal"
DEFAULT_LIBRARY = (
    FIRMWARE_DIR / ".pio" / "libdeps" / "esp32dev" / "Adafruit PWM Servo Driver Library"
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_core_tests import find_unix_compiler  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, default=DEFAULT_LIBRARY,
                        help="directory holding Adafruit_PWMServoDriver.cpp")
    args = parser.parse_args()

    library = args.library.resolve()
    driver_source = library / "Adafruit_PWMServoDriver.cpp"
    if not driver_source.is_file():
        print(f"Adafruit_PWMServoDriver.cpp not found in {library}", file=sys.stderr)
        return 2
    compiler = find_unix_compiler()
    if compiler is None:
        print("no g++ or clang++ on PATH", file=sys.stderr)
        return 2

    sources = [
        HAL_TEST_DIR / "test_servo_bus.cpp",
        FIRMWARE_DIR / "src" / "hal" / "servo_bus.cpp",
        driver_source,
    ]
    # The fakes come first so they stand in for Arduino, Wire and BusIO.
    includes = [
        HAL_TEST_DIR / "fakes",
        library,
        FIRMWARE_DIR / "src",
        FIRMWARE_DIR / "lib" / "jensen_core",
    ]
    with tempfile.TemporaryDirectory(prefix="jensen_hal_") as build:
        binary = Path(build) / ("hal_tests.exe" if sys.platform == "win32" else "hal_tests")
        command = [compiler, "-std=c++17", "-O1", "-Wall",
                   *[f"-I{path}" for path in includes],
                   *[str(path) for path in sources], "-o", str(binary)]
        built = subprocess.run(command)
        if built.returncode != 0:
            return built.returncode
        return subprocess.run([str(binary)]).returncode


if __name__ == "__main__":
    sys.exit(main())
