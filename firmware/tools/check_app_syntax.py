#!/usr/bin/env python3
"""Type-check the ESP32 application sources without the Xtensa toolchain.

``firmware/src`` is Arduino code and normally needs the full PlatformIO
espressif32 toolchain to build. This script compiles it (compile only, no link)
against the declaration-only stubs in ``firmware/test/hostcheck/stubs``, which
catches typos, wrong signatures, missing includes and const-correctness
mistakes — most of what a real build catches — in a couple of seconds and with
no toolchain download.

It is not a substitute for building the firmware. Only
``pio run -e esp32dev`` produces a flashable image, and only that build
verifies the real library APIs.

Usage
-----
    python firmware/tools/check_app_syntax.py [-v]
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

FIRMWARE_DIR = Path(__file__).resolve().parent.parent
SRC_DIR = FIRMWARE_DIR / "src"
CORE_DIR = FIRMWARE_DIR / "lib" / "jensen_core"
STUB_DIR = FIRMWARE_DIR / "test" / "hostcheck" / "stubs"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_core_tests import VSWHERE, find_unix_compiler, find_vcvars  # noqa: E402


def sources() -> list[Path]:
    files = sorted(SRC_DIR.rglob("*.cpp"))
    if not files:
        raise SystemExit(f"no sources found under {SRC_DIR}")
    return files


def include_dirs() -> list[Path]:
    return [STUB_DIR, SRC_DIR, CORE_DIR]


def check_with_unix(compiler: str, build_dir: Path, verbose: bool) -> int:
    failures = 0
    for source in sources():
        cmd = [
            compiler,
            "-std=c++17",
            "-fsyntax-only",
            "-Wall",
            "-Wextra",
            *[f"-I{path}" for path in include_dirs()],
            str(source),
        ]
        if verbose:
            print(" ".join(cmd))
        result = subprocess.run(cmd, capture_output=True, text=True)
        status = "ok  " if result.returncode == 0 else "FAIL"
        print(f"  {status} {source.relative_to(FIRMWARE_DIR)}")
        if result.returncode != 0:
            failures += 1
            sys.stdout.write(result.stdout)
            sys.stderr.write(result.stderr)
    return failures


def check_with_msvc(build_dir: Path, verbose: bool) -> int:
    vcvars = find_vcvars()
    if vcvars is None:
        raise SystemExit(
            "No C++ compiler found. Install the Visual Studio Build Tools "
            '("Desktop development with C++") or put g++/clang++ on PATH.'
        )

    includes = " ".join(f'/I"{path}"' for path in include_dirs())
    lines = [
        "@echo off",
        f'set "PATH=%PATH%;{VSWHERE.parent}"',
        f'call "{vcvars}" >nul',
        "if errorlevel 1 exit /b 1",
        f'cd /d "{build_dir}"',
    ]
    for source in sources():
        # /c compiles without linking; the stubs have no definitions.
        lines.append(f'cl /nologo /std:c++17 /EHsc /W3 /permissive- /c {includes} "{source}"')
        lines.append("if errorlevel 1 exit /b 1")
    lines.append("exit /b 0")

    script = build_dir / "check.bat"
    script.write_text("\r\n".join(lines) + "\r\n", encoding="ascii")

    result = subprocess.run(["cmd", "/c", str(script)], capture_output=True, text=True)
    output = (result.stdout or "") + (result.stderr or "")
    if verbose or result.returncode != 0:
        sys.stdout.write(output)
    else:
        # cl echoes each translation unit's file name as it starts it.
        for source in sources():
            print(f"  ok   {source.relative_to(FIRMWARE_DIR)}")
    return 0 if result.returncode == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true", help="show compiler output")
    args = parser.parse_args()

    print("Jensen Hand — application syntax check (stubbed Arduino API)\n")
    build_dir = Path(tempfile.mkdtemp(prefix="jensen_syntax_"))
    try:
        compiler = find_unix_compiler()
        if compiler is not None:
            failures = check_with_unix(compiler, build_dir, args.verbose)
        elif platform.system() == "Windows":
            failures = check_with_msvc(build_dir, args.verbose)
        else:
            raise SystemExit("No C++ compiler found; install g++ or clang++.")
    finally:
        shutil.rmtree(build_dir, ignore_errors=True)

    if failures:
        print(f"\n{failures} file(s) failed to compile")
        return 1
    print("\nall application sources compile against the stub API")
    return 0


if __name__ == "__main__":
    sys.exit(main())
