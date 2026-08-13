#!/usr/bin/env python3
"""Build and run the Jensen Hand control-core tests on the host.

The control core (``firmware/lib/jensen_core``) contains no Arduino or ESP-IDF
code, so it compiles and runs anywhere with a C++17 compiler. This script finds
one, builds the core together with ``firmware/test/native``, and runs the
resulting binary.

It exists so the tests can be run without installing PlatformIO. If you have
PlatformIO, ``pio test -e native`` from the ``firmware`` directory does the same
thing.

Usage
-----
    python firmware/tools/run_core_tests.py [-v] [--keep]

Exit status is the test binary's, so this is usable directly in CI.
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
CORE_DIR = FIRMWARE_DIR / "lib" / "jensen_core"
TEST_DIR = FIRMWARE_DIR / "test" / "native"

VSWHERE = Path(
    os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"


def sources() -> list[Path]:
    files = sorted(CORE_DIR.rglob("*.cpp")) + sorted(TEST_DIR.rglob("*.cpp"))
    if not files:
        raise SystemExit(f"no sources found under {CORE_DIR} or {TEST_DIR}")
    return files


def find_unix_compiler() -> str | None:
    for name in ("g++", "clang++", "c++"):
        if shutil.which(name):
            return name
    return None


def find_vcvars() -> Path | None:
    """Locate vcvars64.bat via vswhere, then via the usual install roots."""
    if VSWHERE.exists():
        try:
            out = subprocess.run(
                [
                    str(VSWHERE),
                    "-latest",
                    "-products",
                    "*",
                    "-requires",
                    "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                    "-property",
                    "installationPath",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            root = out.stdout.strip().splitlines()
            if root:
                candidate = Path(root[0]) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
                if candidate.exists():
                    return candidate
        except OSError:
            pass

    program_files = [
        os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
        os.environ.get("ProgramFiles", r"C:\Program Files"),
    ]
    for base in program_files:
        for edition in ("BuildTools", "Community", "Professional", "Enterprise"):
            for year in ("2022", "2019"):
                candidate = (
                    Path(base)
                    / "Microsoft Visual Studio"
                    / year
                    / edition
                    / "VC"
                    / "Auxiliary"
                    / "Build"
                    / "vcvars64.bat"
                )
                if candidate.exists():
                    return candidate
    return None


def build_with_msvc(build_dir: Path, verbose: bool) -> Path:
    vcvars = find_vcvars()
    if vcvars is None:
        raise SystemExit(
            "No C++ compiler found. Install the Visual Studio Build Tools "
            '("Desktop development with C++") or put g++/clang++ on PATH.'
        )

    exe = build_dir / "jensen_core_tests.exe"
    src = " ".join(f'"{path}"' for path in sources())
    # vcvars needs vswhere on PATH in some installations.
    installer_dir = VSWHERE.parent
    script = build_dir / "build.bat"
    script.write_text(
        "@echo off\r\n"
        f'set "PATH=%PATH%;{installer_dir}"\r\n'
        f'call "{vcvars}" >nul\r\n'
        "if errorlevel 1 exit /b 1\r\n"
        f'cd /d "{build_dir}"\r\n'
        f'cl /nologo /std:c++17 /EHsc /W4 /permissive- /O2 /I"{CORE_DIR}" /I"{TEST_DIR}" '
        f'{src} /Fe:"{exe}"\r\n'
        "exit /b %errorlevel%\r\n",
        encoding="ascii",
    )

    result = subprocess.run(["cmd", "/c", str(script)], capture_output=not verbose, text=True)
    if result.returncode != 0:
        if not verbose:
            sys.stdout.write(result.stdout or "")
            sys.stderr.write(result.stderr or "")
        raise SystemExit(f"compilation failed (exit {result.returncode})")
    return exe


def build_with_unix(compiler: str, build_dir: Path, verbose: bool) -> Path:
    exe = build_dir / "jensen_core_tests"
    cmd = [
        compiler,
        "-std=c++17",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        f"-I{CORE_DIR}",
        f"-I{TEST_DIR}",
        *[str(path) for path in sources()],
        "-o",
        str(exe),
    ]
    if verbose:
        print(" ".join(cmd))
    result = subprocess.run(cmd, capture_output=not verbose, text=True)
    if result.returncode != 0:
        if not verbose:
            sys.stdout.write(result.stdout or "")
            sys.stderr.write(result.stderr or "")
        raise SystemExit(f"compilation failed (exit {result.returncode})")
    return exe


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true", help="show compiler output")
    parser.add_argument("--keep", action="store_true", help="keep the build directory")
    args = parser.parse_args()

    build_dir = Path(tempfile.mkdtemp(prefix="jensen_core_tests_"))
    try:
        compiler = find_unix_compiler()
        if compiler is not None:
            exe = build_with_unix(compiler, build_dir, args.verbose)
        elif platform.system() == "Windows":
            exe = build_with_msvc(build_dir, args.verbose)
        else:
            raise SystemExit("No C++ compiler found; install g++ or clang++.")

        return subprocess.run([str(exe)]).returncode
    finally:
        if args.keep:
            print(f"build directory: {build_dir}")
        else:
            shutil.rmtree(build_dir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
