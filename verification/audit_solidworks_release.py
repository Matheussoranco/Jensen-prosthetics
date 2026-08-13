#!/usr/bin/env python3
"""Conservative pre-release screen for native SolidWorks files.

The script hashes each file and searches decoded ASCII/UTF-16 strings for
metadata labels and operator-supplied identity terms. Passing this automated
screen is necessary but not sufficient: the manifest always requires two
independent reviewers because compressed SolidWorks containers are not fully
inspectable with open tooling.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

LABELS = re.compile(r"author|last saved by|company|manager|computer|user ?name|created by", re.I)
ASCII = re.compile(rb"[ -~]{4,}")
UTF16 = re.compile(rb"(?:[ -~]\x00){4,}")


def strings(data: bytes) -> list[str]:
    values = [match.group().decode("ascii", "replace") for match in ASCII.finditer(data)]
    values += [match.group().decode("utf-16le", "replace") for match in UTF16.finditer(data)]
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--identity-term", action="append", default=[], help="name, email, host, or company to reject")
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    files = sorted([*args.source.rglob("*.SLDPRT"), *args.source.rglob("*.SLDASM"),
                    *args.source.rglob("*.sldprt"), *args.source.rglob("*.sldasm")])
    records = []
    any_findings = False
    identity = [term.casefold() for term in args.identity_term if term.strip()]
    for path in dict.fromkeys(files):
        data = path.read_bytes()
        findings = []
        for value in strings(data):
            if LABELS.search(value) or any(term in value.casefold() for term in identity):
                findings.append(value[:200])
        findings = sorted(set(findings))
        any_findings = any_findings or bool(findings)
        records.append({"path": str(path.resolve()), "size": len(data),
                        "sha256": hashlib.sha256(data).hexdigest(), "findings": findings})
    manifest = {
        "automated_screening": "failed" if any_findings else ("passed" if records else "no_files"),
        "limitations": "Compressed/proprietary regions may remain opaque to this open-tool string screen.",
        "files": records,
        "independent_review": {"reviewer_1": "pending", "reviewer_2": "pending"},
        "release_approved": False
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if not records:
        print("no native SolidWorks files found", file=sys.stderr)
        return 2
    if any_findings:
        print("identity/metadata findings require remediation", file=sys.stderr)
        return 1
    print("automated screen passed; two independent reviews are still required")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
