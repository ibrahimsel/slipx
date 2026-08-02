#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""NFR-09: hold the version numbers in agreement, and keep the two axes apart.

There are two version numbers in SlipX and they are versioned independently:

    the distribution / slipx_core version   what a consumer pins
    the slipx_schema version                what a car file declares

They are equal today by coincidence and must not become equal by construction.
A schema addition the core never sees must not force a core release, and a core
ABI break must not invalidate every car file in existence. This script asserts
the first number is written consistently in the four places it appears, and
asserts nothing whatsoever about the second beyond it being a valid semver:
that is the point.

The four places, and why there is no way to make there be one:

    pyproject.toml            read by the build backend before CMake runs
    CMakeLists.txt            read by consumers doing find_package
    include/slipx/version.hpp reaches the run manifest (SIM-06) and any C++
                              consumer with no Python anywhere near it
    slipx/version.py          importable without the extension being built

Each is authoritative for a different consumer, and generating three of them
from the fourth would mean a build step between a git checkout and a readable
version, which is worse. So they are duplicated and checked.

CMake's project(VERSION) cannot express a PEP 440 pre-release suffix, so the
rule there is looser: PROJECT_VERSION must equal the numeric release part of
the full version. 0.1.0a1 in pyproject requires 0.1.0 in CMakeLists.

Run:  python3 tools/version_check.py [--expect VERSION]

--expect is used by the release workflow, which passes the git tag. That is
what stops a tag and a distribution from disagreeing about what was released.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# PEP 440, restricted to the subset this project intends to use. Anything
# outside it (epochs, local versions, .post releases) is refused rather than
# accepted quietly, because a version this script does not understand is a
# version the release workflow's ordering check cannot be trusted about.
VERSION_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)((?:a|b|rc)\d+)?(\.dev\d+)?$")


class Site:
    """One place a version is written, and how to read it back out."""

    def __init__(self, path: str, pattern: str, label: str) -> None:
        self.path = REPO_ROOT / path
        self.pattern = re.compile(pattern, re.MULTILINE)
        self.label = label

    def read(self) -> str:
        if not self.path.exists():
            raise SystemExit(f"{self.path}: does not exist")
        text = self.path.read_text(encoding="utf-8")
        match = self.pattern.search(text)
        if match is None:
            raise SystemExit(
                f"{self.path.relative_to(REPO_ROOT)}: no version found.\n"
                f"  expected something matching {self.pattern.pattern!r}\n"
                f"  If the file was reformatted, fix the pattern in "
                f"tools/version_check.py in the same commit. A version check "
                f"that silently stops looking is worse than no version check."
            )
        return match.group(1)


# The full version string, suffix included.
FULL = [
    Site("pyproject.toml", r'^version\s*=\s*"([^"]+)"', "pyproject.toml"),
    Site(
        "src/core/slipx_core/include/slipx/version.hpp",
        r'kVersion\s*=\s*"([^"]+)"',
        "slipx/version.hpp  kVersion",
    ),
    Site(
        "src/bindings/slipx/slipx/version.py",
        r'^__version__\s*=\s*"([^"]+)"',
        "slipx/version.py   __version__",
    ),
]

# The numeric release part only.
CMAKE = Site("CMakeLists.txt", r"^\s*VERSION\s+(\d+\.\d+\.\d+)\s*$",
             "CMakeLists.txt    project(VERSION)")

# The C++ triple, which has to agree with the release part of kVersion.
HPP = REPO_ROOT / "src/core/slipx_core/include/slipx/version.hpp"
TRIPLE = [
    (r"kVersionMajor\s*=\s*(\d+)", "kVersionMajor"),
    (r"kVersionMinor\s*=\s*(\d+)", "kVersionMinor"),
    (r"kVersionPatch\s*=\s*(\d+)", "kVersionPatch"),
]

SCHEMA = Site("src/core/slipx_schema/slipx_schema/version.py",
              r'^SCHEMA_VERSION\s*=\s*"([^"]+)"', "slipx_schema")


def release_part(version: str) -> str:
    match = VERSION_RE.match(version)
    if match is None:
        raise SystemExit(
            f"'{version}' is not a version this project uses.\n"
            f"  expected MAJOR.MINOR.PATCH with an optional aN, bN or rcN "
            f"pre-release suffix and an optional .devN suffix."
        )
    return f"{match.group(1)}.{match.group(2)}.{match.group(3)}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--expect",
        help="require this exact version. The release workflow passes the git "
             "tag, so a tag that disagrees with the tree is a failed release "
             "rather than a published surprise.",
    )
    parser.add_argument(
        "--print", dest="print_only", action="store_true",
        help="print the distribution version and exit. Used by the release "
             "workflow to name artefacts.",
    )
    args = parser.parse_args()

    versions = {site.label: site.read() for site in FULL}
    distinct = set(versions.values())

    if args.print_only:
        if len(distinct) != 1:
            raise SystemExit("versions disagree; run without --print")
        print(distinct.pop())
        return 0

    failures: list[str] = []

    for label, value in versions.items():
        print(f"  {label:34s} {value}")

    if len(distinct) != 1:
        failures.append(
            "the distribution version is not written the same way everywhere:\n"
            + "\n".join(f"      {k:34s} {v}" for k, v in versions.items())
        )
        version = None
    else:
        version = distinct.pop()
        release = release_part(version)

        cmake_version = CMAKE.read()
        print(f"  {CMAKE.label:34s} {cmake_version}")
        if cmake_version != release:
            failures.append(
                f"CMakeLists.txt says {cmake_version}, but the release part of "
                f"{version} is {release}. CMake cannot carry a pre-release "
                f"suffix, so it carries the numeric part and nothing else."
            )

        text = HPP.read_text(encoding="utf-8")
        triple = []
        for pattern, name in TRIPLE:
            match = re.search(pattern, text)
            if match is None:
                failures.append(f"{HPP.name}: {name} not found")
                triple = []
                break
            triple.append(match.group(1))
        if triple:
            joined = ".".join(triple)
            print(f"  {'slipx/version.hpp  triple':34s} {joined}")
            if joined != release:
                failures.append(
                    f"version.hpp's triple is {joined}, but kVersion is "
                    f"{version} whose release part is {release}. The triple is "
                    f"what a consumer compares in a preprocessor conditional; "
                    f"a triple that disagrees with the string makes that "
                    f"comparison answer a question nobody asked."
                )

        if args.expect is not None and version != args.expect:
            failures.append(
                f"expected {args.expect}, tree says {version}. The tag and the "
                f"distribution have to be the same release: a version on PyPI "
                f"can never be reused, so publishing under the wrong number is "
                f"not something a later commit can repair."
            )

    # The schema version is reported, never compared. NFR-09 is the reason.
    schema = SCHEMA.read()
    release_part(schema)  # still has to be a version, just not the same one
    print(f"\n  {SCHEMA.label:34s} {schema}   (independent, NFR-09)")
    if version is not None and schema == version:
        print(
            "\n  Note: the schema version and the distribution version are "
            "equal right now.\n  That is a coincidence and this check will "
            "never enforce it. If a release\n  bumps one and you find yourself "
            "reaching for the other to match, stop:\n  that is the coupling "
            "NFR-09 exists to prevent."
        )

    if failures:
        print("\nVERSION CHECK FAILED (NFR-09).\n", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("\nversion is written consistently.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
