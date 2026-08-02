#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""NFR-01: Apache-2.0 throughout, and no copyleft dependency in slipx_core.

Two separate claims, checked separately.

The licence claim is strategic, not legal housekeeping. The two-year goal is
slipx_core turning up as a dependency in projects we do not maintain, and a
copyleft licence anywhere in the core makes that legally awkward for exactly
the commercial and academic embedders the strategy targets. One GPL header,
added once for convenience, ends it.

Run:  python3 tools/licence_scan.py
Exit: 0 clean, 1 problems found.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

SPDX_LINE = "SPDX-License-Identifier: Apache-2.0"

# Every source file carries the identifier, so a file that travels on its own
# into somebody else's tree travels with its licence.
SOURCE_SUFFIXES = {".cpp", ".cc", ".hpp", ".h", ".py"}

# Generated, vendored or trivially non-substantial files.
EXEMPT_PATTERNS = [
    # build/ and any build-* variant. The CORE-01 check documented alongside
    # this one configures into build-core/, and CMake drops its own compiler
    # probe sources there, so scanning it fails the licence check for a file
    # CMake wrote. .gitignore already covers the same set.
    re.compile(r"(^|/)build(-[^/]*)?/"),
    re.compile(r"(^|/)\.venv/"),
    re.compile(r"(^|/)__pycache__/"),
    re.compile(r"(^|/)_skbuild/"),
    re.compile(r"build_info\.hpp$"),  # generated from the .in, which is checked
]

# Licences that must never appear in the dependency tree of slipx_core, and
# that we would rather not have anywhere in the repository.
COPYLEFT_MARKERS = [
    ("GNU General Public License", "GPL"),
    ("GNU Lesser General Public License", "LGPL"),
    ("GNU Affero", "AGPL"),
    ("Mozilla Public License", "MPL"),
    ("Common Development and Distribution", "CDDL"),
    ("Eclipse Public License", "EPL"),
]

# The dependencies each layer is allowed to acquire, and the licence of each.
# Kept here rather than only in NOTICE so that a new dependency has to be added
# in a place where somebody is thinking about its licence.
DECLARED_DEPENDENCIES = {
    "slipx_core": {},  # and it must stay empty (CORE-01)
    "slipx_schema": {"PyYAML": "MIT", "jsonschema": "MIT"},
    "slipx": {"pybind11": "BSD-3-Clause"},
    "slipx_sim": {},
    "tests": {"GoogleTest": "BSD-3-Clause", "pytest": "MIT"},
}


def is_exempt(relative: str) -> bool:
    return any(pattern.search(relative) for pattern in EXEMPT_PATTERNS)


def check_spdx_headers() -> list[str]:
    problems = []
    for path in sorted(REPO_ROOT.rglob("*")):
        if not path.is_file() or path.suffix not in SOURCE_SUFFIXES:
            continue
        relative = str(path.relative_to(REPO_ROOT))
        if is_exempt(relative):
            continue

        # Only the first few lines: an identifier buried at the bottom of a
        # file is an identifier nobody reads.
        head = "\n".join(
            path.read_text(encoding="utf-8", errors="replace").splitlines()[:8]
        )
        if SPDX_LINE not in head:
            problems.append(
                f"{relative}: no '{SPDX_LINE}' in the first 8 lines. Every "
                f"source file carries it, so that a file which travels into "
                f"somebody else's tree travels with its licence (NFR-01)."
            )
    return problems


def check_licence_file() -> list[str]:
    problems = []
    licence = REPO_ROOT / "LICENSE"
    if not licence.exists():
        return ["LICENSE is missing (NFR-01)"]

    text = licence.read_text(encoding="utf-8")
    if "Apache License" not in text or "Version 2.0" not in text:
        problems.append("LICENSE is not the Apache License 2.0 (NFR-01)")

    for marker, name in COPYLEFT_MARKERS:
        if marker in text:
            problems.append(f"LICENSE mentions {name}; it must be Apache-2.0 alone")

    if not (REPO_ROOT / "NOTICE").exists():
        problems.append("NOTICE is missing; Apache-2.0 section 4(d) expects one")
    return problems


def check_no_copyleft_text() -> list[str]:
    """No copyleft licence text anywhere in the tree.

    Catches a vendored dependency dropped in wholesale, which is the realistic
    way this goes wrong: not a considered decision, but a directory copied in
    to get something working.
    """
    problems = []
    for path in sorted(REPO_ROOT.rglob("*")):
        if not path.is_file():
            continue
        relative = str(path.relative_to(REPO_ROOT))
        if is_exempt(relative) or relative.startswith(".git/"):
            continue
        if path.suffix not in SOURCE_SUFFIXES | {".txt", ".md", ".cmake", ".toml"}:
            continue
        # This file names the licences it is looking for, so it would match
        # itself.
        if relative == "tools/licence_scan.py":
            continue

        text = path.read_text(encoding="utf-8", errors="replace")
        for marker, name in COPYLEFT_MARKERS:
            if marker in text:
                problems.append(
                    f"{relative}: contains {name} licence text. No copyleft "
                    f"dependency may appear in slipx_core, and a copyleft file "
                    f"anywhere in the tree is one edit away from being in it "
                    f"(NFR-01)."
                )
    return problems


def check_core_has_no_dependencies() -> list[str]:
    """The core's declared dependency set is empty, and stays empty."""
    if DECLARED_DEPENDENCIES["slipx_core"]:
        return [
            "slipx_core has declared dependencies. It must have none: it "
            "depends on the C++ standard library and nothing else (CORE-01, "
            "NFR-01). Adding one is a change to the project's strategy, not "
            "to its build."
        ]
    return []


def main() -> int:
    problems: list[str] = []
    problems += check_licence_file()
    problems += check_spdx_headers()
    problems += check_no_copyleft_text()
    problems += check_core_has_no_dependencies()

    if problems:
        print("Licence scan failed (NFR-01):\n", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    declared = sum(len(v) for v in DECLARED_DEPENDENCIES.values())
    print(
        f"Licence scan clean: Apache-2.0 throughout, {declared} declared "
        f"dependencies, all permissive, none in slipx_core."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
