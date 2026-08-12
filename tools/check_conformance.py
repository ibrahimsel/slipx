#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""NFR-02 and NFR-03: check a build against the published reference hashes.

The subtlety this script exists for: SlipX promises bit-identical results
across runs on a FIXED (platform, compiler, flag set), and explicitly does not
promise them across platforms (NFR-03). A single pinned hash in a test would
therefore be wrong. It would pass on the machine it was recorded on and fail on
every other, and the only way to make it pass everywhere would be to weaken the
check to a tolerance, which is where nondeterminism hides.

So the reference file is keyed by build. A run is compared against the row that
matches its architecture, compiler and configuration, and:

  matching row, hashes agree      the determinism promise held
  matching row, hashes differ     NFR-02 is broken. This is a bug.
  no matching row                 nothing is claimed. The script prints the
                                  row to add and exits 0, unless --require-row.

The last case is the honest one for a contributor on a machine nobody has run
this on before, and --require-row is what the reference CI runners use so that
a missing row on THEM is a failure rather than a shrug.

It also checks, before touching a build at all, that the hash the wheel job
pins literally still matches the published row. See check_pin below.

Run:  python3 tools/check_conformance.py [--build-dir build] [--require-row]
      python3 tools/check_conformance.py --pin-only    (no build needed)
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
REFERENCE_FILE = REPO_ROOT / "conformance" / "reference_hashes.tsv"

# The wheel job pins the published L1/rk4 hash as a literal rather than
# reading this file, deliberately: it is asserting that an independently built
# artefact agrees with a published number, and a job that read the number out
# of the tree it built from would agree with itself. The cost of that second
# copy is that it can be left behind when the row moves, which is exactly what
# happened when ADR-0032 moved twelve rows and the workflow kept the old
# value. The copy stays; this check is what stops it going stale silently.
CI_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci.yml"
PIN_PATTERN = re.compile(r"^\s*EXPECTED_HASH:\s*([0-9a-f]{16})\s*$", re.MULTILINE)
PINNED_PROCESSOR = "x86_64"
PINNED_CASE = ("L1", "rk4")

COLUMNS = [
    "system_processor",
    "compiler_id",
    "compiler_major",
    "build_type",
    "tier",
    "integrator",
    "hash",
]

# The cases the reference file covers. Both tiers and both integrators,
# because a determinism bug that only shows up under semi-implicit Euler is
# still a determinism bug, and the cheap integrator is the one a large RL
# rollout will actually use.
CASES = [("L0", "rk4"), ("L0", "semi_implicit_euler"),
         ("L1", "rk4"), ("L1", "semi_implicit_euler"),
         ("L2", "rk4"), ("L2", "semi_implicit_euler")]


def read_reference() -> list[dict[str, str]]:
    if not REFERENCE_FILE.exists():
        return []
    rows = []
    for line in REFERENCE_FILE.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split("\t")
        if len(fields) != len(COLUMNS):
            raise ValueError(
                f"{REFERENCE_FILE}: expected {len(COLUMNS)} tab-separated "
                f"fields, got {len(fields)}: {line!r}"
            )
        rows.append(dict(zip(COLUMNS, fields)))
    return rows


def check_pin(reference: list[dict[str, str]]) -> list[str]:
    """Check the wheel job's literal hash against the published row.

    Repository-only: it reads two files and builds nothing, so it can run in
    the policy job and fail in seconds rather than after a wheel is built.
    """
    problems: list[str] = []
    if not CI_WORKFLOW.exists():
        return [f"{CI_WORKFLOW} not found"]

    pins = PIN_PATTERN.findall(CI_WORKFLOW.read_text(encoding="utf-8"))
    if len(pins) != 1:
        return [
            f"expected exactly one EXPECTED_HASH pin in "
            f"{CI_WORKFLOW.relative_to(REPO_ROOT)}, found {len(pins)}"
        ]
    pin = pins[0]

    tier, integrator = PINNED_CASE
    rows = [
        r for r in reference
        if r["system_processor"] == PINNED_PROCESSOR
        and r["tier"] == tier and r["integrator"] == integrator
    ]
    if not rows:
        return [
            f"no {PINNED_PROCESSOR} {tier}/{integrator} row to pin against in "
            f"{REFERENCE_FILE.relative_to(REPO_ROOT)}"
        ]

    published = {r["hash"] for r in rows}
    if len(published) > 1:
        # NFR-03 allows this and the reference file is shaped to express it.
        # A single literal in the workflow cannot be, so the wheel job would
        # have to name the build it pins rather than assume there is one hash.
        return [
            f"the {PINNED_PROCESSOR} {tier}/{integrator} rows no longer agree "
            f"({', '.join(sorted(published))}), so a single pin in the wheel "
            f"job is ambiguous about which build it means"
        ]

    expected = published.pop()
    if pin != expected:
        problems.append(
            f"{CI_WORKFLOW.relative_to(REPO_ROOT)} pins EXPECTED_HASH="
            f"{pin}, but the published {PINNED_PROCESSOR} {tier}/{integrator} "
            f"hash is {expected}"
        )
    return problems


def run_case(binary: Path, tier: str, integrator: str) -> dict:
    """Run one case and return its manifest."""
    manifest_path = binary.parent / f"conformance_{tier}_{integrator}.manifest.json"
    result = subprocess.run(
        [
            str(binary),
            "--tier", tier,
            "--integrator", integrator,
            "--manifest", str(manifest_path),
            "--quiet",
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"{binary} failed for {tier}/{integrator}:\n{result.stderr}"
        )
    with manifest_path.open(encoding="utf-8") as handle:
        return json.load(handle)


def key_of(manifest: dict, tier: str, integrator: str) -> dict[str, str]:
    build = manifest["build"]
    # Compiler MAJOR only. A point release of GCC is not supposed to change
    # floating-point results and in practice does not; requiring an exact
    # version match would mean a new reference row every time a CI image
    # updated, and a check nobody maintains is a check nobody believes.
    compiler_major = build["compiler_version"].split(".")[0]
    return {
        "system_processor": build["system_processor"],
        "compiler_id": build["compiler_id"],
        "compiler_major": compiler_major,
        "build_type": build["build_type"],
        "tier": tier,
        "integrator": integrator,
    }


def format_row(key: dict[str, str], digest: str) -> str:
    return "\t".join([key[c] for c in COLUMNS[:-1]] + [digest])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--build-dir", default="build")
    parser.add_argument(
        "--require-row",
        action="store_true",
        help="fail when this build has no reference row. Used by the reference "
             "CI runners, where a missing row means the file needs updating.",
    )
    parser.add_argument(
        "--write-missing",
        action="store_true",
        help="append rows for cases this build has no reference for. Only ever "
             "run deliberately: a reference hash is a published claim.",
    )
    parser.add_argument(
        "--pin-only",
        action="store_true",
        help="check only that the wheel job's literal hash matches the "
             "published row. Reads two files and needs no build, so the policy "
             "job can run it.",
    )
    args = parser.parse_args()

    reference = read_reference()

    pin_problems = check_pin(reference)
    if pin_problems:
        print("PIN CHECK FAILED.\n", file=sys.stderr)
        for problem in pin_problems:
            print(f"  - {problem}", file=sys.stderr)
        print(
            "\nThe wheel job compares an installed wheel against a hash "
            "written literally in the workflow. When a reference row moves, "
            "that literal moves with it, in the same commit: the reference "
            "file, the CHANGELOG table and the workflow pin are one change.",
            file=sys.stderr,
        )
        return 1
    print("workflow pin matches the published L1/rk4 hash.")
    if args.pin_only:
        return 0

    binary = (
        REPO_ROOT / args.build_dir / "src" / "orchestration" / "slipx_sim"
        / "slipx_conformance"
    )
    if not binary.exists():
        print(f"conformance binary not found at {binary}", file=sys.stderr)
        print("build it with: cmake --build build", file=sys.stderr)
        return 2

    failures: list[str] = []
    missing: list[str] = []
    matched = 0

    for tier, integrator in CASES:
        manifest = run_case(binary, tier, integrator)
        digest = manifest["trajectory_hash"]
        key = key_of(manifest, tier, integrator)

        row = next(
            (r for r in reference if all(r[c] == key[c] for c in COLUMNS[:-1])),
            None,
        )

        label = f"{tier}/{integrator}"
        if row is None:
            missing.append(format_row(key, digest))
            print(f"  {label:28s} {digest}  (no reference row for this build)")
        elif row["hash"] != digest:
            failures.append(
                f"{label}: expected {row['hash']}, got {digest}\n"
                f"      build: {key['system_processor']} {key['compiler_id']} "
                f"{key['compiler_major']} {key['build_type']}\n"
                f"      flags: {manifest['build']['cxx_flags']}"
            )
            print(f"  {label:28s} {digest}  MISMATCH (expected {row['hash']})")
        else:
            matched += 1
            print(f"  {label:28s} {digest}  matches reference")

    if failures:
        print(
            "\nDETERMINISM CHECK FAILED (NFR-02).\n\n"
            "These cases have a reference row for this exact build, so the "
            "hashes should be identical. They are not, which means something "
            "changed the numerical result:\n",
            file=sys.stderr,
        )
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        print(
            "\nIf the change was intended (a physics fix, a new integrator, a "
            "changed conformance scenario), the reference file must be updated "
            "in the same commit and the change called out in the release "
            "notes: every previously published result becomes incomparable.",
            file=sys.stderr,
        )
        return 1

    if missing:
        print(
            f"\n{len(missing)} case(s) have no reference row for this build.\n"
            "That is not a failure: NFR-03 does not promise bit-identity "
            "across platforms, compilers or flag sets, so a build nobody has "
            "recorded is a build about which nothing was claimed.\n\n"
            "To publish these as reference values, add to "
            f"{REFERENCE_FILE.relative_to(REPO_ROOT)}:\n"
        )
        for row in missing:
            print(f"  {row}")

        if args.write_missing:
            with REFERENCE_FILE.open("a", encoding="utf-8") as handle:
                for row in missing:
                    handle.write(row + "\n")
            print(f"\nappended {len(missing)} row(s).")
        elif args.require_row:
            print(
                "\n--require-row was given, so a missing row is a failure: this "
                "is a designated reference runner and its hashes are supposed "
                "to be published.",
                file=sys.stderr,
            )
            return 1

    print(f"\n{matched} of {len(CASES)} case(s) checked against a reference.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
