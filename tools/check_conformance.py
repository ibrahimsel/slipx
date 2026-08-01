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

Run:  python3 tools/check_conformance.py [--build-dir build] [--require-row]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
REFERENCE_FILE = REPO_ROOT / "conformance" / "reference_hashes.tsv"

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
         ("L1", "rk4"), ("L1", "semi_implicit_euler")]


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
    args = parser.parse_args()

    binary = (
        REPO_ROOT / args.build_dir / "src" / "orchestration" / "slipx_sim"
        / "slipx_conformance"
    )
    if not binary.exists():
        print(f"conformance binary not found at {binary}", file=sys.stderr)
        print("build it with: cmake --build build", file=sys.stderr)
        return 2

    reference = read_reference()
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
