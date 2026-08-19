#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The registry's CI check: every entry clears the acceptance bar.

The bar itself is code in the SlipX distribution
(``slipx_schema.rules.check_registry_submission``), so this script stays a
thin runner: load each entry, run the check, verify the validation report
file is really there, and print what a reviewer starts from. Anything
refused is refused by name.

Run:  python tools/check_submission.py [cars/one_entry ...]
      (no arguments: every directory under cars/)
Exit: 0 when every entry clears the bar, 1 otherwise.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

import slipx
from slipx_schema.rules import check_registry_submission

ROOT = Path(__file__).resolve().parent.parent


def check_entry(entry: Path) -> list[str]:
    failures: list[str] = []

    try:
        car = slipx.load_car(entry)
    except Exception as refusal:  # noqa: BLE001 - every refusal is reported
        return [f"does not load: {refusal}"]

    print(car.summary())
    for warning in car.warnings:
        print(f"  warning (reviewer judgement, not refusal): {warning}")

    provenance_path = entry / "provenance.yaml"
    provenance = yaml.safe_load(provenance_path.read_text(encoding="utf-8"))
    failures.extend(
        str(error) for error in check_registry_submission(provenance)
    )

    report = provenance.get("validation_report", "")
    if report and not str(report).startswith(("http://", "https://")):
        if not (entry / str(report)).is_file():
            failures.append(
                f"provenance names validation_report '{report}' and the "
                f"file is not in the entry"
            )

    return failures


def main(argv: list[str]) -> int:
    if argv:
        entries = [Path(argument) for argument in argv]
    else:
        cars = ROOT / "cars"
        entries = sorted(p for p in cars.iterdir() if p.is_dir()) if cars.is_dir() else []

    if not entries:
        print("no entries to check; an empty registry passes trivially")
        return 0

    failed = False
    for entry in entries:
        print(f"== {entry}")
        failures = check_entry(entry)
        for failure in failures:
            print(f"  REFUSED: {failure}")
        failed = failed or bool(failures)

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
