#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""NFR-06: the dependency direction of SRS 2.1, enforced automatically.

    slipx_registry -> slipx_id -> slipx_ros -> slipx_sim
                   -> slipx_scene / slipx_sense
                   -> slipx (Python) -> slipx_schema -> slipx_core

Dependencies flow strictly downward. No component may depend on anything above
it, and slipx_core may depend on nothing but the C++ standard library.

This exists because the rule is the load-bearing one in the whole design and
because it will be broken by accident rather than on purpose. Somebody needs a
logger in the core for an afternoon, or a YAML parser "just for the parameter
loading", and it compiles, and it passes the tests, and the embedding strategy
is quietly over. A build that breaks is the only reliable way to notice.

Run:  python3 tools/dep_lint.py [--verbose]
Exit: 0 clean, 1 violations found.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Layers from the bottom up. A component may include headers from its own layer
# and from anything BELOW it in this list, and nothing else.
#
# One deliberate departure from the order as written in SRS 2.1, which lists
# `slipx` (Python) between slipx_sense and slipx_schema. Taken literally that
# would forbid the bindings from exposing the orchestrator, while the same
# document describes `slipx` as "pybind11 bindings + Gymnasium adapter" and a
# Gymnasium environment is a fixed-step loop over agents, which is precisely
# what slipx_sim is. The two statements cannot both hold.
#
# Resolved in favour of the description: the binding layer sits above
# everything it binds. That preserves what the rule is actually for, since
# nothing below slipx depends on slipx, and slipx_core still depends on
# nothing. SRS 2.1 should be amended to match.
LAYERS: list[tuple[str, Path]] = [
    ("slipx_core", Path("src/core/slipx_core")),
    ("slipx_schema", Path("src/core/slipx_schema")),
    ("slipx_sense", Path("src/world/slipx_sense")),
    ("slipx_scene", Path("src/world/slipx_scene")),
    ("slipx_sim", Path("src/orchestration/slipx_sim")),
    ("slipx", Path("src/bindings/slipx")),
    ("slipx_c", Path("src/bindings/slipx_c")),
    ("slipx_ros", Path("src/integration/slipx_ros")),
    ("slipx_id", Path("src/tooling/slipx_id")),
    ("slipx_registry", Path("src/tooling/slipx_registry")),
]

LAYER_INDEX = {name: i for i, (name, _) in enumerate(LAYERS)}

# Pairs that are siblings rather than layers, where NEITHER may include the
# other (ADR-0037).
#
# The list above has to be a total order, because comparing two components
# needs one, and that forces an order onto pairs the design does not order.
# slipx_scene and slipx_sense are the case: the spec's diagram puts them on
# one line, and the order between them here is an artefact of which was typed
# first. Left as a plain layer comparison it would silently permit the scene
# to include a sensor, which is half of a coupling that is not wanted in
# either direction.
#
# A sensor model is about timing, noise and dropouts; what a ray hits is a
# question for whoever owns geometry. They meet in slipx_sim, which is above
# both and is allowed to know about each.
SIBLINGS: list[tuple[str, str]] = [
    ("slipx_scene", "slipx_sense"),
]

# The include prefix each component owns, so an include can be attributed to a
# layer. slipx_core owns slipx/ at the top level; everything else namespaces
# itself one level down.
INCLUDE_OWNERS: list[tuple[re.Pattern[str], str]] = [
    (re.compile(r"^slipx/sim/"), "slipx_sim"),
    (re.compile(r"^slipx/scene/"), "slipx_scene"),
    (re.compile(r"^slipx/sense/"), "slipx_sense"),
    (re.compile(r"^slipx/ros/"), "slipx_ros"),
    (re.compile(r"^slipx/schema/"), "slipx_schema"),
    (re.compile(r"^slipx/"), "slipx_core"),
]

# What slipx_core is allowed to include, and nothing else (CORE-01).
#
# The C++ standard library, in full. Not Eigen: D-02 was resolved in favour of
# a hand-rolled fixed-size math header, so the core now has NO third-party
# dependency at all and this list has no entry for one. If Eigen is ever
# reinstated, it goes here and the README claim changes with it.
CORE_ALLOWED_SYSTEM_HEADERS = {
    "algorithm", "array", "cassert", "cmath", "cstddef", "cstdint", "cstdio",
    "cstdlib", "cstring", "initializer_list", "limits", "memory", "new",
    "stdexcept", "type_traits", "utility",
}

# Headers that are banned from slipx_core outright, with the reason. Each is
# individually defensible and collectively they are the CORE-01 promise: no
# I/O, no threads, no logging, no allocation in step, no clock (CORE-03,
# CORE-04).
CORE_BANNED_SYSTEM_HEADERS = {
    "chrono": "no wall clock in a deterministic core (CORE-04)",
    "ctime": "no wall clock in a deterministic core (CORE-04)",
    "random": "no ambient RNG; randomness lives above the core (CORE-04)",
    "thread": "no threading inside the integrator (NFR-02)",
    "mutex": "no threading inside the integrator (NFR-02)",
    "atomic": "no threading inside the integrator (NFR-02)",
    "future": "no threading inside the integrator (NFR-02)",
    "condition_variable": "no threading inside the integrator (NFR-02)",
    "execution": "no parallel algorithms in a deterministic core (NFR-02)",
    "iostream": "no I/O in the core (CORE-01)",
    "fstream": "no I/O in the core (CORE-01)",
    "sstream": "no I/O in the core (CORE-01)",
    "filesystem": "no I/O in the core (CORE-01)",
    "vector": "no dynamic allocation in the core's hot path (CORE-01)",
    "string": "no dynamic allocation in the core's hot path (CORE-01)",
    "map": "no allocation, and no unordered iteration either (CORE-04)",
    "set": "no allocation, and no unordered iteration either (CORE-04)",
    "unordered_map": "unordered iteration in a numerical path (CORE-04)",
    "unordered_set": "unordered iteration in a numerical path (CORE-04)",
    "functional": "std::function allocates and is not needed in the core",
    "regex": "nothing in a physics core should be parsing anything",
}

# Third-party roots that must not appear anywhere in slipx_core.
CORE_BANNED_PREFIXES = {
    "Eigen/": "D-02 resolved against Eigen; the core has no third-party deps",
    "eigen3/": "D-02 resolved against Eigen; the core has no third-party deps",
    "rclcpp/": "ROS in the core would end the embedding strategy (CORE-01)",
    "rclpy/": "ROS in the core would end the embedding strategy (CORE-01)",
    "ros/": "ROS in the core would end the embedding strategy (CORE-01)",
    "yaml-cpp/": "parsing is slipx_schema's job (CORE-01)",
    "nlohmann/": "parsing is slipx_schema's job (CORE-01)",
    "pybind11/": "bindings are a layer above (CORE-01)",
    "spdlog/": "no logging framework in the core (CORE-01)",
    "gtest/": "test-only, and never in library sources",
    "benchmark/": "test-only, and never in library sources",
}

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*([<"])([^">]+)[>"]', re.MULTILINE)

CPP_SUFFIXES = {".cpp", ".cc", ".cxx", ".hpp", ".h", ".hh", ".ipp"}


def sources_of(directory: Path, include_tests: bool) -> list[Path]:
    if not directory.is_dir():
        return []
    files = []
    for path in sorted(directory.rglob("*")):
        if path.suffix not in CPP_SUFFIXES:
            continue
        parts = set(path.relative_to(directory).parts)
        if not include_tests and ({"tests", "test", "benchmarks"} & parts):
            continue
        files.append(path)
    return files


def includes_in(path: Path) -> list[tuple[str, str]]:
    """(bracket, header) for every #include in the file."""
    text = path.read_text(encoding="utf-8", errors="replace")
    return [(m.group(1), m.group(2)) for m in INCLUDE_RE.finditer(text)]


def owner_of(header: str) -> str | None:
    for pattern, owner in INCLUDE_OWNERS:
        if pattern.match(header):
            return owner
    return None


def check_layering(verbose: bool) -> list[str]:
    """No component may include a header owned by a layer above it."""
    violations = []
    for name, relative in LAYERS:
        directory = REPO_ROOT / relative
        for path in sources_of(directory, include_tests=True):
            for bracket, header in includes_in(path):
                owner = owner_of(header)
                if owner is None or owner == name:
                    continue
                if LAYER_INDEX[owner] > LAYER_INDEX[name]:
                    violations.append(
                        f"{path.relative_to(REPO_ROOT)}: {name} includes "
                        f"<{header}>, which belongs to {owner}. Dependencies "
                        f"flow strictly downward (SRS 2.1, NFR-06)."
                    )
                elif verbose:
                    print(f"  ok  {name} -> {owner}  ({header})")
    return violations


def check_siblings() -> list[str]:
    """Neither of a sibling pair may include the other (ADR-0037)."""
    forbidden: dict[str, set[str]] = {}
    for one, other in SIBLINGS:
        forbidden.setdefault(one, set()).add(other)
        forbidden.setdefault(other, set()).add(one)

    directories = dict(LAYERS)
    violations = []
    for name, banned in sorted(forbidden.items()):
        for path in sources_of(REPO_ROOT / directories[name], include_tests=True):
            for _, header in includes_in(path):
                owner = owner_of(header)
                if owner in banned:
                    violations.append(
                        f"{path.relative_to(REPO_ROOT)}: {name} includes "
                        f"<{header}>, which belongs to {owner}. Those two are "
                        f"siblings and neither may include the other "
                        f"(ADR-0037); they meet in slipx_sim, which is above "
                        f"both."
                    )
    return violations


def check_core_dependencies() -> list[str]:
    """slipx_core may include the standard library and nothing else."""
    violations = []
    core_dir = REPO_ROOT / "src/core/slipx_core"

    # Library sources only. The core's own tests legitimately use vector,
    # string and GoogleTest; the promise is about what a consumer links, not
    # about what CI compiles.
    for path in sources_of(core_dir, include_tests=False):
        location = path.relative_to(REPO_ROOT)
        for bracket, header in includes_in(path):
            if bracket == '"':
                continue  # a core-internal header; layering already checked it

            for prefix, reason in CORE_BANNED_PREFIXES.items():
                if header.startswith(prefix):
                    violations.append(f"{location}: includes <{header}>. {reason}")
                    break
            else:
                if header in CORE_BANNED_SYSTEM_HEADERS:
                    violations.append(
                        f"{location}: includes <{header}>. "
                        f"{CORE_BANNED_SYSTEM_HEADERS[header]}"
                    )
                elif header not in CORE_ALLOWED_SYSTEM_HEADERS:
                    violations.append(
                        f"{location}: includes <{header}>, which is not on "
                        f"slipx_core's allow-list. If it is a standard header "
                        f"the core genuinely needs, add it to "
                        f"CORE_ALLOWED_SYSTEM_HEADERS in this script, "
                        f"deliberately. If it is anything else, it does not "
                        f"belong in the core (CORE-01)."
                    )
    return violations


def check_core_link_line() -> list[str]:
    """slipx_core's CMakeLists must not link anything."""
    violations = []
    cmake = REPO_ROOT / "src/core/slipx_core/CMakeLists.txt"
    if not cmake.exists():
        return ["src/core/slipx_core/CMakeLists.txt is missing"]

    for number, line in enumerate(cmake.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.split("#")[0].strip()
        if stripped.startswith("target_link_libraries") and "slipx_core" in stripped:
            violations.append(
                f"src/core/slipx_core/CMakeLists.txt:{number}: slipx_core has a "
                f"link line. It must link nothing but the standard library "
                f"(CORE-01, NFR-06)."
            )
        if "find_package" in stripped:
            violations.append(
                f"src/core/slipx_core/CMakeLists.txt:{number}: slipx_core calls "
                f"find_package. Adding a dependency here is a design decision, "
                f"not a convenience (CORE-01)."
            )
    return violations


def check_python_layering() -> list[str]:
    """slipx_schema must not import slipx, in either direction of convenience."""
    violations = []
    schema_dir = REPO_ROOT / "src/core/slipx_schema/slipx_schema"
    pattern = re.compile(r"^\s*(?:from|import)\s+(slipx)\b(?!_)", re.MULTILINE)

    for path in sorted(schema_dir.rglob("*.py")):
        text = path.read_text(encoding="utf-8")
        for match in pattern.finditer(text):
            line = text[: match.start()].count("\n") + 1
            violations.append(
                f"{path.relative_to(REPO_ROOT)}:{line}: slipx_schema imports "
                f"slipx. The dependency runs slipx -> slipx_schema, never "
                f"upward (SRS 2.1, NFR-06)."
            )
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    violations: list[str] = []
    violations += check_core_dependencies()
    violations += check_core_link_line()
    violations += check_layering(args.verbose)
    violations += check_siblings()
    violations += check_python_layering()

    if violations:
        print("Dependency direction violated (NFR-06):\n", file=sys.stderr)
        for violation in violations:
            print(f"  {violation}", file=sys.stderr)
        print(
            "\nThe rule is SRS 2.1: dependencies flow strictly downward, and "
            "slipx_core depends on nothing but the standard library. It is the "
            "single most load-bearing decision in the design, and a change "
            "that breaks it is wrong even if it compiles.",
            file=sys.stderr,
        )
        return 1

    print(f"Dependency direction is clean: {len(LAYERS)} components checked.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
