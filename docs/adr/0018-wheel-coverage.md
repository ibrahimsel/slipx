# ADR-0018: Wheels for five platforms on CPython 3.9 to 3.13

- **Status:** Accepted
- **Date recorded:** 2026-08-02
- **Requirements:** none directly
- **Related:** [ADR-0017](0017-first-release-is-a-pre-release.md),
  [ADR-0020](0020-wheels-assert-nothing-about-their-hash.md)

## Context

`python -m build` on a developer machine produces one Linux wheel tied to that
machine's glibc, which is not installable anywhere else. Without a wheel
matrix, `pip install slipx` falls back to the sdist and requires the user to
have CMake and a C++17 compiler.

For this audience that is close to a wall. A student on a laptop, an instructor
setting an assignment, and a team member who is not the one who builds things
all hit a compiler error rather than a working install.

Each platform added is a platform users are then entitled to expect to keep
working, so the matrix is a commitment and not just a build setting.

## Decision

cibuildwheel builds CPython 3.9 to 3.13 for:

| Platform | Why |
|---|---|
| manylinux_2_28 x86_64 | the baseline |
| manylinux_2_28 aarch64 | **Jetson.** The computer actually bolted to an F1TENTH car |
| macOS arm64 | current laptops |
| macOS x86_64 | older Intel laptops, still common in teaching labs |
| Windows AMD64 | instructors and students not on Linux |

Excluded, with reasons, in `[tool.cibuildwheel]`:

- **PyPy.** pybind11 works there; nobody running a physics rollout is on PyPy.
- **musllinux.** Alpine is not a robotics platform. The sdist remains.
- **32-bit.** The state vectors are double precision throughout.

Python 3.9 is the floor because it is the Python on Ubuntu 20.04, which is still
under a lot of ROS 2 Foxy-era hardware. Dropping it would exclude exactly the
teams this is for.

manylinux_2_28 rather than 2_17: glibc 2.28 covers Ubuntu 20.04 onwards and
every Jetson image in current use, and its image ships a compiler that does not
need coaxing into C++17.

aarch64 builds on a native ARM runner rather than under QEMU. It is a
first-class target here rather than a courtesy, and emulated builds take roughly
an order of magnitude longer for the same result.

## Consequences

25 wheels per release. The jobs run in parallel and the build has no external
dependencies, so wall clock is roughly 15 to 20 minutes, dominated by macOS and
Windows.

**Cost depends on repository visibility.** On a public repository this is free,
including the ARM runners. On a private repository, free ARM runners are not
available at all, and macOS bills at a 10x multiplier against the minute
allowance. That makes the matrix a reason the repository is public rather than
an independent decision.

Windows has never been built before this matrix existed. MSVC is handled in
`cmake/SlipxDeterminism.cmake` via `/fp:precise`, so it should work, and the
TestPyPI rehearsal is where that is found out. If it does not build, the right
response is to drop the platform from the matrix rather than to delay the
release.

Adding a platform later is easy. Removing one after users have it is not, which
is why PyPy and musllinux are absent rather than speculative.
