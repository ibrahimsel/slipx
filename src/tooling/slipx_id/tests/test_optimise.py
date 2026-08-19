# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The optimiser, held to analytical answers.

Every case here has a solution derivable by hand, because an optimiser tested
only against itself converges confidently to whatever it converges to.
"""

from __future__ import annotations

import math

import pytest

from slipx_id.optimise import (
    FitReport,
    levenberg_marquardt,
    linear_least_squares,
)


def test_linear_least_squares_recovers_an_exact_line() -> None:
    # y = 3x + 2 sampled without noise: the closed form must be exact to
    # rounding, with zero cost and zero stated uncertainty.
    xs = [0.0, 1.0, 2.0, 3.0, 4.0]
    rows = [[x, 1.0] for x in xs]
    rhs = [3.0 * x + 2.0 for x in xs]

    fit = linear_least_squares(rows, rhs, ("slope", "intercept"))

    assert fit.value("slope") == pytest.approx(3.0, abs=1e-12)
    assert fit.value("intercept") == pytest.approx(2.0, abs=1e-12)
    assert fit.cost == pytest.approx(0.0, abs=1e-20)
    assert fit.converged


def test_linear_least_squares_matches_the_hand_solution_with_noise() -> None:
    # Fixed, written-out "noise" rather than a random draw: determinism
    # applies to tests as much as to the library. The expected values are the
    # textbook normal-equation solution computed independently.
    xs = [0.0, 1.0, 2.0, 3.0]
    noise = [0.1, -0.1, 0.1, -0.1]
    rows = [[x, 1.0] for x in xs]
    rhs = [2.0 * x + 1.0 + e for x, e in zip(xs, noise)]

    fit = linear_least_squares(rows, rhs, ("slope", "intercept"))

    # Normal equations by hand: y = (1.1, 2.9, 5.1, 6.9), sum x = 6,
    # sum x^2 = 14, sum y = 16, sum xy = 33.8.
    # slope = (4*33.8 - 6*16) / (4*14 - 36) = 39.2 / 20 = 1.96
    # intercept = (16 - 1.96*6) / 4 = 1.06
    assert fit.value("slope") == pytest.approx(1.96, abs=1e-12)
    assert fit.value("intercept") == pytest.approx(1.06, abs=1e-12)
    assert fit.stddev[0] is not None and fit.stddev[0] > 0.0


def test_lm_recovers_an_exponential_decay() -> None:
    # r(t) = a exp(-b t) with a = 5, b = 1.3, from a deliberately poor start.
    ts = [0.1 * i for i in range(30)]
    data = [5.0 * math.exp(-1.3 * t) for t in ts]

    def residuals(p):
        a, b = p
        return [a * math.exp(-b * t) - y for t, y in zip(ts, data)]

    fit = levenberg_marquardt(
        residuals, [1.0, 0.3], ("a", "b"), lower=[0.0, 0.0], upper=[100.0, 10.0]
    )

    assert fit.converged
    assert fit.value("a") == pytest.approx(5.0, rel=1e-8)
    assert fit.value("b") == pytest.approx(1.3, rel=1e-8)
    assert fit.cost < 1e-16


def test_lm_is_deterministic_to_the_bit() -> None:
    ts = [0.2 * i for i in range(20)]
    data = [2.0 * math.exp(-0.7 * t) + 0.01 * math.sin(t * 7) for t in ts]

    def residuals(p):
        a, b = p
        return [a * math.exp(-b * t) - y for t, y in zip(ts, data)]

    first = levenberg_marquardt(residuals, [1.0, 1.0], ("a", "b"))
    second = levenberg_marquardt(residuals, [1.0, 1.0], ("a", "b"))

    assert first.values == second.values  # ==, not approx: bit identity
    assert first.cost == second.cost
    assert first.iterations == second.iterations


def test_lm_respects_bounds() -> None:
    # The unconstrained minimum of (x - 5)^2 is at 5; boxed to [0, 2] the
    # fit must settle on the boundary, not wander past it.
    def residuals(p):
        return [p[0] - 5.0]

    fit = levenberg_marquardt(residuals, [1.0], ("x",), lower=[0.0], upper=[2.0])

    assert fit.value("x") == pytest.approx(2.0, abs=1e-12)
    assert fit.converged


def test_an_unidentifiable_pair_is_reported_as_correlated() -> None:
    # Two parameters that only ever appear as a sum: the data cannot tell
    # them apart, and the report must say so rather than print two confident
    # numbers.
    ts = [0.1 * i for i in range(10)]

    def residuals(p):
        a, b = p
        return [(a + b) * t - 2.0 * t for t in ts]

    fit = levenberg_marquardt(residuals, [0.5, 1.0], ("a", "b"))

    assert fit.value("a") + fit.value("b") == pytest.approx(2.0, rel=1e-6)
    assert (
        any({"a", "b"} == {p, q} for p, q, _ in fit.correlations)
        or fit.stddev[0] is None
    ), "an entangled pair must be reported, one way or the other"


def test_underdetermined_fits_are_refused() -> None:
    with pytest.raises(ValueError, match="underdetermined"):
        levenberg_marquardt(lambda p: [p[0]], [1.0, 1.0], ("a", "b"))
    with pytest.raises(ValueError, match="underdetermined"):
        linear_least_squares([[1.0, 0.0]], [1.0], ("a", "b"))


def test_a_start_outside_the_bounds_is_refused() -> None:
    with pytest.raises(ValueError, match="outside"):
        levenberg_marquardt(
            lambda p: [p[0], p[0]], [5.0], ("x",), lower=[0.0], upper=[2.0]
        )


def test_report_summary_reads() -> None:
    fit = FitReport(
        names=("a",),
        values=(1.5,),
        stddev=(0.1,),
        correlations=(),
        cost=0.5,
        residual_count=10,
        iterations=3,
        converged=True,
    )
    assert "a = 1.5" in fit.summary()
