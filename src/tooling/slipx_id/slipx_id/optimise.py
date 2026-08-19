# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Deterministic least squares on the standard library (ADR-0038).

Levenberg-Marquardt with finite-difference Jacobians, normal equations by
Cholesky, fixed iteration order, fixed step sizes, no randomness anywhere:
the same inputs produce the same fit to the last bit. scipy would be faster
and would not promise that across versions; the problems here are never more
than eight parameters against a few thousand residuals, which plain Python
handles in well under a second.

Confidence comes from the Gauss-Newton covariance at the optimum,
``sigma^2 (J^T J)^-1``, and parameter correlations above a reporting
threshold are named in the result: "these two numbers cannot be told apart
from this data" is a finding, not a failure.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

Vector = List[float]
Matrix = List[List[float]]

#: Relative finite-difference step. Large enough to survive residuals computed
#: through a full simulation, small enough that the secant slope still points
#: where the derivative does.
_FD_RELATIVE_STEP = 1e-6

#: Absolute correlation above which a parameter pair is reported as entangled.
CORRELATION_THRESHOLD = 0.95


@dataclass(frozen=True)
class FitReport:
    """What a fit produced, and how far it should be trusted.

    ``stddev`` entries are one-sigma confidence intervals from the
    Gauss-Newton covariance, and ``None`` when the information matrix was
    singular, which means the data does not constrain that direction at all.
    ``correlations`` lists every pair whose correlation magnitude exceeds
    :data:`CORRELATION_THRESHOLD`: those parameters trade off against each
    other in this data and their individual values deserve suspicion even
    when their joint prediction is good.
    """

    names: Tuple[str, ...]
    values: Tuple[float, ...]
    stddev: Tuple[Optional[float], ...]
    correlations: Tuple[Tuple[str, str, float], ...]
    cost: float  # sum of squared residuals at the optimum
    residual_count: int
    iterations: int
    converged: bool

    def value(self, name: str) -> float:
        return self.values[self.names.index(name)]

    def sigma(self, name: str) -> Optional[float]:
        return self.stddev[self.names.index(name)]

    def summary(self) -> str:
        lines = []
        for name, value, sigma in zip(self.names, self.values, self.stddev):
            if sigma is None:
                lines.append(f"  {name} = {value:.6g}  (unconstrained)")
            else:
                lines.append(f"  {name} = {value:.6g} +/- {sigma:.2g}")
        for a, b, rho in self.correlations:
            lines.append(f"  {a} and {b} are correlated ({rho:+.3f})")
        return "\n".join(lines)


def _cholesky(a: Matrix) -> Optional[Matrix]:
    """Lower-triangular L with L L^T = a, or None when a is not positive
    definite. Plain Cholesky without pivoting: the inputs are J^T J plus a
    non-negative diagonal, so failure means genuine rank deficiency."""
    n = len(a)
    low = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = a[i][j]
            for k in range(j):
                s -= low[i][k] * low[j][k]
            if i == j:
                if s <= 0.0:
                    return None
                low[i][j] = math.sqrt(s)
            else:
                low[i][j] = s / low[j][j]
    return low


def _cholesky_solve(low: Matrix, b: Vector) -> Vector:
    n = len(low)
    y = [0.0] * n
    for i in range(n):
        s = b[i]
        for k in range(i):
            s -= low[i][k] * y[k]
        y[i] = s / low[i][i]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = y[i]
        for k in range(i + 1, n):
            s -= low[k][i] * x[k]
        x[i] = s / low[i][i]
    return x


def _normal_matrix(jacobian: Matrix) -> Matrix:
    n = len(jacobian[0])
    jtj = [[0.0] * n for _ in range(n)]
    for row in jacobian:
        for i in range(n):
            ri = row[i]
            if ri == 0.0:
                continue
            for j in range(i + 1):
                jtj[i][j] += ri * row[j]
    for i in range(n):
        for j in range(i + 1, n):
            jtj[i][j] = jtj[j][i]
    return jtj


def _gradient(jacobian: Matrix, residuals: Sequence[float]) -> Vector:
    n = len(jacobian[0])
    g = [0.0] * n
    for row, r in zip(jacobian, residuals):
        if r == 0.0:
            continue
        for i in range(n):
            g[i] += row[i] * r
    return g


def _clamp(x: Vector, lower: Sequence[float], upper: Sequence[float]) -> Vector:
    return [min(max(v, lo), hi) for v, lo, hi in zip(x, lower, upper)]


def _finite_difference_jacobian(
    residual_fn: Callable[[Sequence[float]], Sequence[float]],
    x: Vector,
    base: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    scales: Sequence[float],
) -> Matrix:
    """Forward differences, one column per parameter, in parameter order.

    A step that would leave the box is taken backwards instead, so the
    Jacobian never evaluates the model outside the bounds the caller set.
    """
    n = len(x)
    m = len(base)
    columns: List[Vector] = []
    for i in range(n):
        h = _FD_RELATIVE_STEP * scales[i]
        forward = list(x)
        forward[i] = x[i] + h
        if forward[i] > upper[i]:
            forward[i] = x[i] - h
            h = -h
        if forward[i] < lower[i]:
            raise ValueError(
                f"parameter {i} has no room inside its bounds for a finite "
                f"difference step; widen the bounds or rescale the parameter"
            )
        perturbed = residual_fn(forward)
        columns.append([(perturbed[k] - base[k]) / h for k in range(m)])
    return [[columns[i][k] for i in range(n)] for k in range(m)]


def _covariance(
    jacobian: Matrix, cost: float, m: int, n: int
) -> Tuple[Optional[Matrix], Optional[Vector]]:
    jtj = _normal_matrix(jacobian)
    low = _cholesky(jtj)
    if low is None:
        return None, None
    # sigma^2 from the residuals, with the degrees of freedom honest: a fit
    # with as many parameters as points has learnt nothing about its noise.
    dof = max(m - n, 1)
    sigma2 = cost / dof
    inverse: Matrix = []
    for i in range(n):
        e = [0.0] * n
        e[i] = 1.0
        inverse.append(_cholesky_solve(low, e))
    covariance = [[inverse[j][i] * sigma2 for j in range(n)] for i in range(n)]
    return covariance, [covariance[i][i] for i in range(n)]


def _correlations(
    covariance: Matrix, names: Sequence[str], threshold: float
) -> Tuple[Tuple[str, str, float], ...]:
    out = []
    n = len(names)
    for i in range(n):
        for j in range(i + 1, n):
            denominator = math.sqrt(covariance[i][i] * covariance[j][j])
            if denominator <= 0.0:
                continue
            rho = covariance[i][j] / denominator
            if abs(rho) >= threshold:
                out.append((names[i], names[j], rho))
    return tuple(out)


def levenberg_marquardt(
    residual_fn: Callable[[Sequence[float]], Sequence[float]],
    initial: Sequence[float],
    names: Sequence[str],
    *,
    lower: Optional[Sequence[float]] = None,
    upper: Optional[Sequence[float]] = None,
    max_iterations: int = 60,
    cost_tolerance: float = 1e-14,
    correlation_threshold: float = CORRELATION_THRESHOLD,
) -> FitReport:
    """Minimise the sum of squared residuals, deterministically.

    ``residual_fn`` takes a parameter vector and returns the residuals; it
    must be a pure function of its argument, for the same reason a policy
    must be (anything else it reads makes the fit unreproducible). Bounds
    are boxes; a step is clamped to them and a clamped step that does not
    improve raises the damping like any other rejected step.
    """
    n = len(initial)
    if n == 0:
        raise ValueError("no parameters to fit")
    if len(names) != n:
        raise ValueError("one name per parameter")
    lo = list(lower) if lower is not None else [-math.inf] * n
    hi = list(upper) if upper is not None else [math.inf] * n
    for i in range(n):
        if not lo[i] <= initial[i] <= hi[i]:
            raise ValueError(
                f"initial value of {names[i]} ({initial[i]:g}) is outside "
                f"its bounds [{lo[i]:g}, {hi[i]:g}]"
            )

    # Fixed scales from the initial guess, so the finite-difference step and
    # the damping do not drift as the iterate moves.
    scales = [max(abs(v), 1e-8) for v in initial]

    x = _clamp(list(initial), lo, hi)
    r = list(residual_fn(x))
    m = len(r)
    if m < n:
        raise ValueError(
            f"{m} residuals cannot constrain {n} parameters; the fit is "
            f"underdetermined before it starts"
        )
    cost = sum(v * v for v in r)

    lam = 1e-3
    iterations = 0
    converged = False
    jacobian = _finite_difference_jacobian(residual_fn, x, r, lo, hi, scales)

    for iterations in range(1, max_iterations + 1):
        jtj = _normal_matrix(jacobian)
        g = _gradient(jacobian, r)

        improved = False
        for _ in range(25):
            damped = [row[:] for row in jtj]
            for i in range(n):
                damped[i][i] += lam * max(jtj[i][i], 1e-30)
            low = _cholesky(damped)
            if low is not None:
                delta = _cholesky_solve(low, [-v for v in g])
                candidate = _clamp(
                    [x[i] + delta[i] for i in range(n)], lo, hi
                )
                candidate_r = list(residual_fn(candidate))
                candidate_cost = sum(v * v for v in candidate_r)
                if candidate_cost < cost:
                    previous_cost = cost
                    x, r, cost = candidate, candidate_r, candidate_cost
                    lam = max(lam * 0.1, 1e-12)
                    improved = True
                    break
            lam *= 10.0
            if lam > 1e14:
                break

        if not improved:
            # No damping level improves: the iterate is at a (possibly
            # bounded) minimum of the model.
            converged = True
            break

        jacobian = _finite_difference_jacobian(residual_fn, x, r, lo, hi, scales)
        if previous_cost - cost <= cost_tolerance * max(cost, 1.0):
            converged = True
            break

    covariance, variances = _covariance(jacobian, cost, m, n)
    if covariance is None or variances is None:
        stddev: Tuple[Optional[float], ...] = tuple(None for _ in range(n))
        correlations: Tuple[Tuple[str, str, float], ...] = ()
    else:
        stddev = tuple(
            math.sqrt(v) if v >= 0.0 else None for v in variances
        )
        correlations = _correlations(covariance, names, correlation_threshold)

    return FitReport(
        names=tuple(names),
        values=tuple(x),
        stddev=stddev,
        correlations=correlations,
        cost=cost,
        residual_count=m,
        iterations=iterations,
        converged=converged,
    )


def linear_least_squares(
    rows: Sequence[Sequence[float]],
    rhs: Sequence[float],
    names: Sequence[str],
    *,
    correlation_threshold: float = CORRELATION_THRESHOLD,
) -> FitReport:
    """Solve ``rows @ x = rhs`` in the least-squares sense, in closed form.

    For the sub-problems that are genuinely linear (coastdown, slip
    stiffness), where iterating would only add ways to be wrong.
    """
    n = len(names)
    m = len(rows)
    if m < n:
        raise ValueError(
            f"{m} equations cannot constrain {n} parameters; the fit is "
            f"underdetermined before it starts"
        )
    jacobian = [list(row) for row in rows]
    jtj = _normal_matrix(jacobian)
    g = _gradient(jacobian, [-v for v in rhs])
    low = _cholesky(jtj)
    if low is None:
        raise ValueError(
            "the normal matrix is singular: at least one parameter "
            "combination is invisible to this data"
        )
    x = _cholesky_solve(low, [-v for v in g])
    residuals = [
        sum(row[i] * x[i] for i in range(n)) - b for row, b in zip(rows, rhs)
    ]
    cost = sum(v * v for v in residuals)
    covariance, variances = _covariance(jacobian, cost, m, n)
    assert covariance is not None and variances is not None  # jtj was PD
    return FitReport(
        names=tuple(names),
        values=tuple(x),
        stddev=tuple(math.sqrt(v) if v >= 0.0 else None for v in variances),
        correlations=_correlations(covariance, names, correlation_threshold),
        cost=cost,
        residual_count=m,
        iterations=0,
        converged=True,
    )
