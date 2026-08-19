# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""System identification for SlipX (ADR-0038).

Turns recordings of the manoeuvre library (``docs/identification/``) into a
parameter set with residuals, confidence intervals and a provenance block.
Sits above ``slipx`` and ``slipx_schema`` and depends downward only; nothing
in the library imports this package.

The fit is staged in the manoeuvre library's order, each stage consuming
earlier results as constants, because a joint fit trades entangled
parameters freely and the manoeuvres were designed precisely so that it
never has to.
"""

from .channels import Channel
from .optimise import (
    CORRELATION_THRESHOLD,
    FitReport,
    levenberg_marquardt,
    linear_least_squares,
)
from .reconstruct import Bench
from .stages import (
    LateralFit,
    fit_c_kappa,
    fit_coastdown,
    fit_lateral,
    fit_mu_x0,
    fit_transient,
)
from .synthetic import ManoeuvreRecording

__all__ = [
    "Bench",
    "CORRELATION_THRESHOLD",
    "Channel",
    "FitReport",
    "LateralFit",
    "ManoeuvreRecording",
    "fit_c_kappa",
    "fit_coastdown",
    "fit_lateral",
    "fit_mu_x0",
    "fit_transient",
    "levenberg_marquardt",
    "linear_least_squares",
]
