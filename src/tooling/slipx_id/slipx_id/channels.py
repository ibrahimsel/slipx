# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Time series as the fitter consumes them.

A channel is a pair of tuples, timestamped values in increasing time order.
Everything the fitter does aligns signals by timestamp and never by sample
index, because a dropped message on a real car costs one sample and must not
become an offset in everything after it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple


@dataclass(frozen=True)
class Channel:
    """One timestamped signal. Immutable, so a stage cannot edit its inputs."""

    times: Tuple[float, ...]
    values: Tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.times) != len(self.values):
            raise ValueError(
                f"{len(self.times)} timestamps against {len(self.values)} "
                f"values; a channel is one value per timestamp"
            )
        for earlier, later in zip(self.times, self.times[1:]):
            if later <= earlier:
                raise ValueError(
                    "timestamps must increase strictly; equal or reversed "
                    "stamps mean two messages claimed the same instant and "
                    "the recording needs looking at"
                )

    @staticmethod
    def of(times: Iterable[float], values: Iterable[float]) -> "Channel":
        return Channel(tuple(times), tuple(values))

    def __len__(self) -> int:
        return len(self.times)

    def between(self, start: float, end: float) -> "Channel":
        """The sub-channel with start <= t <= end."""
        pairs = [
            (t, v) for t, v in zip(self.times, self.values) if start <= t <= end
        ]
        return Channel(tuple(t for t, _ in pairs), tuple(v for _, v in pairs))

    def value_at(self, time: float) -> float:
        """Linear interpolation. Refuses to extrapolate: a value outside the
        recording is a value the recording does not contain."""
        times = self.times
        if not times or time < times[0] or time > times[-1]:
            raise ValueError(
                f"t = {time:g} is outside this channel's span "
                f"[{times[0] if times else float('nan'):g}, "
                f"{times[-1] if times else float('nan'):g}]"
            )
        lo, hi = 0, len(times) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if times[mid] <= time:
                lo = mid
            else:
                hi = mid
        if times[lo] == time:
            return self.values[lo]
        span = times[hi] - times[lo]
        weight = (time - times[lo]) / span
        return self.values[lo] * (1.0 - weight) + self.values[hi] * weight

    def value_before(self, time: float) -> float:
        """Zero-order hold: the last value at or before ``time``.

        For command channels, where a step must stay a step: interpolating
        across a command edge would invent a ramp nobody commanded, and the
        step-steer fit measures exactly the interval this would blur.
        """
        times = self.times
        if not times or time < times[0]:
            raise ValueError(
                f"t = {time:g} is before this channel's first sample"
            )
        lo, hi = 0, len(times) - 1
        if times[hi] <= time:
            return self.values[hi]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if times[mid] <= time:
                lo = mid
            else:
                hi = mid
        return self.values[lo]

    def derivative(self) -> "Channel":
        """Central differences at interior points.

        The endpoints are dropped rather than one-sided: a one-sided
        difference has twice the truncation error and every consumer here
        has samples to spare.
        """
        if len(self.times) < 3:
            raise ValueError("a derivative needs at least three samples")
        times = self.times[1:-1]
        values = tuple(
            (self.values[i + 1] - self.values[i - 1])
            / (self.times[i + 1] - self.times[i - 1])
            for i in range(1, len(self.times) - 1)
        )
        return Channel(times, values)

    def mean(self) -> float:
        if not self.values:
            raise ValueError("an empty channel has no mean")
        return sum(self.values) / len(self.values)


def unwrap_angles(values: Sequence[float]) -> Tuple[float, ...]:
    """Remove 2-pi jumps so an angle can be differentiated.

    A heading crossing pi flips sign in one sample; differentiating that
    reads as a spin at several hundred radians per second. Unwrapping keeps
    each step within (-pi, pi] of the previous sample.
    """
    if not values:
        return ()
    out = [values[0]]
    for value in values[1:]:
        step = value - out[-1]
        step -= 2.0 * math.pi * round(step / (2.0 * math.pi))
        out.append(out[-1] + step)
    return tuple(out)


def wrap_angle(angle: float) -> float:
    """Into (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi
