# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Migrations between known minor versions (SCH-01).

A migration is a pure function from one document to the next. It must set
``schema_version`` to the version it produces, and it must fail loudly rather
than dropping a field it does not understand: a migration that quietly
discards data is worse than one that refuses to run.

A migration must also never invent a value. 0.2.0 added fields (``c_kappa``,
the ``esc`` block, the pack voltage endpoints), every one of them optional at
the schema level for exactly this reason: a migrated 0.1.0 file gains nothing
it did not carry, and a tier that needs an absent field refuses by name
(ADR-0030). So the 0.1.0 to 0.2.0 step is the identity for every kind, and it
is registered explicitly rather than special-cased, because a gap in the chain
is treated as a release bug and an implicit identity would hide real gaps.
"""

from __future__ import annotations

from typing import Callable, Dict, Tuple

from .version import CURRENT, Version

Document = Dict[str, object]
Migration = Callable[[Document], Document]

#: (kind, from_minor) -> migration producing from_minor + 1.
#:
#: Keyed by minor rather than by full version because patch releases do not
#: change the document shape; if one ever does, it was not a patch release.
_MIGRATIONS: Dict[Tuple[str, int], Migration] = {}


def register(kind: str, from_minor: int) -> Callable[[Migration], Migration]:
    """Register a migration from ``kind`` at ``from_minor`` to ``from_minor + 1``."""

    def decorate(fn: Migration) -> Migration:
        key = (kind, from_minor)
        if key in _MIGRATIONS:
            raise ValueError(f"duplicate migration registered for {key}")
        _MIGRATIONS[key] = fn
        return fn

    return decorate


def migrate(kind: str, document: Document, version: Version) -> Document:
    """Bring ``document`` forward to the current minor, one step at a time.

    Stepwise rather than in one jump: a chain of small, individually tested
    migrations is the only version of this that stays correct as the chain
    grows.

    Raises:
        KeyError: if a step in the chain is missing, which means a released
            version has no path forward and a file somebody wrote in good faith
            can no longer be read. Loud, because that is a release bug.
    """
    current = version
    working = dict(document)
    while current.minor < CURRENT.minor:
        key = (kind, current.minor)
        if key not in _MIGRATIONS:
            raise KeyError(
                f"no migration for {kind} from schema {current} to "
                f"{current.major}.{current.minor + 1}.0; this is a release "
                f"bug, not a problem with your file"
            )
        working = _MIGRATIONS[key](working)
        current = Version(current.major, current.minor + 1, 0)
        working["schema_version"] = str(current)
    return working


def available() -> list[tuple[str, int]]:
    """Registered migrations, for diagnostics and tests."""
    return sorted(_MIGRATIONS.keys())


# ------------------------------------------------------------- 0.1.0 -> 0.2.0
#
# 0.2.0 only added optional fields, so nothing moves and nothing is invented.
# See the module docstring, and ADR-0030 for what was added and why it is all
# optional.

def _identity(document: Document) -> Document:
    return dict(document)


_KINDS_0_1_0 = ("car", "dynamics", "limits", "sensors", "provenance", "tyre")

for _kind in _KINDS_0_1_0:
    register(_kind, 1)(_identity)


# ------------------------------------------------------------- 0.2.0 -> 0.3.0
#
# 0.3.0 added a whole document kind, the track manifest, and changed no field
# of any kind that already existed (ADR-0036). So this step is the identity
# too, for a different reason from the last one: not "the new fields are
# optional" but "there are no new fields here at all".
#
# `track` is registered at this step as well, and it is the odd one out. No
# track file written at 0.2.0 exists, because there were no tracks at 0.2.0.
# The entry is here so that a file somebody hand-wrote with the wrong version
# in it gets read and then validated against the real schema, which produces a
# message about the field that is actually wrong, rather than the "this is a
# release bug" a gap in the chain reports.

for _kind in _KINDS_0_1_0 + ("track",):
    register(_kind, 2)(_identity)

# And the step before it, for the same reason: so that a track file carrying
# any older version at all is read and then judged on its contents.
register("track", 1)(_identity)


# ------------------------------------------------------------- 0.3.0 -> 0.4.0
#
# 0.4.0 loosened the tyre compound from a two-value enum to the same
# pattern-constrained vocabulary surfaces already use, and added the optional
# provenance `data` block naming the recordings a fit consumed (ADR-0041).
# A loosening and an optional addition move no field, so the step is the
# identity for every kind: every 0.3.0 document is already a valid 0.4.0 one.

for _kind in _KINDS_0_1_0 + ("track",):
    register(_kind, 3)(_identity)


# ------------------------------------------------------------- 0.4.0 -> 0.5.0
#
# 0.5.0 makes the sensor file carry what the sensor models consume
# (ADR-0048). Two of its changes touch fields a 0.4.0 file could hold:
#
# - ``latency.jitter_stddev`` becomes ``latency.jitter``, the half-width of
#   the uniform jitter the models actually implement. The value is converted
#   by the square root of three, which preserves the variance the author
#   stated: the same physical quantity under a new parameterisation, not an
#   invented number.
# - The free ``noise`` object is gone, replaced by typed per-sensor blocks.
#   Its keys were never defined, so no migration can restructure a non-empty
#   one; per the module docstring, that fails loudly rather than dropping
#   data. An empty one carried nothing and is removed.
#
# Every other kind moves by identity.

_SQRT_3 = 3.0 ** 0.5


@register("sensors", 4)
def _sensors_0_4_to_0_5(document: Document) -> Document:
    out = dict(document)
    migrated = []
    for entry in document.get("sensors", []):  # type: ignore[union-attr]
        working = dict(entry)
        noise = working.pop("noise", None)
        if noise:
            name = working.get("name", "?")
            raise ValueError(
                f"sensor '{name}' carries a free-form 'noise' object, whose "
                f"keys were never defined, so no migration can restructure "
                f"it without guessing. Restate it in the 0.5.0 shape: a "
                f"typed block named after the sensor type (ADR-0048)."
            )
        latency = working.get("latency")
        if isinstance(latency, dict) and "jitter_stddev" in latency:
            latency = dict(latency)
            sigma = latency.pop("jitter_stddev")
            if isinstance(sigma, (int, float)):
                # Half-width of the uniform jitter with the same variance the
                # author stated as a standard deviation.
                latency["jitter"] = float(sigma) * _SQRT_3
            else:
                latency["jitter"] = sigma  # let validation name the problem
            working["latency"] = latency
        migrated.append(working)
    out["sensors"] = migrated
    return out


for _kind in ("car", "dynamics", "limits", "provenance", "tyre", "track"):
    register(_kind, 4)(_identity)
