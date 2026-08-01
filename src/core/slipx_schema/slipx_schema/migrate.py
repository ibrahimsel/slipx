# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Migrations between known minor versions (SCH-01).

At 0.1.0 there is nothing to migrate: there has been no earlier release. The
mechanism ships anyway, and is tested against a synthetic migration, because
the alternative is writing it under pressure the first time a real schema
change lands and a registry full of contributed files needs to survive it.

A migration is a pure function from one document to the next. It must set
``schema_version`` to the version it produces, and it must fail loudly rather
than dropping a field it does not understand: a migration that quietly
discards data is worse than one that refuses to run.
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
