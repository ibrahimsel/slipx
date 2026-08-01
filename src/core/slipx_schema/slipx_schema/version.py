# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Schema versioning and the SCH-01 compatibility rule.

slipx_schema is versioned independently of slipx_core (NFR-09). A schema
addition the core never sees must not force a core release, and a core ABI
break must not invalidate every car file in existence.

The compatibility rule, in one sentence: a parser accepts its own major
version at its own minor or older, migrates older minors forward, and refuses
everything else.

Refusing a NEWER minor deserves a word, because tolerating it is the more
common choice. A newer minor may add a field, and a field this parser does not
know about is a field it will ignore. Ignoring a parameter that the file's
author believed was in effect is exactly the silent behaviour SCH-02 exists to
prevent: the car runs, produces a plausible number, and is not the car that was
described.
"""

from __future__ import annotations

from dataclasses import dataclass

#: The schema version this package implements and writes.
SCHEMA_VERSION = "0.1.0"


@dataclass(frozen=True, order=True)
class Version:
    major: int
    minor: int
    patch: int

    @classmethod
    def parse(cls, text: str) -> "Version":
        parts = text.split(".")
        if len(parts) != 3:
            raise ValueError(
                f"'{text}' is not a semantic version; expected MAJOR.MINOR.PATCH"
            )
        try:
            return cls(int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError as exc:
            raise ValueError(
                f"'{text}' is not a semantic version; expected MAJOR.MINOR.PATCH"
            ) from exc

    def __str__(self) -> str:
        return f"{self.major}.{self.minor}.{self.patch}"


CURRENT = Version.parse(SCHEMA_VERSION)


def compatibility(version: Version) -> tuple[bool, str]:
    """Whether ``version`` can be handled, and why not when it cannot.

    Returns:
        ``(True, "")`` when the file can be parsed, possibly after migration.
        ``(False, reason)`` otherwise, where ``reason`` is written for the
        person holding the file rather than for a log.
    """
    if version.major != CURRENT.major:
        return False, (
            f"file declares schema {version}, this parser implements "
            f"{CURRENT}. A different major version means fields have changed "
            f"meaning, so the file is refused rather than guessed at. Install "
            f"a slipx release implementing schema {version.major}.x, or "
            f"migrate the file."
        )
    if version.minor > CURRENT.minor:
        return False, (
            f"file declares schema {version}, this parser implements "
            f"{CURRENT}. A newer minor may set fields this parser does not "
            f"know about, and silently ignoring a parameter its author "
            f"believed was in effect would produce a car that is not the one "
            f"described. Upgrade slipx."
        )
    return True, ""
