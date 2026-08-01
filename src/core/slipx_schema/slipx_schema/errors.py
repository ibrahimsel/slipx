# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Errors and warnings raised while loading a car directory.

SCH-02 requires a validation failure to name the offending field path and the
permitted range. That is not decoration. The people this library is for are
students debugging a config file at midnight before a competition, and
"ValidationError: 0.06 is not of type 'string'" with a stack trace into a
third-party library tells them nothing about which of six files is wrong.

The other half of SCH-02 is that silent defaulting is prohibited. A missing
required field is an error here, never a quietly substituted value, because a
defaulted parameter is indistinguishable from a measured one once it is in
memory.
"""

from __future__ import annotations

from dataclasses import dataclass, field


class SlipxSchemaError(Exception):
    """Base for everything this package raises."""


@dataclass
class FieldError:
    """One validation failure, located and explained.

    Attributes:
        path: Dotted path to the offending field, e.g.
            ``geometry.track_front``. Empty string for a failure at the root of
            the document.
        message: What is wrong, in plain words.
        permitted: The permitted range or set, as text. Empty when the failure
            is not a range failure (a missing required field, for instance).
        file: Which file in the car directory the failure is in.
        requirement: The requirement ID this check exists because of, so a
            reader can go and find out why the rule is there.
    """

    path: str
    message: str
    permitted: str = ""
    file: str = ""
    requirement: str = ""

    def __str__(self) -> str:
        location = f"{self.file}: " if self.file else ""
        where = self.path if self.path else "<document root>"
        text = f"{location}{where}: {self.message}"
        if self.permitted:
            text += f" (permitted: {self.permitted})"
        if self.requirement:
            text += f" [{self.requirement}]"
        return text


class ValidationError(SlipxSchemaError):
    """One or more fields failed validation.

    Carries every failure found rather than only the first. A car file with
    four mistakes in it should take one run to fix, not four.
    """

    def __init__(self, errors: list[FieldError], context: str = "") -> None:
        self.errors = errors
        self.context = context
        header = context or "validation failed"
        body = "\n".join(f"  - {e}" for e in errors)
        super().__init__(f"{header}:\n{body}")


class SchemaVersionError(SlipxSchemaError):
    """The file's ``schema_version`` cannot be handled (SCH-01).

    An unknown MAJOR is refused outright: a major bump means fields changed
    meaning, and guessing at a file from the future is how a car silently
    becomes a different car. An unknown newer MINOR is also refused, for the
    same reason in miniature; older known minors are migrated.
    """


class CarDirectoryError(SlipxSchemaError):
    """The car directory is missing a file, or names one that is not there."""


@dataclass
class Warning_:
    """A value that validates but looks wrong.

    Warnings are for the SCH-04 plausibility checks, where the honest position
    is that the value is possible but improbable. Refusing them would make the
    parser wrong about unusual cars; hiding them would make it useless for the
    common case where somebody has entered millimetres.
    """

    path: str
    message: str
    file: str = ""
    requirement: str = ""

    def __str__(self) -> str:
        location = f"{self.file}: " if self.file else ""
        text = f"{location}{self.path}: {self.message}"
        if self.requirement:
            text += f" [{self.requirement}]"
        return text


@dataclass
class Report:
    """The outcome of validating a car directory."""

    errors: list[FieldError] = field(default_factory=list)
    warnings: list[Warning_] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def raise_if_failed(self, context: str = "") -> None:
        if self.errors:
            raise ValidationError(self.errors, context)
