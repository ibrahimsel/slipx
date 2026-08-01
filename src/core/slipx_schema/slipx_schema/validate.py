# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""JSON Schema validation with usable errors (SCH-02).

jsonschema does the checking. This module does the part that matters to the
person holding the broken file: turning a ValidationError into a field path and
a permitted range.
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, List

import jsonschema

from .errors import FieldError

SCHEMA_DIR = Path(__file__).parent / "schema"

#: Every schema document shipped with this package.
SCHEMA_FILES = {
    "car": "car.schema.json",
    "dynamics": "dynamics.schema.json",
    "limits": "limits.schema.json",
    "sensors": "sensors.schema.json",
    "provenance": "provenance.schema.json",
    "tyre": "tyre.schema.json",
}


@lru_cache(maxsize=None)
def load_schema(kind: str) -> Dict[str, Any]:
    """Read one schema document by kind, e.g. ``"dynamics"``."""
    if kind not in SCHEMA_FILES:
        raise KeyError(f"unknown schema kind '{kind}'; known: {sorted(SCHEMA_FILES)}")
    with (SCHEMA_DIR / SCHEMA_FILES[kind]).open(encoding="utf-8") as handle:
        return json.load(handle)


_FILENAME_TO_KIND = {filename: kind for kind, filename in SCHEMA_FILES.items()}


def _inline_cross_file_refs(node: Any, depth: int = 0) -> Any:
    """Replace ``$ref`` to a sibling schema FILE with that file's contents.

    Cross-file references are resolved by inlining rather than by handing
    jsonschema a resolver, for two reasons. The mechanism for supplying one
    changed between jsonschema 4.17 and 4.18 (RefResolver gave way to the
    referencing library), and pinning either would make this package fight
    with whatever else is in a student's environment. And a resolver that can
    be handed a base URI is a resolver that can be pointed at the network,
    which is not acceptable in a validator that has to work on a competition
    laptop with no connectivity.

    Internal ``#/$defs/...`` references are left alone; jsonschema handles
    those without any resolution machinery.
    """
    if depth > 16:
        raise RecursionError("schema $ref nesting is too deep; is there a cycle?")

    if isinstance(node, dict):
        ref = node.get("$ref")
        if isinstance(ref, str) and ref in _FILENAME_TO_KIND:
            target = dict(load_schema(_FILENAME_TO_KIND[ref]))
            target.pop("$schema", None)
            target.pop("$id", None)
            resolved = _inline_cross_file_refs(target, depth + 1)
            # Anything alongside the $ref (a description, usually) is kept and
            # wins, matching how a reader would expect the local note to
            # override the referenced one.
            extra = {k: v for k, v in node.items() if k != "$ref"}
            resolved.update(_inline_cross_file_refs(extra, depth + 1))
            return resolved
        return {k: _inline_cross_file_refs(v, depth + 1) for k, v in node.items()}

    if isinstance(node, list):
        return [_inline_cross_file_refs(v, depth + 1) for v in node]

    return node


@lru_cache(maxsize=None)
def _validator(kind: str):
    schema = _inline_cross_file_refs(load_schema(kind))
    cls = jsonschema.validators.validator_for(schema)
    cls.check_schema(schema)
    return cls(schema)


def _path_of(error: jsonschema.ValidationError) -> str:
    """Dotted path to the offending field, with array indices in brackets."""
    parts: List[str] = []
    for element in error.absolute_path:
        if isinstance(element, int):
            parts.append(f"[{element}]")
        else:
            parts.append(f".{element}" if parts else str(element))
    return "".join(parts)


def _permitted_of(error: jsonschema.ValidationError) -> str:
    """The permitted range or set, read back out of the failing keyword.

    SCH-02 asks for the permitted range in the message. Recovering it from the
    schema rather than restating it in prose means the message cannot drift
    away from the rule it is describing.
    """
    schema = error.schema
    if not isinstance(schema, dict):
        return ""

    if "enum" in schema:
        return "one of " + ", ".join(repr(v) for v in schema["enum"])
    if "const" in schema:
        return repr(schema["const"])

    bounds = []
    if "exclusiveMinimum" in schema:
        bounds.append(f"> {schema['exclusiveMinimum']}")
    elif "minimum" in schema:
        bounds.append(f">= {schema['minimum']}")
    if "exclusiveMaximum" in schema:
        bounds.append(f"< {schema['exclusiveMaximum']}")
    elif "maximum" in schema:
        bounds.append(f"<= {schema['maximum']}")
    if bounds:
        unit = ""
        description = schema.get("description", "")
        if "[" in description and "]" in description:
            unit = " " + description[description.rindex("[") : description.rindex("]") + 1]
        return " and ".join(bounds) + unit

    if "pattern" in schema:
        return f"matching {schema['pattern']}"
    if error.validator == "type":
        return f"type {schema.get('type')}"
    if "minLength" in schema or "maxLength" in schema:
        lo = schema.get("minLength", 0)
        hi = schema.get("maxLength", "unbounded")
        return f"length {lo} to {hi}"
    return ""


def _message_of(error: jsonschema.ValidationError) -> str:
    """A message aimed at the file's author rather than at a library user."""
    if error.validator == "required":
        # The most common failure, and the one where jsonschema's own wording
        # is least helpful. Silent defaulting is prohibited, so a missing
        # field is always an error and the message says why it is not simply
        # filled in.
        missing = error.message.split("'")[1] if "'" in error.message else "?"
        return (
            f"required field '{missing}' is missing. It has no default: "
            f"silently substituting a value would make a guessed parameter "
            f"indistinguishable from a measured one"
        )
    if error.validator == "additionalProperties":
        return (
            f"{error.message}. Unknown fields are refused rather than ignored, "
            f"because an ignored field is a parameter its author believed was "
            f"in effect"
        )
    return error.message


def validate_document(
    kind: str, document: Any, file: str = "", requirement: str = "SCH-02"
) -> List[FieldError]:
    """Validate one document against its schema.

    Returns every failure, sorted by field path, rather than stopping at the
    first: a file with four mistakes should take one run to fix.
    """
    errors: List[FieldError] = []
    for error in sorted(_validator(kind).iter_errors(document), key=lambda e: list(e.absolute_path)):
        errors.append(
            FieldError(
                path=_path_of(error),
                message=_message_of(error),
                permitted=_permitted_of(error),
                file=file,
                requirement=requirement,
            )
        )
    return errors
