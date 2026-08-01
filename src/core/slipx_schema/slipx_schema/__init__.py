# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""slipx_schema: car and track description, and the reference parser.

Sits between the ``slipx`` Python package and ``slipx_core`` in the dependency
stack, and depends on neither. That is the point: ``slipx_core`` must build and
pass its full test suite with this package absent (CORE-01), because parameters
enter the core as a plain struct and parsing is somebody else's problem. This
is somebody else.

Typical use::

    from slipx_schema import load_car

    car = load_car("examples/cars/reference_1_10")
    print(car.summary())        # leads with the provenance label (NFR-08)
    car.params.mass             # ready to hand to slipx_core

Versioned independently of ``slipx_core`` (NFR-09). See ``version.py`` for the
compatibility rule.
"""

from .errors import (
    CarDirectoryError,
    FieldError,
    Report,
    SchemaVersionError,
    SlipxSchemaError,
    ValidationError,
    Warning_,
)
from .loader import load_car, validate_car
from .model import Car, Provenance, Tyre, VehicleParameters
from .rules import (
    LENGTH_MAX_M,
    LENGTH_MIN_M,
    ROBORACER_RULESET,
    WIDTH_MAX_M,
    WIDTH_MIN_M,
)
from .validate import SCHEMA_FILES, load_schema, validate_document
from .version import SCHEMA_VERSION, Version, compatibility

__all__ = [
    "SCHEMA_VERSION",
    "SCHEMA_FILES",
    "Car",
    "CarDirectoryError",
    "FieldError",
    "LENGTH_MAX_M",
    "LENGTH_MIN_M",
    "Provenance",
    "ROBORACER_RULESET",
    "Report",
    "SchemaVersionError",
    "SlipxSchemaError",
    "Tyre",
    "ValidationError",
    "VehicleParameters",
    "Version",
    "WIDTH_MAX_M",
    "WIDTH_MIN_M",
    "Warning_",
    "compatibility",
    "load_car",
    "load_schema",
    "validate_car",
    "validate_document",
]

__version__ = SCHEMA_VERSION
