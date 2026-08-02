# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Distribution version.

Tracks slipx_core's version, and is deliberately not the same number as
slipx_schema's: the two are versioned independently (NFR-09) because a schema
addition the core never sees must not force a core release, and a core change
must not invalidate every car file.

Zero major: the API is not stable and semantic versioning's guarantees begin at
1.0.0. The ``a1`` suffix is PEP 440 for a pre-release: it is the first artefact
published to an index, it exists so that the packaging path itself is exercised
against a version nobody should pin, and pip will not select it once a final
release exists.

This string is checked against pyproject.toml, CMakeLists.txt and
slipx/version.hpp by tools/version_check.py, which CI runs. There are four
places a version is written and no way to make there be one; the check is what
stops them drifting.
"""

__version__ = "0.1.0a1"
