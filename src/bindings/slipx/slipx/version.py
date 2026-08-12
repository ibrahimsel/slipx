# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Distribution version.

Tracks slipx_core's version, and is deliberately not the same number as
slipx_schema's: the two are versioned independently (NFR-09) because a schema
addition the core never sees must not force a core release, and a core change
must not invalidate every car file.

Zero major: the API is not stable and semantic versioning's guarantees begin at
1.0.0. Until 1.0.0 a minor bump may break the API, and this one does: 0.2.0 is
the first final release, and ``pip install slipx`` selects it in preference to
the 0.1.0a1 pre-release that came before it. There is no 0.1.0 and there never
will be.

This string is checked against pyproject.toml, CMakeLists.txt and
slipx/version.hpp by tools/version_check.py, which CI runs. There are four
places a version is written and no way to make there be one; the check is what
stops them drifting.
"""

__version__ = "0.2.0"
