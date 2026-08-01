# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Distribution version.

Tracks slipx_core's version, and is deliberately not the same number as
slipx_schema's: the two are versioned independently (NFR-09) because a schema
addition the core never sees must not force a core release, and a core change
must not invalidate every car file.

Zero major: the API is not stable and semantic versioning's guarantees begin at
1.0.0.
"""

__version__ = "0.1.0"
