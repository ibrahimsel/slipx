# ADR-0021: The README is the PyPI long description

- **Status:** Accepted
- **Date recorded:** 2026-08-02
- **Requirements:** NFR-08
- **Related:** [ADR-0013](0013-provenance-labels-are-printed.md),
  [ADR-0017](0017-first-release-is-a-pre-release.md)

## Context

A PyPI project page needs a long description. The alternative to reusing the
README is a shorter document written for PyPI.

That alternative is tempting because the README is long and a package page
could be a summary. It is wrong here for a specific reason: the README's job is
to state the scope and the limitations plainly, and a second document doing the
same job is a second place those limitations have to be kept accurate. They
would drift, and the direction of drift is predictable. The shorter, more
promotional document is the one that loses the sentence about no parameter set
having been validated.

There is also a claim-discipline requirement that a package page must satisfy
regardless of which document is used: it must not imply any parameter set is
validated, and it should say which tiers actually exist. The README already does
both in its second paragraph.

## Decision

`readme = "README.md"` in `pyproject.toml`. The whole README is the PyPI long
description.

Consequently every link and image in the README must be an absolute URL, because
PyPI serves the text from a different origin and resolves nothing relative to
the repository. A comment at the top of `README.md` says so, so that the next
person to add a link knows before rather than after.

The metadata is held to the same discipline as the prose. The `description`
field says what the library is and does not characterise how good it is:
"validated", "accurate" and "high-fidelity" are all claims this project is not
entitled to make about any shipped parameter set, and a one-line summary is the
worst place to start making them. `Development Status :: 3 - Alpha` stays until
L2 exists and at least one parameter set has a validation report attached; it is
not a placeholder to be bumped when the code feels finished.

## Consequences

The PyPI page is long. That is acceptable; the people it is for read
documentation.

Two concrete bugs were caught by adopting this rule and then applying it: the
banner image and one documentation link were repository-relative and would have
rendered broken on the package page, and the "there is no PyPI release yet"
paragraph would have been published by the release that falsified it.

The README now serves three audiences at once: GitHub, PyPI, and the sdist. A
change that makes sense on one has to be checked against the other two.
`twine check --strict` runs in the release workflow and catches the mechanical
half of that.
