# ADR-0019: Publish with Trusted Publishing, not an API token

- **Status:** Accepted
- **Date recorded:** 2026-08-02
- **Requirements:** none directly
- **Related:** [ADR-0017](0017-first-release-is-a-pre-release.md),
  [ADR-0018](0018-wheel-coverage.md)

## Context

Uploading to PyPI from CI needs credentials. The two options are a long-lived
API token stored as a repository secret, and PyPI's Trusted Publishing, which
exchanges a GitHub OIDC token for a short-lived, tightly scoped upload
credential.

A token is simpler to set up and has three properties worth naming. It is
long-lived, so it is valid until somebody remembers to rotate it. Its scope is
the account or the project rather than a particular workflow, so anything that
can read the secret can publish. And it is invisible in use: a token upload
leaves no evidence tying the artefact to the commit it was built from.

For a project whose entire argument is that provenance should be checkable, and
which hashes the compiler ID into its run manifests, a shared secret in a
settings page is the wrong shape.

## Decision

Publication uses PyPI Trusted Publishing (OIDC). No API token exists in the
repository or its settings.

The publisher is scoped to owner `ibrahimsel`, repository `slipx`, workflow
`release.yml`, and a named GitHub environment: `testpypi` for the rehearsal and
`pypi` for the real index. A run outside that combination cannot mint a
credential.

The release workflow is built around the same principle:

- **Dispatch only.** Nothing publishes on a push or a tag.
- **A separate `publish` boolean**, so a full build-and-verify run with nothing
  uploaded is the default way to use it.
- **PyPI uploads must be dispatched from a tag.** An artefact on PyPI is
  permanent and has to be traceable to a commit that cannot move afterwards.
- **A GitHub environment with a required reviewer on `pypi`**, so the
  irreversible step needs an approval that a workflow file cannot grant itself.
- **No `skip-existing` on the real index.** A collision with an existing version
  means the version number was wrong, and the right outcome is a loud failure
  rather than a quiet partial release.

## Consequences

Setup is more involved than pasting a token: two GitHub environments, and a
pending publisher registered on each index before the first upload. Pending
publishers are the mechanism for a name that does not yet exist, so this is also
how the name is reserved.

The `id-token: write` permission is required on the publish jobs. It is granted
per job rather than workflow-wide, so the build and verification jobs cannot
mint a credential even if something in them is compromised.

Required reviewers on an environment need a public repository or a paid plan.
On a private repository on the free plan the gate silently does not exist, which
was discovered while configuring it, and is one more reason the repository is
public.

Nothing here prevents a human uploading with `twine` and a token from a laptop.
That path is deliberately not documented, because a release that did not go
through the workflow did not go through the version check, the sdist content
check or `twine check --strict`.
