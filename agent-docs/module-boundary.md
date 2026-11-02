# Module boundary gate

**Run this gate before implementing production behavior and before approving its PR.**
Existing code, filenames, passing tests and layout rows do not waive this gate.

## Reusable CAD behavior

- MUST state the accepted input class and resulting CAD behavior in the owning
  design doc before adding a route. Use reusable geometry, topology, tolerance
  and provenance invariants.
- NEVER add production dispatch, admission predicates, proof exceptions or
  retries tailored to a named application, manufactured part or test fixture.
  Put part construction in caller or consumer repositories; implement the
  reusable CAD capability in this module. Follow [Test scope](../agent-docs/testing.md#test-scope)
  for repository fixtures.
- Geometry classes may have specialized algorithms. MUST define their support
  through geometric invariants independent of the part being modeled.
- MUST fail the gate when a route is justified only by one modeled part or
  workflow. Stop before implementation and report the unsupported CAD operation
  and the reusable capability needed to support it.
- MUST demonstrate the declared input class with independent cases that vary
  its relevant geometry and construction. A renamed fixture, neutral filename
  or move into `internal/` does not establish reusable behavior.

## Root package placement

- MUST put private algorithms, geometry kernels, helpers, and report/request
  plumbing in `internal/` packages. Keep exported types, their methods, root
  topology code and thin adapters in root.
- MUST design new unexported helpers for internal packages, taking plain data
  (`r3.Vec`, floats, slices) or internal types. NEVER pass `*Document`, `*Body`,
  `*Face`, `*Edge` or `*Mesh` merely to reach the data; unwrap it in a thin root adapter.
- MUST document why each unexported root adapter needs root access. Name the
  private field or invariant it owns and the internal operation it calls.
  Root-owned history or handles justify an adapter, not the algorithms behind it.
- MUST fail the gate when private algorithms remain in root without that
  boundary. Extract them before implementation continues or the PR is approved.
- MUST keep extraction PRs focused on narrow leaf helpers.

## PR evidence

- MUST include the gate evidence in the PR description for production changes:
  the owning design section and supported input class, the focused cases that
  exercise it, and each new unexported root adapter's required root access.
  State when a change leaves an item unchanged.
- MUST reject the PR when this evidence is missing or either gate section fails.
  Fix the module boundary before approving the PR; layout entries and green CI
  do not substitute for this check.
