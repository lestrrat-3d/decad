# CLAUDE.md

Guidance for working in this repository. Read before making structural changes.
Update when a design variable gets resolved.

## What this is

A **headless CAD engine** in Go — the 3D modeling layer above the `sketch` 2D
constraint engine. Build solids in code, then interrogate them programmatically.

**North-star use case:** a *headless 3D verification oracle*. A coding agent
models a part here and proves it sound — watertight, correct volume, no
interference, no wall thinner than the tool — BEFORE committing to write real
CAD software code (e.g. an Autodesk Fusion add-in). Be wrong in the cheap place.

**Current state: the public API is landing incrementally against an approved
design.** What the package exports today is the leading edge of that surface;
everything it does not yet export remains design-only, and every capability the
design consumes exists in its dependencies — there is no open dependency gap.
`docs/api-design.md` is the core contract for the whole surface.
`docs/layout.md`'s Design documents table lists every companion design and what
it owns, and names what every other file in the package owns.

## Read before you write

| Before writing | Read |
|---|---|
| Any file, to find what owns what | `docs/layout.md` — one row per root `.go` file and per design doc |
| Any public type | `docs/api-design.md`, and every companion design `docs/layout.md`'s "Design documents" table lists |
| Recipe codecs or evaluation entry points | `docs/recipe-replay-design.md` |
| Evaluator, topology or feature code | `docs/evaluator-design.md` |
| Tessellation, export or mesh-boolean operand code | `docs/tessellation-design.md` |
| Free-form geometry or per-segment-kind dispatch | `docs/spline-design.md` |
| `evaluateBoolean` dispatch, `Union`/`Cut`/`Intersect`, or any code combining two recorded sections through a private `sketch` scene | `docs/prism-boolean-design.md` |
| Any modify op, option codec or modify payload | `docs/modify-design.md`, `docs/modify-reach-design.md` |
| Anything the surrounding `.go` file already documents | that file's own doc comments |

## Hard rules

- **Layering is `decad -> sketch -> r3 -> units`.** decad imports all three
  directly. NEVER import decad from any of them; they do not know it exists.
- **NEVER re-derive a 2D answer.** Profile closure, DOF, constraint conflicts,
  sketch validity, an intersection, a cut parameter, a projection onto a curve →
  ask `sketch`, consume its answer. Where `sketch` reports its own answer
  approximate — a `Partial` fragment whose cut is sampled, or an uncertified
  `Partial` fragment (`BoundaryEdge.TExact` false;
  `docs/sketch-seam-design.md`) — decad **rejects**. It never repairs,
  projects, fits, or infers the exact answer. A whole (non-`Partial`) edge
  records from the entity's own data and never consults `TExact`. Building a
  private `sketch` scene from decad's OWN recorded entities and asking it to
  arrange them is not re-deriving an answer — the moments engine already does
  this for authentication (`moments_validate.go`), and
  `docs/prism-boolean-design.md` extends it to combining two recorded sections:
  decad selects among the regions `sketch` returns; it never computes the
  crossing, cut parameter, or containment itself.
- **A decad-side check may only FALSIFY an upstream claim, never bless one.**
  Admission is decided by what `sketch` says — `BoundaryEdge.TExact` for a
  `Partial` fragment — never by a test decad runs on the
  geometry it was handed. A residual against a source curve is admissible in exactly
  one direction: **large ⇒ the claim is disproven ⇒ reject**; **small ⇒ proves
  nothing** — a sampled cut can lie arbitrarily close to the curve, so a small
  residual NEVER admits an input (`docs/sketch-seam-design.md`). A check that can accept
  is an admission gate, and an admission gate on a residual is unsound. Reject-only,
  always.
- **NEVER hand-roll coordinate math.** Vectors, frames, local↔world transforms →
  `r3`. Its `Frame` is orthonormal, so the inverse is the transpose, never a
  matrix solve.
- **Shapes belong HERE.** `r3` excludes them by charter; solids/surfaces/meshes/
  topology are this module's job.
- **NEVER add a public API that contradicts the design docs** —
  `docs/api-design.md` and every design document listed in the Layout table.
  Extending them is fine; changing a decision means changing the doc first.
- **NEVER expose triangles as the representation, indices as selectors, or a bare
  `float64` measurement. NEVER give a boolean a target-out parameter or let it
  mutate an operand.** These are the forward-compatibility invariants that keep
  an exact-kernel future reachable (`docs/api-design.md` §3). Scalar quantities —
  values and their error bounds alike — are `units.Value`. Exactly two things are
  not scalar quantities and so fall outside the rule (`docs/api-design.md` §5.2):
  the **coordinate** — an `r3.Vec`, or a plane-local `Point2` — which is a length in
  millimetres by convention; and the **curve parameter** — a spline's degree, knots
  and weights, a recorded segment's parameter range (`TStart`/`TEnd`), a conic's
  fullness `Rho` — which is a dimensionless index into a parameterisation, not a
  measurement of anything.
  Neither is a licence for a bare float anywhere else.
- **NEVER add a `go.mod` module without recording the decision here.** Approved:
  - `github.com/lestrrat-3d/sketch` — parametric 2D constraint engine.
  - `github.com/lestrrat-3d/r3` — 3D coordinate math (`Vec`, `Frame`,
    `Transform`).
  - `github.com/lestrrat-3d/units` — typed quantities (`Value`, `Kind`).
    Direct: decad's `Measurement` and `Recipe` quantities are `units.Value`.
    It is the same module `sketch` uses for its dimensions (`sketch` has no
    in-tree units package), so there is no parallel unit system to reconcile.
  - `github.com/lestrrat-go/option/v3` — functional options (house library). Used
    by feature options.
  - `github.com/stretchr/testify/require` — assertions, **test code only**.
    NEVER import from production code.
  - `github.com/lestrrat-3d/solidlens` — pure-Go headless raster renderer for
    triangle meshes. Required by the `_gallery` module ALONE, which is where the
    README images are rendered. It is deliberately NOT a dependency of the
    decad library: nothing under the root module imports it, and it appears in
    neither the root `go.mod` nor its `go.sum`.
- **Correctness must be observable.** Every capability ships with a test
  asserting on computed geometry (coordinates, volumes, residuals) — NEVER
  merely "it ran".

## Conventions

- Go style, testing and file-layout rules: `~/.claude/docs/go.md`. Tests use
  `testify/require` (never `assert`), external `_test` package, `t.Context()`.
- User-facing usage → executable Go examples in `examples/` with verified
  `// Output:` blocks. NEVER README-only snippets.
- Docs state **current state only** — no changelogs, no "was X, now Y".
- Design docs live in `docs/<topic>-design.md`, and every one of them carries a
  row in `docs/layout.md`, as does every non-test `.go` file in the root.

## Verification

```
go test ./...      # must pass
go vet ./...       # must pass
golangci-lint run  # v2.12.2, config in .golangci.yml
```
