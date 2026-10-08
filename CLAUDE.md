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

**The public API lands incrementally against approved designs; unshipped
APIs are design-only.**
`docs/api-design.md` is the core contract for the whole surface.

## Read before you write

| Before writing | Read |
|---|---|
| Any file, to find what owns what | `docs/layout.md` — one row per root `.go` file and design doc |
| Any public type | `docs/api-design.md`, and every companion design listed in `docs/layout.md` |
| Collision geometry or rigid-body dynamics | `docs/collision-dynamics-design.md`, `docs/multibody-dynamics-design.md` |
| Linkages and joints | `docs/linkage-check-design.md` |
| Evaluator, topology or feature code | `docs/evaluator-design.md` |
| Tessellation, export or mesh-boolean operands | `docs/tessellation-design.md` |
| STEP export | `docs/step-export-design.md` |
| Free-form geometry or per-kind dispatch | `docs/spline-design.md` |
| `Union`/`Cut`/`Intersect`, `evaluateBoolean` dispatch, or any code combining recorded sections through a private `sketch` scene | `docs/prism-boolean-design.md`, `docs/general-boolean-design.md` |
| Mirror, pattern, `Placed`/`PlacedCopy` | `docs/mirror-pattern-design.md` |
| Any modify op, option codec or modify payload | `docs/modify-design.md`, `docs/modify-reach-design.md` |
| `stackedPrismPayload`, blind `Cut`, slabs | `docs/stacked-prism-design.md` |
| Sheet-body, surface-result, patch or stitch code | `docs/surface-design.md` |
| `Trim`, `Extend` or `Split` code | `docs/surface-intersection-design.md` |
| Anything the surrounding `.go` file documents | its doc comments |
| Answering what is missing, or closing/adding a refusal | `docs/missing-features.md` |

## Hard rules

- **Layering is `decad -> sketch -> r3 -> units`.** decad imports all three
  directly. NEVER import decad from any of them; they do not know it exists.
- **Ask `sketch` for 2D answers by default.** Profile closure, DOF, constraint
  conflicts, sketch validity, an intersection, a cut parameter, a projection
  onto a curve → ask `sketch`, consume its answer. decad computes a 2D answer
  itself only where that clearly wins on performance or correctness, and the
  design doc owning that code states the reason (e.g. the coplanar contact
  patch of two exact planar faces, clipped in exact rational arithmetic:
  `docs/multibody-dynamics-design.md` §9.4). Building a private `sketch` scene
  from decad's OWN recorded entities and asking it to arrange them is the
  default's usual shape (`internal/momentinput/record_validation.go`,
  `docs/prism-boolean-design.md`, `docs/surface-intersection-design.md`):
  decad selects among the regions, chains and cells `sketch` returns. The
  soundness half is absolute: where `sketch` reports its own answer
  approximate — an uncertified `Partial` fragment (`BoundaryEdge.TExact`
  false; `docs/sketch-seam-design.md`) — decad **rejects**. It never repairs,
  projects, fits, or infers the exact answer. A whole (non-`Partial`) edge
  records from the entity's own data and never consults `TExact`.
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
    decad's inputs and `Measurement`s are `units.Value`, the module `sketch`
    uses too, so no parallel unit system exists.
  - `github.com/lestrrat-3d/step` — AP214; `export` package only.
  - `github.com/lestrrat-go/option/v3` — functional options (house library). Used
    by feature options.
  - `github.com/stretchr/testify/require` — assertions, **test code only**.
    NEVER import from production code.
  - `_gallery` module ALONE; the root `go.mod`/`go.sum` never list them:
    - `github.com/lestrrat-3d/solidlens` — pure-Go mesh rasterizer; README
      images.
    - `github.com/lestrrat-3d/kinetograph` — animates decad bodies; landing
      clip (`go run . clip`) and dynamics scenes (`go run . dynamics`).
- **Tooling lives in its own nested module.** `_gallery/` renders the README
  images and landing clip, and `_shardgen/` packs the race shards. Both carry
  their own `go.mod` and an `_` prefix, so the root module, its linter and
  `go list ./...` never see them. A generator NEVER joins the library's module.
- **Correctness must be observable.** Every capability ships with a test
  asserting on computed geometry (coordinates, volumes, residuals) — NEVER
  merely "it ran".

## Conventions

- Go style, testing and file-layout rules: `~/.claude/docs/go.md`. Tests use
  `testify/require` (never `assert`), `t.Context()`. Exported-API-only test →
  `apitest/`.
- User-facing usage → executable Go examples in `examples/` with verified
  `// Output:` blocks. NEVER README-only snippets.
- Docs state **current state only** — no changelogs, no "was X, now Y".
- Design docs live in `docs/<topic>-design.md`, and every one of them carries a
  row in `docs/layout.md`, as does every non-test `.go` file in the root.

## Verification

- **ALWAYS update `.github/test-shards.txt` or
  `.github/test-shards-apitest.txt` after adding, renaming, or removing a
  test, fuzz target, or example.**
- **ALWAYS run `go test . ./apitest/ -run '^TestCI'` before pushing any
  test-name change.**

```
go test ./...      # must pass
go vet ./...       # must pass
golangci-lint run  # v2.12.2, config in .golangci.yml
```
