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
`docs/api-design.md` is the core contract for the whole surface: a
recipe/evaluator split, a B-rep-shaped surface, immediate-mode features,
selectors instead of handles, and `Exactness` on every measurement. Two
companion designs carry its deep ends: `docs/sketch-seam-design.md` (the trim
contract and recording IR at the `sketch` seam) and
`docs/verification-design.md` (the tolerance gate and noise floor). Read them
before writing any public type. `docs/evaluator-design.md` is the v1
evaluator's own design — topology construction, mass properties, staging
(`ErrUnsupported`), the mesh boolean, the Verify implementation; read it
before writing evaluator/topology/feature code.

## Hard rules

- **Layering is `decad -> sketch -> r3 -> units`.** decad imports all three
  directly. NEVER import decad from any of them; they do not know it exists.
- **NEVER re-derive a 2D answer.** Profile closure, DOF, constraint conflicts,
  sketch validity, an intersection, a cut parameter, a projection onto a curve →
  ask `sketch`, consume its answer. Where `sketch` reports its own answer
  approximate — a `Partial` fragment whose cut is sampled, `BoundaryEdge.TExact`
  false (`docs/sketch-seam-design.md`) — decad **rejects**. It never repairs,
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
  `docs/api-design.md` and its companions `docs/sketch-seam-design.md` and
  `docs/verification-design.md`. Extending them is fine; changing a decision
  means changing the doc first.
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
- **Correctness must be observable.** Every capability ships with a test
  asserting on computed geometry (coordinates, volumes, residuals) — NEVER
  merely "it ran".

## Layout

| Path | Responsibility |
|---|---|
| `docs/api-design.md` | **The core public API contract.** Recipe/evaluator split, forward-compat invariants, feature + selector + verification surface. Points at the two companion designs. |
| `docs/sketch-seam-design.md` | The recording contract at the `sketch` seam: trim contract (`TExact`), the `CurveSegment` recording IR, `ErrUnrecordableProfile`. |
| `docs/verification-design.md` | How `Verify` judges every bounded result: report + statuses, `WithTolerance`, the diameter-anchored noise floor. |
| `docs/evaluator-design.md` | The v1 evaluator: evaluate-from-the-record rule, topology model + provenance roles, mass properties on decad's own records, per-feature build tables, explicit staging via `ErrUnsupported`, the exact-predicate mesh boolean, Verify's implementation, the increment plan. |
| `doc.go` | Package doc: scope + the layering contract. |
| `errors.go` | The core §12 sentinel error vocabulary — one branchable identity per case an agent must branch on. |
| `measurement.go` | The bounded-result shapes (core §5.3/§6): `Exactness`, `Measurement`, `VecMeasurement`, `Box`. |
| `record.go` | The recording IR (seam §2): `PlaneRecord`, `Point2`, `ProfileRecord`, `LoopRecord`, the ten sealed `CurveSegment` variants, and their tagged JSON codec (pointer variants normalize to values; nil pointers and unknown/missing kind tags are rejected). |
| `seam.go` | The seam conversion: `RecordProfile(s, p)` — the §7 gates (`ErrForeignProfile`/`ErrStaleProfile`/`ErrInvalidProfile`), the `TExact` admission gate, and the reject-only falsifier (`falsifyRange`, evaluating through `sketch/geom`'s own evaluators — the only `Polyline` read, endpoints only, check-never-record). |
| `extent.go` | The linear extent vocabulary (core §8.1): `Direction` (named text codec), the sealed `Extent`/`SideExtent` tiers (`Distance`/`ThroughAll`/`Symmetric`/`TwoSided`; `DistanceSide`/`ThroughAllSide`) and their tagged codecs. `ToFace` and the angular set land with selectors/Revolve (evaluator §11). |
| `recipe.go` | The Recipe IR (core §6.2): `Recipe`/`Step`/`StepRef`/`OpKind` (named text codec)/`BodyRef`/`Selector` root/`StepOpts` (`ExtrudeOpts`), and Step's wire codec — absent fields omitted, every sealed field dispatched through its closed-set codec, selectors rejected until the query types land. |
| `moments.go` | The mass-property engine (evaluator §4): exact Green's-theorem boundary integrals over a `ProfileRecord` — `Area()` (a `Measurement`, Kind Area, Exact/zero-bound) and `Centroid()` (a `VecMeasurement` whose Value is PLANE-LOCAL (u, v, 0) mm) — plus `SecondMoments()` (∫u²dA/∫uv dA/∫v²dA about the plane origin, Kind SecondMomentOfArea — what a revolve centroid needs) — closed forms for line/circle/arc walks incl. pointer variants; a `CircleSeg` whose `CCW` contradicts its range order is `ErrDegenerate`; free-form kinds are `ErrUnsupported` until their increments. |
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks) that double as living documentation. Never `package main`. |
| `.github/workflows/` | `ci.yml` (lint → test/tidy/govulncheck), `codeql.yml`. |

## Conventions

- Go style, testing and file-layout rules: `~/.claude/docs/go.md`. Tests use
  `testify/require` (never `assert`), external `_test` package, `t.Context()`.
- User-facing usage → executable Go examples in `examples/` with verified
  `// Output:` blocks. NEVER README-only snippets.
- Docs state **current state only** — no changelogs, no "was X, now Y".
- Design docs live in `docs/<topic>-design.md`.

## Verification

```
go test ./...      # must pass
go vet ./...       # must pass
golangci-lint run  # v2.12.2, config in .golangci.yml
```
