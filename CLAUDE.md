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

## Documentation split

- `docs/` holds user-facing documentation, including the canonical API reference.
- `agent-docs/` holds design decisions and maintenance instructions for agents.

## Read before you write

| Before writing | Read |
|---|---|
| Any file, to find what owns what | `agent-docs/layout.md` — one row per root `.go` file and design doc |
| Any public type | `docs/api-design.md`, and every companion design listed in `agent-docs/layout/` |
| Collision geometry or rigid-body dynamics | `agent-docs/collision-dynamics-design.md`, `agent-docs/multibody-dynamics-design.md` |
| Linkages and joints | `agent-docs/linkage-check-design.md` |
| Evaluator, topology or feature code | `agent-docs/evaluator-design.md` |
| Tessellation, export or mesh-boolean operands | `agent-docs/tessellation-design.md` |
| STEP export | `agent-docs/step-export-design.md` |
| Free-form geometry or per-kind dispatch | `agent-docs/spline-design.md` |
| `Union`/`Cut`/`Intersect`, `evaluateBoolean` dispatch, or any code combining recorded sections through a private `sketch` scene | `agent-docs/prism-boolean-design.md`, `agent-docs/general-boolean-design.md` |
| Mirror, pattern, `Transform`/`Copy` | `agent-docs/mirror-pattern-design.md` |
| Any modify op, option codec or modify payload | `agent-docs/modify-design.md`, `agent-docs/modify-reach-design.md` |
| `stackedPrismPayload`, blind `Cut`, slabs | `agent-docs/stacked-prism-design.md` |
| Sheet-body, surface-result, patch or stitch code | `agent-docs/surface-design.md` |
| `Trim`, `Extend` or `Split` code | `agent-docs/surface-intersection-design.md` |
| Anything the surrounding `.go` file documents | its doc comments |
| Answering what is missing, or closing/adding a refusal | `agent-docs/missing-features.md` |
| Tests and CI shards | `agent-docs/testing.md` |
| Adding or importing a `go.mod` module | `agent-docs/modules.md` |

## Hard rules

- **Layering is `decad -> sketch -> r3 -> units`.** decad imports all three
  directly. NEVER import decad from any of them.
- **Ask `sketch` for 2D answers by default.** Profile closure, DOF, constraint
  conflicts, sketch validity, an intersection, a cut parameter, a projection
  onto a curve → ask `sketch`, consume its answer. decad computes a 2D answer
  itself only where that clearly wins on performance or correctness, and the
  design doc owning that code states the reason (e.g. the coplanar contact
  patch of two exact planar faces, clipped exactly:
  `agent-docs/multibody-dynamics-design.md` §9.4). Building a private `sketch` scene
  from decad's OWN recorded entities and asking it to arrange them is the
  default's usual shape (`internal/momentinput/record_validation.go`,
  `agent-docs/prism-boolean-design.md`, `agent-docs/surface-intersection-design.md`):
  decad selects among the regions, chains and cells `sketch` returns. The
  soundness half is absolute: where `sketch` reports its own answer
  approximate — an uncertified `Partial` fragment (`BoundaryEdge.TExact`
  false; `agent-docs/sketch-seam-design.md`) — decad **rejects**. It never repairs,
  projects, fits, or infers the exact answer. A whole (non-`Partial`) edge
  records from the entity's own data and never consults `TExact`.
- **A decad-side check may only FALSIFY an upstream claim, never bless one.**
  Admission is decided by what `sketch` says — `BoundaryEdge.TExact` for a
  `Partial` fragment — never by a test decad runs on the
  geometry it was handed. A residual against a source curve is admissible in exactly
  one direction: **large ⇒ the claim is disproven ⇒ reject**; **small ⇒ proves
  nothing** — a sampled cut can lie arbitrarily close to the curve, so a small
  residual NEVER admits an input (`agent-docs/sketch-seam-design.md`). A check that can accept
  is an admission gate, and an admission gate on a residual is unsound. Reject-only,
  always.
- **NEVER hand-roll coordinate math.** Vectors, frames, local↔world transforms →
  `r3`. `Frame` is orthonormal to rounding: inverse = transpose, never a
  solve; exact readings charge the defect (`massmoment.MapCharge`).
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
- **NEVER add a `go.mod` module without recording the decision in
  `agent-docs/modules.md`.**
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
- Keep the root package to the public API where practical. Put everything the
  user does not see — algorithms, geometry kernels, helpers, report and request
  plumbing — in an `internal/` package. The root keeps the exported types, their
  methods and thin wrappers that call into `internal/`. Unexported root code
  that cannot move (e.g. it needs unexported fields of a root type) says why in
  its doc comment.
- New unexported root helpers take plain data (`r3.Vec`, floats, slices), NEVER
  `*Document`/`*Body`/`*Face`/`*Edge`/`*Mesh`, to fit `internal/`. Root keeps
  wrappers; topology code stays. Narrow leaf helpers in PRs.
- User-facing usage → executable Go examples in `examples/` with verified
  `// Output:` blocks. NEVER README-only snippets.
- Docs state **current state only** — no changelogs, no "was X, now Y".
- Design docs live in `agent-docs/<topic>-design.md`, and every one of them carries a
  row in `agent-docs/layout/`, as does every non-test `.go` file in the root.

## Verification

```
go test ./...      # must pass
go vet ./...       # must pass
golangci-lint run  # v2.12.2, config in .golangci.yml
```
