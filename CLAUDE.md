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

**Current state: scaffolding + an approved API design.** No public API exists yet.
`docs/api-design.md` is the contract for the one that lands: a recipe/evaluator
split, a B-rep-shaped surface, immediate-mode features, selectors instead of
handles, and `Exactness` on every measurement. Read it before writing any public
type.

## Hard rules

- **Layering is `decad -> sketch -> r3`.** NEVER import decad from either; they
  do not know it exists.
- **NEVER re-derive a 2D answer.** Profile closure, DOF, constraint conflicts,
  sketch validity → ask `sketch`, consume its answer.
- **NEVER hand-roll coordinate math.** Vectors, frames, local↔world transforms →
  `r3`. Its `Frame` is orthonormal, so the inverse is the transpose, never a
  matrix solve.
- **Shapes belong HERE.** `r3` excludes them by charter; solids/surfaces/meshes/
  topology are this module's job.
- **NEVER add a public API that contradicts `docs/api-design.md`.** Extending it
  is fine; changing a decision means changing the doc first.
- **NEVER expose triangles as the representation, indices as selectors, or a bare
  `float64` measurement.** These are the forward-compatibility invariants that keep
  an exact-kernel future reachable (`docs/api-design.md` §3). Scalar quantities —
  values and their error bounds alike — are `units.Value`. The single exception is
  the `r3.Vec` coordinate, which is a length in millimetres by convention
  (`docs/api-design.md` §5.2); it is not a licence for any other bare float.
- **NEVER add a `go.mod` module without recording the decision here.** Approved:
  - `github.com/lestrrat-3d/sketch` — parametric 2D constraint engine.
  - `github.com/lestrrat-3d/r3` — 3D coordinate math (`Vec`, `Frame`).
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
| `docs/api-design.md` | **The public API contract.** Recipe/evaluator split, forward-compat invariants, feature + selector + verification surface. |
| `doc.go` | Package doc: scope + the layering contract. |
| `wiring_test.go` | Dependency smoke test — solves a `sketch` profile, lifts it to world space via `r3.Frame`. Asserts nothing about decad. **Delete when real decad code imports both deps.** |
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
