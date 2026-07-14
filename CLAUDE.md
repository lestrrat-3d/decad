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
| `extent.go` | The extent vocabulary (core §8.1): `Direction` (named text codec), the sealed linear `Extent`/`SideExtent` tiers (`Distance`/`ThroughAll`/`Symmetric`/`TwoSided`; `DistanceSide`/`ThroughAllSide`) and the sealed angular `AngularExtent`/`SideAngular` tiers (`AngleExtent`/`FullRevolution`/`SymmetricAngle`/`TwoSidedAngle`; `AngleSide`), each with its own tagged codec — the two sets are disjoint in the type system and on the wire. `ToFace`/`ToFaceAngular` land with selector resolution (evaluator §11). |
| `recipe.go` | The Recipe IR (core §6.2): `Recipe`/`Step`/`StepRef`/`OpKind` (named text codec)/`BodyRef`/`Selector` root/`StepOpts` (`ExtrudeOpts`), and Step's wire codec — absent fields omitted, every sealed field (selectors included) dispatched through its closed-set codec, the extent/angular one-of keying enforced on both wire directions. |
| `selector.go` | The selector vocabulary (core §9): `EdgeSelector`/`FaceSelector`, the `*EdgeQuery`/`*FaceQuery` variants of `Selector` (predicate conjunction + `Exactly`/`AtLeast` cardinality), the `EdgePredicate`/`FacePredicate` constructors (`Convex`/`Concave`/`ParallelTo`/`LongerThan`/`CreatedBy`/`Circular`; `Planar`/`Cylindrical`/`NormalTo`/`FaceCreatedBy`), and their tagged codec. The variants seal in with POINTER receivers, so there is no value form to normalize to: nil pointers are rejected (`errNilSelector` wraps `ErrDegenerate`) and the clone helpers deep-copy — no caller-owned pointer survives into a recorded step, none escapes `Recipe()`. Resolution (`SelectEdges`/`SelectFaces`, evaluator §7) is staged `ErrUnsupported` until evaluator increment 2. |
| `moments.go` | The mass-property engine (evaluator §4): exact Green's-theorem boundary integrals over a `ProfileRecord` — `Area()` (a `Measurement`, Kind Area, Exact/zero-bound) and `Centroid()` (a `VecMeasurement` whose Value is PLANE-LOCAL (u, v, 0) mm) — plus `SecondMoments()` (∫u²dA/∫uv dA/∫v²dA about the plane origin, Kind SecondMomentOfArea — what a revolve centroid needs) — closed forms for line/circle/arc walks incl. pointer variants; a `CircleSeg` whose `CCW` contradicts its range order is `ErrDegenerate`; free-form kinds are `ErrUnsupported` until their increments. |
| `topology.go` | The topology model (evaluator §3): `Body`→`Lump`→`Shell`→`Face`→`Loop`→`Edge`→`Vertex` with the core §6/§6.1 accessors, sealed `Surface` (`Plane`/`Cylinder`/`Cone`/`Sphere`/`Torus` shipped; `SurfaceKind` discriminants) and `Curve` (`Line3`/`Circle3`/`Arc3`) sets, multi-role `Face.Origins()`, bounded `Vertex.Position()`. Bodies are immutable after construction. |
| `document.go` | `Document` (`New`/`Bodies`/`Recipe`), the atomic commit tail (record→evaluate→commit; a failed evaluation leaves recipe and document untouched), retire/`requireLive` gates, `Body.Placed` (re-evaluates the payload under the composed rigid motion), and `magnitudeIn` (Kind/finite/non-negative validation). |
| `extrude.go` | Increment-1 `Document.Extrude` (evaluator §5): gates via `RecordProfile`, `WithTaper` recorded-then-`ErrUnsupported`, extent resolution to the signed interval (`Distance`/`Symmetric`/`TwoSided`-of-distance-sides; through-all staged), and the analytic prism evaluator — shared-edge manifold topology, Exact volume/area/centroid, exact directional-extreme bounds. |
| `revolve.go` | `Document.Revolve` (evaluator §6): the sealed `Axis` vocabulary (`SketchLine`/`ConstructionAxis`/`EdgeAxis` + tagged codec; `Step.Axis` keyed to `OpRevolve` on the wire like `Angular`), the §6 axis gates (coplanar with the profile plane; region in one closed half-plane, either side; contact only as a segment endpoint or an on-axis `LineSeg` — interior tangency is `ErrDegenerate`), angular-extent resolution to the sweep interval (a 2π magnitude IS a full revolution, past it is `ErrDegenerate`; `EdgeAxis` resolution staged `ErrUnsupported` with the selectors), and the analytic revolve evaluator — `Cylinder`/`Cone`/planar-annulus/`Sphere`/`Torus` walls per segment kind, caps only on partial sweeps (a full sphere/torus face carries no loops), latitude-circle seam vertices, Pappus-exact volume/area/centroid, exact cylindrical-extreme bounds. |
| `verify.go` | Increment-1 `Document.Verify` (evaluator §10/§11, verification design): `Report`/`BodyReport`/`Status` (worst-wins precedence) + `Trustworthy()`, `Interference`/`Clearance` row types, the `VerifyOption` set (`WithTolerance` validated; the survey options accepted, parameter-validated, and read `Suspect` unanswered), the structural boundary audit (validity by construction), the Exact-passes tolerance gate, and box-disjointness pair proofs — an undecided pair reads `Suspect`, never a fabricated row. |
| `tessellate.go` | `Mesh` + `Body.Tessellate(tol)` (core §11, evaluator §9): an OUTPUT, never the representation. Chordings are chosen once per boundary curve and shared by every face meeting it, so the mesh is watertight and consistently oriented by construction; facets remember their source `*Face`; `Mesh.Bound` is the proven max chord sagitta (≤ tol; zero for an all-planar body). Tolerance validated like every magnitude, zero → `ErrDegenerate`; a body this evaluator did not build → `ErrUnsupported`. |
| `triangulate.go` | The cap triangulator behind `Tessellate`: polygon-with-holes → CCW triangles by hole bridging (Eberly max-u visibility) + reflex-blocked ear clipping — correct for non-convex outlines with holes, deterministic; a stalled clip is `ErrDegenerate`, never a wrong mesh. |
| `export.go` | `Body.STL`/`Body.OBJ` (core §11): deterministic writers over `Tessellate` — ASCII STL (chosen for diffable, byte-stable output), OBJ vertex table + 1-based faces. `WithChordTolerance` is an `STLOBJOption` (jwx combined-interface pattern) both accept; without it the tolerance defaults to 1/1000 of the body's bounding-box diagonal. |
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
