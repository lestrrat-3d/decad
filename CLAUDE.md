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
`docs/api-design.md` is the core contract for the whole surface. The Layout
table's Design documents section lists every companion design and what it owns.

## Read before you write

| Before writing | Read |
|---|---|
| Any public type | `docs/api-design.md`, and every companion design the Layout table's "Design documents" section lists |
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
- **Correctness must be observable.** Every capability ships with a test
  asserting on computed geometry (coordinates, volumes, residuals) — NEVER
  merely "it ran".

## Layout

Every row names what a file owns and where its detail lives. A row is a
POINTER, never a summary — see "Layout rows" under Conventions before editing
one.

### Design documents

| Path | Responsibility |
|---|---|
| `docs/api-design.md` | The core public API contract: recipe/evaluator split, forward-compat invariants, and the feature, selector and verification surface. Points at every companion design. |
| `docs/sketch-seam-design.md` | The recording contract at the `sketch` seam: the trim contract (`TExact`), the `CurveSegment` recording IR, and `ErrUnrecordableProfile`. |
| `docs/recipe-replay-design.md` | The stored-recipe contract: strict versioned encoding, validation and liveness, error precedence, resource limits, and whole-recipe atomic evaluation. |
| `docs/verification-design.md` | How `Verify` judges every bounded result: the report and its statuses, interference cost and deadlines, `WithTolerance`, and the diameter-anchored noise floor. |
| `docs/payload-verification-design.md` | How `Verify` covers each evaluator payload: per-payload proofs, boundary certificates, bounded validity/clearance/survey algorithms, and required tests. |
| `docs/evaluator-design.md` | The v1 evaluator: the evaluate-from-the-record rule, topology and provenance roles, mass properties, per-feature build tables, staging via `ErrUnsupported`, and the mesh boolean. |
| `docs/tessellation-design.md` | The tessellation contract for every payload: shared curve samples, manifold proofs, source faces, boundary certificates, and boolean handoff. |
| `docs/clearance-design.md` | The clearance kernel: stationarity-tier candidate enumeration, the face-pair distance table, the disjointness proof, and how rows feed the report. |
| `docs/interference-design.md` | Non-mutating pairwise overlap proof: the four-way pair relation, containment volume reuse, read-only mesh intersection, cancellation, and refusal rules. |
| `docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` in four normative tables (receiver, refusals, result, consumers), plus the section-rewrite reduction, the exact offset, and the build-time audit. |
| `docs/spline-design.md` | The free-form kinds: per-kind exactness tiers, refusals and their sentinels, exact rational Tier A moments and their work budget, proven brackets, and reach per capability. |
| `docs/modify-reach-design.md` | The approved modify extension: tangent-chain expansion, asymmetric chamfers, cap-loop blends, allowed shells, proof gates, payload topology and staging. |
| `docs/loft-design.md` | The count-free `Loft` design in four normative tables (pairing, refusals, result, consumers), its exact-rational mass properties, and the wall-crossing audit. |
| `docs/prism-boolean-design.md` | The analytic reduction for `Union`/`Cut`/`Intersect` over co-directional coplanar prisms: the reject-only entry gate, the private `sketch` scene, and the section-displacement bound. |

### Seam, records and recipes

| Path | Responsibility |
|---|---|
| `doc.go` | Package doc: scope, the evaluator support-and-refusal map, and the layering contract (`decad -> sketch -> r3 -> units`). |
| `errors.go` | The core §12 sentinel error vocabulary, plus the H2 typed `BooleanError` (`Op`/`Inputs`/`Code`) whose `BooleanErrorCode` set wraps `ErrBooleanFailed` or `ErrUnsupported`. See `docs/api-design.md` §12, §8. |
| `measurement.go` | The bounded-result shapes: `Exactness`, `Measurement`, `VecMeasurement`, `Box`. See `docs/api-design.md` §5.3, §6. |
| `record.go` | The recording IR: `PlaneRecord`, `ProfileRecord`, `LoopRecord`, and the ten sealed `CurveSegment` variants with their tagged codec. NURBS validation rules are documented on their own functions. See `docs/sketch-seam-design.md` §2. |
| `seam.go` | The seam conversion `RecordProfile(s, p)`: admits, authenticates and records a profile, then applies the `TExact` admission gate and reject-only falsifier. See `docs/sketch-seam-design.md` §1, §7. |
| `extent.go` | The extent vocabulary: the sealed linear `Extent`/`SideExtent` and angular `AngularExtent`/`SideAngular` tiers, deliberately disjoint, each with a tagged codec. `ToFace`/`ToFaceAngular` record their body as a `StepRef`. See `docs/api-design.md` §8.1. |
| `recipe.go` / `recipe_wire.go` | The Recipe IR, Step's wire codec, and the strict/versioned root codec returning a path-aware `RecipeError`. Placement and per-op field presence are enforced on both wire directions. See `docs/recipe-replay-design.md` §2, §3.2, §6. |
| `recipe_decode.go` | The bounded JSON preflight scanner behind `Recipe`'s decoder: charges every array element and string byte against the fixed resource-limit ceilings before typed allocation. See `docs/recipe-replay-design.md` §7. |
| `selector.go` | The selector vocabulary: `EdgeQuery`/`FaceQuery`, predicate conjunction plus `Exactly`/`AtLeast` cardinality, and their tagged codec. Resolution is a filter pipeline over live topology; a failing resolution returns a `SelectionError`. See `docs/api-design.md` §9. |
| `selection_error.go` | `SelectionError` (wraps `ErrNoMatch`/`ErrCardinality`) and the canonical `*Query.String()` rendering it and a verification `Diagnostic` both reuse. See `docs/api-design.md` §9. |
| `codec_error.go` | The path-aware codec error machinery behind every recorded-step decode: builds the JSON-style path a `RecipeError` reports. See `docs/recipe-replay-design.md` §6. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | The mass-property engine (evaluator §4): closed-form Green's-theorem boundary integrals for `Area`, `Centroid`, `SecondMoments`; Tier A free-form terms route through one record-level work preflight. See `docs/spline-design.md` §5.2. |
| `moments_trig.go` | `moments.go`'s certified sine/cosine primitive: `turnSinCosInterval` proves an enclosure of sin/cos of an exact rational turn without ever comparing against π. See this file's own doc comment. |
| `spline_bezier.go` | The exact reduction of `docs/spline-design.md` §5.1: converts a recorded free-form curve to piecewise polynomial Bézier control points over `big.Rat`, with no rounding. Owns the §5.2 work-budget charges. |
| `spline_length.go` | `docs/spline-design.md` §6.1: brackets a free-form curve's arc length between its chord and control polygon, narrowed by exact dyadic de Casteljau bisection to a fixed depth. |
| `spline_extreme.go` | `docs/spline-design.md` §6.2: brackets a Tier A free-form span's directional extreme over `gu·u + gv·v`, reducing to `clearance_poly.go`'s certified root engine via the Bernstein convex-hull property. Owns Table R row R18's refusal. See the file's own doc comment. |
| `spline_fit.go` | `docs/spline-design.md` §5.1.2's fit-spline reduction: converts a recorded `FitSplineSeg` into the same `bezierSpan` chain the other Tier A kinds produce, over its own closed form rather than knot insertion. See the file's own doc comment. |
| `spline_moments.go` | The exact integration of `docs/spline-design.md` §5.1 over Bézier spans, reusing `clearance_poly.go`'s `ratPoly`. `addFreeform` feeds `moments.go`'s region-level rational accumulator. |

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | The topology model (evaluator §3): `Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge`→`Vertex`, plus sealed `Surface`/`Curve` variant sets. Convexity, exactness rules and immutability are on the types' own doc comments; see `docs/evaluator-design.md` §3. |
| `document.go` | `Document` (`New`/`Bodies`/`Recipe`), its atomic commit tail, and retire/liveness gates. `Body.Placed`/`Duplicate`/`PlacedCopy` re-evaluate the payload under a composed motion; see their doc comments and `docs/evaluator-design.md` §8. |
| `extrude.go` | `Document.Extrude` (evaluator §5) plus `segmentWalk`/`walkKind`, the boundary vocabulary every feature reads through, and the analytic prism evaluator `evalPrism`/`prismPayload`. See doc comments; `docs/evaluator-design.md` §5, `docs/prism-boolean-design.md` §7, `docs/spline-design.md` §6.2. |
| `revolve.go` | `Document.Revolve` (evaluator §6): the sealed `Axis` vocabulary, the axis-contact/incidence gates, angular-extent resolution, and the analytic revolve evaluator `evalRevolve`/`revolvePayload`. See doc comments on each; `docs/evaluator-design.md` §6. |
| `stops.go` | Body-relative stop resolution for `ToFace`/`ToFaceAngular`/`ThroughAll`/`ThroughAllSide` (evaluator §5/§6/§11, core §8.1/§6.2): each stop body resolves at the call and records as a `StepRef`, never consumed. See doc comments on `resolveToFace`/`resolveThroughAll`/`resolveToFaceAngular`. |
| `loft.go` | `docs/loft-design.md` PR 1b: `Document.Loft`/`LoftContext`, the public entry point over `loft_build.go`'s evaluator. Owns gates S9–S11 and S4's arity half; the step commits only after `evalLoft` succeeds. See doc comments; `docs/loft-design.md` §2/§4/§10. |
| `loft_build.go` | `docs/loft-design.md` PR 1a: `loftPayload` and `evalLoft` — Table P pairing, Table S gates S1–S5, the flat-triangle wall/cap construction, and the four published measurements. See doc comments; `docs/loft-design.md` §5–§8. |
| `loft_audit.go` | The build-time crossing audit of `docs/loft-design.md` §6: `loftCrossingAudit` proves the assembled triangle set manifold and watertight, reusing `boolean_exact.go` and `boolean_mesh.go`'s `triTriClassify` unchanged. Gate order is S6, S8, S7. |
| `loft_moments.go` | `docs/loft-design.md` §8's mass-property engine: `loftMassAccumulator`, an exact-rational tetrahedron sum over the assembled triangle set, publishing Volume/Centroid/Bounds with one final rounding. Area is never Exact. |

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | `Body.Fillet` rewrites a straight prism's section into a tangent arc at each selected corner and rebuilds through `evalPrism`. It also defines the shared `cornerBlend` machinery Chamfer reuses. See `docs/modify-design.md` §6. |
| `chamfer.go` | `Body.Chamfer` bevels a straight prism's lateral corners with a chord between setback feet, sharing `cornerBlend` machinery with `fillet.go`. A cap-loop selection instead routes to `capblend.go`. See `docs/modify-design.md` §7. |
| `fillet_audit.go` | The shared §5 audit of a modify op's rewritten section — orientation, self-consuming trim, crossing/contact, and nesting — run by Fillet and Chamfer, and reused by Shell's offset audit. See `docs/modify-design.md` §5. |
| `shell.go` | `Body.Shell` removes a prism's cap faces and offsets the section into a wall of thickness `t`, building a tube (both caps) or a `cupPayload` (one cap). See `docs/modify-design.md` §8. |
| `shell_offset.go` | The exact per-feature section offset (`P ⊖ t` / `P ⊕ t`) behind `Shell`, plus the §5 audit wrapper run on the offset section. See `docs/modify-design.md` §7-§8. |
| `shell_cup.go` | `cupPayload` and `evalCup`: the two-co-directional-prism body a one-cap `Shell` builds, with Exact mass properties and roles. See `docs/modify-design.md` §9; clearance stays staged (§12 D6). |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Builds the complete-cap-loop chamfer: `capBlendPayload` plus the selection classification and build gates in `buildCapBlend`. Gate order and sentinels are documented per function; see `docs/modify-reach-design.md` §8.3/§4. |
| `capblend_geom.go` | Builds the `capBlendPayload` topology in `buildCapBand`: trimmed side walls, cap faces, and Plane/Cone chamfer band patches. Patch orientation and area bookkeeping are documented per function; see `docs/modify-reach-design.md` §8.3. |
| `capblend_contour.go` | Proves the cap contour's displacement bound used by every cap-level reading, via interval arithmetic over the same offset-intersection cases `shell_offset.go` and `fillet.go` use. See `docs/modify-reach-design.md` §8.4. |
| `capblend_centroid.go` | Computes closed-form first moments for the cap-blend payload's centroid: exact-rational Plane patch moments, a Fourier sum for Cone patches, and a bounding-box ceiling on the result. See `docs/modify-reach-design.md` §8.4. |
| `capblend_moments.go` | `evalCapBlendContext` builds the cap-blend body and its bounded area/volume/centroid via closed-form divergence-theorem integrals per patch, exact where representable. See `docs/modify-reach-design.md` §8.4. |
| `capblend_survey.go` | The cap-blend payload's undercut and minimum-radius surveys: exact per-patch normal ranges, and reuse of the receiver's own unchanged-profile survey for radius. See `docs/modify-reach-design.md` §12 Table DX (DX7/DX8). |

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | Implements `Document.Verify`: builds the report, runs the tolerance gate, and partitions body pairs into disjoint, overlapping, or undecided. `bodyGateDiameter` picks each body's own tolerance-gate diameter; some payloads have none. See `docs/verification-design.md` §1-§3. |
| `clearance.go` | The pair kernel: `clearancePair` proves one pair's four-way relation and, when disjoint, a proven gap interval. The coplanar-plane certificate runs first and short-circuits later checks. See `docs/clearance-design.md` §1/§2/§6. |
| `clearance_degen.go` | The degeneracy oracle every cell asks before emitting a constant/`Exact` candidate: three-valued `degYes`/`degNo`/`degUnknown`, `degYes` only from exact rational arithmetic, never a tolerance. See `docs/clearance-design.md` §4/§5. |
| `clearance_cells.go` | The §3 candidate sink and §4 face-interior table: enumerates stationarity tiers per face pair, folds admission into contributions, and reduces offset-surface pairs to spine-pair criticals. See `docs/clearance-design.md` §3/§4. |
| `clearance_tiers.go` | The curve and vertex tiers of §3: face-edge, edge-edge, and vertex cells over §4's curve-tier table. Constant-distance families emit only on the degeneracy oracle's `degYes`. See `docs/clearance-design.md` §3/§4. |
| `clearance_geom.go` | The kernel's boundary model: builds trimmed carrier faces, edges and vertices from a body's payload and runs the §2 nesting ray casts, closed form except a Sturm-certified torus quartic. See `docs/clearance-design.md` §2/§3. |
| `clearance_poly.go` | The certified-bracket machinery of §4/§5: isolates stationarity polynomials by Sturm sequences over exact rationals, then brackets each critical value by a proven Lipschitz bound. See `docs/clearance-design.md` §4/§5. |
| `survey.go` | The analytic wall, undercut, and min-radius surveys on prism, revolve, and cup payloads. An undecided answer reads `Suspect`, never a silent pass. See `docs/verification-design.md` §6. |
| `survey2d.go` | The 2D closed-form inscribed-disk kernel behind the wall survey, shared with the modify section audit via `elemOf`. Its candidate set is exact for line/arc boundaries. See `docs/verification-design.md` §6. |
| `budget.go` | `workBudget`, the shared bounded work counter read-only and pre-commit audit phases poll via `step`/`err`. It holds closures, never a stored `context.Context`. See `docs/interference-design.md` §7.2. |
| `interference.go` | The pairwise overlap measurement behind `Verify`: containment and equality certificates reuse an operand's volume; every other supported pair runs read-only `OpIntersect` under the positive-volume gate. See `docs/interference-design.md` §4-§8. |

### Booleans

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` surface over the mesh-boolean evaluator and the typed `BooleanError` mapping; tries the prism-boolean analytic reduction first. See `Union`'s doc comment and `docs/evaluator-design.md` §9. |
| `prism_boolean.go` | The analytic `Union` reduction over co-directional coplanar prisms, dispatched from `performBoolean` ahead of the mesh path. See the file's own doc comment and `docs/prism-boolean-design.md`. |
| `boolean_mesh.go` | The exact-predicate mesh-boolean pipeline: contact classification, subdivision, stitching, and the closed-mesh audit. See the file's own doc comment and `docs/evaluator-design.md` §9. |
| `boolean_cut.go` | Per-facet exact subdivision along contact segments into classified regions, in rational 2D on the facet's own plane. See the file's own doc comment. |
| `boolean_exact.go` | The exact-arithmetic kernel behind the mesh boolean: adaptive orient3d, rational predicates, and the reject-only segment pre-filter. The filter's admission-radius derivation lives in `segAdmissionRadius2`'s and `tooFar`'s own doc comments. |
| `boolean_body.go` | Builds a `facetedPayload` into a `Body`: face/loop/edge topology from the stitched mesh, and measurements integrated exactly with composed proven bounds. See the file's own doc comment and `docs/evaluator-design.md` §9. |
| `bounds.go` | The single owner of every proven error bound a faceted measurement reports, one helper per mechanism, including `docs/prism-boolean-design.md` §7's section-displacement terms. See the file's own doc comment. |

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh` and `Body.Tessellate`: prism, cup, and faceted payloads build; revolve stays `ErrUnsupported`. See the file's own doc comment and `docs/tessellation-design.md`. |
| `triangulate.go` | The cap triangulator behind `Tessellate`: hole bridging plus reflex-blocked ear clipping, correct for non-convex outlines with holes. See the file's own doc comment. |
| `export.go` | `Body.STL`/`Body.OBJ`: deterministic writers over `Tessellate`, with `WithChordTolerance`'s documented default. See the file's own doc comment. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks) that double as living documentation. Never `package main`. |
| `.github/workflows/` | `ci.yml` (lint → test/tidy/govulncheck), `codeql.yml`. |

## Cross-cutting notes

**Section displacement.** `prismPayload.sectionDelta`
(`docs/prism-boolean-design.md` §7) is the proven displacement between a
payload's recorded section and the section its construction denotes. `evalPrism`
and `tessellate.go` compose it into every measurement and mesh bound they
publish; every other consumer that cannot state or bound it withholds its answer
rather than measure the recorded section as the denoted one. That per-consumer
list lives in `docs/prism-boolean-design.md`'s Implementation notes.

**Cap contour displacement.** The cap-loop chamfer's cap contour is a computed
offset carrying its own proven displacement, the same idea as `sectionDelta` one
construction over. The two terms are independent and are never composed
together; see `docs/modify-reach-design.md`'s Implementation notes.

**Modify audit cancellation.** `Body.FilletContext`, `Body.ChamferContext` and
`Body.ShellContext` share one cancellable `workBudget` through their pre-commit
audits, and `cupWall` extends it under `Document.Verify`'s context. See
`docs/modify-design.md` §5 for polling and cancellation details.

**Context-aware placement.** `Body.PlacedContext`, `Body.DuplicateContext` and
`Body.PlacedCopyContext` accept the caller's context for cancellable rebuild and
placement. See `docs/evaluator-design.md` §8.

## Conventions

- Go style, testing and file-layout rules: `~/.claude/docs/go.md`. Tests use
  `testify/require` (never `assert`), external `_test` package, `t.Context()`.
- User-facing usage → executable Go examples in `examples/` with verified
  `// Output:` blocks. NEVER README-only snippets.
- Docs state **current state only** — no changelogs, no "was X, now Y".
- Design docs live in `docs/<topic>-design.md`, and every one of them carries a
  Layout row.
- **Layout rows are pointers, not summaries.** A row states what the file owns
  in one or two sentences and names where the detail lives, within 300
  characters. NEVER grow a row to record an invariant, a derivation, a sign
  convention or a refusal — that belongs in the owning design doc or the
  function's own doc comment, and a row that restates it drifts from the code.
  `claude_md_layout_test.go` enforces every mechanical rule above, and, because
  a guard measures only the text it reads, it classifies the WHOLE file rather
  than the Layout section alone: a line it reads as a heading or as part of a
  table, without being the one spelling declared for it, fails the test rather
  than being skipped, since a skipped line escapes the cap and the path check
  both. So headings here are ATX (`## Heading`), the file carries exactly one
  `## Layout` heading, and a table outside that section exists only if the
  guard declares it — adding one to this file means declaring it there first.
  That guard's `parseCLAUDEMD` doc comment owns the complete rule list, and the
  file-level comment above it owns the two things the rules deliberately leave
  to the byte budget.

## Verification

```
go test ./...      # must pass
go vet ./...       # must pass
golangci-lint run  # v2.12.2, config in .golangci.yml
```
