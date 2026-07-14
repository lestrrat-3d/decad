# Evaluator Design (v1)

How the v1 evaluator turns a `Recipe` step's exact inputs into a `Body` — the
topology it builds, the measurements it computes and the bounds it proves, how
selectors resolve against it, how the tessellation-backed boolean works, and
how `Verify`'s checks are implemented. Companion to `docs/api-design.md` (the
core contract this evaluator implements — references of the form "core §N"),
`docs/sketch-seam-design.md` (the profile records it consumes), and
`docs/verification-design.md` (how its answers are judged). Nothing here
changes those contracts; where this document stages a capability, the staging
is explicit and rejected loudly, never silently approximated.

## 1. Two sources of truth, one direction

**The evaluator consumes the `Step`'s own records, never the live inputs.** A
feature call gates its live inputs (core §7), records them — the profile as
`ProfileRecord` via the seam conversion, the plane as `PlaneRecord`, a
placement as `TransformRecord`, quantities as `units.Value` — and then
evaluates **from the record**. The live `*sketch.Profile` is never read after
recording. This is what makes core §2's promise mechanical: re-evaluating a
`Recipe` re-runs exactly the same inputs, so a re-evaluated model is the same
model, and a second evaluator answers the same question better rather than a
different question.

The one thing the evaluator reads from the live profile, at feature-call time
only, is a **falsifier input**: `Profile.Area` (sketch's own area answer) is
compared against the area the evaluator computes from the records (§4). A
large mismatch rejects the call — the record and the profile disagree, which
is a bug somewhere — and a small one proves nothing and admits nothing, the
same one-sided shape as the seam's range falsifier.

## 2. What v1 is, and is not

Per core §2: **v1 is analytic where construction is free, and
tessellation-backed exactly where exactness dies — the boolean.** A body built
by features alone (extrude, revolve, placed) carries analytic faces and Exact
measurements. A body touched by a boolean carries `Faceted` faces and
Approximate measurements with proven bounds. There is no third state.

**Staging is explicit.** v1 lands in increments (§11), and an intent the
recipe can record but this evaluator cannot yet build is **rejected at the
call** with the sentinel `ErrUnsupported` (core §12) — never built
approximately, never silently narrowed. The recipe/evaluator split is exactly
what makes this honest: the *vocabulary* is complete from the start, and
`ErrUnsupported` names the evaluator's reach, not the API's.

## 3. The topology model

Concrete structs with unexported fields; the accessors of core §6/§6.1 are the
whole public surface. Identity is the pointer, scoped to one body — never an
index (core §3 invariant #3).

| Type | Holds |
|---|---|
| `Body` | owning `*Document`, origin `FeatureRef`, `[]*Lump`, cached measurements (computed at build, immutable after) |
| `Lump` | `[]*Shell` |
| `Shell` | `[]*Face`, `void bool` |
| `Face` | tagged `Surface`, `[]*Loop` (first outer), origin `FeatureRef`, back-ref to its body |
| `Loop` | ordered `[]*coedge` (edge + sense), `outer bool` |
| `Edge` | tagged `Curve`, `Start`/`End` `*Vertex`, ALL adjacent faces (exactly 2 on a closed manifold body; `Edge.Faces()` reports the actual count, which is precisely how `len != 2` surfaces non-manifold topology — core §6.1), `convex bool` |
| `Vertex` | position (mm), the proven bound on it |

Rules:

- **A `Body` is immutable after construction.** Every operation builds a new
  body; concurrency safety (core §12) falls out.
- **Provenance is structural.** `FeatureRef` identifies the producing
  `StepRef` plus a stable role within it — `side(i, j)` (loop `i`, segment
  `j`), `capStart`, `capEnd`, and the revolve/boolean analogs. Roles derive
  from the recorded step, so re-evaluation reproduces them and `CreatedBy`
  selects the same faces under every run.
- **Canonicalize at build.** Adjacent coplanar side faces merge; a full
  cylinder is one face with two circular-edge loops and no seam edge. v1
  counts already match the analytic answer, so vN does not churn them
  (core §3).
- **Every vertex carries its bound.** Feature-built vertices are exact (bound
  zero); boolean-built vertices carry the tessellation's chord bound. The
  verification gate reads these (verification §4).

## 4. Mass properties — decad's own, on decad's own records

The hard rule "NEVER re-derive a 2D answer" forbids re-deciding what sketch
has decided: closure, validity, cuts, intersections, projections. It does
**not** put the recorded region's *mass properties* out of reach — once
recorded, the region is decad's own geometry, and volumes, centroids and
areas of the bodies built from it are decad's 3D job. The boundary is:

- **sketch decides topology and admissibility** — what closes, what is valid,
  where curves cross, what a trim is. decad consumes.
- **decad integrates its own records** — area, first moments, arc length,
  extremes of the recorded loops, by closed form per segment kind: shoelace
  over segment chords plus the exact circular-segment correction for
  `CircleSeg`/`ArcSeg` fragments (the same correction family geom uses;
  independent code on decad's own data, asserted against sketch's answer by
  the §1 falsifier at every feature call).

Increment 1 implements the closed forms for `LineSeg`/`CircleSeg`/`ArcSeg`;
the free-form kinds arrive with their increments (§11) and reject
`ErrUnsupported` until then.

## 5. Extrude

Input: the recorded profile + plane, a linear `Extent`, options. The sweep
direction is the plane normal `U × V` for `Along` (core §8.1).

Faces, per recorded loop segment (roles in parentheses):

| Segment | Side face |
|---|---|
| `LineSeg` | `Plane` (`side(i,j)`) |
| `CircleSeg` (whole) | full `Cylinder`, no seam edge |
| `CircleSeg` (fragment) / `ArcSeg` | `Cylinder` patch bounded by two line edges + two arc edges |
| free-form kinds | later increments: `NURBSSurface` where the control net is exactly derivable from the record (ellipse/elliptical-arc/conic — exact rational forms; spline/closed-spline/NURBS — extruded control net); `FitSplineSeg` stays `ErrUnsupported` until geom exports its interpolant's B-spline form — decad NEVER re-runs the interpolation solve (seam §2) |

Caps: one planar face per end (`capStart`/`capEnd`), the outer loop plus one
loop per hole, holes wound opposite.

Measurements (untapered, increment-1 kinds): all **Exact**, bound zero. The
extent resolves to a signed sweep interval `[z0, z1]` along the plane normal
(`Along` positive; `Against`, `Symmetric` and `TwoSided` sides place `z0`/`z1`
on their own senses), and every formula reads the interval: `Volume =
A·(z1−z0)` (`A` the recorded region's area per §4), `Centroid` the region
centroid lifted `(z0+z1)/2` along the normal — `h/2` only in the one-sided
`Along` case — `Bounds` from per-segment analytic extremes swept over
`[z0, z1]`, `Area` from cap areas + side areas (`segment length · (z1−z0)`;
arc length `rθ` exact). Extents: `Distance`,
`Symmetric`, and `TwoSided` of distance sides land in increment 1 — the three
whose interval the step's own quantities determine. `ThroughAll` and
`ThroughAllSide` have no finite stop geometry of their own (they stop at the
far side of every body the sweep meets), so they are body-relative exactly
like `ToFace`/`ToFaceAngular`: all four land in increment 2 with selectors
(§7), the stop an intersection of the sweep direction with analytic target
surfaces — closed form — and `ErrUnsupported` until then. A nonzero
`WithTaper` is recorded exactly and is `ErrUnsupported` in v1: a tapered
extrude of a general region is an offset problem (self-intersecting offsets),
and a wrong-but-confident prism is the failure decad exists to prevent.

## 6. Revolve

Same recording shape; the axis must be validated non-degenerate and coplanar
with the profile plane, and the profile must lie strictly on one side of the
axis (a region crossing its axis sweeps a self-intersecting solid — rejected,
`ErrDegenerate`). Faces: a `LineSeg` parallel to the axis → `Cylinder`;
inclined → `Cone`; perpendicular → planar annulus; `ArcSeg`/`CircleSeg` →
`Torus` (`Sphere` when the arc center lies on the axis). Partial sweeps get
two planar cap faces. Volume/centroid by Pappus on the §4 region integrals —
exact. Increment 2.

## 7. Selectors

Resolution is a filter pipeline over the live body's topology: gather
(`Body.Edges()`/`Faces()`), apply each predicate as a pure function of the
analytic data (`Edge.IsConvex`, `Curve`/`Surface` variants, lengths, normals),
then enforce cardinality (`Exactly`/`AtLeast`; `ErrCardinality` at zero when
asserted, else `ErrNoMatch` — core §12 precedence). Predicates on a `Faceted`
body read the facet data the body holds; a predicate that needs analytic
identity a faceted face no longer has (e.g. `Cylindrical()`) simply does not
match it — matching is decided on what the face IS, never guessed from what
it approximates. Queries and predicates are recordable values (core §6.2);
resolution happens at feature-call time and at re-evaluation identically,
because provenance roles are stable (§3).

## 8. Document, recipe, and re-evaluation

`Document` owns the live body set and the step list. Every feature call:
validate live inputs (gates of core §7/§8) → build the `Step` record as a
value (records + `StepRef` substitution for body references), NOT yet
appended → evaluate from that record (§1) → only on success, commit
atomically: append the step, retire consumed bodies, register the result. A
failed evaluation — `ErrUnsupported` included — leaves the recipe and the
document untouched: a rejected operation is not intent, and a recipe holding
it would re-reject on every re-evaluation. `Body.Placed` transforms
analytic geometry exactly — every v1 surface variant maps to itself under an
isometry (plane→plane, cylinder→cylinder, …), with `IsReflection` flipping
face orientation handling. Re-evaluation (`vN`, or replay for testing) walks
the steps in order and must reproduce the same body count, the same
provenance roles, and measurements within each evaluator's own exactness — a
replay test in the suite asserts this on every example model.

## 9. The boolean — where exactness dies, and how

Increment 4, the deep end. Strategy:

- **Tessellate both operands** with a caller-visible chord tolerance
  (`Tessellate(tol)`'s machinery): per-surface analytic tessellators with a
  proven per-facet deviation bound δ_t; facets remember their source face.
- **Robust mesh boolean in-repo, stdlib-only.** The curated-dependency rule
  stands: no third-party mesh kernel. The algorithm is the exact-predicate
  route: triangle/triangle intersection and point classification decided by
  adaptive-precision sign tests that fall back to `math/big.Rat` exactly at
  the boundary cases — a sign decided exactly is a topology decision that
  cannot flip (core §2.1's whole fear), so the output is watertight **by
  construction** on the tessellated geometry. Retriangulation along
  intersection curves, classification by exact winding tests, stitching by
  shared exact vertices.
- **Output**: faces are `Faceted`, grouped by source analytic face so
  provenance (`CreatedBy`) and face-level selection survive the boolean;
  vertices carry bound δ_t; measurements integrate the mesh exactly and
  report `Approximate` with the verification-design bound shapes
  (volume bound ≈ δ_t · area).
- Rejected alternatives: a third-party kernel (dependency rule; also the
  supply-chain surface); float-only BSP classification (the flipped-sign
  nonsense solid of core §2.1); snapping/welding heuristics (silently moves
  geometry — decad never repairs).

## 10. Verify — how each answer is computed

- **Feature-built bodies** are valid **by construction, and the proof is the
  construction**: a prism/revolve over a sketch-proven-closed, decad-recorded
  region is watertight and manifold structurally (every edge bounds exactly
  two faces by the build tables of §5/§6), and cannot self-intersect when the
  revolve side condition holds. `Verify` still runs the structural audit (an
  invariant check, cheap) but its verdict is decided, not sampled. All
  quantities Exact → the tolerance gate passes them at any `rel`.
- **Faceted bodies** are judged per verification §6: the held boundary
  against its own proven bound — watertightness/manifoldness read off the
  stitched mesh (exact, §9), self-clearance via a spatial grid, decisive
  beyond the bound or `Suspect`.
- **Pairs**: proven disjoint when the two bodies' bounds-inflated boxes are
  disjoint (a box bounds its body, so box separation proves body separation),
  or when a clearance computation clears the summed bounds; proven
  overlapping when a witness point lies inside both beyond their bounds
  (analytic containment tests for analytic bodies; exact mesh classification
  for faceted ones). A pair proven neither joins neither list and reads
  `Suspect` — from increment 3 on, the report is honest before the boolean
  exists, because box-proofs already decide the common far-apart case.
- **Wall thickness / undercuts / min radius**: the analytic surveys of
  verification §6 over the v1 surface set (plane/cylinder/cone/sphere/torus
  pair tables for opposition and pinch; per-variant normal ranges for pull
  membership). Increment 5; until then the options are accepted, the survey
  is not run, and the asked-but-unanswered question reads `Suspect` — never
  a silent pass.

## 11. Increments

Each increment is a PR series behind the same public contract; nothing ships
half-silent — unreachable capability is `ErrUnsupported` at the call.

| # | Lands |
|---|---|
| 1 | topology model, `Document`/`Recipe`/`Step` wiring, `Extrude` for line/circle/arc profiles with `Distance`/`Symmetric`/`TwoSided`-of-distance-sides extents, mass properties (§4), `Placed`, structural `Verify` (validity by construction, quantities, tolerance gate; every pair undecided → `Suspect` unless box-proven disjoint) |
| 2 | `Revolve` (angular extents), selector vocabulary + resolution, the body-relative stops (`ToFace`/`ToFaceAngular`/`EdgeAxis`, `ThroughAll`/`ThroughAllSide`) |
| 3 | pair proofs (box-disjointness, analytic clearance), `WithClearances` |
| 4 | tessellation + the exact-predicate mesh boolean, `Faceted` bodies, faceted `Verify`, `Tessellate`/`STL`/`OBJ` |
| 5 | fillet/chamfer on analytic prism edges, shell, `WithMinWallThickness`/`WithPullDirection`/`WithMinRadius` surveys |
| 6 | free-form side surfaces (`NURBSSurface` from recorded control data), tapered extrude if a sound offset story exists |

## 12. Open questions

- **`FitSplineSeg` side faces** (§5) wait on geom exporting its interpolant's
  B-spline form; decad will not duplicate the solve.
- **Tapered extrude** (§5) needs an offset formulation that rejects
  self-intersecting offsets rather than producing them.
- **Fillet/chamfer reach** in increment 5 is straight-prism edges first;
  general edge chains are open.
