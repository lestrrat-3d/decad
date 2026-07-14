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
measurements with proven bounds. There is no third state.

`Exactness` on a boolean's measurement is decided by that measurement's own
proven bound, never by the fact that a boolean produced it (core §5.3: a zero
bound IS the claim that the value is exactly representable). A nonzero bound
reads `Approximate`. A **zero** bound reads `Exact`, and exactly one boolean
measurement can reach it: the **VOLUME** of an all-planar pair whose contact
points round exactly, which is integrated in exact rational arithmetic over the
held mesh and whose rounding term is then genuinely zero. Every other boolean
measurement is Approximate — a length, an area or a centroid is a float sum of
square roots, and the last ulp is not free, so its bound is never zero (§9,
`bounds.go`).

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
| `Face` | tagged `Surface`, `[]*Loop` (first outer), origin roles (≥1 `FeatureRef`s, exposed as `Face.Origins()` — a canonicalization merge UNIONS the merged faces' roles, and `FaceCreatedBy` matches on ANY of them, so provenance survives the merge) , back-ref to its body |
| `Loop` | ordered `[]*coedge` (edge + sense), `outer bool` |
| `Edge` | tagged `Curve`, `Start`/`End` `*Vertex`, ALL adjacent faces (exactly 2 on a closed manifold body; `Edge.Faces()` reports the actual count, which is precisely how `len != 2` surfaces non-manifold topology — core §6.1), `convex bool` |
| `Vertex` | position (mm), the proven bound on it |

Rules:

- **A `Body` is immutable after construction.** Every operation builds a new
  body; concurrency safety (core §12) falls out.
- **Provenance is structural.** `FeatureRef` identifies the producing
  `StepRef` plus a stable role within it — `side(i, j)` (loop `i`, segment
  `j`), `capStart`, `capEnd`, and the revolve/boolean analogs. Roles derive
  from the recorded step, so re-evaluation reproduces them, and the
  provenance predicates — `CreatedBy` for edges, `FaceCreatedBy` for faces
  (core §9) — select the same entities under every run.
- **Canonicalize at build.** Adjacent coplanar side faces merge; a full
  cylinder is one face with two circular-edge loops and no seam edge. v1
  counts already match the analytic answer, so vN does not churn them
  (core §3).
- **Every vertex carries its bound.** Feature-built vertices are exact (bound
  zero); boolean-built vertices carry the tessellation's chord bound. The
  verification gate reads these (verification §4).
- **`convex` is the walked boundary, decided at build** (core §6.1). The
  profile walk carries the material on its left — outer loop counter-clockwise,
  every hole clockwise (§4) — and every edge reads that walk, never a 3D
  dihedral measured after the fact. A junction edge — a prism's vertical edge,
  and a revolve's swept junction, which is a partial sweep's arc and a full
  revolution's latitude circle alike — takes the sign of the cross product of
  the incoming and outgoing walk tangents: a left turn is convex. A rim edge (a
  wall's own copy in a cap plane) takes the sense of the wall it runs along —
  a circular wall by its own turn, counter-clockwise convex and clockwise
  concave, which is the same test that decides whether the wall's material lies
  outside its cylinder; a straight wall, having no turn, by the role of its loop
  (outer convex, hole concave). The on-axis edge shared by both caps of a
  partial revolve is convex when the sweep is under π. Consequence, and it is
  the intended one: a hole's rim edges are concave, and so are the rims of a
  concave round on the outer loop.

## 4. Mass properties — decad's own, on decad's own records

The hard rule "NEVER re-derive a 2D answer" forbids re-deciding what sketch
has decided: closure, validity, cuts, intersections, projections. It does
**not** put the recorded region's *mass properties* out of reach — once
recorded, the region is decad's own geometry, and volumes, centroids and
areas of the bodies built from it are decad's 3D job. The boundary is:

- **sketch decides topology and admissibility** — what closes, what is valid,
  where curves cross, what a trim is. decad consumes.
- **decad integrates its own records** — area, first moments, the second and
  mixed moments (`∫u² dA`, `∫uv dA` — what a revolve centroid needs, §6),
  arc length, and extremes of the recorded loops, by closed form per segment
  kind: shoelace-family boundary integrals over segment chords plus the
  exact circular-segment corrections for `CircleSeg`/`ArcSeg` fragments (the
  same correction family geom uses; independent code on decad's own data,
  asserted against sketch's answer by the §1 falsifier at every feature
  call).

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
on their own senses), and every formula reads the interval: `Volume = A·h`
with `h = |z1−z0|` (`A` the recorded region's area per §4 — measures are
magnitudes, so an `Against` sweep never reads negative), `Centroid` the
region centroid lifted `(z0+z1)/2` along the normal — the SIGNED midpoint,
`h/2` only in the one-sided `Along` case — `Bounds` from per-segment analytic
extremes swept over the signed `[z0, z1]`, `Area` from cap areas + side areas
(`segment length · h`; arc length `rθ` exact). Extents: `Distance`,
`Symmetric`, and `TwoSided` of distance sides land in increment 1 — the three
whose interval the step's own quantities determine. `ThroughAll` and
`ThroughAllSide` have no finite stop geometry of their own (they stop at the
far side of every body the sweep meets), so they are body-relative exactly
like `ToFace`: all three land in increment 2 with selectors (§7), the stop an
intersection of the sweep direction with analytic target surfaces — closed
form — and `ErrUnsupported` until then. A through-all dependency is ambient
at the CALL but never in the RECORD: core §6.2's depends-on rule covers this
case explicitly — the feature call resolves which live bodies actually bound
the stops and records each one's `StepRef` in the step's `Inputs`, in stop
order — so re-evaluation reaches the same stops with no ambient body-set
dependency. (`ToFaceAngular` is the revolve analog and lands there, §6/§11.) A nonzero
`WithTaper` is recorded exactly and is `ErrUnsupported` in v1: a tapered
extrude of a general region is an offset problem (self-intersecting offsets),
and a wrong-but-confident prism is the failure decad exists to prevent.

## 6. Revolve

Same recording shape; the axis must be validated non-degenerate and coplanar
with the profile plane, and the axis must not pass through the region's
INTERIOR: the region lies in one closed half-plane of the axis. Boundary
contact is allowed in exactly two forms — a segment ENDPOINT on the axis (it
sweeps to a pole or an apex, the sphere and cone-tip case), and a whole
`LineSeg` lying ALONG the axis (it sweeps nothing and emits no face, per the
build table below). Any other contact — a curve tangent to the axis at an
interior point (a circle kissing it would sweep a self-touching horn torus),
or a segment crossing it — is rejected, `ErrDegenerate`, as is a region with
interior on both sides; that is what keeps §10's valid-by-construction claim
true.
Faces: a `LineSeg` lying ON the axis emits no face at all — it sweeps a
zero-area set, and the neighboring segments' faces close the solid there;
parallel to the axis (off it) → `Cylinder`; inclined → `Cone` (an
endpoint on the axis is its apex); perpendicular → planar annulus (a disk
when it reaches the axis); `ArcSeg`/`CircleSeg` → `Torus`, or `Sphere` when
the arc's center lies on the axis — an endpoint ON the axis closes at a pole,
and an endpoint off it leaves a latitude-circle edge (a spherical band when
neither endpoint reaches the axis). The free-form segment kinds follow the
same staging as extrude (§5): `NURBSSurface` surfaces of revolution where the
control net is exactly derivable from the record, in increment 6, and
`ErrUnsupported` until then — `FitSplineSeg` included, on the same grounds.
Partial sweeps get two planar cap faces. Volume by Pappus on the §4 first moments; the solid centroid from the §4
second and mixed moments (`∫u² dA`, `∫uv dA`) — a full revolution's centroid
lies on the axis with its axial position from the mixed moment, and a partial
sweep's is closed form in the sweep angle. `Area` by Pappus's first theorem
per side face — swept arc length × the sweep angle × the segment CURVE's
centroidal radius about the axis, a boundary first moment with closed forms
for `LineSeg`/`ArcSeg`/`CircleSeg` — plus the §4 region area for each partial-
sweep cap. `Bounds` from per-face analytic extremes: each face's radial
extreme about the axis (a cylinder's radius, a cone's two end radii, a
torus/sphere's center distance ± minor/radius) and axial range, with a
partial sweep's angular interval deciding which cardinal directions are
reached — the same extreme analysis extrude uses, in cylindrical coordinates.
All exact. Increment 2.

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

- **Tessellate both operands** with an evaluator-internal chord tolerance —
  a documented default derived from the pair's own diameter (the booleans of
  core §8 expose no tolerance parameter, on purpose). What IS caller-visible
  is the proven bound the output carries: the tolerance's whole effect
  surfaces as `Bound`/`Exactness`, judged by the caller's `WithTolerance` at
  Verify. The machinery is `Tessellate(tol)`'s, with per-surface analytic
  tessellators, a proven per-facet deviation bound δ_t, and facets that
  remember their source face.
- **Robust mesh boolean in-repo, stdlib-only.** The curated-dependency rule
  stands: no third-party mesh kernel. The algorithm is the exact-predicate
  route: triangle/triangle intersection and point classification decided by
  adaptive-precision sign tests that fall back to `math/big.Rat` exactly at
  the boundary cases — a sign decided exactly is a topology decision that
  cannot flip (core §2.1's whole fear), so the output is watertight **by
  construction** on the tessellated geometry. Retriangulation along
  intersection curves, classification by exact winding tests, stitching by
  shared exact vertices.
- **One symmetric classifier per facet pair.** Two closed triangles are convex
  sets, so their intersection is empty, a point, a segment, or a 2-D region —
  and the pair pass computes exactly *which*, never "how many of A's vertices
  sit on B's plane, and whose geometry do I look on". A 2-D region is a
  coplanar face-on-face tangency (`ErrDegenerate`); a point is carried and
  refused unless some crossing chain owns it as an endpoint; a segment is a
  rim. Which triangle's boundary an endpoint lies on is decided by testing the
  point against *each* triangle, not by which list it was drawn from. The
  classification is direction-free by construction, which is what keeps it from
  disagreeing with itself.
- **Graze versus crossing is not a property of a facet pair.** When a contact
  segment runs ALONG a facet edge, the pair cannot tell whether the operand's
  boundary touches the other's plane and comes back (a graze — a tangency no
  side classification can be proven for, `ErrDegenerate`) or genuinely passes
  through it (an ordinary crossing, and a real rim). Only the edge's TWO
  adjacent facets can: their apex vertices strictly on one side is a graze,
  straddling is a crossing. So the pair pass only reports the in-plane edge,
  and the mesh pass makes the call once, with the adjacency in hand. A crossing
  edge subdivides the facet it passes through (never the facet it runs along —
  a segment on a facet's own boundary cuts nothing off it) and its regions
  classify by exact parity, because the other operand's boundary there is a
  DIHEDRAL and one plane of a dihedral decides nothing.
- **A tangency the chords cannot see is refused, never assumed away.** A chord
  polygon lies strictly inside the curved surface it approximates, so a true
  tangency between two operands can vanish from the tessellation — and whether
  it vanishes depends on where the chord samples fell. A pre-pass therefore
  proves, per analytic FACE pair, that no touch can be hiding: if the true
  surfaces touch, that point is within δ_A of A's facets and δ_B of B's, so
  the facet sets come within δ_A + δ_B of each other. A face pair whose facets
  already MEET is decided (the contact is exact and the predicates own it); a
  face pair that comes within δ_A + δ_B *without* meeting is the undecidable
  one, and it is refused (`ErrUnsupported`). Reject-only: it may refuse a valid
  model whose operands genuinely pass that close, and that is the accepted
  price. Deciding such a pair for real is the clearance kernel's job
  (`docs/clearance-design.md`), not a chord's. A planar face with straight
  edges triangulates exactly, and a `Faceted` face IS its polygons, so both are
  held with zero error and the pre-pass has nothing to prove about them.
- **An operand facet that collapsed is refused, never skipped.** A rigid
  placement's own rounding can flatten a facet of an already-faceted body to
  zero area. Such a facet has no plane and no interior, so every contact
  predicate here is blind to it — a point or tangent contact the other operand
  makes there would be classified by nothing at all — and it must not ride
  through the boolean on its component's verdict. The operand is refused
  (`ErrUnsupported`) rather than examined in part.
- **The final rounding to float64 welds, and what the weld takes it must
  answer for.** Two exact result vertices closer than an ulp round to one held
  vertex, and the facets they span collapse. A collapsed facet is a zero-area
  triangle: it moves neither the volume integral nor the area sum of the held
  mesh, and its two real directed edges cancel, so closure survives it. But the
  facet it stood for was not zero-area *before* the weld, and two things it
  carried have to be accounted for, in two different ways:
  - **The volume and the area it carried are BOUNDED.** The rounding's volume
    error is what its vertex displacement sweeps out over the surface the
    displacement acted ON — the stitched surface *before* any facet was
    dropped from it. Charging it against the mesh that survived would leave the
    dropped facets' own swept volume out of the bound entirely. So the swept
    volume is charged against the pre-round surface, and the area the weld
    dropped — which the held mesh can no longer report — joins the operands'
    chord deficit in the area bound (`bounds.go`: `sweptVolumeAllow`,
    `perturbedAreaUpper`).
  - **A component welded out of existence is REFUSED.** When *every* facet of a
    connected component collapses, that whole shell disappears — a lump gone
    from the body, with its volume, its place in `Lumps()` and its reach in the
    bounds box. No bound answers for that: the loss is topological, and the
    closure audit does not catch it, because the components that remain still
    close. It is refused (`ErrUnsupported`). Every other collapse is an edge
    contraction inside a component that survives, and the two bounds above
    cover it.
- **Output**: faces are `Faceted`, one per CONNECTED PATCH of a source analytic
  face. Each patch keeps that source's origins, so provenance (`FaceCreatedBy`)
  and face-level selection survive the boolean — but the source face is not the
  face. A boolean can cut one source into pieces that no longer touch (a blind
  trench crosses a cap and leaves two separate strips of it standing), and each
  piece is its own face, bounded from outside by its own loop. Grouping by
  source alone would hand both strips to one face, which then has two outer
  boundaries and can call only one of them outer — reporting the other as a
  *hole* in a patch it is not part of, a wrong topology answer on the surface
  agents traverse. So the key is the patch: the facets of one source reachable
  from each other across shared edges. Within a patch, which loop bounds it from
  outside is decided, not guessed: on a planar patch the boundary is walked with
  the material on its left, so about the patch's own outward normal the outer
  loop turns positive and every genuine hole turns negative, and the outer
  loop's area vector — the patch's area plus its holes' — is the largest. Never
  the longest perimeter: a serpentine slot can out-measure the boundary it is cut
  into. A curved patch has no such plane and no loop of it is a hole in another
  (a hole wall's two rims bound a tube), so there the longest boundary stands as
  the deterministic bookkeeping choice; validity never reads it. Measurements
  integrate the mesh exactly and report the verification-design
  bound shapes (volume bound ≈ δ_t · area, by the symmetric-difference
  identity, which the rim never enters) — `Approximate`, except the §2
  Exact-volume case: an all-planar pair whose contacts round exactly leaves the
  exact volume integral with a genuinely zero bound.
- **The rim is NOT bounded by δ_t.** A vertex an operand's tessellation
  contributed lies on that operand's surface; a vertex the BOOLEAN creates does
  not lie on either. It is the crossing of two chord PLANES, and the true
  intersection curve is anywhere within δ_A of the one and δ_B of the other —
  a tube of half-width **(δ_A + δ_B)/sin θ** about it, θ the crossing angle.
  So the boundary bound is that trim-amplified displacement, computed from a
  proven lower bound on sin θ taken exactly from the facet normals, and it is
  what every boundary measurement composes from (`Vertex.Position`,
  `Faceted.Bound`, `FacetedCurve.Bound`, `Box`, and the perimeter term of every
  area bound). It has no finite ceiling as the operands approach tangency: when
  the inflated bound reaches the pair's own diameter it has stopped bounding
  anything, and the operation is refused (`ErrUnsupported`) rather than
  reported with a number nobody can use. Every bound has exactly one owner
  (`bounds.go`); no measurement site computes one inline.
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
  or when a clearance computation clears the summed bounds. An
  `Interference` row carries a bounded overlap VOLUME (verification §1), and
  a bare witness point cannot supply one — so a row is emitted only once the
  boolean machinery can intersect the pair and bound the volume (increment
  4). Until then a pair that cannot be proven disjoint is undecided: it joins
  neither list and reads `Suspect` — never a fabricated row, and never a
  false `Sound`. Box-disjointness proofs run from increment 1 (they are
  cheap, and they decide the common far-apart case); increment 3 adds the
  clearance computation and `WithClearances`.
- **Wall thickness / undercuts / min radius**: the analytic surveys of
  verification §6, answered outright on this evaluator's own payloads
  (`survey.go`/`survey2d.go`): the wall reading reduces exactly to the 2D
  spanning-disk problem (a prism's profile with the height as the vertical
  fit; a revolve's meridian section, mirrored for a full turn), undercuts
  are per-face exact normal-range membership, and the min radius is the
  tightest concave principal radius. All readings Exact; a payload the
  surveys cannot decide leaves the asked question undecided → `Suspect`,
  never a silent pass.

## 11. Increments

Each increment is a PR series behind the same public contract; nothing ships
half-silent. The staging surfaces differently by what is being staged, per
the sections above: an intent the evaluator cannot BUILD — a feature call, an
extent, a segment kind — is `ErrUnsupported` at the call (§2), while a
`Verify` question the evaluator cannot yet ANSWER is accepted and reads
`Suspect` (§10) — an asked-but-undecided answer, never an error and never a
silent pass.

| # | Lands |
|---|---|
| 1 | topology model, `Document`/`Recipe`/`Step` wiring, `Extrude` for line/circle/arc profiles with `Distance`/`Symmetric`/`TwoSided`-of-distance-sides extents, mass properties (§4), `Placed`, structural `Verify` (validity by construction, quantities, tolerance gate; every pair undecided → `Suspect` unless box-proven disjoint) |
| 2 | `Revolve` (angular extents), selector vocabulary + resolution, the body-relative stops (`ToFace`/`ToFaceAngular`/`EdgeAxis`, `ThroughAll`/`ThroughAllSide`) |
| 3 | analytic clearance proofs and `WithClearances` (box-disjointness proofs already run from increment 1, §10/§11 row 1) |
| 4 | tessellation + the exact-predicate mesh boolean, `Faceted` bodies, faceted `Verify`, `Tessellate`/`STL`/`OBJ` |
| 5 | fillet/chamfer on analytic prism edges, shell |
| 6 | free-form side surfaces (`NURBSSurface` from recorded control data), tapered extrude if a sound offset story exists |

## 12. Open questions

- **`FitSplineSeg` side faces** (§5) wait on geom exporting its interpolant's
  B-spline form; decad will not duplicate the solve.
- **Tapered extrude** (§5) needs an offset formulation that rejects
  self-intersecting offsets rather than producing them.
- **Fillet/chamfer reach** in increment 5 is straight-prism edges first;
  general edge chains are open.
