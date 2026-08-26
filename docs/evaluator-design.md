# Evaluator Design (v1)

How the v1 evaluator turns a `Recipe` step's exact inputs into a `Body` — the
topology it builds, the measurements it computes and the bounds it proves, how
selectors resolve against it, how the tessellation-backed boolean works, and
how `Verify`'s checks are implemented. Companion to `docs/api-design.md` (the
core contract this evaluator implements — references of the form "core §N"),
`docs/sketch-seam-design.md` (the profile records it consumes),
`docs/verification-design.md` (how its answers are judged), and
`docs/recipe-replay-design.md` (strict loading, graph validation,
whole-recipe atomicity, and the package-owned evaluator boundary). Nothing here
changes those contracts. `docs/tessellation-design.md` owns the tessellation
contract and the private operand proofs the boolean consumes. The
payload-specific proofs and implementation order live in
`docs/payload-verification-design.md`. `docs/interference-design.md` owns the
read-only use of this evaluator's intersection volume inside `Verify`. Where
this document stages a capability, the staging is explicit and rejected loudly,
never silently approximated.

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

Per core §2: **v1 uses analytic construction for features, and tessellation for
every boolean outside the analytic reduction's admitted class.** Feature bodies
carry analytic faces, as does a `Union`/`Cut`/`Intersect` result that reduction admits (§9).
Every measurement carries its own proof: exactly representable results are
`Exact`; float/transcendental closed forms and tessellated results are
`Approximate` with proven bounds.

`Exactness` on a boolean's measurement is decided by that measurement's own
proven bound, never by the fact that a boolean produced it (core §5.3: a zero
bound IS the claim that the value is exactly representable). A nonzero bound
reads `Approximate` and a **zero** bound reads `Exact`, on either path. Which
measurements can reach zero depends on the path that built the result.

**On the mesh path (§9), exactly one measurement can reach it:** the **VOLUME**
of an all-planar pair whose contact points round exactly, which is integrated in
exact rational arithmetic over the held mesh and whose rounding term is then
genuinely zero. Every other measurement of a mesh-built result is Approximate —
a length, an area or a centroid is a float sum of square roots, and the last ulp
is not free, so its bound is never zero (§9, `bounds.go`).

**On the analytic reduction (§9), the result is a swept payload and reads by §4
like any other one**, so a volume, an area, a centroid and a `Box` can all reach
a zero bound. `docs/prism-boolean-design.md` §7 owns when each of them does; this
document does not restate its rule.

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
| `Loop` | ordered `[]CoEdge` (directed edge uses in boundary-walk order), `outer bool` |
| `CoEdge` | shared `*Edge` plus walk sense; public `Edge`/`Start`/`End`/`IsForward` accessors |
| `Edge` | tagged `Curve`, `Start`/`End` `*Vertex`, ALL adjacent faces (exactly 2 on a closed manifold body; `Edge.Faces()` reports the actual count, which is precisely how `len != 2` surfaces non-manifold topology — core §6.1), `convex bool` |
| `Vertex` | position (mm), the proven bound on it |

Rules:

- **A `Body` is immutable after construction.** Every operation builds a new
  body; concurrency safety (core §12) falls out.
- **Provenance is structural.** `FeatureRef` identifies the producing
  `StepRef` plus a stable role within it — `side(i, j)` (loop `i`, segment
  `j`) for a swept wall; `side(i, j, k)` for a Loft wall triangle;
  `capStart`, `capEnd`, and the revolve/boolean analogs. This bullet owns the
  role MECHANISM, not every grammar it carries: `docs/loft-design.md` §7 owns
  what `i`, `j` and `k` index for a Loft, and a Loft's `j` is NOT the
  swept-wall segment index this bullet glosses beside it. Roles derive from
  the recorded step, so
  re-evaluation reproduces them, and the provenance predicates — `CreatedBy`
  for edges, `FaceCreatedBy` for faces (core §9) — select the same entities
  under every run.
- **Canonicalize at build.** Adjacent coplanar side faces merge, except no
  Loft wall triangle merges with another face. This keeps
  `side(i,j,0)`/`side(i,j,1)` distinct within a cell and preserves every
  cross-cell rung between coplanar Loft triangles. A full cylinder is one
  face with two circular-edge loops and no seam edge. v1 counts already match
  the analytic answer, so vN does not churn them (core §3).
- **Every vertex carries its bound.** Exactness is decided by what the
  coordinate was READ FROM, never by which builder placed it. A coordinate the
  record states is exact (bound zero); every computed one carries its own
  computation's proven displacement. A swept vertex is read from two
  independent coordinates and carries both: its plane-local pair from the
  section — displaced where the payload carries a section displacement (§9,
  prism-boolean §7) — and its sweep level from the extent, displaced where the
  level was computed rather than stated (a `ToFace`/`ThroughAll` stop resolved
  in float, a magnitude rescaled from a non-base unit, a chamfered end's
  setback). A Boolean result that takes §9's mesh path carries the tessellation
  chord bound. A Boolean result that analytic reduction admits carries its
  payload's own bound instead, as `docs/prism-boolean-design.md` §7 owns. A
  cap-loop chamfer's cap-level feet the offset solve's displacement
  (modify-reach §8.4). A loft's vertices carry the payload's own `delta`,
  which is the placement term for a placed loft and the computed station's
  `stationRound` wherever the build holds a station loft §5.2's table does not
  pin — a chorded circular pair's, and a `LineSeg` end at a trimmed parameter
  alike — positive under `r3.Identity()` and so never conditioned on the body
  having been placed. A loft is NOT swept, so
  its section displacement `sectionDelta` is not a vertex term at all: it
  bounds a chord's departure from the curve it chords, and the facet reading
  that composes it is loft §5.2's own. **`docs/loft-design.md` §5.2's table
  owns every loft term, its condition and its refusal; this bullet names them
  and restates none.** The verification gate reads these (verification §4).
- **Every loop exposes its stored direction.** `Loop.CoEdges()` returns copied
  `CoEdge` values in boundary-walk order. Each use's `Start`/`End` follows that
  walk and `IsForward` states whether it matches the shared `Edge` orientation.
  `Loop.Edges()` returns the same edge identities and order without direction
  for compatibility.
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
  (outer convex, hole concave). A free-form wall's own curvature can turn sign
  inside one span, so it reads neither test — `docs/spline-design.md` §6.5 owns
  its proof, its straight-walk case, and its refusal where the curve's own
  curvature sign is not proven. The on-axis edge shared by both caps of a
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

Caller-built and decoded records carry no live sketch answer. Public
`ProfileRecord` mass-property methods MUST validate before integration:
finite supported fields/ranges first, then region topology through a private
`sketch` reconstruction that exactly matches the recorded walks. Whole-circle
regions use direct disk containment/separation, preserving valid thin annuli
below sketch's general arrangement threshold. Reject malformed or overflowing
input; NEVER return a non-finite `Exact` result.

Increment 1 implements the closed forms for `LineSeg`/`CircleSeg`/`ArcSeg`.
`docs/spline-design.md` owns the free-form kinds entirely: Table F there assigns
each an exactness tier and what a measurement over it may claim, §5 gives the
construction, Table R owns every refusal and its sentinel — the permanent and the
upstream-blocked ones among them — and §10 owns the landing order. This section
restates none of it; read it there.

## 5. Extrude

Input: the recorded profile + plane, a linear `Extent`, options. The sweep
direction is the plane normal `U × V` for `Along` (core §8.1).

Faces, per recorded loop segment (roles in parentheses):

| Segment | Side face |
|---|---|
| `LineSeg` | `Plane` (`side(i,j)`) |
| `CircleSeg` (whole) | full `Cylinder`, no seam edge |
| `CircleSeg` (fragment) / `ArcSeg` | `Cylinder` patch bounded by two line edges + two arc edges |
| free-form kinds | `NURBSSurface` — `docs/spline-design.md` §7 owns the variant and its exactness, Table C the extrude reach, Table F the per-kind tier, Table R the refusals |

Caps: one planar face per end (`capStart`/`capEnd`), the outer loop plus one
loop per hole, holes wound opposite.

Measurements (untapered, increment-1 kinds): closed form with outward numerical
bounds; only proved exactly representable results are **Exact**. The
extent resolves to a signed sweep interval `[z0, z1]` along the plane normal
(`Along` positive; `Against`, `Symmetric` and `TwoSided` sides place `z0`/`z1`
on their own senses), and every formula reads the interval: `Volume = A·h`
with `h = |z1−z0|` (`A` the recorded region's area per §4 — measures are
magnitudes, so an `Against` sweep never reads negative), `Centroid` the
region centroid lifted `(z0+z1)/2` along the normal — the SIGNED midpoint,
`h/2` only in the one-sided `Along` case — `Bounds` from per-segment analytic
extremes swept over the signed `[z0, z1]`, beside the frame and placement's own
rounding: `xform.Apply`/`xform.ApplyDir` are isometries only in EXACT
arithmetic, so a frame that is not axis-aligned or a placement that is not the
identity rounds, and `Bounds` (a `capBlendPayload` result's included) charges
that rounding rather than reading the placed extreme as an exact leaf. Beside
it, the same reading charges the rounding its own FINAL SUMMATION of those
terms into one published coordinate commits. That is a separate mechanism, not
a consequence of the first: a placement can leave every coefficient exactly
right and still round when they are added, which is exactly what a pure
translation does, so a reading charging only the coefficients publishes a
translated box as `Exact` with a zero bound. Both terms are zero for an
unplaced, axis-aligned payload, which keeps the ordinary prism's box `Exact`
as before, and a placed one is `Exact` only where its own endpoint sum is
representable too. `Area` from cap areas + side areas
(`segment length · h`; arc length `rθ` carries its evaluation bound). Each END
of the interval carries its own proven **axial displacement** — how far the
level recorded there sits from the level the extent denotes. A level the caller
stated denotes itself and carries zero; a level the resolution computed carries
that computation's own rounding (a `ToFace`/`ThroughAll` stop resolved in float
against another body's face, a magnitude rescaled from a non-base unit, a
chamfered end's setback). It is the axial twin of the section displacement:
the two are tracked apart and neither stands in for the other — one moves a
coordinate IN the plane, the other moves a level ALONG the normal — while a
reading both displace, a side vertex or `Bounds`, sums them. Every
level-derived reading takes it: `h` and the
volume, wall area and centroid built on it, `Bounds`, the mesh bound, the side
vertices and the vertical edge lengths. Extents: `Distance`,
`Symmetric`, and `TwoSided` of distance sides land in increment 1 — the three
whose interval the step's own quantities determine. `ThroughAll` and
`ThroughAllSide` have no finite stop geometry of their own (they stop at the
far side of every body the sweep meets), so they are body-relative exactly
like `ToFace`: all three land in increment 2 with selectors (§7), the stop an
intersection of the sweep direction with analytic target surfaces — closed
form — and `ErrUnsupported` until then. A through-all payload must provide a
directional extent BESIDE the proven displacement that reading's own ends
carry, and the stop consumes both: the level it resolves takes that
displacement as its own axial displacement, so a far side a bracket holds — a
revolved solid's partial-sweep extreme, or a boundary extreme riding a computed
arc radius or a walked endpoint the record does not state (§6) — never publishes
itself as the level it denotes. The in-path
test is decided OUTSIDE that displacement: material beyond the sketch plane by
more than it is met, material short of the plane by more than it is not, and an
interval the displacement straddles the plane with is `ErrUnsupported` rather
than a guessed dependency (`docs/spline-design.md` §6.4). A prism with a nonzero
section displacement still returns `ErrUnsupported`: that term moves a
coordinate IN the plane and the extent reading does not carry it, so the stop
has no stated displacement to charge. A through-all
dependency is ambient at the CALL but never in the RECORD: core §6.2's depends-on rule covers this
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
contact is allowed only through an exact axis-incidence audit. At each on-axis
point across all recorded loops, the incident walk ends MUST be exactly one
off-axis walk end and one `LineSeg` end lying ALONG the axis, from the same
loop. The off-axis walk sweeps to one pole/apex fan; the on-axis line sweeps
nothing. A second off-axis sector at that point, repeated loop incidence,
isolated endpoint tangency, missing on-axis continuation, interior curve
tangency, or crossing segment is `ErrDegenerate`, as is a region with interior
on both sides. Thus a circle kissing the axis remains a rejected self-touching
horn when it is segmented into arcs whose shared tangent point is a walk
endpoint; segmentation cannot turn the same geometric incidence into a pole.
This audit is what keeps §10's valid-by-construction claim true.
Faces: a `LineSeg` lying ON the axis emits no face at all — it sweeps a
zero-area set, and the neighboring segments' faces close the solid there;
parallel to the axis (off it) → `Cylinder`; inclined → `Cone` (an
endpoint on the axis is its apex); perpendicular → planar annulus (a disk
when it reaches the axis); `ArcSeg`/`CircleSeg` → `Torus`, or `Sphere` when
the arc's center lies on the axis — an endpoint ON the axis closes at a pole,
and an endpoint off it leaves a latitude-circle edge (a spherical band when
neither endpoint reaches the axis). The free-form segment kinds emit
`NURBSSurface` faces; `docs/spline-design.md` §7 owns the variant and its
exactness, Table C the revolve reach, Table R the refusals, and §10 the revolve
increment.
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
`Bounds` is `Exact` only where every one of those extremes is proven exactly
representable; a sweep amplitude no `float64` holds, a boundary extreme a
computed arc radius or a computed walk endpoint carries, the boundary scan's own
`gu·u + gv·v` arithmetic, the axis frame or
the placement's own rounding, or the reading's own summation of those terms
into a published endpoint, publishes the proven bound its own arithmetic
derives instead. The scan's arithmetic is charged apart from the extremes it
reads, at the SECTION's own coordinate envelope: a placement makes both
coefficients non-trivial, so multiplying them by a section coordinate rounds
even where every candidate position is a value the record states verbatim and
the extremes' own term is zero. No other term in the reading scales with that
magnitude, and it is zero wherever one coefficient is zero and the other is `0`
or `±1`, which is the axis-aligned unplaced case. The axis frame contributes
three terms. The first is its
resolved direction and anchor's own proven displacement (`axisInPlane`'s
`dUBound`/`dVBound`/`aUBound`/`aVBound`, already folded into the region's
moments by `axisMoments`, and now into `Bounds` and the meridian
minimum-radius survey the same way), and each of those four is the ROUNDING
its own evaluation committed, proven over the rationals — never a magnitude
envelope over the value it bounds, which for an anchor would grow with the
axis's distance from the frame origin while the projection's own error stayed
zero. The second is, like the placement, the rounding `xform.Apply`/
`xform.ApplyDir` commit whenever the frame is not axis-aligned or the
placement is not the identity. The third is the extreme reading's OWN anchor
shift — the products and the subtraction that carry a plane-local extreme into
axis coordinates — which rounds at the anchor's magnitude rather than the
section's, so a far-offset axis rounds here even where the boundary scan
reports zero. The endpoint summation is charged separately from all three,
since a pure translation leaves all four coefficients exactly right and rounds
only when they are added. Every one of these terms is zero for an axis-aligned,
unplaced revolve whose anchor projects and shifts exactly, which keeps the
ordinary revolve's box Exact as before.
A walk endpoint is a computed one
wherever the record does not state it: a trimmed line's or arc's own bound, and
every circle's, is evaluated rather than read, and the bound it carries is the
per-component gap from a certified enclosure of the point the record denotes. That bound belongs to the directional extent reading itself,
not to the box, because a body-relative stop reads the same extent (§5) and
must see the same uncertainty rather than an exact coordinate. All mass results
carry the §4 numerical bounds. Increment 2.

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
it would re-reject on every re-evaluation.

Immediate calls and stored-recipe evaluation share one recorded-step helper per
operation. The helper consumes the step's records + already-built input bodies,
returns one body + consumed-body list, and never commits. One package-owned
commit tail serves both paths. A separate replay implementation is forbidden.

`Body.PlacedContext`, `Body.DuplicateContext`, and `Body.PlacedCopyContext`
pass the caller's context through payload placement. Faceted placement polls
that context while transforming vertices, auditing and rebuilding topology, and
recomputing measurements. Cancellation returns before the document commits.
`Placed`, `Duplicate`, and `PlacedCopy` remain compatibility wrappers using
`context.Background()`.

Whole-recipe `Evaluate` applies selected recipe limits while taking its deep
normalized snapshot, before any private slice can grow past a ceiling. It then
validates the complete graph and walks it in a private document. It checks
context + work budget in every long loop. Geometry-dependent dependencies such
as `ThroughAll` are recomputed against replayed live bodies and MUST equal
recorded `Inputs`. Failure returns no document.
`docs/recipe-replay-design.md` §§4–7 is normative.

`Body.Placed` transforms analytic geometry exactly in EXACT arithmetic — every
v1 surface variant maps to itself under an isometry (plane→plane,
cylinder→cylinder, …), with `IsReflection` flipping face orientation handling
— but the isometry's FLOAT evaluation rounds wherever the frame is not
axis-aligned or the placement is not the identity. Every reading built from
`xform.Apply`/`xform.ApplyDir` therefore carries that rounding as its own
proven displacement rather than reading the placed coordinate as an exact
leaf: `prismPayload`/`capBlendPayload`'s `Bounds` (§5) and
`revolvePayload`'s (§6) each charge it, composed outward with whatever other
displacement the same reading already carries — and beside it each charges the
rounding of its OWN recombination of the placed terms into a published
coordinate, which a pure translation commits even where the isometry's float
evaluation rounded nothing.

Replay tests cover every example model + every current `OpKind`. Same-evaluator
replay reproduces live-body order and provenance roles. Measurements reproduce
within each evaluator's own exactness/bounds; alternate evaluators may build a
different topology split but MUST preserve role/query meaning.

## 9. The boolean — where exactness dies, and how

Increment 4, the deep end. Strategy:

`docs/prism-boolean-design.md` is the approved analytic reduction
`performBoolean` dispatches, ahead of the tessellation path below, for
co-directional coplanar prism pairs: `Union`'s select-all/merge/chain path and
`Cut`/`Intersect`'s clean-nesting structural match. `Verify`'s own interference
evaluation (below) dispatches that same reduction for `OpIntersect` through
`evaluateAnalyticIntersect`, a read-only twin that builds the admitted payload
and never commits, so it consumes neither operand. `evaluateBoolean` remains
this section's mesh path: a pair neither dispatch admits falls back to it
unchanged, as does every other operation.

**Interference evaluation MUST be read-only; committing a public boolean stays
a wrapper.** Interference PR 1 (`docs/interference-design.md` §11) factors one
internal `evaluateBoolean(ctx, op, a, b)` over tessellation, exact-predicate
classification, cutting, stitching, bound composition, and the rational volume
integral. It never appends a step, retires an operand, registers a body, or
mints a recipe reference. `UnionContext` / `CutContext` / `IntersectContext`
gate their operands, pass the caller context through evaluation and faceted-body
construction, then commit the step atomically. `Union` / `Cut` / `Intersect`
call those variants with `context.Background()` for compatibility. `Verify`
passes its own context to the read-only analytic `OpIntersect` twin first, and
to this same mesh evaluator for a pair that twin does not admit; it consumes
only a bounded intersection volume from whichever path answers, never calls
public `Intersect`, and never builds a transient document body
(`docs/interference-design.md` §5, §5.2). This split is implemented; public
consuming behavior is unchanged.

Expected geometric non-results — empty held intersection, an undecidable
contact arrangement, and `ErrUnsupported` staging — are private typed outcomes:
the public wrapper maps them to its existing errors, while `Verify` maps them to
an undecided pair. An invariant failure in source mapping, closure, orientation,
or exact classification remains an error from `Verify`; it is never hidden as
`Suspect`. The shared evaluator checks `ctx` at phase boundaries and at bounded
work intervals inside quadratic/refinement loops, as interference §7 specifies.

- **Tessellate both operands** with an evaluator-internal chord tolerance —
  a documented default derived from the pair's own diameter, raised past either
  operand's own section displacement, which no mesh of that operand can go
  below (`docs/prism-boolean-design.md` §7, tessellation §5). The booleans of
  core §8 expose no tolerance parameter, on purpose. What IS caller-visible
  is the proven bound the output carries: the tolerance's whole effect
  surfaces as `Bound`/`Exactness`, judged by the caller's `WithTolerance` at
  Verify. The machinery and its payload staging are
  `docs/tessellation-design.md`'s: every mesh carries a two-sided boundary
  bound, source faces, and area slack. A boolean-admissible mesh additionally
  carries a proven occupied-volume symmetric-difference allowance. Export
  needs the boundary proof; a boolean rejects an operand that does not carry
  the occupied-volume proof.
- **Robust mesh boolean in-repo, stdlib-only.** The curated-dependency rule
  stands: no third-party mesh kernel. The algorithm is the exact-predicate
  route: triangle/triangle intersection and point classification decided by
  adaptive-precision sign tests that fall back to exact rational arithmetic
  exactly at the boundary cases — a sign decided exactly is a topology
  decision that cannot flip (core §2.1's whole fear), so the output is
  watertight **by construction** on the tessellated geometry. The exact
  fallback is carried as homogeneous integer coordinates — an integer
  numerator triple over one shared positive denominator — rather than
  `math/big.Rat`: every predicate is a homogeneous form of fixed degree in the
  differences, so its sign is invariant under scaling by a positive
  denominator, and the exactness guarantee is unchanged. A point is reduced to
  its canonical form only at vertex emission, because welding is by exact
  identity and a homogeneous point has many spellings. Retriangulation along
  intersection curves, classification by exact winding tests, stitching by
  shared exact vertices.
- **One symmetric classifier per facet pair.** Two closed triangles are convex
  sets, so their intersection is empty, a point, a segment, or a 2-D region —
  and the pair pass computes exactly *which*, never "how many of A's vertices
  sit on B's plane, and whose geometry do I look on". A 2-D region is a
  coplanar face-on-face tangency: classifying material side over every
  positive-area coplanar patch is the designed route
  (`docs/interference-design.md` §5.2), staged as that document's §11 PR4, and
  this evaluator refuses the case until that increment lands. A point is
  carried and refused unless some crossing chain owns it as an endpoint; a
  segment is a rim. When the operands name valid solids, an unlanded 2-D
  region and an unowned point are contacts this evaluator refuses, so the
  boolean surfaces them as
  `BooleanUnsupportedContact` (wrapping `ErrUnsupported`; `docs/api-design.md`
  §8 / H2), never `ErrDegenerate` — which stays reserved for a genuinely
  malformed region, a zero-area or self-crossing one. The classifier still
  DETECTS the region-or-point shape; only the public error a valid such contact
  yields is decided against H2. Which triangle's boundary an endpoint lies on is
  decided by testing the point against *each* triangle, not by which list it was
  drawn from. The classification is direction-free by construction, which is what
  keeps it from disagreeing with itself.
- **Graze versus crossing is not a property of a facet pair.** When a contact
  segment runs ALONG a facet edge, the pair cannot tell whether the operand's
  boundary touches the other's plane and comes back (a graze — a tangency no
  side classification can be proven for, which surfaces on a valid model as
  `BooleanUnsupportedContact` / `ErrUnsupported`, `docs/api-design.md` §8 / H2,
  never `ErrDegenerate`) or genuinely passes
  through it (an ordinary crossing, and a real rim). Only the edge's TWO
  adjacent facets can: their apex vertices strictly on one side is a graze,
  straddling is a crossing. So the pair pass only reports the in-plane edge,
  and the mesh pass makes the call once, with the adjacency in hand. A crossing
  edge subdivides the facet it passes through (never the facet it runs along —
  a segment on a facet's own boundary cuts nothing off it) and its regions
  classify by exact parity, because the other operand's boundary there is a
  DIHEDRAL and one plane of a dihedral decides nothing.
- **A tangency the held facets cannot see is refused, never assumed away.** A
  chord polygon can miss a touch on its analytic patch, and an inherited
  faceted boundary certificate can place the true patch away from its held
  polygon. A pre-pass therefore proves, per current operand FACE pair, that no
  touch can be hiding: if the true patches touch, that point is within δ_A of
  A's facets and δ_B of B's, so the facet sets come within δ_A + δ_B of each
  other. An analytic face's δ is its complete `sourceBound`, including curved
  trim displacement and every coordinate-construction/final-placement rounding
  allowance. A faceted face's δ is its inherited certified
  displacement, falling back to the
  payload's global composed boundary `Delta` when no tighter face value exists.
  Mesh facets carry this value as internal `sourceBound` beside their current
  operand `*Face`; grouping facets for the pre-pass takes the maximum bound for
  that face. NEVER derive zero from `KindFaceted` alone.
  Upward-round `b = δ_A + δ_B`. When `b > 0`, a held-facet meet is decidable
  exactly when the facet sets INTERPENETRATE — cross with a signed penetration
  depth strictly GREATER than `b`. Then the true patches provably cross: A's
  true surface lies within δ_A of A's facets and B's within δ_B of B's, so even
  in the worst case — each true surface pulled back toward its own interior by
  its own bound — the two true surfaces still overlap. The chord error alone,
  bounded by `b`, cannot open a penetration deeper than `b`, so a depth past `b`
  is a proof of true crossing and the pair is ADMITTED. Every other close pair
  is undecidable and refused (`ErrUnsupported`): a pair whose held facets merely
  touch, a pair that overlaps by AT MOST `b`, and a pair whose facets stay apart
  but within `b`. A displacement within the bounds can pull such surfaces apart
  — chording a concave hole wall inward, for one, adds held material into the
  hole and makes truly disjoint surfaces meet — so a held meet at distance 0 or
  a shallow penetration ≤ `b` proves nothing when `b > 0`. The interpenetration
  depth is the implementer's to compute: it is the maximum signed penetration of
  the two facet sets, decided by the exact predicates, never assumed cheaply
  available. Reject-only in the undecidable band: it may refuse a valid model
  whose operands genuinely pass that close, and that is the accepted price. A
  TANGENCY — facets that touch without crossing — has no positive-bound contact
  certificate in this evaluator increment and stays refused; deciding such a
  pair for real is the clearance kernel's job (`docs/clearance-design.md`), not
  a held facet's. A planar face still carries
  every curved-trim, coordinate-construction, and placement displacement in its
  `sourceBound`; it has zero error only when its held trimmed polygon and stored
  coordinates are both proved exact. A faceted face has zero
  error only when its inherited boundary certificate proves that its true patch
  equals the held polygons (`docs/tessellation-design.md` §2/§7, payload
  verification §5).
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
- **Output**: faces are `Faceted`, one per CONNECTED PATCH of a current operand
  face. Each patch keeps that face's origins, so provenance (`FaceCreatedBy`)
  and face-level selection survive the boolean — but the operand face is not the
  face. A boolean can cut one operand face into pieces that no longer touch (a blind
  trench crosses a cap and leaves two separate strips of it standing), and each
  piece is its own face, bounded from outside by its own loop. Grouping by
  operand face alone would hand both strips to one face, which then has two outer
  boundaries and can call only one of them outer — reporting the other as a
  *hole* in a patch it is not part of, a wrong topology answer on the surface
  agents traverse. So the key is the patch: facets of one operand face reachable
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
  bound shapes (volume composes the operands' own proven mesh
  symmetric-difference allowances by the set identity; the rim never enters
  that allowance) — `Approximate`, except the §2
  Exact-volume case: an all-planar pair whose contacts round exactly leaves the
  exact volume integral with a genuinely zero bound.
- **The rim is NOT bounded by δ_t.** A vertex an operand's tessellation
  contributed lies on that operand's surface; a vertex the BOOLEAN creates does
  not lie on either. It is the crossing of two chord PLANES, and the true
  intersection curve is anywhere within δ_A of the one and δ_B of the other —
  a tube of half-width **(δ_A + δ_B)/sin θ** about it, θ the crossing angle.
  So the pre-weld boundary bound is that trim-amplified displacement, computed
  from a proven lower bound on sin θ taken exactly from the facet normals. The
  final face bound adds, with upward rounding, the maximum displacement from
  each incident exact stitched vertex to its stored welded binary64 coordinate;
  the global boundary certificate takes the upward-rounded maximum of those
  complete face bounds. That complete value is what every boundary measurement
  composes from (`Vertex.Position`,
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
- **Faceted bodies** are judged per verification §6 and payload verification
  §5/§6: exact held-mesh facts plus an internal source certificate, with a BVH
  proving every remote triangle/triangle and nonadjacent trim-edge distance.
  A clean complete certificate whose remote feature scale exceeds twice its
  boundary displacement proves validity; a shallow defect remains `Suspect`.
  The tolerance gate is then applied to every bounded result the report
  carries; `Approximate` alone never assigns `Suspect`. The body's gate
  diameter is cached from the greatest distance between any two vertices in
  the complete held faceted payload, not from the smaller set exposed as B-rep
  boundary-loop vertices, and is published through verification §3's shared
  reader, which rounds that distance toward zero. A placement rebuild recomputes the diameter after
  transforming the payload vertices and polls its context through the
  quadratic scan. The area floor sums each unique topological edge's held
  length once, never its two coedge uses.
- **Pairs**: use the four-way relation and proof order of
  `docs/interference-design.md`. Bounds separation or the analytic clearance
  kernel proves disjoint/touching; nesting and admitted crossings remain
  explicitly `pairOverlapping`, never folded into undecided. A strict
  full-containment or analytic equality certificate reuses an operand's
  volume. Every other candidate overlap runs the read-only `OpIntersect`
  evaluator and emits a row
  only when the volume interval proves positivity (`Value - Bound > 0`). Empty,
  contact, unsupported, or coarse results remain undecided and read `Suspect`;
  internal failures return from `Verify`. Box-disjointness proofs run from
  increment 1; increment 3 adds analytic clearance; increment 4 adds read-only
  transversal intersection volume, with later staged breadth listed by the
  interference design.
- **Wall thickness / undercuts / min radius**: the analytic surveys of
  verification §6, answered outright on this evaluator's own payloads
  (`survey.go`/`survey2d.go`): prism/revolve wall reduces exactly to the 2D
  spanning-disk problem (a prism's profile with the height as the vertical
  fit; a revolve's meridian section, mirrored for a full turn). `Verify`
  passes one shared work counter through the prism/revolve survey; candidate
  generation streams each disk directly into validation, and generation,
  validation, containment, and boundary scans all poll that counter. A
  canceled context therefore aborts the survey and returns its error from
  `Verify`. Undercuts are per-face exact normal-range membership, and the min
  radius is the tightest concave principal radius. A cup uses its exact morphology theorem:
  exact zero for an allowance-qualified pinch, otherwise its shell thickness
  carrying that thickness's own millimetre-conversion displacement as the
  reading's bound (payload verification §4.1).
  A faceted body uses certified source-normal/curvature patches and bounded
  medial candidates. A payload or certificate the surveys cannot decide leaves
  the asked question undecided → `Suspect`, never a silent pass (payload
  verification §4/§8–§10).

`Verify` MUST run requested surveys before the total numeric gate, then apply
verification §3's field table to every present body reading. The gate compares
base-unit magnitudes and uses the inclusive `Bound <= rel × Ref` rule. One
shared scalar-gate path handles body scalar readings, `Clearance.Gap`, and
`Interference.Volume`; a gap-only shortcut MUST NOT become a second contract.
The wall interval verdict and undercut predicate verdict remain separate inputs
to status aggregation, as verification §6 specifies.

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
| 4 | tessellation per `docs/tessellation-design.md` + the exact-predicate mesh boolean, `Faceted` bodies, faceted `Verify`, `Tessellate`/`STL`/`OBJ`; supplies the geometry and bounds shared by public booleans and read-only interference evaluation |
| 5 | fillet/chamfer on analytic prism edges, shell |
| 6 | bounded canonical recipe encode, strict versioned decode, full operation/reference validation with deterministic error precedence, resource budgets, shared recorded-step dispatch, atomic public `Evaluate`, replay/property/fuzz suite |
| 7 | tapered extrude if a sound offset story exists |

Free-form support is `docs/spline-design.md`'s own increment plan (§10 there).
Its stages do not consume a global evaluator increment number.

Loft follows `docs/loft-design.md` §12's count-free four-PR delivery plan.
PR 1 adds `Document.Loft`, its four measurements, and structural/tolerance
`Verify`; PR 2 adds tessellation, mesh-boolean admission, and placement; PR 3
lands same-kind `CircleSeg`/`ArcSeg` correspondence; PR 4 reserves N-section
and guide-rail/centerline lofts, and stages the analytic clearance adapter and
non-constant-section wall survey.
Every unlanded Loft `Verify` question remains `Suspect`; a call this evaluator
cannot yet build returns `ErrUnsupported`.

Payload verification §13 gives count-free stages for the cup adapter and
faceted validity/clearance/survey work. Every later question stays `Suspect`
until its stage lands. These stages do not consume global evaluator increment
numbers.

## 12. Open questions

- **Free-form reach is decided.** `docs/spline-design.md` owns it: the
  whole-entities-only scope, the exactness tiers, the refusals, and the upstream
  ask that retires `EllipticalArcSeg` (§9 there). `FitSplineSeg` is Tier A
  (Table F), and its own build-path refusal (R6) is retired (§10 P4b), not
  through any upstream ask.
- **Tapered extrude** (§5) needs an offset formulation that rejects
  self-intersecting offsets rather than producing them.
- **Modify reach is decided.** `docs/modify-reach-design.md` extends increment
  5's modify ops with staged sub-items (RX/SX/BX/DX), including exact admitted
  cases and permanent `ErrUnsupported` boundaries; these stages do not consume a
  global evaluator increment number.
