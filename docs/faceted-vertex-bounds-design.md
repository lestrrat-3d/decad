# Faceted Vertex Bounds Design

Per-vertex displacement bounds on faceted bodies, carried through every mesh
boolean, so that a chain of booleans over planar-faced bodies is limited by
the bound of the geometry each new pair touches rather than by one global
held-mesh bound per body. Companion to `docs/tessellation-design.md`, whose
§2 proof record, §7 faceted restatement and §11 boolean composition this
design refines, and to `docs/evaluator-design.md` §9, which owns the mesh
boolean. References of the form "core §N" are to `docs/api-design.md`;
"tessellation §N" to `docs/tessellation-design.md`; "evaluator §N" to
`docs/evaluator-design.md`; "sweep §N" to `docs/sweep-design.md`.

Nothing here changes a public type. `Vertex.Bound`, `Faceted.Bound`,
`FacetedCurve.Bound`, `Measurement.Bound`, `Box.Bound` and `Mesh.Bound` keep
their shapes; what changes is which proven number each one reports. The one
behaviour change a caller can observe is §5's chain-depth refusal, which
core §8 "The chain depth" states.

## 1. Problem

A boolean's result is a `facetedPayload` carrying ONE displacement bound,
`meshBound`, for every held vertex of the body (`boolean_body.go`). Every
reading composes from it: every `Vertex.Bound`, every face's
`Faceted.Bound`, every rim edge's `FacetedCurve.Bound` and length bound, the
area bound (`meshBound × perimeter`), and the box bound (`Radius3D(meshBound)`).
Tessellating the result restates it with that one number as every face's
`sourceBound` (`tessellateFaceted`), and the next boolean composes its rim
bound from it: `rimDelta(ma.bound, mb.bound, sinMin, dPair)` divides the SUM
of the two operands' global bounds by the sine of the SHALLOWEST crossing
anywhere in the pair (tessellation §11 step 7).

So one union's rim bound becomes the whole body's bound, and the next union
amplifies it again, at every vertex, whether or not the new pair touches the
old rim. The factor per union is `1/sin θ_min` plus the weld, which the
prototype in §6 measured at ×1.5–1.9 per union on a 40°-tilted branch tree
and the mtilt hand-off measured at ×4 on placed 16-gon prisms crossing more
shallowly. Core §8's chain-depth gate refuses the result as an operand once
`meshBound` exceeds the next pair's chord tolerance (`2e-5 ×` the pair
diameter, `boolChordFactor`), which the measured tree reached at its 46th
union. Long before that, the published volume, box and vertex bounds have
grown from `1e-14 mm` to `1e-3 mm` for a body whose held vertices never moved.

Tessellation §2 already allows a `facetedPayload` to "retain a tighter
displacement per live faceted face". A per-FACE bound is not enough: a trunk
wall is one face cut by every branch that enters it, and a cone wall is one
face cut at both ends, so a per-face figure carries every rim's amplification
across the whole face. The unit that stays local is the held vertex.

## 2. The bound model

Every held mesh has, for each vertex `v`, a **vertex bound** `β(v)`:

> There is a point `v*` of the true boundary with `|v − v*| ≤ β(v)`. `v*` is
> the point the held vertex stands for: the exact rational vertex a mitred
> sweep rounded, the exact crossing point of two true faces for a rim vertex,
> or any nearest true point for a vertex of a chorded analytic mesh.

A facet `t = (a, b, c)` then has the **facet bound**

```text
δ(t) = max(β(a), β(b), β(c))
```

and the claim tessellation §2's `sourceBound(face)` makes for a face is made
per facet: the true piece of the boundary that facet stands for lies within
`δ(t)` of the facet, two-sided, under the certified correspondence the facet's
own producer proved. `sourceBound(face)` is the maximum `δ(t)` over the
face's facets, and `Mesh.Bound()` is the maximum over the mesh, so both keep
tessellation §2's meaning unchanged.

**Why the facet bound is the corner maximum.** Every point `x` of a held
facet is a convex combination `Σ λ_i v_i` of its corners. The point
`x* = Σ λ_i v_i*` lies on the plane through the three true points, and
`|x − x*| ≤ Σ λ_i β(v_i) ≤ δ(t)`. For a PLANAR true face the three true
points lie in that face's plane, so `x*` is a point of the true face's plane
and the affine triangle `(a*, b*, c*)` is the facet's true piece exactly;
this is the model `docs/tessellation-design.md` §2's `stitchPayload` and
`chainPayload` rows already use ("the largest `Vertex.Bound()` over the
vertices that face's own triangles touch"), and `PerturbedTriangleAreaAllow`
already charges area under it. For a CURVED true face the affine triangle is
not the true piece, and the corner bounds alone do not cover the chord
sagitta between them; §2.1 states how such a mesh enters this model.

### 2.1 What each mesh publishes

A `Mesh` keeps a private per-vertex record beside `faceBound`:

```text
vertexBound []float64   // one entry per vertex, millimetres; nil when absent
```

| Producer | `vertexBound` | Why it is sound |
|---|---|---|
| `mitredSweepPayload` (sweep §16) | each vertex's own rounding gap, `Radius3D(ProvenUpRound(gap_v))`, where `gap_v` is the largest per-coordinate gap between that vertex's exact rational and its float (the per-vertex value the build already computes before taking `delta` as the maximum, `sweep_mitre_build.go`) | the true faces are the exact polygons on the rationals, so every facet's true piece is the affine triangle on its corners' rationals |
| `facetedPayload` | the payload's own per-vertex record (§3), restated unchanged | composed by §3 from operand records |
| `stitchPayload`, `chainPayload` | every corner's `Vertex.Bound()` | those rows already state the corner-maximum model |
| every chorded analytic payload (prism, cup, revolve, loft, cap-blend) | ABSENT (`nil`) | the facet's own sagitta is not a corner property |

**A mesh with no per-vertex record reads `β(v)` as the largest `faceBound`
over the faces whose facets touch `v`.** That is a two-sided statement the
mesh already made: `v` lies on a facet whose face's true patch is within
`faceBound` of it, so a true point within `faceBound` of `v` exists, and
`δ(t) = max corner β ≥ faceBound(face(t))` keeps every facet's own published
displacement, including its chord sagitta, because every corner of `t` touches
`face(t)`. The reading is looser than `faceBound` only at a corner shared with
a coarser neighbouring face, which is the conservative direction. A curved
payload that later wants a per-vertex record must fold its facet departure
into every corner's `β` itself; a separate per-facet departure term is not
part of this design.

## 3. Composition through one boolean

The mesh boolean (evaluator §9; `internal/meshbool`'s `MeshBoolean`,
`CutTriangle` and `StitchFacetsContext`, driven by `boolean.go`'s
`evaluateBoolean`) produces three kinds of result vertex. Each
gets its bound from one rule, and no rule reads a global figure.

### 3.1 An operand vertex keeps its own bound

A kept vertex that was a vertex of operand A lifts to the exact rational of
its float (`prepBoolMeshContext`) and rounds back to the identical float at the
final weld (`StitchFacetsContext`), so its weld displacement is exactly zero and
`β_result(v) = β_A(v)`. The same holds for operand B.

### 3.2 A rim vertex takes the pair's own trim amplification

A rim vertex `r` is an exact point of the contact segment of ONE facet pair
`(t_A, t_B)` (`TriContact.P0`/`P1`). Its bound before the weld is

```text
β_pre(r) = upRound( (δ(t_A) + δ(t_B)) / sin θ(t_A, t_B) )
```

with `sin θ(t_A, t_B)` the proven lower bound `SinLowerBound(c.Sin2)` of THAT
pair's crossing angle, read from the contact's own exact `Sin2` rather than
from the pair-wide minimum the global composition of §1 divides by. The
argument is that composition's, applied per pair: the true piece of `t_A` lies within `δ(t_A)`
of `t_A`'s plane and `t_B`'s within `δ(t_B)` of `t_B`'s, the intersection of
two slabs of those half-widths crossing at `θ` is a tube of half-width
`(δ(t_A) + δ(t_B))/sin θ` about the exact crossing line, and the true rim
point `r*` lies in that tube. A rim vertex two contact segments share (a
chain vertex) takes the larger of the two values. The pair-diameter refusal
stays per vertex: a `β_pre(r)` at or above the pair diameter `dPair`, or a
non-finite one, refuses the operation with the same `ErrUnsupported`.

Both operand facets of a contact are known exactly where the contact is
classified, so `MeshBoolean` computes the pair's `RimBound` from its `Sin2`,
`δ(t_A)` and `δ(t_B)` there and records it against each segment endpoint's
exact point. `KeepSide` then gives every kept facet corner its pre-weld bound
(`KeptFacet.Beta`): an operand corner its own `β`, a recorded rim point its
`RimBound`, a point that is both the larger, and any other point §3.3's.

### 3.3 A new vertex on an operand facet that is not a rim point

The conforming pass and the artificial split line of `CutTriangle` create
vertices on a facet's own edges that no contact segment ends at. Such a vertex
lies on the held facet, so by §2's facet claim it takes that facet's bound:
`β_pre(x) = δ(t)`, with `t` the operand facet for a cutter point and the kept
facet being split for a conforming insertion. A point two facets place takes
the larger of the two.

### 3.4 The weld, per vertex

`StitchFacetsContext` already measures every stitched vertex's per-coordinate
rounding gap before taking the maximum as `round`. The per-vertex figure is
kept: `weld(v) = Radius3D(ProvenUpRound(gap_v))`, zero exactly for every
operand vertex (§3.1), and

```text
β_result(v) = upRound( β_pre(v) + weld(v) )
```

The global `round` stays what `SweptVolumeAllow(round, preArea)` charges the
volume with; §4.2 states what else still reads it.

### 3.5 Why the result's facets satisfy §2's claim

A result facet `f` is a sub-triangle of exactly one operand facet `t_A`
(subdivision never merges). Its corners are operand vertices of `t_A` (§3.1),
rim vertices (§3.2) or edge vertices (§3.3), and by §2's affine argument every
point of `f` is within `δ(f) = max corner β_result` of the affine triangle on
its corners' true points. For a planar true face that triangle is `f`'s true
piece. The true piece's boundary along the cut is the true rim, and every
point of it is within `β(r)` of the held rim by §3.2, so the correspondence
is two-sided at the cut as well as in the interior. For a curved operand face
`δ(f) ≥ faceBound(face(t_A))` holds by §2.1's reading, which is the two-sided
statement tessellation §11 already makes for the whole face; this design
localises WHICH facets also carry a rim amplification, never what one facet's
displacement means. The volume composition is unchanged, since the rim never
enters `volSymDiff` (tessellation §11 step 6); the hidden-tangency pre-pass
of the NEXT boolean reads each face's `sourceBound` as the maximum `δ(t)` over
that face's facets, which is at most today's global `Delta`, so every
admission it makes is one the global figure would also have made, and every
refusal it drops is one a tighter proven bound makes unnecessary. The gate
stays reject-only: it compares a proven bound against a proven depth and
never reads the geometry to admit.

## 4. Consumers

### 4.1 The faceted payload record

`facetedPayload` gains `vertexBound []float64`, one entry per held vertex.
`meshBound` stays, redefined as the maximum `δ(t)` over the held facets, so
every reader that wants one body-wide figure keeps one that is never looser
than today's: `Verify`'s planar faceted contact kernel
(`contact_faceted_pair.go`'s `planarFacetedSolid`, `contact_faceted_support.go`,
`contact_faceted_sweep.go`), the mass-property reader
(`mass_properties_faceted.go`, which checks `mesh.bound == pp.meshBound`), and
the translation-only zero-bound fast path (`facetedTranslationOnly`), whose
condition becomes "every `vertexBound` entry is zero". A nil `vertexBound`
on a payload is an invariant failure, never a fallback: the composing boolean
always writes it.

### 4.2 Body readings

| Reading | Global reading of §1 | Per-vertex reading |
|---|---|---|
| `Vertex.Bound` | `meshBound` | `β(v)` |
| `Faceted.Bound` (a face) | `meshBound` | `max δ(t)` over the face's facets |
| `FacetedCurve.Bound`, `Edge.Length` bound | `meshBound`; `ChainLengthBound(n, meshBound, len)` | `max β` over the chain's vertices; `ChainLengthBound(n, that max, len)` |
| face area bound | `meshBound × perimeterUpper + areaSlack + SumSlop + Σ_t FacetAreaTermSlop(t)` | `min(δ_f × perimeterUpper, Σ_t PerturbedTriangleAreaAllow(t, δ(t))) + areaSlack + SumSlop + Σ_t FacetAreaTermSlop(t)`, with `δ_f` the face's own `Faceted.Bound` and the sum the `stitchPayload` row's term over the face's facets, through `AbsSumUpper` |
| body area bound | the same with the summed face perimeters | the sum of every face's geometric term, plus `areaSlack + SumSlop + Σ_t FacetAreaTermSlop(t)` over every facet |
| `Volume` | `symA + symB + SweptVolumeAllow(round, preArea)` | unchanged; `round` stays the global weld maximum here |
| `Centroid` | from `volSymDiff` and `dPair` | unchanged |
| `Box.Bound` | `Radius3D(meshBound)` | `Radius3D(e)`, with `e` the largest, over the six extremes, of `max(max_v (v.x + B(v)) − m, m − max_v (v.x − β(v)))` for a held maximum `m` (the minimum mirrored), where `B(v)` is the largest `δ(t)` over the facets touching `v`, formed exactly and rounded up |

Both area terms are proven upper bounds on the same displacement, so the
smaller is one too: the perimeter term is at most the global one because
`δ_f ≤ meshBound` (§4.1), and the facet sum is the tighter of the two on a
face of few facets whose rims carry a large bound.

`SumSlop` bounds the float loop that sums the facet areas, and
`FacetAreaTermSlop` bounds each facet's own float area term. The second is
charged at the scale of the facet's edge products, not its area, because a
sliver facet's cross product cancels: its rounding can exceed any relative
charge on the area it cancels to.

For the box, every true boundary point lies within `δ(t)` of a point of some
held facet `t`, whose coordinate on the axis is at most its largest corner's,
so the true maximum is at most `max_v (v.x + B(v))`; and every held vertex has
a true point within `β(v)`, so the true maximum is at least
`max_v (v.x − β(v))`. A vertex inside the held box whose own reach passes the
extreme widens `e`, which the extreme vertex's own bound alone would miss.

### 4.3 Placed

`facetedPayload.placed` adds `RigidRoundAllow(|v|_max, |translation|_max)` to
each vertex's own `β(v)`, at that vertex's own largest coordinate magnitude
rather than the body's, since the rounding a rigid motion commits on one
vertex depends on that vertex's inputs alone (`internal/proofbound/bounds.go`,
`RigidRoundAllow`). `meshBound` is recomputed as the maximum `δ(t)` after the
addition. The exact-source record for a translation-only placement is kept or
dropped under the same conditions as today, with §4.1's zero condition.

### 4.4 Faceted restatement

`tessellateFaceted` publishes the payload's `vertexBound` on the `Mesh`,
`faceBound[f] = max δ(t)` over the facets `faceOf` maps to `f`, and
`bound = meshBound`. Tessellation §7's refusal of a public `Tessellate` call
whose tolerance is below the held `meshBound` stays: a caller who asked for a
finer mesh than the body holds still gets `facetedBoundError`. Only the
boolean's own request changes (§5).

### 4.5 The prepared operand

`meshbool.BoolMesh` carries the operand mesh's per-vertex bounds (or §2.1's derived
reading when the mesh publishes none) and exposes `δ(t)` per facet, which is
what the contact classifier records on each `Xseg` (§3.2) and what §5's gate
reads.

## 5. The chain-depth comparison

A global comparison, which asks every operand for a mesh at the pair's chord
tolerance `tol` and refuses a faceted operand holding `meshBound > tol`
inside `tessellateFaceted` before any contact is examined, compares the wrong
number once bounds are per vertex: one coarse rim far from the new contact
would refuse a pair whose every new rim is composed from fine geometry.

**What the gate must certify.** Every rim vertex this operation creates is
composed from operand facets whose bounds are at most the pair's own chord
tolerance, so that

```text
β_pre(r) ≤ 2·tol / sin θ(t_A, t_B)
```

holds for every rim it makes: the same ceiling a rim of two freshly chorded
analytic operands has. Geometry the pair's contacts do not touch keeps the
bound it already published, which the caller could read before planning the
chain (core §8), and is never the reason for a refusal.

**The rule.** The boolean asks a RESTATING operand for its mesh at
`max(tol, heldFloor)`, where `heldFloor` is the payload's own `meshBound`
(`facetedPayload`) or `delta` (`mitredSweepPayload`, `coilPayload`), read by a
`heldFloorOf(body)` reader beside `sectionDisplacementOf`; a restatement
returns the same vertices at any tolerance at or above its floor, so the mesh
is the one the boolean needs and the request never refuses.

**A held primitive raises the pair's tolerance.** An operand built as one
fixed held mesh — a coil or a mitred sweep, whose `delta` is a proven bound
of that one construction rather than a figure a chain grows — raises the
pair's chord tolerance itself to that bound, as a displaced prism section
already raises it past twice its displacement (`pairChordFrom`, through
`heldPrimitiveFloorOf` beside `sectionDisplacementOf`). Every facet such an
operand touches the partner with then holds `δ(t) ≤ delta ≤ tol`, so the gate
below admits it. A boolean result's `meshBound` does not raise the pair
tolerance: it is the figure a chain grows, and the gate exists to refuse it.
The partner pays for the raise: a chorded analytic partner is meshed at the
raised `tol`, so its mesh may be coarser than the pair's size alone would
ask, and the result's rims and area and volume bounds compose from that
coarser mesh. Its published bounds still hold, since every tessellation
proves its `sourceBound`, `areaSlack` and `volSymDiff` for the tolerance it
was asked at. Otherwise the request for a chorded analytic operand stays
`tol`. After the contact classification has run
and before any facet is cut, the gate walks every operand facet the
classification reports as meeting the other operand (`ContactPoint`,
`ContactSegment`, or within the pre-pass slack) and refuses with
`ErrUnsupported`, through `meshbool.BooleanExpectedStaging`, when any such
facet of a RESTATING operand has `δ(t) > tol` — which a held primitive's
facets never do (`meshbool.RefuseCoarseHeldContact`,
called from `MeshBoolean` on the classified contacts and from the root's
hidden-tangency pre-pass on the facets within its slack;
`booleanOperandStaging` restates it in the boolean's own terms). The message names the operand (`Cut`'s
target or tool, else first or second), the bound of the touched facet, the
pair's chord tolerance, and says that a boolean takes no tolerance. A chorded
analytic operand is not gated: its `sourceBound` may exceed `tol` by
tessellation §1's Tolerance row today, and nothing here changes that.

**Reject-only.** The gate compares two proven numbers, a facet's published
bound and the pair's tolerance, and refuses when the first exceeds the second.
It admits nothing: a facet it passes enters the composition with its own
bound exactly as §3 states, and no measurement of the handed geometry decides
the answer. It can refuse a pair whose rims would have been fine, exactly as
today's global comparison can, and that stays the accepted price.

**What changes for a caller.** A chain of booleans over planar-faced bodies
is refused only where a new contact lands on earlier rim geometry whose
bound has outgrown the pair tolerance; §6 measured no such refusal over fifty
unions. A result can carry a `Bound` above the next pair's tolerance in
untouched regions and still serve as an operand. Core §8's "The chain depth"
paragraph, evaluator §9's operand and rim bullets and tessellation §7/§11
state this rule.

## 6. Measured growth

Dated illustration, measured on 2026-10-06 against `c19a0468` with a
post-processing prototype of §3's rules (not committed), on a tree of
mitred sweeps: a 16-gon trunk of circumradius 5 mm along a two-span path
`(0,0,0)→(0,0,30)→(3,0,60)` scaled `1, 0.9`, and fifty 16-gon branches
tapering 1.5 → 1 mm along a three-span path, each tilted 40° from the trunk
axis, its root 2.6 mm from the axis, 1.1 mm higher than the previous root and
turned by the golden angle. Each union is `Union(tree, branch)`.

| Union | today's `meshBound` | pair `tol` | §3 max `β` | max `β` among that union's rims | trunk `δ(t)` at the contact |
|---|---|---|---|---|---|
| 1 | 1.4e-14 | 1.26e-3 | 1.4e-14 | 1.4e-14 | 6.2e-15 |
| 10 | 3.0e-12 | 1.31e-3 | 3.3e-14 | 3.3e-14 | 1.7e-14 |
| 20 | 7.4e-10 | 1.32e-3 | 6.2e-14 | 5.5e-14 | 2.6e-14 |
| 30 | 2.1e-7 | 1.32e-3 | 7.6e-14 | 6.4e-14 | 2.4e-14 |
| 40 | 8.2e-5 | 1.32e-3 | 1.8e-13 | 1.8e-13 | 6.2e-14 |
| 45 | 2.2e-3 | 1.33e-3 | 2.2e-13 | 2.2e-13 | 8.2e-14 |
| 50 | 1.9e-2 | 1.43e-3 | 2.5e-13 | 2.5e-13 | 1.9e-13 |

All figures in millimetres. Today's bound grows ×1.5–1.9 per union and core
§8's gate refuses the 46th union; the per-vertex maximum grows ×18 over all
fifty, and the trunk facets each new branch cuts carry at most `1.9e-13`,
seven orders under the tolerance. The final body holds 5712 vertices; a union
took 20 ms at the start and 560 ms at the end, unchanged by the bookkeeping.

Two clustered variants put consecutive branches 7° apart at 2.2 mm steps and
25° apart at 1.1 mm steps, so that branches cut each other and the same trunk
walls repeatedly: after 22 and 10 unions today's bound reached `8.4e-10` and
`2.2e-10` against per-vertex maxima of `3.8e-13` and `4.8e-14`. Both runs
then stopped on an `ErrBooleanFailed` from the facet cutter
("a boolean subdivision polygon closed clockwise"; "exact ear clipping
stalled"), an invariant failure outside this design's scope.

Interpolating the corner bounds at the cut point instead of taking their
maximum (an affine bound over the facet, with the rim denominator reduced by
the two facets' bound gradients) gave `1.1e-13` instead of `2.5e-13` after
fifty unions and at most ×2.5 in the clustered runs; it is not scheduled.

**Curved operands, estimated.** A chorded cone wall's facets carry
`δ(t) ≈ tol` (the sagitta), so a cone-to-cone union's rims carry
`≈ 2·tol/sin θ`, about `8·tol` at the hand-off's crossings, exactly what
the hand-off measured (`0.0093 → 0.051 mm` over two unions). Under §5's rule
the next union is refused only when its contacts touch those rims; a branch
whose root is buried in the previous segment's END region touches facets that
still carry their own `≈ tol`, so an end-to-end chain of cones continues at
every length, with each joint's rims carrying `≈ 8·tol` and nothing compounding
across joints. A chain whose every union cuts the previous rim compounds
exactly as today.

## 7. Curved operands

| Option | Gains | Costs |
|---|---|---|
| **A. Per-vertex bounds, chain-depth gate stays global** (PRs 1–3 alone) | planar chains run to fifty unions and beyond with bounds near `1e-13`; no contract change; `Bound`s shrink for every faceted body | a curved operand's result still carries `≈ 8·tol` as its global bound and is refused at the next union, so mtilt's request 1 stays refused |
| **B. Per-vertex bounds and the local gate of §5** (PRs 1–4) | curved chains continue wherever a new contact avoids earlier rims, with the rim ceiling `2·tol/sin θ` certified per union; planar chains are additionally protected against one coarse rim far from the contact | core §8's chain-depth paragraph, evaluator §9 and tessellation §7/§11 are rewritten; a result can publish a `Bound` above the next pair's tolerance and still be an operand, which callers judging with `WithTolerance` at `Verify` already handle |
| **C. Drop the chain-depth gate** | simplest code; nothing is ever refused before contact | the only ceiling left is `rimDelta`'s pair-diameter refusal, so a chain can publish a bound of millimetres on a part of centimetres without any refusal naming it; core §8's promise that a chain is limited by a stated comparison is gone |

**Recommendation: B.** It is A plus one gate and three companion edits, the
gate certifies a stated inequality, and it is the only option under which the
hand-off's curved chain continues.

## 8. Increments

Files are named by role. "The mesh-boolean pipeline" is `internal/meshbool`
(the contact gather, the facet cutter and the stitch) together with the
root's `boolean_mesh.go`, which prepares each operand for it.

| PR | Lands | Files | Tests and the red leg each must show |
|---|---|---|---|
| **1** | `Mesh.vertexBound` and its setter in the mesh proof record (`tessellate.go`); the mitred sweep build keeps each vertex's own rounding gap and publishes it, with `faceBound[f]` the corner maximum over `f`'s triangles and `delta`/`bound` the overall maximum (`sweep_mitre_build.go`, `sweep_mitre.go`); §2.1's derived reading for a mesh with no record, as one reader the pipeline will call | mesh record, mitred build, mitred restatement | A mitred sweep with its path start at the origin and an axis-aligned first span publishes `β = 0` at every start-cap vertex and `β ≤ delta` everywhere, with equality at one vertex; red leg: publish `delta` at every vertex and the zero assertion fails. The derived reading on a revolve mesh equals the largest incident `faceBound` at a cap-wall corner; red leg: read the vertex's own face alone and the corner assertion fails |
| **2** | the composition of §3: facet bounds on the prepared operand, `Sin2`/`δ` recorded on each contact segment, corner provenance on kept facets, per-vertex weld in the stitch, `facetedPayload.vertexBound`, `meshBound` as the facet maximum; measurements still read `meshBound` | mesh-boolean pipeline, `boolean.go`'s composition, `boolean_body.go`'s payload | Union of two crossing mitred branches (the existing `apitest` fixture): every surviving operand vertex keeps its operand `β` to the bit; every rim vertex's `β` equals `upRound((δ(t_A)+δ(t_B))/SinLowerBound(Sin2)) + weld` recomputed from the fixture's own facet pair; `meshBound` is at most today's. Red legs: divide by the pair-wide `sinMin` and the rim equality fails at a rim whose own crossing is not the shallowest; drop the per-vertex weld and tessellation §14's "rational intersection vertices round inexactly" fixture reports a rim `β` below the measured rounding gap |
| **3** | §4's readings: `Vertex.Bound`, edge bounds, `Faceted.Bound`, area, box; `Placed` per vertex; `tessellateFaceted` publishes the record and per-face `faceBound`; the pre-pass reads per-face slack | `boolean_body.go`, `tessellate.go` | A three-union chain: an untouched trunk vertex reports the trunk's own `delta` as its `Vertex.Bound`, an untouched trunk face reports it as `Faceted.Bound`, and the box bound equals `Radius3D(e)` recomputed from the published vertices; red leg: publish `meshBound` to every vertex and the untouched-vertex equality fails. A fixture whose largest-`β` vertex sits inside the held box but within `β` of one face of it: the box bound covers `m − (v.x − B(v))`; red leg: drop that term and the fixture's exact source box (the exact rationals) escapes the published bound. `Placed` of a boolean result: each vertex's bound grows by `RigidRoundAllow` at its own magnitude; red leg: charge the body-wide magnitude and the near-origin vertex's bound is strictly looser than the assertion allows |
| **4** | §5: `heldFloorOf`, the per-operand request tolerance, the post-classification gate and its message; companion edits to core §8 "The chain depth", evaluator §9's rim bullet, tessellation §7 (the boolean's request) and §11 steps 7–8 and the result-payload paragraph; `booleanOperandStaging` rewritten | `boolean.go`, the mesh-boolean pipeline's contact gather, the three design docs | Twenty unions of §6's fixture succeed with every union's rim `β` at most `2·tol/sin θ` of its own pair. A fixture whose operand carries one rim with `β > tol` away from the new contact succeeds, and the same operand with the new contact placed ON that rim refuses with the new message; red leg: compare the operand's `meshBound` and the first fixture refuses. A cone-to-cone-to-cone chain (the hand-off's request 1 shape) builds three segments; red leg: the global comparison refuses the third |
| **5** (optional, not scheduled) | the interpolated rim rule of §6 | mesh-boolean pipeline | only if a measured chain needs it |

Each PR ships its own doc edits (this document's affected sections and the
companion documents named in its row), updates `.github/test-shards.txt` or
`.github/test-shards-apitest.txt`, and passes `go test ./...`, `go vet ./...`
and `golangci-lint run`.

## 9. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Rim rule | corner maximum (§3.2) | interpolated corner bounds, ×2–2.5 tighter in the clustered runs, with a gradient-corrected denominator to prove |
| Gate scope | restating operands only; chorded analytic operands keep tessellation §1's Tolerance row | gate every operand's contacted facets, which would newly refuse an analytic operand whose `deltaStore` exceeds `tol` |
| Whether the gate exists | §7's option B | option C |
| Weld and placement rounding | per vertex (§3.4, §4.3) | the global maximum at every vertex, which is what today does |
| Per-facet departure term | none; a curved per-vertex record folds it into `β` (§2.1) | a `facetBound []float64` beside `vertexBound`, needed only when a curved payload wants per-vertex bounds below its sagitta |
| Box bound | the six-extreme rule of §4.2 | `Radius3D(meshBound)`, which stays sound and is what §4.1's maximum makes available |
