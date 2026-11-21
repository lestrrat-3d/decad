# Faceted Vertex Bounds Design

Per-vertex displacement bounds on faceted bodies, carried through every mesh
boolean, so that a chain of booleans over planar-faced bodies is limited by
the bound of the geometry each new pair touches rather than by one global
held-mesh bound per body. Companion to `agent-docs/tessellation-design.md`, whose
§2 proof record, §7 faceted restatement and §11 boolean composition this
design refines, and to `agent-docs/evaluator-design.md` §9, which owns the mesh
boolean. References of the form "core §N" are to `agent-docs/api-design.md`;
"tessellation §N" to `agent-docs/tessellation-design.md`; "evaluator §N" to
`agent-docs/evaluator-design.md`; "sweep §N" to `agent-docs/sweep-design.md`.

No public type changes. `Vertex.Bound`, `Faceted.Bound`, `FacetedCurve.Bound`, `Measurement.Bound`,
`Box.Bound` and `Mesh.Bound` report proven per-vertex numbers. The one caller-visible behaviour is §5's
chain-depth refusal, which core §8 "The chain depth" states.

## 1. Problem

A global composition would give a boolean's `facetedPayload` ONE displacement bound, `MeshBound`, for every held
vertex of the body (`internal/facetedrecord/payload.go`). Every reading would compose from it: every `Vertex.Bound`, every face's
`Faceted.Bound`, every rim edge's `FacetedCurve.Bound` and length bound, the
area bound (`MeshBound × perimeter`), and the box bound (`Radius3D(MeshBound)`).
Restating the result (`facetedrecord.Restate`) would give every face that one number as `sourceBound`, and
the next boolean would divide the SUM of the two operands' global bounds by the sine of the SHALLOWEST crossing
anywhere in the pair (tessellation §11 step 7).

So one union's rim bound would become the whole body's bound, and the next union would amplify it again at every
vertex, whether or not the new pair touches the old rim. The factor per union is `1/sin θ_min` plus the weld, so
a chain of shallow crossings compounds geometrically. A global chain-depth gate would refuse the result as an operand
once `MeshBound` exceeds the next pair's chord tolerance (`meshbool.ChordFactor` `×` the pair diameter). Long before,
the published volume, box and vertex bounds would grow for a body whose held vertices never moved.

Tessellation §2 allows a `facetedPayload` to "retain a tighter displacement per live faceted face". A per-FACE
bound is not enough: a trunk wall is one face cut by every branch that enters it, and a cone wall is one face
cut at both ends, so a per-face figure carries every rim's amplification across the whole face. The held vertex
is the unit that stays local.

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
this is the model tessellation §2's `stitchPayload` and `chainPayload` rows use ("the largest `Vertex.Bound()` over the
vertices that face's own triangles touch"), and `PerturbedTriangleAreaAllow`
already charges area under it. For a CURVED true face the affine triangle is
not the true piece, and the corner bounds alone do not cover the chord
sagitta between them; §2.1 states how such a mesh enters this model.

### 2.1 What each mesh publishes

A mesh node (`topo.Mesh`) keeps a per-vertex record beside `FaceBound`:

```text
VertexBound []float64   // one entry per vertex, millimetres; nil when absent
```

| Producer | `vertexBound` | Why it is sound |
|---|---|---|
| `mitredSweepPayload` (sweep §16) | each vertex's own rounding gap, `Radius3D(ProvenUpRound(gap_v))`, where `gap_v` is the largest per-coordinate gap between that vertex's exact rational and its float (the per-vertex value the build already computes before taking `delta` as the maximum, `internal/sweepmitre/placement.go`) | the true faces are the exact polygons on the rationals, so every facet's true piece is the affine triangle on its corners' rationals |
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

The mesh boolean (evaluator §9; `internal/meshbool`'s `MeshBoolean`, `CutTriangle` and `StitchFacetsContext`, driven by `boolean.go`'s
`evaluateBoolean`) produces three kinds of result vertex. Each gets its bound from one rule, and no rule
reads a global figure.

### 3.1 An operand vertex keeps its own bound

A kept vertex that was a vertex of operand A lifts to the exact rational of
its float (`booleaneval.PrepBoolMesh`) and rounds back to the identical float at the
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
from the pair-wide minimum §1's global composition divides by. The argument is that composition's, applied per
pair: the true piece of `t_A` lies within `δ(t_A)`
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

A face of the result whose held facets are exactly coplanar, hole-free, with
every vertex claim zero and float corners is re-triangulated on its corners
before the weld, so it holds no vertex the cutter placed and its vertices
weld at zero (`agent-docs/coplanar-contact-design.md` §7).

`StitchedMesh.RoundingVolumeBound` charges occupied-volume error. Its global
bound is `SweptVolumeAllow(round, preArea)`. After positive-reach repair it
also sums each pre-round triangle's swept-volume bound using the largest
rounding distance of that triangle's corners, and takes the smaller proven
bound. Collapsed triangles remain in that sum. The bound is owned by
`proofbound.PerturbedAreaAndRoundingVolumeContext`; inherited vertex bounds
do not enter this rounding charge.

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
that face's facets, which is at most the global `Delta`, so every
admission it makes is one the global figure would also have made, and every
refusal it drops is one a tighter proven bound makes unnecessary. The gate
stays reject-only: it compares a proven bound against a proven depth and
never reads the geometry to admit.

### 3.6 Shallow crossings

A rim vertex's bound divides the pair's summed facet bounds by `sin θ`, so two facets that cross at a shallow
angle give a large bound however small their own bounds are. A spiral bevel member whose rim pairs cross at
a very small `sin θ` therefore carries a large rim bound, and `VerifyAll` at a tolerance below it refuses with
"requested tolerance ... is below the faceted body's minimum mesh bound".

These pairs have nearly equal outward normals, so for Union and Intersect the result's corner at the rim is
nearly flat, and the nearest point of the true boundary is much closer than `(δ_A + δ_B)/sin θ`. A flat-corner
rule of the form `min((δ_A + δ_B)/sin θ, √2 · max(δ_A, δ_B))` for `n_A · n_B ≥ 0` (`-n_B` for Cut) is NOT part of
this design. It rests on a band premise: that the true solid lies inside every point farther than `δ` on a
facet's inner side and outside every point farther than `δ` on its outer side, within a ball of radius
`√2 · max(δ_A, δ_B)` about the rim vertex. Boundary closeness does not establish it:

- Two thin plates thinner than `δ`, nearly parallel and co-oriented, are boundary-close to their meshes, yet
  their true Intersect is empty near the rim and the nearest true boundary lies about `2δ/θ` away.
- Another facet of the same operand inside the ball breaks the premise for the points past it.
- The flat-corner bound is a distance to the result's boundary. It is not a distance to the true crossing of the
  two true faces, so §2's definition of `v*` for a rim vertex and §3.5's per-corner affine argument would change
  with it.

A sound version needs a one-sided-band flag published by every producer (including the chorded loft and revolve
payloads, which have no vertex record), an exact rational clearance check of every other facet of the operand
against the ball, and a redefined meaning of the bound with every consumer updated. Until then the rule above
stays as §3.2 states it.

### 3.7 Keeping a rim's bound beside the rim

A kept piece of an operand facet has a rim corner `u` whose bound is far above the facet's own `δ(t)` and, as its
other corners, operand corners or points `δ(t)` covers. The piece's facet bound is the largest corner bound, so the
rim's bound stands for the whole piece. A facet that runs the height of a tube wall carries it along the whole
height, and the next boolean that crosses the piece, however far from the rim, reads it and amplifies it again.

`RimBound` reads the facet's slab (faceted-source-retention §11), so on a retained `Plane` or `Cylinder` the later
boolean reads how far the true face lies from the piece's plane, which the rim's corner bound does not raise, and a
chain of rims on one such wall does not compound. On any other curved face the slab is the facet bound.

`meshbool.SplitSpread` splits such a piece. On each edge from a rim corner `u` to another corner `v` it places a
candidate point `x` at `1/2^j` of the edge from `u`, with the largest `j` that keeps `x` at least `4 β(u)` from `u`
and deeper than a multiple of the clearance below from the other operand's held boundary. The piece becomes a rim
piece, which keeps `β(u)`, and far pieces: those with no rim corner. The corners of a far piece are operand corners
and split points. A split point takes `β(x) = δ(t)` (§3.3), because it lies on the held facet `t`.

The claim is over the far piece, not over its split points. A far piece's facet bound is `δ(t)`, the largest corner
bound, and the next boolean reads it for every point of the piece. The distance from a point of the piece to a convex
other operand is not concave along a segment, so it can dip below the clearance between two points that both clear it,
and a proof at the split points leaves the points between them unproven.

A far piece `T` carries `δ(t)` when every point `q` of it is farther than `δ(t) + δ_o` from the held boundary of the
other operand, `δ_o` the largest bound that operand holds. The true piece of `t` within `δ(t)` of `q` is a point `p*`
of the true surface, and it is part of the result's boundary only if the other operand's true boundary does not trim
it. The held and the true boundary of the other operand differ by at most `δ_o`, and the segment `q p*` is at most
`δ(t)` long. So when every point of `T` is farther than `δ(t) + δ_o` from the held other boundary, the segment meets
neither boundary, and `p*` is on the same side of the true other boundary as `q`.

`SplitSpread` proves that for the whole closed triangle `T`. `farPiecesClear` takes each facet of the other operand
whose box comes within the clearance of `T`'s box, grown by the corners' rounding. It proves the distance from `T` to
that facet above `δ(t) + δ_o`, first by a float separating axis with its rounding charged (`triGapAbove`), then by
`DisjointTriDistanceAbove`, the exact rational distance. A facet neither proof clears, whether it crosses `T`,
touches it or stands too near, refuses the split. A candidate split is emitted only when every far piece it makes is
proven. Otherwise the next candidate is tried: the clearance margin takes the values 1, 8, 64 and 512
(`spreadMargins`), so there are at most four candidates. A piece with no passing candidate stays whole. The test is
reject-only: a float distance at or below the clearance refuses without the exact proof, and only an axis gap or the
exact distance admits.

Which pieces never pass:

- A far piece with a corner on the other operand's boundary, such as a chain corner that holds a low bound, never
  passes: its distance to the other operand is 0 there. The piece stays whole and keeps the rim's bound.
- A far piece that reaches within the clearance of the rim `u` or of the chain beside it never passes. A flat triangle
  whose rim corner lies almost on the opposite edge makes such a piece however far the split points stand, and it
  stays whole.

Further conditions:

- The pieces keep the orientation, the source face and the operand facet's bound. They only add vertices, and a
  rim corner, an operand corner and every other bound of the piece are unchanged. The stitch's conforming pass
  inserts a split point into the facet across a shared edge, and that facet keeps its own bound. A neighbour that
  is not split at that point takes the point with the neighbour's own facet bound, which is at least `β(u)`, so
  the insertion undoes the split for the far pieces that use that point and never lowers a bound.
- A piece on a planar face is never split. A split point lies on no boundary loop of its face, and
  `internal/retainedblend` re-triangulates a face from its boundary loops and refuses a vertex off them
  (`a face holds a vertex off its boundary`).
- Only a document that sets `decad.WithRimSplit` is split, and a two-body `Union` that §5's gate refuses, which
  rebuilds its recorded `Union` history with splitting on (`agent-docs/prism-boolean-design.md` Part R, "Rim-split
  rebuild"). A vertex that no boundary loop of a curved face reaches
  is refused by the retained-blend reading too (`a face holds a vertex off its boundary`, or `the blend consumes the
  wall`), so a document that edge-finishes its mesh results keeps the faces that expect it. Only a rim corner
  above the pair's chord tolerance is split. `Fillet` and `Chamfer` may refuse a split face. A blend that succeeds
  reads the far pieces' facet bounds, which the proof above covers, and its result bound is lower for it.
  `WithRimSplit` adds no refusal for bounds, because no bound a blend reads is unproven. The refusals that remain come
  from the split vertex lying off the face's boundary loops.
- A bounded replay applies the same pass. It charges the scan of the other operand's facets, a flat 64 work
  (`spreadExactWork`) per exact distance proof, and the records of each split. It reserves the output slice's storage
  before it allocates it or grows it, and every append to that slice goes through the reservation. The exact-arithmetic
  temporaries (`proof.Xlerp`, `PointRoundBound`, the rationals inside `DisjointTriDistanceAbove`) reserve no bytes;
  they are bounded by a constant multiple of the corner sizes, and their cost is the flat 64 work.
- The rim-split choice lives on the evaluation context, not in the recorded body. A replay of a recorded boolean must
  run under the same option, or it builds a different mesh.

## 4. Consumers

### 4.1 The faceted payload record

`facetedPayload` carries `VertexBound []float64`, one entry per held vertex. `MeshBound` is the maximum `δ(t)`
over the held facets, so every reader that wants one body-wide figure gets one never looser than the global
composition's: `Verify`'s planar faceted contact kernel
(`internal/planarpose/pose.go`, through `planarband.FacetedSolid`; `internal/contactfaceted/floor.go`), the
mass-property reader (`internal/massdispatch/dispatch.go`, which reads `pp.MeshBound`), and the
translation-only zero-bound fast path (`internal/planarsnapshot/faceted.go`'s `translationOnly`), whose
condition is "every `VertexBound` entry is zero". A nil `VertexBound`
on a payload is an invariant failure, never a fallback: the composing boolean
always writes it.

### 4.2 Body readings

| Reading | Global reading of §1 | Per-vertex reading |
|---|---|---|
| `Vertex.Bound` | `MeshBound` | `β(v)` |
| `Faceted.Bound` (a face) | `MeshBound` | `max δ(t)` over the face's facets |
| `FacetedCurve.Bound`, `Edge.Length` bound | `MeshBound`; `ChainLengthBound(n, MeshBound, len)` | `max β` over the chain's vertices; `ChainLengthBound(n, that max, len)` |
| face area bound | `MeshBound × perimeterUpper + AreaSlack + SumSlop + Σ_t FacetAreaTermSlop(t)` | `min(δ_f × perimeterUpper, Σ_t PerturbedTriangleAreaAllow(t, δ(t))) + AreaSlack + SumSlop + Σ_t FacetAreaTermSlop(t)`, with `δ_f` the face's own `Faceted.Bound` and the sum the `stitchPayload` row's term over the face's facets, through `AbsSumUpper` |
| body area bound | the same with the summed face perimeters | the sum of every face's geometric term, plus `AreaSlack + SumSlop + Σ_t FacetAreaTermSlop(t)` over every facet |
| `Volume` | `symA + symB + SweptVolumeAllow(round, preArea)` | `symA + symB + StitchedMesh.RoundingVolumeBound()` (§3.4) |
| `Centroid` | from `VolSymDiff` and `DPair` | unchanged |
| `Box.Bound` | `Radius3D(MeshBound)` | `Radius3D(e)`, with `e` the largest, over the six extremes, of `max(max_v (v.x + B(v)) − m, m − max_v (v.x − β(v)))` for a held maximum `m` (the minimum mirrored), where `B(v)` is the largest `δ(t)` over the facets touching `v`, formed exactly and rounded up |

Both area terms are proven upper bounds on the same displacement, so the
smaller is one too: the perimeter term is at most the global one because
`δ_f ≤ MeshBound` (§4.1), and the facet sum is the tighter of the two on a
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

### 4.3 Transform

`facetedPayload.Placed` adds `RigidRoundAllow(|v|_max, |translation|_max)` to
each vertex's own `β(v)`, at that vertex's own largest coordinate magnitude
rather than the body's: the rounding a rigid motion commits on one vertex depends on that vertex's inputs
alone (`internal/proofbound/bounds.go`,
`RigidRoundAllow`). A vertex whose exact image under the stored basis and
translation is itself a float (`proof.XptDenotes`) commits no rounding and adds
nothing, so a whole-number translation of whole-number coordinates leaves every
bound where it was. `MeshBound` is recomputed as the maximum `δ(t)` after the
addition. The exact-source record for a translation-only placement is kept or dropped under §4.1's zero condition.
Ordinary placement keeps a point loft's `PointSection` source record only for
an exactly identity delta; a later float composition to identity cannot
restore it. An exact rotation copy drops the record regardless of angle.

Placement carries every analytic group tag (`Plane`, `Cylinder`, `Cone`,
`Sphere`, `Torus`) to its float image under the delta
(`surfaceplace.TransformSurface`) unless the delta is exactly the identity.
The moved tag is a bounded stand-in (`agent-docs/api-design.md` §6.1). A retained
group adds the departure of `agent-docs/faceted-source-retention-design.md` §3.4 to
its `normalBound`; any other group's `normalBound` is 2, the largest distance
of two unit normals. A denoted
group proves `NormalAt` against the denoted patch's exact image and does not
read `normalBound`. A reflection flips only a `Plane` group's orientation. A
`NURBSSurface` tag is a marker with no geometry and does not move.

An exact-rotation placement (`WithCopyRotation`) places the held mesh by the
same float transform, but denotes an exactly orthogonal rational rotation. It
adds `|Q - D|·|v|_max + |t_Q - t_D|` to each vertex's rounding allowance in
place of the stretch factor and determinant charge, where `Q` is the exact
rotation and `D` the float one. It drops every analytic surface tag and
`denoted` patch, because those describe the float placement.

### 4.4 Faceted restatement

`facetedrecord.Restate` publishes the payload's `VertexBound` on the mesh, `faceBound[f] = max δ(t)` over the facets `FaceOf` maps to
`f`, and `bound = MeshBound`. Tessellation §7's refusal of a public `Tessellate` call whose tolerance is below
the held `MeshBound` holds: a caller who asks for a finer mesh than the body holds gets `ErrUnsupported`. The
boolean's own request is separate (§5).

### 4.5 The prepared operand

`meshbool.BoolMesh` carries the operand mesh's per-vertex bounds (or §2.1's derived
reading when the mesh publishes none) and exposes `δ(t)` per facet, which is
what the contact classifier records on each `Xseg` (§3.2) and what §5's gate
reads.

## 5. The chain-depth comparison

The gate is local. A global comparison (every operand asked for a mesh at the pair's chord tolerance `tol`,
a faceted operand with `MeshBound > tol` refused in `facetedrecord.Restate` before any contact is examined)
compares the wrong number once bounds are per vertex: one coarse rim far from the new contact would refuse a
pair whose every new rim is composed from fine geometry.

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
`max(tol, heldFloor)`, where `heldFloor` is the payload's own `MeshBound`
(`facetedPayload`) or `delta` (`mitredSweepPayload`, `coilPayload`,
`twistedSweepPayload`), read by a
`booloperand.HeldFloor` reader beside `booloperand.sectionDisplacement`; a restatement
returns the same vertices at any tolerance at or above its floor, so the mesh
is the one the boolean needs and the request never refuses.

**A held primitive raises the pair's tolerance.** An operand built as one
fixed held mesh — a coil, mitred sweep, or twisted sweep with proven `delta`,
or one certified `LoftFaces` or circular-to-fitted `LoftSections` result with
proven `MeshBound` — raises the
pair's chord tolerance itself to that bound, as a displaced prism section
already raises it past twice its displacement (`meshbool.PairChordFrom`, through
`booloperand.heldPrimitiveFloor` beside `booloperand.sectionDisplacement`). Every facet such an
operand touches the partner with then holds `δ(t) ≤ heldBound ≤ tol`, so the gate
below admits it. A boolean result's `MeshBound` does not raise the pair
tolerance: it is the figure a chain grows, and the gate exists to refuse it.
Rigid placement of either loft keeps the primitive classification while
charging its new bound; scaling and a Boolean result clear it.
The analytic partner is meshed at the smaller of the pair tolerance and the
larger of its diameter-derived chord tolerance and its own floor
(`meshbool.OperandChord`). A held primitive's raised pair tolerance therefore
does not coarsen that partner. The result's rim, area and volume bounds compose
the bounds each operand publishes. After the contact classification has run
and before any facet is cut, the gate walks every operand facet the
classification reports as meeting the other operand (`ContactPoint`,
`ContactSegment`, or within the pre-pass slack) and refuses with
`ErrUnsupported`, through `meshbool.BooleanExpectedStaging`, when any such
facet of a RESTATING operand has `δ(t) > tol`, where a facet of a retained `Plane` states
the slab half-width of `agent-docs/faceted-source-retention-design.md` §11 in place of `δ(t)` — which a held primitive's
facets never do (`meshbool.RefuseCoarseHeldContact`,
called from `MeshBoolean` on the classified contacts and from the hidden-tangency
pre-pass (`meshbool.RefuseUndecidableProximity`, run by `internal/booleaneval/eval.go`) on the facets within its slack;
`meshbool.StagingError` restates it in the boolean's own terms). The message names the operand (`Cut`'s
target or tool, else first or second), the bound of the touched facet, the
pair's chord tolerance, and says that a boolean takes no tolerance. A chorded
analytic operand is not gated: its `sourceBound` may exceed `tol` by tessellation §1's Tolerance row.

**Chain tolerance.** A document may state the coarsest facet bound a contact
may touch (`decad.WithChainTolerance`). The gate then compares a touched facet
against the larger of the pair's chord tolerance and that figure. The pair's
chord tolerance, and so the partner's mesh, is unchanged; only the refusal
moves. Every rim the contact makes still composes from the proven bounds of
the facets it touches, so a larger tolerance admits a larger bound on the
result and no unproven one.

**Reject-only.** The gate compares two proven numbers, a facet's published
bound and the pair's tolerance, and refuses when the first exceeds the second.
It admits nothing: a facet it passes enters the composition with its own
bound exactly as §3 states, and no measurement of the handed geometry decides
the answer. It can refuse a pair whose rims would have been fine, as a global comparison can; that is the accepted price.

**What changes for a caller.** A chain of booleans over planar-faced bodies
is refused only where a new contact lands on earlier rim geometry whose bound has outgrown the pair tolerance;
A result can carry a `Bound` above the next pair's tolerance in
untouched regions and still serve as an operand. A `Union` that the gate refuses because earlier rims spread along
long facets of a curved wall is rebuilt from its recorded `Union` history with §3.7's split on
(`agent-docs/prism-boolean-design.md` Part R, "Rim-split rebuild"), so each step's contact composes from the
face's own bound where the split proves it. Core §8's "The chain depth"
paragraph, evaluator §9's operand and rim bullets and tessellation §7/§11
state this rule.

### Cylindrical sides at an aligned pad

An unshifted Cut by a direct Loft can retry a coarse contact on a stock whose
aligned pad rim still identifies the native flange. The proof registers all facets
of one convex circular side over the heights that can reach the complete tool.
Projection in angle and height must cover the native strip once, with outward
orientation, matched endpoints and a bounded radial distance on each facet.

The complete current shell must exclude any foreign trim boundary from the open
strip. A held point whose original error ball lies strictly inside its angular and
axial limits supplies a surviving point on the same true face. The registered cap
height includes the exact displacement from its rounded held level. A radial
closing chord to the following walk contributes its full length. Other native
planes may touch an endpoint. If a plane crosses near an endpoint, the proof
can use a smaller angular interval. Each facet receiving the registered bound must
keep both its original error ball and its registered image inside the certified
angular range. At the transition, the larger of its original slab bound and
registered bound covers both assignments. The full facet bound must cover the
registered distance in either case. Exterior angular and axial intervals exclude
a continuation of the same face beyond its native end or cap.

Only the coarse-contact gate reads these local distances. Cutting and publishing
the intersection curves use the operand's original bounds.

## 6. Measured growth

The fixture used by §8's tests is a tree of mitred sweeps: a 16-gon trunk of circumradius 5 mm along a two-span
path `(0,0,0)→(0,0,30)→(3,0,60)` scaled `1, 0.9`, and fifty 16-gon branches tapering 1.5 → 1 mm along a three-span
path, each tilted 40° from the trunk axis, its root 2.6 mm from the axis, 1.1 mm higher than the previous root and
turned by the golden angle. Each union is `Union(tree, branch)`.

Under §3, a trunk facet that a new branch cuts carries its own `δ(t)`, so a rim `β` stays near the operand's own
rounding gap. A global bound grows by `1/sin θ` plus the weld per union (§1). The comparison is the pair
tolerance against the per-vertex maximum.

Interpolating the corner bounds at the cut point instead of taking their maximum (an affine bound over the facet,
with the rim denominator reduced by the two facets' bound gradients) would tighten rim bounds; it is not scheduled.

**Curved operands, estimated.** A chorded cone wall's facets carry
`δ(t) ≈ tol` (the sagitta), so a cone-to-cone union's rims carry
`≈ 2·tol/sin θ`, about `8·tol` where `sin θ ≈ 1/4` (the hand-off's crossings). Under §5's rule
the next union is refused only when its contacts touch those rims; a branch
whose root is buried in the previous segment's END region touches facets that
still carry their own `≈ tol`, so an end-to-end chain of cones continues at
every length, with each joint's rims carrying `≈ 8·tol` and nothing compounding
across joints. A chain whose every union cuts the previous rim compounds
exactly as a global bound would.

## 7. Curved operands

| Option | Gains | Costs |
|---|---|---|
| **A. Per-vertex bounds, chain-depth gate stays global** (PRs 1–3 alone) | planar chains run to fifty unions and beyond with per-vertex bounds; no contract change; `Bound`s shrink for every faceted body | a curved operand's result still carries `≈ 8·tol` as its global bound and is refused at the next union, so mtilt's request 1 stays refused |
| **B. Per-vertex bounds and the local gate of §5** (PRs 1–4) | curved chains continue wherever a new contact avoids earlier rims, with the rim ceiling `2·tol/sin θ` certified per union; planar chains are additionally protected against one coarse rim far from the contact | core §8's chain-depth paragraph, evaluator §9 and tessellation §7/§11 are rewritten; a result can publish a `Bound` above the next pair's tolerance and still be an operand, which callers judging with `WithTolerance` at `Verify` already handle |
| **C. Drop the chain-depth gate** | simplest code; nothing is ever refused before contact | the only ceiling left is the pair-diameter refusal of `RimBound`, so a chain can publish a bound of millimetres on a part of centimetres without any refusal naming it; core §8's promise that a chain is limited by a stated comparison is gone |

**Chosen: B.** The gate certifies a stated inequality, and B is the only option under which the hand-off's
curved chain continues.

## 8. Increments

All of PRs 1–4 are implemented; PR 5 is not. Files are named by role. "The mesh-boolean pipeline" is `internal/meshbool` (the contact gather, the facet
cutter and the stitch) together with `internal/booleaneval/eval_mesh.go`, which prepares each operand for it.

| PR | Lands | Files | Tests and the red leg each must show |
|---|---|---|---|
| **1** | the mesh node's `VertexBound` (`internal/topo/mesh.go`) with its setter and reader, `topobuild.SetVertexBounds` and `topobuild.VertexBounds` (`internal/topobuild/mesh.go`); the mitred sweep build keeps each vertex's own rounding gap and publishes it, with `faceBound[f]` the corner maximum over `f`'s triangles and `delta`/`bound` the overall maximum (`internal/sweepmitre/build.go`); §2.1's derived reading for a mesh with no record, as one reader the pipeline calls | mesh record, mitred build, mitred restatement | A mitred sweep with its path start at the origin and an axis-aligned first span publishes `β = 0` at every start-cap vertex and `β ≤ delta` everywhere, with equality at one vertex; red leg: publish `delta` at every vertex and the zero assertion fails. The derived reading on a revolve mesh equals the largest incident `faceBound` at a cap-wall corner; red leg: read the vertex's own face alone and the corner assertion fails |
| **2** | the composition of §3: facet bounds on the prepared operand, `Sin2`/`δ` recorded on each contact segment, corner provenance on kept facets, per-vertex weld in the stitch, `facetedPayload.VertexBound`, `MeshBound` as the facet maximum; measurements still read `MeshBound` | mesh-boolean pipeline, `boolean.go`'s composition, the payload build | Union of two crossing mitred branches (the existing `apitest` fixture): every surviving operand vertex keeps its operand `β` to the bit; every rim vertex's `β` equals `upRound((δ(t_A)+δ(t_B))/SinLowerBound(Sin2)) + weld` recomputed from the fixture's own facet pair; `MeshBound` is at most today's. Red legs: divide by the pair-wide `sinMin` and the rim equality fails at a rim whose own crossing is not the shallowest; drop the per-vertex weld and tessellation §14's "rational intersection vertices round inexactly" fixture reports a rim `β` below the measured rounding gap |
| **3** | §4's readings: `Vertex.Bound`, edge bounds, `Faceted.Bound`, area, box; `Transform` per vertex; `facetedrecord.Restate` publishes the record and per-face `faceBound`; the pre-pass reads per-face slack | `internal/booleaneval/eval_mesh.go`, `internal/facetedrecord/restate.go` | A three-union chain: an untouched trunk vertex reports the trunk's own `delta` as its `Vertex.Bound`, an untouched trunk face reports it as `Faceted.Bound`, and the box bound equals `Radius3D(e)` recomputed from the published vertices; red leg: publish `MeshBound` to every vertex and the untouched-vertex equality fails. A fixture whose largest-`β` vertex sits inside the held box but within `β` of one face of it: the box bound covers `m − (v.x − B(v))`; red leg: drop that term and the fixture's exact source box (the exact rationals) escapes the published bound. `Transform` of a boolean result: each vertex's bound grows by `RigidRoundAllow` at its own magnitude; red leg: charge the body-wide magnitude and the near-origin vertex's bound is strictly looser than the assertion allows |
| **4** | §5: `booloperand.HeldFloor`, the per-operand request tolerance, the post-classification gate and its message; companion edits to core §8 "The chain depth", evaluator §9's rim bullet, tessellation §7 (the boolean's request) and §11 steps 7–8 and the result-payload paragraph; `meshbool.StagingError` rewritten | `boolean.go`, the mesh-boolean pipeline's contact gather, the three design docs | Twenty unions of §6's fixture succeed with every union's rim `β` at most `2·tol/sin θ` of its own pair. A fixture whose operand carries one rim with `β > tol` away from the new contact succeeds, and the same operand with the new contact placed ON that rim refuses with the new message; red leg: compare the operand's `MeshBound` and the first fixture refuses. A cone-to-cone-to-cone chain (the hand-off's request 1 shape) builds three segments; red leg: the global comparison refuses the third |
| **5** (not implemented) | the interpolated rim rule of §6 | mesh-boolean pipeline | only if a measured chain needs it |

## 9. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Rim rule | corner maximum (§3.2) | interpolated corner bounds, with a gradient-corrected denominator to prove |
| Gate scope | restating operands only; chorded analytic operands keep tessellation §1's Tolerance row | gate every operand's contacted facets, which would newly refuse an analytic operand whose `deltaStore` exceeds `tol` |
| Whether the gate exists | §7's option B | option C |
| Weld and placement rounding | per vertex (§3.4, §4.3) | the global maximum at every vertex, the global composition of §1 |
| Per-facet departure term | none; a curved per-vertex record folds it into `β` (§2.1) | a `facetBound []float64` beside `vertexBound`, needed only when a curved payload wants per-vertex bounds below its sagitta |
| Box bound | the six-extreme rule of §4.2 | `Radius3D(MeshBound)`, which stays sound and is what §4.1's maximum makes available |
