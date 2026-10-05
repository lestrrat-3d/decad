# Contact Geometry Design

Navigation: pair verdict → §1–§2; manifold → §3; source-box proofs → §4;
contact cases → §5; refusal → §6; delivery tests → §7.

This document owns the pair pose query and the geometry passed to rigid-body
response. `docs/collision-dynamics-design.md` maps the overall system, and
`docs/contact-sweep-design.md` owns the two-body sweep. `docs/clearance-design.md`
owns boundary-distance and touching proofs; `docs/interference-design.md`
owns positive-volume overlap;
`docs/tessellation-design.md` owns source-face displacement and mesh audits.
This document adds witnesses and normal certificates without weakening any of
those admission gates.

Current code certifies relations and face manifolds for source boxes at
signed-permutation poses. An oriented source-box path also certifies relations
under arbitrary proper read poses by projecting the exact transformed corners
on all face and edge-cross axes. Two boxes whose three source edge directions
match can publish a four-point manifold at one isolated oblique face touch.
One horizontal rotated box face strictly inside an axis-aligned face publishes
a four-point manifold. At isolated touch, a partly overhanging horizontal
face publishes every vertex of its exact clipped polygon. Two opposed
axis-normal faces can publish
one bounded interior witness when one projected face center lies strictly
inside the other face. Other rotated contacts publish no
manifold. A full source semicircle sphere against a source box also receives
an exact rational relation proof and a point manifold at one isolated face
support. At identity query poses, the analytic clearance kernel can certify
a relation for other admitted solids without a contact manifold. Other curved
and faceted witness and normal proofs remain design contracts.

A full circular source prism at a signed-axis pose can certify separation
from a source box across one axial face. Its complete projected disk must lie
strictly inside that box face. The source circle, prism extent, and placement
produce an exact outer box, and the axial support gap is the true pair gap.
An overlapping outer box, side approach, or projected disk reaching a face
edge returns `Undecided`; this path publishes no cylinder manifold.

Two full source semicircle spheres receive the exact relation proof from their
recorded centers and radii. A nonzero center offset with crossing sphere
surfaces publishes one bounded point manifold along the center line. Coincident
centers keep the relation and withhold the manifold.

## 1. Claims and entry point

```go
func (d *Document) ContactPair(ctx context.Context, a, b *Body,
    poseA, poseB r3.Transform, req ContactRequest) (*ContactReport, error)

type ContactRequest struct {
    PointResolution  units.Value // positive Length
    NormalResolution units.Value // positive Angle
}

type ContactRelation int // Separated, Touching, Overlapping, Undecided

type ContactReport struct {
    A, B       *Body
    PoseA      r3.Transform
    PoseB      r3.Transform
    Request    ContactRequest
    Relation   ContactRelation
    Gap        *Measurement // Separated or Touching only
    Overlap    *Measurement // bounded Volume if proven; optional on Overlapping
    Manifold   *ContactManifold
    Reason     ContactReason // typed, only for Undecided or absent manifold
}
```

The actual Go names may change, but these fields and their absence rules are
required. The report always names the original bodies and their caller-supplied
poses. Poses act **after** each body's recorded placement, as `Body.Placed`
does. Two identical body pointers, a nil or foreign body, a retired body, a
sheet, an unsound solid, an invalid transform, or an invalid request is an
input error with no report. Use the existing typed errors for the relevant
cause. A valid solid payload that cannot be classified is `Undecided`, not an
input error. Validate bodies, poses, kinds, finiteness, and positive
resolutions before reading `ctx`; then propagate `ctx.Err()` unchanged.

`Separated` proves disjoint interiors and a strictly positive minimum
boundary gap. `Touching` proves disjoint interiors with the complete
zero-distance set certified by clearance's touching gate or §4's source-box
proof; `Gap` is `Exact` zero. `Overlapping` proves shared interior. An
optional `Overlap` volume is
present only after interference's independent volume gate passes; a boundary
crossing or containment witness alone may prove the relation. `Undecided`
asserts none of those three relations and has no gap or manifold. A known
relation may still have no usable manifold. `Reason` then names the missing
certificate, for example `ContactNoNormalProof`, `ContactAmbiguousFeature`,
`ContactPointTooCoarse`, or `ContactPayloadUnsupported`; callers branch on
that code, not its message. Do not reuse `Diagnostic` or `Verify.Status` as
the pair verdict.

`ContactPair` always asks the analytic distance kernel for a gap on a
separated pair; box separation alone does not supply a minimum. If the
analytic kernel proves separation but cannot measure the gap, return
`Undecided` with a gap-specific reason. Keep the private four-way
`pairVerdict` separate from the public `ContactRelation`; map it only after
the transient pose's own displacement has been charged. A later overlap
volume proof may upgrade an undecided analytic relation, but may not weaken a
stronger earlier certificate.

## 2. Transient pose and proof transfer

Build both transient bodies through the same private payload placement path
that `VerifyMotion` uses. Reserve a transient producer identity; never call
public `Placed`, `PlacedCopy`, `Duplicate`, `Intersect`, or `commit`. The true
query pose is the exact composition of the read `r3.Transform` entries with
the body's read placement entries. Bound the difference between that exact
composition and the float transient payload with the existing
`poseDeviation`/placement machinery. Charge each body's displacement once
into every separation, witness-point, and contact-set bound. A held positive
gap becomes a proven positive gap only if its lower endpoint remains above
zero after both charges.

Transfer a held overlap to the query pose by subtracting the two bodies'
conservative swept-volume allowances from its lower volume bound. The
allowances use each body's area upper bound and its own pose deviation, as in
`docs/motion-check-design.md` §5.1. A positive remainder proves overlap;
otherwise return `Undecided` unless another exact set certificate applies.
Transfer a held exact touch only if the complete contact and material-side
certificate also applies to the exact query poses. A nonzero placement or
composition displacement does not turn an `Exact` zero into an approximate
touch: the relation becomes `Undecided` unless a new exact posed certificate
proves it. In particular, the first box-impact slice requires an exact
axis-aligned posed-box contact path rather than relying on the existing
zero-displacement coplanar certificate alone.

For that path, admit only a rectangular prism whose recorded closed section
and extent prove its **occupied set** is exactly one box, with zero source
boundary displacement. A body's `Bounds()` rectangle or a box around its
facets does not pass this gate. Admit its existing placement and both query
poses only when every read rotation is an exact signed permutation of the
coordinate axes. Interpret their finite float entries as exact dyadic
rationals. Apply the composed transforms to the source box corners with
exact rational products and sums, without taking the rounded transient
payload's AABB as the answer. This constructs the exact occupied axis-aligned
box at the caller's query pose. The rational box can certify a face touch
despite nonzero `bodyGeom.delta` on a separately built transient body. If a
pose comes from a `PoseSegment`, also enclose the difference between its read
float transform and that segment's ideal screw pose using the
`VerifyMotion` pose-deviation machinery. A relation is transferred to the
ideal path only if the charged enclosure proves it; an exact rational touch
of the float pose by itself does not certify an ideal-path touch. Endpoint
equality has a separate exact check against the endpoint the caller stated.

## 3. Manifold contract

```go
type ContactManifold struct {
    Points []ContactPoint // nonempty, stable feature order
}

type ContactPoint struct {
    OnA, OnB       VecMeasurement // world positions; Length ball bounds
    Normal         VecMeasurement // unit vector A toward B; Dimensionless bound
    NormalAngle    units.Value    // proven angular radius, Angle
    Separation     Measurement    // signed Length, B minus A along Normal
    FaceA, FaceB   *Face          // original live faces when certified
    FeatureA, FeatureB ContactFeature // stable face/edge/vertex source identity
}
```

The public representation must keep all scalar quantities typed. `OnA` and
`OnB` each enclose a point on the corresponding true trimmed boundary. The
normal's ball bound and `NormalAngle` both enclose every true normal assigned
to that entry; the angular radius is at most the requested
`NormalResolution`. The point bounds are each at most `PointResolution`.
`Separation` encloses the signed normal projection of the two true witness
points, including their point and normal errors; positive means apart,
negative means penetration. For a certified touch its interval contains
zero. A report may omit `FaceA` or `FaceB` only when the contact feature has no
single certified owning face; it must still carry stable source identity.
Neither a mesh triangle index nor a transient face pointer crosses the API.

`Manifold != nil` means every published entry passes these bounds and the
set is sufficient for the response law over the whole certified contact set.
For a planar patch, derive the exact trimmed intersection region and publish
all extremal vertices, including holes and disconnected components, in
deterministic loop order. A line contact publishes its clipped endpoints.
A point contact publishes one point. A finite solver reduction for a curved
contact must bound the force and torque effect of the omitted continuum; until
that reduction exists, relation proof may be published with `Manifold == nil`.
Duplicate geometric points from neighboring faces are merged only under a
certified equality; near coordinates never define identity. The manifold is
read-only, deterministic, and never implies a particular impulse.

One outward material normal per side is needed to orient the contact normal
from A toward B. A face interior with a smooth analytic normal can supply
one. At an edge or vertex, enumerate every adjacent face normal and its
material side; the candidate set is a cone, not an average. Publish a normal
only if a unique direction or a bounded cone meets `NormalResolution` and
the solver can consume all its admissible directions. Otherwise omit the
manifold with `ContactAmbiguousFeature`. A concave rim, multiple simultaneous
features, and zero-area coplanar trims follow this rule. Normal orientation
comes from body material sides, not triangle winding alone.

An overlapping pair may have a usable manifold only when the kernel proves
the boundary contact features and a bounded penetration direction/depth
that the response correction can use. A positive `Interference.Volume`, a
nesting witness, or intersecting held facets cannot manufacture those data.
Full containment without a separating boundary contact has no manifold.
The solver must decline that event rather than choosing a closest face.

## 4. Geometry paths

| Path | Relation proof | Witness and normal proof |
|---|---|---|
| Source-certified boxes at admitted poses | Exact rational interval relation | Rational face patches and clipped lines/points; exact material-side axis normals. |
| Analytic prism/revolve carriers | Existing clearance tiers, casts, and certified contact families | Retain admitted stationary feet and complete contact-set geometry; evaluate analytic normals with trim and orientation bounds. |
| Verified faceted boundary | Existing bounded pair proofs | Map candidates to original faces; charge source displacement and pose error; require source normal enclosures. |
| Unsupported or coarse payload | Existing box separation may still prove clear | Return relation with no manifold, or `Undecided` if the relation itself lacks proof. |

The analytic path extends private candidate results; it does not infer a
manifold from a public `Clearance` row. A candidate on an unbounded carrier is
admitted only when its feet are proved inside the trimmed source faces.
Enumerate face-interior, face-edge, edge-edge, face-vertex, edge-vertex, and
vertex-vertex tiers, including cone apex and spindle-axis singularities, as
`docs/clearance-design.md` §3 requires. Proof of a global minimum still needs
every rival's lower bound. Stationary feet supply upper witnesses, never a
global minimum or a contact by themselves.

The bounded-mesh path requires `BoundaryVerified()` and finite private
`sourceBound(face)` for each candidate. `VolumeVerified()` is additionally
required for an occupied-volume overlap proof, but alone proves neither a
point nor a normal. A source-face displacement is a two-sided position bound,
not a normal bound: obtain a source analytic derivative enclosure, a
certified local normal cone, or refuse. A triangle normal with no such source
proof cannot enter a manifold. The source-face mapping, oriented material
side, and trim audit must pass; an inconsistent mapping is an evaluator error
with no report. Refine tessellation when a chord bound covers possible
contact and point/normal bounds may yet meet the request. At the deterministic
work or resolution floor, return an explicit undecided relation or absent
manifold according to which proof is missing.

For two admitted rational boxes, compare all three axis intervals exactly.
A positive gap on any axis excludes contact, but a global minimum distance
still needs the full box-distance calculation. Strictly positive overlap on
all axes proves `Overlapping`; equality on at least one axis with no positive
separation proves `Touching` because the complete boxes have disjoint
interiors and their boundaries meet. Two or three equal axes are valid
edge/vertex **relation** certificates, even when the normal cone makes the
manifold unavailable. A single equality axis with strict projected overlap
gives an opposed-face patch. Clip the two rectangles with exact rational
arithmetic and use that patch for the manifold. Convert each rational point
to `r3.Vec` with an outward Length ball bound; its axis normal is exact.

For shallow box penetration, calculate the six directed translations that
would move B clear of A. A manifold is available only when exactly one
strictly smallest positive translation is certified, the selected faces
cross rather than one box being fully contained in the other, and their
projected overlap is strictly two-dimensional. Use the two original true
face points at each clipped patch vertex and the chosen A-to-B axis normal;
their signed separation is the negative translation depth, bounded by the
point conversion. A tied minimum, strict containment, edge-only projection,
or requested point resolution below the conversion bound leaves the
`Overlapping` relation intact and the manifold absent. This is the C1 box
penetration path consumed by a sweep's later proven-overlap endpoint.

For a source box at a general read pose, transform all eight recorded corners
with exact dyadic operations. The resulting parallelotope is tested on both
sets of face-normal axes and all nonzero edge-cross axes. Positive separation
on one axis proves `Separated`; equality on at least one axis with no positive
separation proves `Touching`; strict projected overlap on every axis proves
`Overlapping`. A separated report bounds the minimum distance from below by
the largest normalized axis gap and from above by a certified vertex-to-face
or vertex-to-vertex witness. If those bounds cannot publish a positive gap,
the relation is `Undecided`. For one opposed face equality, two boxes with
parallel corresponding edges use the first box's rational dual basis to clip
their complete face rectangles. Each clipped corner records its original
face identities, a rational point with outward conversion bound, and a
face normal with outward vector and angular bounds. A second equality axis,
nonparallel source edges, or a request tighter than those bounds withholds
the manifold. Other rotated face, edge, vertex, and shallow-overlap manifolds
still need their own complete trimmed contact-set and source-feature proofs.

The horizontal rotated-face path requires one source box at a signed-axis
pose and another whose transformed source edges keep one face horizontal.
At isolated touch, clip the rotated face's exact dyadic corners against all
four signed-axis face half-planes with exact rational intersections. Publish
every distinct polygon vertex when its signed area is nonzero and every
rounded witness meets `PointResolution`. Preserve both original face
identities and the exact vertical material normal. A zero-area projected
intersection keeps the proven relation without a manifold.

For penetration, the existing contained-face path requires all four rotated
corners strictly inside the other projected face. Its depth must be strictly
smaller than every side margin, and the bodies must cross at opposed Z
supports rather than contain one another. It publishes the four rotated
corners. A partly overhanging penetration or tilted face keeps its proven
relation without a clipped manifold.

### 4.1 Source-box contact set for sweeps

Keep a private `sourceBoxContactProof` with the exact occupied interval of
each box on **all three** world axes at the certified pose. Retain the source
box corners, exact read start placements, original face identities, every
axis where opposed supports are equal, and each remaining axis's exact
projected overlap interval. It is a proof about the complete occupied sets,
not just the manifold's sampled patch. At one face touch, its contact set is
the Cartesian product of the two projected overlap intervals on the common
support plane. If one projected interval shrinks to a point, retain the
edge-contact set; if both do, retain the vertex-contact set. The public
`ContactPair` report states only the relation at this pose. Its manifold
may be absent at an edge or vertex even when this private set is complete.

The sweep may consume this private proof only when it certifies that its
ideal time-zero poses are the same exact read placements. For a path proved
to be pure affine translation, the sweep can advance every interval endpoint
by the certified translation at time `u`. The contact kernel supplies exact
support-plane and projected-interval facts; the sweep owns all comparisons
over time, event times, and status. In particular, the sweep can use an
outward lower bound `w > 0` on the relative translation rate along an opposed
support axis. The whole-body gap along that axis is then at least `w*u` for
every `u > 0`. This is a one-sided departure proof even though the gap at
`u = 0` is exactly zero. Keep the product symbolic until a positive lower
gap at a chosen `h` is representable; a rounded zero proves nothing.

When the normal support gap stays zero, the exact projected intervals let
the sweep prove persistent face contact during tangential sliding. At an
exact projected-overlap endpoint equality, the complete contact set becomes
an edge or vertex. The sweep may resume clear-interval proof only after a
separate axis has a strictly positive gap. A positive normal velocity at a
single manifold point, sampled separated poses, a rounded near-equality,
or a rotating path cannot substitute for these whole-box proofs. If no
strict interval claim follows, the sweep returns its undecided outcome.

### 4.2 Source semicircle sphere against a source box

Admit a full-revolution solid with one untrimmed spherical face only when its
recorded section is one complete semicircular arc and its on-axis diameter.
The arc's center and endpoints must lie exactly on an axis with exact cardinal
direction; the two axial radii must match as rationals. Require zero section
displacement and signed-permutation frame, placement, and query pose. A sphere
surface tag without this occupied-set record cannot enter this path.

Map the recorded center and radius to exact rational world coordinates. Clamp
the center to the exact box intervals and compare squared distance to squared
radius. A strict excess proves separation, equality proves touch, and a
strict deficit proves overlap. A separated report encloses the true gap after
outward square-root and float conversion. A one-axis exterior center with its
whole projected radius strictly inside the other box intervals can publish
the opposed box face and spherical point. Shallow penetration also requires
the sphere not to reach the opposite box face. An edge, corner, internal
center, or wider patch keeps the proven relation but withholds the manifold.
Reversing body order reverses the exact axis normal and witness fields.

### 4.3 Two source semicircle spheres

Admit each sphere through §4.2's source record and pose gates. Compare the
exact squared distance between their centers with the square of the radius
sum. A strict excess proves separation; equality proves touch; a deficit
proves overlap. Enclose a separated gap by outward square-root conversion.
Publish one point manifold when the center difference is nonzero and its
distance exceeds the absolute radius difference. The latter gate excludes a
sphere wholly inside the other. Enclose the center-line unit normal by a
bounded square root of the exact squared distance. Both witnesses lie on the
original spherical faces along that direction, with point bounds including
normal conversion. Their signed separation encloses center distance minus
radius sum and both witness errors. A request tighter than the normal or point
bounds keeps the relation but withholds the manifold. Reversing body order
reverses the normal and swaps witnesses.

## 5. Contact cases and ordering

| Case | Required result |
|---|---|
| Positive gap, including a body inside a cavity | `Separated` only after nesting or shell tests exclude shared interior; publish a bounded global minimum gap. |
| Strict full containment of a solid in material | `Overlapping`; no contact manifold from containment alone. |
| Coplanar positive-area patch with opposed material normals | `Touching` after all other crossings are excluded; publish the full trimmed patch reduction if its bounds meet the request. |
| Coplanar patch with matching material sides | `Overlapping` only when shared interior is proven; no touch manifold from the patch alone. |
| Edge-only or vertex-only meeting | `Touching` on the new exact source-box path; `Undecided` under today's other clearance paths. A manifold requires a resolved normal cone. |
| Smooth tangency or ruling contact | `Touching` only for a complete certified family in clearance §6; the manifold may still be unavailable if point or normal bounds fail. |
| Transversal boundary crossing | `Overlapping` if the proof transfers to both exact query poses; the unique shallow source-box penetration path above may yield a manifold. Other crossings require a separate proof. |
| Near miss inside displacement bounds | `Undecided`; never promote a held-mesh gap or sample to `Separated`. |

Input order is preserved: reversing A and B swaps point and face fields,
reverses normals, and leaves the geometric relation and gap interval
unchanged within bounded arithmetic. No contact point order may depend on a
map iteration, BVH traversal, worker completion, or tessellation cache hit.
Sort source features by original body topology order, then exact parameter
order along each feature, then exact rational coordinate order for tied
generated points. Stable tie breaks use A before B. Repeated calls with the
same inputs publish identical reports, including reasons and bounds.

## 6. Refusal, cancellation, and non-mutation

Expected geometric limits return `Undecided` or a known relation with an
absent manifold and a typed `ContactReason`. These include a staged payload,
an uncertain trim, bounds spanning zero, a missing normal enclosure, an
ambiguous edge cone, and a requested resolution the current proof cannot
reach. An internal invariant failure, such as a missing source face or a
broken exact predicate, returns an error and no report. A point/normal bound
must be finite; an unbounded result is absence, never `+Inf` in a published
manifold. Context cancellation at any phase returns `ctx.Err()` and no
report. Poll within candidate, tessellation, containment, and refinement
loops under the shared work-counter discipline of clearance §3 and
interference §7.2.

The query changes no document membership, liveness, topology, payload,
measurement, or producer identity. Per-body tessellation caches may accept
only a complete successful mesh under their existing contract; partial or
canceled work never enters a cache. The report holds copies of result slices
and values. It may point only to original bodies and their original source
features, never transient ones. `ContactPair` is safe for concurrent reads
under the existing `Document.Verify` concurrency rule.

## 7. Delivery and computed tests

| Stage | Delivered path | Computed obligation |
|---|---|---|
| C1 | Exact posed source boxes and read-only pair report | Check 3 mm gap, face touch, shallow overlap, containment refusal, tied depth, and document state. |
| C2 | Analytic prism/revolve contact families | Check point, ruling, patch, void-wall orientation, and near-tangent refusal against calculated geometry. |
| C3 | Verified faceted source witnesses and certified normal cones | A curved source lies within each point ball; its analytic normal lies within the angular cone; a render-only mesh and a displacement bound without normal proof yield no manifold. |
| C4 | Remaining shipped solid payloads and cost pass | Each payload computes certified contact or gives its typed missing-proof reason. |

C1 uses two 10 mm boxes 3 mm apart and reads a 3 mm gap. A face touch
reads zero and four patch corners. A unique shallow overlap reads negative
separation and the expected axis normal. Strict containment and tied
translation depths have no manifold. No C1 query changes the next producer
identity. C2 evaluates a plane/sphere point, a plane/cylinder ruling, and an
opposed coplanar patch, including void-wall orientation and near-tangent
refusal. C4 runs prism, cup, revolve, loft, faceted, cap blend, stitched
solid, and admitted sweep payloads.

C1 also checks the private complete-set certificate with two opposed boxes.
An outward relative translation proves positive gap for every positive
time. A tangential translation retains a face patch until its projected
overlap ends, reports an exact edge-contact set at the exit, and proves
positive gap only afterward. A point's positive normal velocity alone must
not pass either test.

Every stage also checks pair reversal, deterministic ordering, wrong kinds,
invalid/retired/foreign/sheet bodies, cancellation during a real candidate
walk, and non-mutation. At least one test passes a real `ContactPair`
manifold into a dynamics response and checks the resulting position and
velocity; a hand-written manifold does not cover that boundary. Tests at
the point and normal resolution thresholds assert both sides of the gate:
a finite bound just within the request publishes an entry, and a wider bound
omits it without changing a proven relation. A small positive held gap whose
placement allowance spans zero must never report `Separated`.
