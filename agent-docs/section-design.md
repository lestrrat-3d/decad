# Planar section design

`Body.Section` reads a solid at a requested plane and returns a new Sketch whose
profiles denote the material in that plane. It does not change the body or its
document. An exact returned Sketch may enter any admitted profile consumer. A
bounded section may enter only the straight solid `Extrude` path below.

The bounded classes using `ObliqueCirclePolygon` use exact station polygons
for their geometric proofs, then round station coordinates for Sketch. The
rounded Sketch polygon is a two-sided proxy: a vertex may lie outside the
exact circle or ellipse.
`sectionclip.ObliqueCirclePolygon` bounds each rounded vertex against its exact
station in local and world coordinates and proves that their matched convex
rings remain convex through interpolation. Matching convex combinations extend
the vertex bound to both occupied polygon regions. The chord and radial terms
then bound both directions between the exact station region and the source
section. Their sum is the two-sided `sectionDelta`. Only `Extrude` accepts the
revision-bound Sketch and carries `sectionDelta` into its prism; direct profile
consumers refuse it. The prism's volume, bounds, mesh and later admitted
operations charge or refuse that displacement without assuming containment.

## Public result

```go
func (b *Body) Section(ctx context.Context, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, error)
```

The returned Sketch uses `plane` as its construction frame. The profiles are
fresh, valid Sketch snapshots; changing the Sketch makes them stale under the
ordinary seam rules or the bounded-source certificate below. The result is a
slice because a section can hold several material regions: the full Revolve
classes and the mesh Boolean history class below, and the orthogonal class
(`agent-docs/orthogonal-section-design.md`), return one profile per region, and
every other admitted class returns one.
The caller may read a retired body. A nil body or context, or an invalid frame,
is `ErrDegenerate`; a sheet or unsound solid is `ErrNotSolid`.

## Parallel exact prism class

The parallel class admits a `prismPayload` with zero `sectionDelta`,
`z0Delta`, and `z1Delta`. It refuses a plane that touches either cap. A body
outside this prism class tries the authenticated sphere, cone and full Revolve
classes below or returns `ErrUnsupported` without changing the document.

The gate reads the stored source frame and rigid placement as exact dyadic
values. It proves that the placed images of source `U`, `V`, and `N` equal the
requested frame's `U`, `V`, and `N`, component by component. It proves that the
placed source origin differs from the requested origin by a scalar multiple of
the requested normal. The scalar gives the requested sweep level, which must
lie strictly between the recorded sweep levels. These are exact comparisons
on stored values, not a parallelism or distance tolerance. A placed body is
admitted when its transform passes the same identities.

For every admitted level the section is the prism's recorded profile, including
its holes. No 3D edge intersection is needed. Reconstruct its entities on a new
Sketch at the requested frame. Ask Sketch for its profiles, record the fresh
boundary through the seam's `TExact` checks, then require its segments to match
the prism record. A source segment that cannot be recreated or whose
trim Sketch does not certify returns `ErrUnsupported`. A reconstructed profile
that does not match the source never becomes a section result.

## Exact slab of a stacked prism

A stacked prism is a stack of slabs of exact regions, each with its own levels.
A blind-pocket `Cut`, a stacked `Union` boss, a closed shell with a cavity and a
mirrored or patterned stack all produce one. `Body.Section` reads any of them
through `stackedunion.OperandFromPayload` and `sectionclip.SectionBrepStack`,
with the same gates as the BRep stack below. A plane strictly inside the
pocketed slab of a blind pocket returns the outer outline with the pocket as a
hole, and a plane strictly inside the solid slab returns the full outline.
The gates refuse a nonzero section displacement, a nonzero displacement at
either level of the selected slab, a level that is not strictly inside a slab
(an interface such as the pocket floor, an end cap or a level outside the body),
a slab with more than one region, and a frame that does not preserve the
recorded coordinates.

## Exact slab of an authenticated Boolean BRep

An A1 planar `Union` may return a BRep with an authenticated stack of prism
slabs. `Body.Section` reads that stack through `stackedunion.OperandFromPayload`, rather
than inferring a section from the visible faces. It requires zero global
section displacement and zero displacement at both ends of the selected slab.
The slab must contain exactly one recorded region. A BRep without this stack,
or a stack whose profile carries a displacement, returns `ErrUnsupported`.

For each slab, use the parallel prism gate above with its recorded frame,
placement and two levels. The requested plane must lie strictly inside one
slab; an interface, cap, exterior level or unsupported frame is refused.
The selected region is reconstructed on a fresh Sketch and checked against its
record through the same seam proof as an exact prism. This is an exact planar
section, including any recorded holes, and can enter ordinary profile
consumers. It does not treat a tessellated BRep face as a source profile.

## Mesh Boolean result read through its operation history

A mesh `Union`, `Cut` or `Intersect` result holds a mesh and keeps no section
record. `Body.Section` reads its section from the two operands its public
operation record names, when that record proves the result denotes the
operation over them exactly. The query reads the retained operands, retired or
not, and consumes nothing. It replays no feature call.

### Admission of the result

The body must be a `facetedPayload` that misses both smooth-loft classes and
that `boolchain.Expandable` accepts (`agent-docs/boolean-chain-regroup-design.md`,
"Eligibility of an expandable body"): no point-cone trim and no operation
warning naming an accepted change, such as a gap shift or an embedding reach.
Its producer must be a public `Cut`, `Union` or `Intersect` with exactly two
consumed inputs; a `Cut` reads its target, then its tool. Such a body denotes
`A op B` over its two recorded inputs exactly (same document, "Assumptions").
This is the set-equality certificate that `agent-docs/operation-history-design.md`
requires before a source payload is reused. A gap-shifted result, a `Copy` or
`Transform` of a result, a many-body `Union` and a body without a Boolean
producer return `ErrUnsupported`.

### Admission of each operand

Each operand must have an exact section at the requested plane in one of these
existing classes: the parallel exact prism, a strictly interior slab of a
stacked prism or stack-backed BRep, the cardinal vertical plane of a
rectangular or whole-circle prism, or a full Revolve cut through or normal to
its axis. The first three are read through
`sectionclip.SectionBrepStackRecord`, because `stackedunion.OperandFromPayload`
states a plain prism as a one-slab stack. A full Revolve is read through
`sectionclip.RevolveAxialSection`, then `sectionclip.RevolveNormalSection`.
Each class keeps all its own gates. A sphere or cone that only its own
authenticated class admits, and a class-B BRep, which keeps no slab stack, have
no operand section here.

An operand section is a list of regions, each with its holes. A revolve
section may hold several; every other class holds one. Every segment must be a
whole `LineSeg`, a whole `ArcSeg` or a whole `CircleSeg` that is a loop on its
own. A narrowed range of any kind enters the scene at a computed walked end, so
it returns `ErrUnsupported`, as does any other segment kind. An operand that is
itself a mesh Boolean result is not read through its own history, and an
operand the plane misses has no section, so the result refuses.

One requested frame rarely suits both operands. A box swept along Z is read in
(X, Y), but a box swept along X must be read in a frame whose V axis is ±X. So
each operand tries eight candidate frames in a fixed order: the requested
(U, V), then (−U, −V), (V, −U) and (−V, U), then the reflections (U, −V),
(−U, V), (V, U) and (−V, −U). The first candidate is the requested frame
itself. Every other candidate keeps the requested origin and is rebuilt by
`r3.NewFrame`, which normalizes and may move the held axes of a tilted frame,
so its held U and V must equal the signed permutation of the requested held
axes exactly; otherwise it is skipped. All eight span the
requested plane. The first candidate a class admits gives the operand's
section. A point at `u'U' + v'V'` is the same world point at the signed
permutation of `(u', v')` in the requested frame, because U' and V' are the
requested axes negated or swapped. Negating and swapping binary64 values is
exact, so every point, arc centre and pinned arc end maps exactly and every
radius is unchanged. A reflection (`U'×V' = −N`) reverses every loop's walk,
so outer loops stay counter-clockwise and holes clockwise in the requested
frame. Under it a line swaps its ends, an arc swaps its pinned ends and keeps
its range, because the image of a counter-clockwise sweep from S to E is the
counter-clockwise sweep from E' to S', and a circle keeps its winding and range,
because the reflection and the reversed walk cancel. A whole circle's parameter
origin moves with the frame, which a closed whole edge does not observe.

### Proof that the section commutes with the Boolean

Write `X°` for the interior of a closed set X in R³, and `int_P`, `cl_P` for
interior and closure inside the plane P. The section of a solid X is its
material region `σ(X) = cl_P(int_P(X ∩ P))`: the slice with any
lower-dimensional pieces removed. The Booleans are regularized: `A ∪ B`,
`cl(A° ∩ B°)` and `cl(A° \ B)` (boolean-chain-regroup-design, "Assumptions").
The 2D operations on regions in P are the same regularized operations.

**Hypothesis H for an operand X at P.** (H1) `X° ∩ P = int_P(X ∩ P)`, and
(H2) `X ∩ P = cl_P(X° ∩ P)`. Under H, `σ(X) = cl_P(X° ∩ P) = X ∩ P` and
`int_P σ(X) = X° ∩ P`.

**Every admitted operand class satisfies H.** None of these arguments reads a
segment's kind, so they hold for sections bounded by arcs and circles, with
holes, and of several regions alike.

- Parallel prism or stack slab: the plane is normal to the sweep at a level
  strictly inside the slab's open level interval I. Consecutive slabs meet only
  at their end levels (`agent-docs/stacked-prism-design.md` I4). So near P the
  operand is `Σ × I` and its interior is `int Σ × I`, with Σ the recorded
  region, holes included. `X ∩ P = Σ`, `X° ∩ P = int Σ`, and `Σ = cl(int Σ)`
  because a recorded region is the closure of its interior.
- Rectangular prism, vertical plane: in its placed axes the operand is
  `[a, b] × [c, d] × [z0, z1]` and P is `u = k`, where `a < k < b` holds by
  the class's strict opposite signs at opposite corners. Then
  `X ∩ P = [c, d] × [z0, z1]` and `X° ∩ P = (c, d) × (z0, z1)`.
- Whole-circle prism, vertical plane at offset `d` with `d² < r²`: a secant
  line meets the open disk in the open chord, so `X ∩ P = [−h, h] × [z0, z1]`
  and `X° ∩ P = (−h, h) × (z0, z1)`.
- Full Revolve, plane holding the axis: X is unchanged by every turn about the
  axis. If a disk D in P about a point p lies in `X ∩ P`, the turns of D about
  the axis sweep a ball about p, both off the axis and on it, so p ∈ X°. That
  is H1. `X ∩ P = M ∪ M'` is a union of two closures of interiors, and the
  closure of the interior of a union contains the union of the closures of the
  interiors, so `cl_P int_P(X ∩ P) = X ∩ P`, which with H1 is H2.
- Full Revolve, plane normal to the axis at level h: the class admits h only
  when no segment end lies on it and every crossing of the level line with the
  meridian boundary is transversal and enumerated. So the level line meets
  ∂M exactly at the sorted radii, and its points strictly between two
  consecutive radii lie in int M or outside M. `X ∩ P` is therefore the closed
  disks and annuli of the material intervals, and a point of their interior
  turns from a point of int M (or an axis point with a half disk of M about
  it), so it lies in X°.

**Lemma.** If A and B satisfy H at P, then `σ(A op B) = σ(A) op σ(B)` for the
regularized 3D op and its regularized 2D counterpart.

- Union. `R ∩ P = (A ∩ P) ∪ (B ∩ P) = σA ∪ σB`, and `σ(R) ⊆ R ∩ P`. The sets
  `A° ∩ P` and `B° ∩ P` are open in P and lie in `R ∩ P`, so they lie in
  `int_P(R ∩ P)`. Their closures σA and σB therefore lie in σ(R). So
  `σ(R) = σA ∪ σB`.
- Intersect. `R = cl(A° ∩ B°) ⊆ A ∩ B`, so
  `int_P(R ∩ P) ⊆ int_P(A ∩ P) ∩ int_P(B ∩ P) = A° ∩ B° ∩ P` by H1.
  `A° ∩ B° ∩ P` is open in P and lies in `R ∩ P`, so equality holds:
  `int_P(R ∩ P) = int σA ∩ int σB`, and `σ(R) = cl_P(int σA ∩ int σB)`.
- Cut. The 2D result is `cl_P(int σA \ σB) = cl_P((A° \ B) ∩ P)`. That set is
  open in P (B is closed) and lies in `R ∩ P`, so `σ(R)` contains its closure.
  Conversely `R ⊆ A`, and `R ∩ B° = ∅` because B° is open and disjoint from
  `A° \ B`, hence from its closure. With H1 for both,
  `int_P(R ∩ P) ⊆ int σA \ int σB`. A point of `int σA` on the boundary of σB
  is a limit of points of `int σA \ σB`: `int σA` is open, and every
  neighbourhood of a boundary point leaves σB. So
  `cl_P(int σA \ int σB) = cl_P(int σA \ σB)`, and σ(R) lies in it.

The identity is a theorem over the two recorded sets. It uses no geometric test
and neither operand's class proves anything about the other, and it places no
condition on how many regions or holes either section or the result has. Where
H fails it can fail. For a notch cut by a box whose bottom cap lies in P, the
result's material at P is the whole square, while `σA \ σB` removes the tool's
footprint. Every admitted class refuses a plane on its own cap, which is this
case, and the revolve normal class refuses a shoulder level for the same
reason.

### The 2D operation

The operand sections, both in the requested frame, form one private Sketch
scene (`prismcells.BuildProfileScene` with the identity map), the default shape
`agent-docs/prism-boolean-design.md` §4.1 describes. Every region and hole of
both operands enters. Whole segments enter at their recorded points, centres
and radii, so the scene's walk charge is zero. The scene's coincident-span gap
must be zero. A span Sketch merged from two near-coincident carriers would
otherwise join or split regions that the exact sets do not.

Each cell's membership in A and in B is Sketch's answer, not decad's.
`prismcells.DeclareOperandLoops` declares every recorded loop of both operands,
outer loops and holes, as a Sketch loop of the scene's own entities in recorded
segment order, before the scene is published. `prismcells.ContainedMembership`
then reads each cell's `Profile.Containments()` verdicts: a cell is in an
operand when Sketch proves it inside the outer loop of one of the operand's
regions and outside every hole of that region. A verdict Sketch leaves
unproven, a missing verdict and an invalid cell refuse. This covers the cells
an edge-direction reading cannot reach, such as a section lying apart from the
other. A loop so thin that Sketch's map of it is degenerate, such as a dart a
few 2^-30 mm wide, reads `ReasonCoincident` and refuses here.

Sketch leaves out a cycle whose area falls below its per-cycle floor without
flagging it, and still proves the other cells' verdicts against the loop. A
triangle of area 3·2^-38 mm² inside a square is such a loop: the square's cell
reads outside it, yet no cell carries it as a hole, so its loss would return
the whole square. `prismcells.LoopEntitiesWalked` therefore requires every
entity of every recorded loop to be walked over its whole parameter domain by
the published cells, counting a window a coincident span withdraws from it
under the other operand's entity. It reads only the ranges Sketch reported, so
it can only refuse.

`prismcells.ClassifyNested` reads membership a second way, from Sketch's edge
directions (§4.2) over every loop of the cell, holes included: a cell walks
each of its loops with itself on the left, as an operand walks its outer loops
counter-clockwise and its holes clockwise, so a matching direction puts the
cell on that operand's material side, and membership crosses an edge of the
other operand unchanged. This reading can only falsify: a cell it reaches
must agree with Sketch's verdict, and a cell it does not reach proves nothing.
Membership is constant on each open cell and the arrangement's edges enclose
no area, so the closure of the cells `a ∨ b`, `a ∧ ¬b` or `a ∧ b` selects is
the regularized 2D operation of the lemma.

`prismcells.MergeRegionsExact` drops the walls between selected cells and
chains the survivors into simple loops by their exact walked ends. A vertex two
survivors leave refuses; this catches two regions that meet at a shared chain
vertex. Selected cells that share a wall are one region, by Sketch's own cell
adjacency. Which loop of a region is its outer loop is again Sketch's verdict:
every merged loop is declared as a loop of spans of the scene's entities, the
scene is published again, each republished representative cell must walk the
same edges as before, and that cell must lie inside exactly one of its
region's loops, the outer one, and outside the others, its holes. An unproven
verdict refuses. No coordinate is compared to place a loop. The seam's
loop-closure check can only refuse.

Each loop is then stated exactly or refused. A whole segment records from its
entity's own data. A cut end must be one paired Sketch line crossing that
`Sketch.CertifyLineLineJunction` solves from the two lines' endpoint bits as an
exact binary64 point. Exact rational membership on both source lines then
restates each line as a whole line between exact points; a whole arc between
them keeps its record, and its walked ends must meet its neighbours' ends
exactly. A cut arc or circle has no such certificate. Its recorded parameter
t names the point at angle 2πt about its centre, which binary64 coordinates
hold only at a quarter turn. `Sketch.CertifyLineCircleJunction` and
`Sketch.CertifyQuarterArcJunctions` return rational enclosures of a line-arc
crossing, not a binary64 vertex. The line-circle certificate takes only a
complete circle and declines a crossing straight across the centre along
either axis, which is where a quarter turn lies, and Sketch certifies no
arc-arc crossing. So no cut end on an arc or circle can be stated as one exact
point both neighbours share. A cut arc or circle returns `ErrUnsupported`, as
does a cut end Sketch does not certify; the cut charge must be zero, and decad
never rounds or fits a junction itself.

The regions are rebuilt through `sectionclip.ReconstructExactSections` on the
requested plane, all in one fresh Sketch. Each record must match a valid
profile whose fresh certified record equals it, and the profiles return in
record order; a face no record names, such as the empty space inside a hole
around an island, is left out. The result is one ordinary exact profile per
region and may enter any profile consumer.

## Exact interior section of a rectangular LoftSections result

The three-, four-, and five-to-thirty-two-section independent rectangular
builders retain their admitted exact half-width and half-height curves. For
three sources these are one quadratic over the full height; for four they are
one cubic; for five or more they are the piecewise cubic spans. The certificate
is attached only to the original faceted body. Placement, scaling, Boolean
operations and modify rewrites do not carry it.

An interior Section requires the canonical world-XY frame at the source's
common in-plane origin. The requested level must lie strictly between the
outer caps; an intermediate source level is allowed. Evaluate the retained
curves with exact rationals at that level. Both half-dimensions must be
positive and exactly representable as finite binary64 values. Their rectangle
is the source solid's exact cross-section. Reconstruct its four whole lines
through Sketch and require the returned valid region to match before returning
it. A cap-level, oblique, shifted, inexact or altered-body request refuses.

## Exact interior section of a three-section homothetic LoftSections result

The original three-section whole-line builder retains its authenticated base
walk, common world-XY origin, outer cap levels, and exact quadratic scale
`q(t) = 1 + bt + at²`. Its source gate proves one positive-area, hole-free
walk star-shaped about the scale origin, positive source scales, and a
nonnegative middle Bernstein control. The two end controls are positive, so
`q(t) > 0` throughout the cap interval. At a fixed interior level, the source
solid's material region is exactly the base polygon scaled by `q(t)`.

Require the canonical world-XY frame at the common in-plane origin, identity
placement, and a level strictly between the outer caps. Evaluate the level
fraction and scale as exact rationals. Require every scaled vertex coordinate
to be finite and exactly representable in binary64; then rebuild the complete
line walk in a fresh Sketch and require one matching valid profile. An inexact
coordinate, cap-level or exterior request, shifted or rotated frame, or copied
or scaled body returns `ErrUnsupported`. Placement and similarity drop the
source certificate.

## Parallel section of an authenticated matching-profile Loft

An authenticated two-section solid Loft with identical full profile records,
including any recorded holes, and zero segment alignment may return an exact
interior section. The two source frames and requested frame must have
canonical world-XY axes, matching in-plane origins and strictly ordered
levels. Identity placement, zero blend-construction displacement and
authenticated source records are required.
Each paired outer or hole boundary point has the same local coordinates in
both records, so its ruled path is vertical. Every level strictly between the two caps has
the recorded profile as its material region. Reconstruct that profile on a
fresh Sketch and require its matching valid region before returning it.
Cap-level, oblique, translated and twisted requests return `ErrUnsupported`.
Differing sections use the line-only or circular-hole classes below when
admitted. Cut and Fillet rewrites refuse because their inserted holes
or connector arcs were not authenticated by both source Sketch profiles.

## Parallel section of an authenticated line-only Loft

An authenticated two-section solid Loft with full `LineSeg` boundaries and
zero segment alignment may return an exact interior world-XY section. Both
source frames and the requested frame must have canonical world-XY axes and
the same in-plane origin. The source levels must be strictly ordered, and the
requested level must lie strictly between them. Identity placement, zero
vertex displacement, zero section displacement and unmodified source records
are required. A trimmed line or a curved boundary returns `ErrUnsupported`.

For each paired line cell, Loft's material wall is its two held triangles.
At the requested level, intersect the lower and upper endpoint rungs and the
fixed diagonal from the lower line's start to the upper line's end. These
three points form two section lines, including the diagonal kink when the
wall quad is not planar. Compute every coordinate from the two recorded
profiles and the exact rational level fraction. Refuse any coordinate that
cannot be represented exactly as finite binary64. Repeat for the outer loop
and each paired hole, then reconstruct the resulting region on a fresh
Sketch and require its matching valid profile. Cut and Fillet rewrites refuse.

## Parallel section of an authenticated circular-hole Loft

An authenticated two-section solid Loft with identical outer records, one
whole circular hole in each section and zero segment alignment may return an
exact section. The direct Loft circle-pair gate first proves the section
origins' exact axial relation. The sections must use matching canonical
world-XY axes, identity placement and zero blend-construction displacement.
Cut-rewritten Lofts are excluded because their inserted holes do not come from
both source Sketch profiles.
The requested frame must have those same axes and in-plane origin, at a level
strictly between the two sections.

The unchanged outer record denotes the same boundary at every level. The
ruled hole wall connects points at equal circle angles, so a fixed sweep
fraction gives a circle whose center and radius are the linear interpolations
of the two authenticated records. Compute that fraction and each interpolated
scalar as exact rationals. Refuse a coordinate or radius that is not exactly
representable as finite binary64, or a nonpositive radius. Reconstruct the
outer record and interpolated circle on a fresh Sketch, then require Sketch to
publish a valid matching profile. This exact profile can enter ordinary
`Extrude`; the source Loft and its document do not change. A rotated or
oblique section, a cap-level section, and a placed Loft keep `ErrUnsupported`.

## Parallel section of an authenticated circular-boundary Loft

An authenticated two-section solid Loft with one whole counterclockwise
outer circle and any number of paired whole clockwise circular holes may
return an exact interior section. Every paired loop has zero alignment.
The source planes, source frames and requested frame use canonical world-XY
axes and the same in-plane origin. The Loft has identity placement and no
Cut or Fillet rewrite. Its held circular chords may carry a positive mesh
displacement; the section reads the authenticated source circles themselves.

Each Loft wall pairs points at the same circle angle. At a fixed fraction
between its cap levels, every boundary is therefore a circle whose center
and radius are the linear interpolation of its paired source records. The
fraction comes from the exact rational cap and request levels. Each center
coordinate and radius must be finite and exactly representable as binary64.
The radius must remain positive. Prove containment of each hole in the outer
circle and separation of every hole pair over the entire cap interval.
Each signed gap is a quadratic in the sweep fraction: the squared radius
gap minus squared center distance for containment, or squared center
distance minus squared radius sum for separation. Require both endpoints
and any interior minimum to be strictly positive in exact rational
arithmetic. Endpoint separation alone does not prove that moving holes
stay apart. Reconstruct all circles on a fresh Sketch and require one valid
matching material region before returning it. A cap-level or oblique
request, a shifted request origin, a placed Loft or an inexact interpolated
circle returns `ErrUnsupported`.

## Cardinal plane of an authenticated source sphere

This class reads the occupied-ball certificate from `sourceSphereAtPose` for a
solid, full `Revolve` of one complete semicircle and its on-axis diameter. The
certificate rejects a displaced meridian, altered face topology, and a placed
body whose motion does not map signed world axes to signed world axes. A sphere
face tag alone does not establish the occupied ball.

Require the requested U, V and normal to be three distinct signed world axes,
with U×V exactly equal to the normal. Read the certified center, radius, plane
origin and axes as exact dyadic values. The signed normal offset `d` gives
`q = r² − d²`. Require `q > 0`; a tangent or exterior plane has no admitted
material profile. Use the downward dyadic square root and an exact square
comparison to find whether a binary64 radius `h` satisfies `h² = q`.

Project the center minus the plane origin onto U and V with exact dyadic dot
products. Require both local coordinates to be exactly representable as
finite binary64 values. The occupied ball meets the plane in a disk centered
there with radius `sqrt(q)`; this whole-ball identity does not require a sampled
meridian crossing. When `h² = q`, reconstruct a whole counterclockwise circle
through Sketch and require its fresh profile record to match before returning
it. The exact circle may enter ordinary profile consumers, including `Extrude`.

Otherwise find the downward and upward binary64 square-root brackets by exact
square comparisons. Refuse a zero lower radius or a nonfinite upper radius.
Sample the lower circle at the same rational half-angle stations and with the
same strict-convexity, feature-scale and 0.05 mm gates as the bounded complete
ellipse below. The exact station polygon lies inside the true disk; its rounded
Sketch proxy need not. The lower-circle chords have the rational `d²/4`
unit-circle sagitta bound. Every point of the true disk can first move
radially to the lower disk within the upper-minus-lower
radius gap, then move to the lower polygon within the chord gap. Twice the
radius gap bounds the plane-local L1 displacement; multiplying by the larger
held basis length upper bound covers world displacement. Add these gaps and
the rounded polygon-vertex errors without cancellation to `sectionDelta`.
The same radial map and matching convex combinations cover both occupied
regions, not only their boundaries.

Reconstruct the polygon through fresh Sketch reference lines and attach the
private revision-bound certificate below. The Sketch polygon is a two-sided
proxy, not an exact circle. Only straight solid `Extrude` with an
independent `Distance` may consume it; all other profile consumers refuse it.

## Cardinal plane of an authenticated right cone

This class admits a sound solid made by a full `Revolve` of three complete
meridian lines: one joins the base centre to the rim, one joins the rim to the
apex, and one joins the apex to the base centre along the axis. It requires
one cone face, one planar base face, one rim edge, no holes, zero meridian
displacement, an exact resolved axis, a cardinal source frame and a signed
cardinal placement. A cone face tag without that source record does not pass.

Require a cardinal requested frame whose normal is parallel to the placed
base-to-apex axis. Read the source corners, placed centres and requested plane
as exact dyadic values. The base centre and apex must lie on strictly opposite
sides of the plane; a cap, apex, tangent or exterior cut returns
`ErrUnsupported`. The three lines enclose a disk whose radius decreases
linearly from the base radius to zero at the apex. Divide the exact signed
base-plane distance by the exact base-to-apex height, and calculate the cut
radius as a rational value. Require the projected circle centre to have
finite, exactly representable binary64 coordinates. An exactly representable
positive radius reconstructs one complete counterclockwise circle in Sketch.
Require Sketch's fresh profile record to match before returning it. That exact
profile can enter ordinary profile consumers, including `Extrude`.

For an inexact radius, round the rational value down and up to binary64 with
exact rational comparisons. Refuse a zero lower radius or a nonfinite upper
radius. The lower disk lies inside the true section, which lies inside the
upper disk. The gap between their radii bounds radial displacement; twice
that gap bounds plane-local L1 displacement. The held plane-basis lengths
convert it to a world-distance bound. Sample the lower circle with the same
rational half-angle stations, chord bound, convexity check, feature-scale gate
and 0.05 mm limit as the bounded sphere section. The radial map and convex
combinations of matched chord endpoints cover both occupied regions.
Round the station coordinates for Sketch; the held polygon may cross the true
circle, and its rational-to-binary64 vertex errors join the radial and chord
terms in the two-sided `sectionDelta`. Reconstruct the bounded polygon through
the private revision-bound Sketch certificate. Only straight solid `Extrude`
with an independent `Distance` may consume this bounded profile; its prism
proof carries the displacement.

## Plane through or normal to the axis of a full Revolve

A full solid `Revolve` that misses the sphere and cone classes above may take
one of two exact classes, both read from its recorded meridian by
`sectionclip.RevolveAxialSection` and `sectionclip.RevolveNormalSection`
(`internal/sectionclip/revolve_meridian.go`). A common-axis Boolean result
that keeps a revolve record reaches them through the same gates.

### Source admission

The record must be a full turn of a solid with zero section displacement. Its
resolved axis must lie along one source-plane axis with zero anchor and
direction bounds, zero radial admission allowance and zero snap allowance. The
source frame must be cardinal and the placement a signed-axis transform. No
binary64 direction off the world axes has unit length, so the cardinal axis is
also what makes the meridian map exact. Every loop must hold whole lines, arcs
and circles with finite coordinates; a partial range, a spline or any other kind
refuses. An arc's two pinned points must lie exactly equally far from its
centre, so the record names one circular arc under any reflection.

Read every point in exact axis coordinates: `a = (p − A)·D` along the oriented
axis and `ρ = cross(D, p − A)` across it, with `A` the anchor and `D` the axis.
That map is a rotation of the source plane, so loops keep their winding. A
point or arc or circle centre with `ρ < 0` refuses. So does one with
`0 < ρ ≤ SnapTol`, because the build reads such a point as on the axis and a
wall about such a centre as a sphere. The placed anchor `A_w` and axial
direction `D_w` are exact dyadic images through the frame and placement.

The solid is `X = {A_w + a·D_w + ρ·(cos φ·R + sin φ·S)}` over the meridian
region `M` and every angle, with `R`, `S` the unit world axes across `D_w`. The
admitted planes are cardinal and their stored normal is the exact cross
product of their axes.

### Plane holding the axis

The plane must hold the anchor exactly and its normal `n` must be
perpendicular to `D_w`. Both are signed world axes, so either `n` is parallel
to `D_w` and `e = n × D_w` is zero, or `e` is the unit in-plane direction
across the axis. With `t` the signed offset along `e`, a plane point is
`A_w + a·D_w + t·e`, and it lies in `X` exactly when `(a, |t|) ∈ M`. So
`X ∩ P = M ∪ M'`, where `M'` is the mirror `t = −ρ`. Both copies are closures
of their interiors, so their union is the regularized section. The requested
local coordinates are a signed permutation of `(a, t)` plus the anchor's
offset. Its determinant is `(D_w × e)·n = n·n = 1` when the plane holds the
axis, and zero when `n` is parallel to `D_w`, which refuses. Every mapped coordinate must
be finite and exactly representable in binary64.

The union's loops depend on how `M` meets the axis.

- No point of the outer loop is on the axis. Then `M` and `M'` are disjoint and
  the section is two regions, each with its copy of every hole.
- The outer loop holds exactly one cyclically contiguous run of whole axis
  lines (both ends at `ρ = 0`). Every other segment must meet the axis at most
  at its ends, and every vertex but the run's two ends must have `ρ > 0`; a
  second run or contact puts a chain junction on the axis and refuses. Then
  `M ∩ M'` is the run, and the union is one region. Its outer loop is the
  off-axis chain from the run's end back to its start, followed by the mirror of
  that chain walked backwards. The holes are every hole of `M` and its mirror.
- A vertex on the axis outside a run pinches `M ∪ M'` at a point, and refuses.

A hole must clear the axis entirely. A line meets the axis inside it only when
both ends are on it. A whole circle clears it when `ρ_c² > r²`. On an arc's
circle, `ρ = ρ_c + r·sin θ` is least at the point `B` straight towards the axis,
and the points with `ρ ≤ 0` form one arc about `B`. With both ends at `ρ ≥ 0`,
the arc keeps `ρ > 0` inside it exactly when `ρ_c² > r²` or `B` is neither inside
its sweep nor one of its ends; exact cross products about the centre decide
it. An arc arriving on the axis tangent to it ends at `B` and refuses.

The copy's map has determinant 1 and keeps the meridian's winding. The
mirror's map is the copy's composed with a reflection, so it reverses the
winding and the mirror is walked backwards: a line swaps its range, an arc
swaps its pinned ends and keeps its range, and a circle keeps its record,
because the reflection and the reversed walk cancel.

### Plane normal to the axis

The plane normal must equal `±D_w` exactly. The level is
`h = (O − A_w)·D_w` and the circle centre is the anchor's local coordinates,
which must be finite and exact. The plane meets `X` in
`{ρ : (h, ρ) ∈ M}` turned about the centre.

Every segment end must lie off level `h`; a cap, shoulder or vertex level
refuses. Then a line crosses the level only between ends on strictly opposite
sides, at the rational `ρ = ρ0 + (h − a0)(ρ1 − ρ0)/(a1 − a0)`. An arc or circle
crosses it at `ρ_c ± w` with `w² = r² − (h − a_c)²`. A negative square misses,
and `w` must be the exact binary64 square root, found by the downward dyadic
square root and an exact square comparison. An arc keeps only the candidates
inside its counter-clockwise sweep, decided by exact cross products about its
centre. Every crossing is transversal. Each radius must be a dyadic rational
held exactly as a finite binary64 value with `ρ ≥ 0`. An arc or circle crossing
at `ρ = 0` refuses; a line can cross there only when it is an axis line.

Sorted, the radii must be strictly increasing and even in number. A tangent
level gives one radius twice and refuses, as do two loops meeting on the level.
Along the level line, membership in `M` flips at each transversal crossing, so
the material is the union of the intervals between the first and second
radius, the third and fourth, and so on. Each interval becomes one region: a
counter-clockwise outer circle, with a clockwise hole circle when its inner
radius is positive. No crossing means the plane misses the body and refuses.

### Reconstruction

`sectionclip.ReconstructExactSections` charges each region and
`momentinput.ReconstructProfilesOnFrame` builds every entity on one fresh
Sketch on the requested frame. Each record must match a valid Sketch profile
whose fresh certified record equals it, and the profiles return in record
order; a region Sketch arranges that no record names, such as the gap between
two rings, is left out. The profiles are exact and may enter any profile consumer,
including `Extrude`.

## Oblique plane of an authenticated source sphere

This class uses the same `sourceSphereAtPose` occupied-ball certificate. It
requires a noncardinal requested frame and a strictly interior plane cut.
A sphere placed by an unsupported rotation still fails the source certificate.

Read the held U and V as exact dyadic vectors and form their Gram entries
`guu = U·U`, `gvv = V·V`, and `guv = U·V`. Set
`m = min(guu - |guv|, gvv - |guv|)` and
`M = max(guu + |guv|, gvv + |guv|)`. Require `m > 0`. For every local vector
`x`, exact arithmetic then proves
`m|x|² <= |U*x.u + V*x.v|² <= M|x|²`. The true plane-local section can be an
ellipse because held frame axes are only approximately orthonormal.

Let `w` be the held plane origin minus the ball center and `b=(U·w,V·w)`.
Solve `G*c=-b` over exact rationals for the local ellipse center `c`, where
`G` is the Gram matrix. Require its exact determinant to be positive. The
completed-square radius is `q = r² - |w|² - b·c`; refuse `q <= 0` for tangent
and exterior planes. Exact square comparisons round `sqrt(q/M)` downward
to an inner local radius and `sqrt(q/m)` upward to an outer local radius.
Refuse a zero inner radius or a nonfinite bracket. Both disks use the exact
rational center `c`. The inner disk lies inside the true section, which lies
inside the outer disk.
Sample the inner circle with the rational half-angle stations and convexity,
feature-scale and 0.05 mm gates of the bounded circle polygon above. The
rounded Sketch polygon is a two-sided proxy even though the exact station
polygon lies inside the inner disk. The outer-minus-inner radius bounds radial
movement from the true section to the inner disk. Twice that gap bounds its
local L1 move; multiplying by the larger
held basis length upper bound covers world movement. Add the chord and rounded
vertex errors without cancellation to `sectionDelta`. The radial mapping and
convexity cover both occupied planar regions and their boundaries.

Reconstruct the resulting polygon with the private revision-bound Sketch
certificate. Only straight solid `Extrude` with independent `Distance` may
consume it. The prism frame-map charge separately covers the held U,V,N map
when the downstream sweep is not exactly orthonormal.

## Cardinal vertical plane of an exact cylinder

The second class reads an exact straight prism whose authenticated source is
one whole, counterclockwise `CircleSeg` with positive radius and no holes.
The source section and both cap levels carry zero displacement. The exact
placed images of the source U, V and normal must each be a signed world
coordinate axis; an image that does not have unit length and cardinal direction
returns `ErrUnsupported`.

The requested plane's V axis must equal either sense of the placed sweep
axis. Its U axis must equal either sense of one placed source radial axis.
The requested normal must equal the exact U×V cross product. The plane may
be offset from the placed circle centre's sweep axis. Compute its signed
normal offset `d` in exact dyadic arithmetic. For a recorded radius `r`,
require `q = r² − d² > 0`. At zero offset, retain the recorded radius as the
half-chord. At nonzero offset, use the downward dyadic square root and an
exact square comparison to require a binary64 half-chord `h` with `h² = q`.
A tangent plane, an exterior plane, a chord without such an exact `h`, or a
frame that fails the cardinal equalities returns `ErrUnsupported`.

The source circle denotes a disk; sweeping it through the recorded interval
denotes disk × interval. The admitted plane cuts the disk on a chord,
so the section is the Cartesian product of the exact interval
`[uCenter − h, uCenter + h]` and the two recorded cap levels. Compute the
four corners in the requested frame using
exact dyadic arithmetic. A corner whose local coordinate is not finite and
exactly representable as `float64` returns `ErrUnsupported`; no rounded
intersection is published as an exact Sketch line. Reconstruct the four-line
profile on the requested plane through Sketch and require its fresh recorded
profile to match that exact rectangle. The parallel class keeps its existing
circle result and its cap-plane refusal.

## Cardinal vertical plane of an exact rectangular prism

This class admits a hole-free exact straight prism whose source profile has
four complete lines forming one axis-aligned rectangle in its own section
coordinates. A blend record, nonzero section or cap displacement, or a
nonpositive sweep interval returns `ErrUnsupported`.

Prove with exact dyadic arithmetic that the placed source U, V, and sweep axes
are signed world coordinate axes. The requested V must equal either sense of
the sweep axis; its U must equal either sense of one source radial axis; its
normal must equal their cross product. Compute the plane's signed normal
coordinate at opposite rectangle corners and require strict opposite signs.
This refuses tangent and exterior planes without a distance tolerance.

The section is the product of the uncut source radial interval and the sweep
interval. Derive both local interval ends from exact images of the source
corners and cap levels. Refuse any corner coordinate that is not finite and exactly
representable as `float64`. Reconstruct the resulting four-line profile in a
fresh Sketch and require Sketch's certified record to match before returning
it. The parallel prism and cylinder classes keep their existing gates.

## Bounded complete ellipse of an exact whole-circle prism

This class accepts the authenticated, hole-free, whole counterclockwise circle
of an exact straight `prismPayload`. It refuses a blend record, nonzero section
or cap displacement, and a nonpositive sweep interval. The requested plane may
be oblique and the source prism may have a rigid placement. The parallel and
cardinal classes run first.

Read stored source, placement, circle and requested-frame coefficients as exact
dyadics. The requested plane is the affine span of its held U and V; use their
exact cross product for plane tests. If that plane is parallel to the placed
sweep axis, return `ErrUnsupported`. Otherwise solve its sweep coordinate as
`z = zCenter + zCos*cos(theta) + zSin*sin(theta)` in exact rationals. Require
`zCenter - |zCos| - |zSin| > z0` and
`zCenter + |zCos| + |zSin| < z1`. These strict inequalities prove that the
entire cut lies between both caps; a cap-crossing or cap-touching curve
returns `ErrUnsupported`.

Solve the placed centre and two radial vectors in the requested held U,V basis,
checking the third world coordinate exactly. Their plane-local images give an
affine map of the unit disk. A zero determinant returns `ErrUnsupported`; the
nonzero map sends its boundary to one complete ellipse and its interior to the
filled section. Reverse the sample order if the determinant reverses winding.
Sample four quadrants with the exact rational half-angle parameter
`((1-t²)/(1+t²), 2t/(1+t²))`, at equally spaced rational `t` values. Try 8,
16, then 32 samples per quadrant. Every sample lies exactly on the source
circle, so its plane-local image is an exact rational ellipse point. Round each
point to finite binary64 coordinates for Sketch reference lines.

For a unit-circle arc with endpoint chord length `d` and angle below 180°, its
maximum radial gap to that chord is at most `d²/4`. Take the largest exact
sample chord squared, divide by four, and multiply it by the sums of absolute
plane-local affine coefficients. This encloses both local coordinate errors
from every ellipse arc to its chord. The held U,V length upper bounds convert
those errors to a world-distance bound. Add the outward-rounded rational-to-
binary64 error of each polygon vertex in both local and world coordinates.
The larger total is `sectionDelta`. The affine image of the sampled circle is
strictly convex, and the existing exact source-to-held polygon test requires
its rounded line ring and every intermediate ring to remain strictly convex.
An ellipse boundary point maps radially to its chord within the gap above;
scaling that mapping from the centre covers both occupied regions. Refuse a
sample count if the delta exceeds 0.05 mm, if four times the delta reaches any
exact polygon edge's local L1 length, or if the rounded convexity proof fails.
Exhausting 32 samples per quadrant returns `ErrUnsupported`.

Reconstruct the rounded polygon through Sketch reference lines and attach the
same private, revision-bound certificate described below. The Sketch area is
the polygon's area, not an exact ellipse-area reading. `Extrude` carries
`sectionDelta` to its prism proof; `MeasureProfile` and other undecorated
consumers refuse the bounded source.

## Bounded one-cap crossing of an exact whole-circle prism

An oblique plane may cross exactly one cap of the same authenticated whole-circle
prism. Read the plane, circle, placement and cap coefficients as exact dyadics.
Solve the sweep level over the source unit disk in rationals as
`zCenter + zCos*x + zSin*y`. Prove one cap line is a strict secant using the
exact squared distance test. Require the other cap to lie strictly outside the
disk and the outer enclosure below. Tangency, two-cap crossings and a plane
parallel to the sweep axis refuse.

Build a source-plane Sketch from the original circle and the exact cap line.
The line's two finite endpoints must be exactly representable as binary64 and
must extend beyond the circle. Require Sketch to publish a valid, hole-free
circle-and-line region whose partial bounds all have `TExact`. Choose the
region by the directed line's interior side, never by sampled area or crossing
coordinates. This Sketch check owns the trim topology; no approximate
intersection may substitute for it.

In parallel, sample the unit circle at rational half-angle stations. Their
convex hull is an inner disk polygon; intersections of adjacent exact tangent
lines make an outer disk polygon. Clip both by the exact cap half-plane, then
map them into the requested plane with the exact rational affine map. The
inner polygon lies inside the true section, which lies inside the outer one.
Sketch supplies trim topology but no displacement bound for the mapped ellipse,
so the rational enclosures supply only that missing bound.
For every outer vertex, find a point on an inner edge using exact rational
projection. The largest outward-rounded local and world distances cover the
outer polygon because a convex neighborhood of the inner polygon contains
every outer vertex. This bounds the two-sided occupied-region displacement.
Add the exact rational-to-binary64 rounding error of every inner vertex.
Require the same strict convexity and edge-scale gates as the complete-ellipse
case, plus the 0.05 mm displacement cap. Exhausting the station budget refuses.

Reconstruct the inner polygon through the existing private bounded-Section
Sketch certificate. Only straight solid `Extrude` can consume it; the
certificate carries the source displacement through the prism volume proof
and `VerifyAll`.

## Bounded oblique plane of an exact rectangular prism

This class accepts a sound, hole-free `prismPayload` whose authenticated source
profile is one complete axis-aligned four-line rectangle. Its section and cap
displacements must be zero, its sweep interval must be positive, and it must
have no blend record. The requested frame may be oblique and the prism may have
a rigid placement. The parallel and cardinal classes run first.

Treat each held source coordinate, frame coefficient and rigid-placement
coefficient as an exact dyadic. The requested plane is the affine span of its
held `U` and `V` vectors. Use their exact cross product for plane-side tests;
do not assume the stored `N` is exactly their cross product. Build the eight
source corners and twelve edges by exact rational arithmetic. A nonzero signed
plane value at every source corner and at least three strictly crossed edges
are required. A tangent, exterior, cap-coincident or vertex-contact cut returns
`ErrUnsupported`. A corner or edge that cannot be bounded with finite
arithmetic also returns `ErrUnsupported`.

Intersect every crossing edge with the plane using its exact rational crossing
parameter. Solve for each intersection's rational coordinates in the requested
`U,V` basis and verify the third world component exactly. Deduplicate and sort
the three to six vertices by exact plane-local orientation. Require a positive,
strictly convex polygon. Round each coordinate to a finite `float64`
representative, then prove that the linearly matched source and rounded vertex
rings stay strictly convex through the entire straight interpolation. This
proves that the rounded loop has the source loop's region and edge pairing.
If the convexity proof, positive-area check, or Sketch reconstruction fails,
return `ErrUnsupported`.

For every vertex, bound the absolute errors in both plane-local coordinates
with exact rationals. For each vertex, take the larger of its plane-local
`|du|+|dv|` and the sum of each error times an outward bound on its held basis
vector's world length. Round the maximum upward to obtain `sectionDelta`.
Linear interpolation along matched edges makes this bound cover both plane-local
and world displacement at every point of each polygon boundary. Both loops are
convex with the same vertex order, so matching convex combinations of their
vertices extend the same bound through the occupied planar regions. Refuse a loop
when four times this bound reaches any exact source edge's plane-local L1
length; the bounded proxy must preserve a useful local feature scale. The
world term is needed even when the held frame differs slightly from an exact
orthonormal frame; the mesh uses `sectionDelta` as a world-distance face bound.
The existing prism frame-map charge separately covers the held basis's effect
on mass and coordinate readings. The rounded line loop is reconstructed as a
fresh Sketch profile on the requested frame and re-recorded through the
ordinary Sketch seam. Its Sketch area describes only the rounded loop; it does
not certify equality with the source section.

Attach a private, revision-bound certificate to the returned Sketch. The
certificate records the reconstructed profile, its frame, the source section
bound, the exact `*sketch.Sketch` owner pointer, and the Sketch revision. The
Sketch stores this value under a private pointer-identity key only decad owns.
Lookup rejects a shallow `copy := *s` because its owner pointer differs, even
if the copied Sketch shares the attachment map. In-place `Sketch.UnmarshalJSON`
must clear every attachment when it
resets the Sketch; an attachment is never serialized. The loop's points and
entities are locked Sketch references with a section-source marker that does
survive serialization. A mutation, missing certificate, mismatched frame or
profile, or deserialized marker returns `ErrUnsupported` at every decad
profile-consumer entry point. The marker alone never grants a bound. Neither
Sketch nor its caller can mint decad's private certificate by supplying a
matching source string.

Admission explicitly checks every certified boundary line and endpoint for
live membership in that Sketch, reference status, the expected `Source()`, and
`IsStale() == false`. Sketch revision does not hash reference source or stale
flags, so matching revisions alone cannot establish this. The central
`sectioncert.RecordProfile` and `recordChain` gates scan all Sketch points and entities for
the section-source marker or attached certificate before recording any profile
or chain. They refuse even a second ordinary region in the same Sketch when a
bounded section marker exists. An in-place load, a reference refresh or a
staleness mark cannot strip the source bound by selecting a different region.

`Extrude` authenticates the fresh Sketch profile and certificate before extent
resolution. It requires bitwise equality of the live Sketch plane's held origin,
U, V and N with the certificate's requested frame. It uses that certified frame
directly, because rebuilding an oblique frame can move its held basis by one bit.
Ordinary profiles keep the usual frame reconstruction.
It passes `sectionDelta` into the new `prismPayload`, so `Volume`,
`Bounds`, `Tessellate(VerifyAll)` and later supported operations charge the
section displacement through their existing proof paths. `Extrude` refuses a
nonzero taper or a section-dependent extent until those paths consume the
bound. `MeasureProfile`, `Revolve`, `Loft`, `Sweep`, `Coil`, `Patch` and all other
profile consumers return `ErrUnsupported` for a bounded Section sketch until
they propagate its source bound. A caller who wants to edit or independently
use the rounded polygon must draw it in a new ordinary Sketch.

Sketch's attachment stores an opaque package-owned value in the Sketch object;
it is not a global pointer map and it is not serialized. Its key identifies
decad's certificate, while the reference-source marker makes loss of the
attachment a refusal rather than an exact source claim. The certificate's
revision and profile comparison prevent a stale or caller-edited snapshot from
reusing its bound.

## Verification

- Section real mesh Boolean results of a 10 mm cube with a second box. A
  Union with a 20×4×8 mm beam along X sectioned through the overlap at z=8
  gives 140 mm², a Cut by a 10 mm box over one top corner at z=7 gives 75 mm²,
  and an Intersect with the beam at z=8 gives 40 mm². Each corner matches
  exactly, and each 2 mm Extrude encloses twice the area and passes
  `Tessellate(0.1 mm, VerifyAll)`. A corner tool drawn on a plane facing down
  takes a reflected candidate frame. Two boxes on one tilted Sketch plane,
  moved along its normal by powers of two, cut on the mesh path and section
  at 75 mm² in that plane's own frame. Refuse a gap-shifted result, the tool's
  cap level and a junction Sketch does not certify. A copied result, a plane
  missing an operand, a slot that splits the section and a section nested in
  the other are axis-square, so the orthogonal grid reads them first. A unit
  fixture refuses two regions that Sketch welds across a 2^-40 mm gap.
- Section real mesh Boolean results whose operand sections hold whole circles
  and holes. A 10 mm cube with a radius 2 mm through hole, cut by a beam along X
  spanning y 7.5..9.5, gives at z=5 a region of 75 − 4π mm² with the round
  hole and a 5 mm² strip. The same plate intersected with a beam spanning
  y 1..9 gives one region of 80 − 4π mm² with the hole. The cube joined to a
  radius 2 mm cylinder along X lying apart gives a 100 mm² square and a 4π mm²
  disk on the plane x=5, each operand read in its own candidate frame. A
  revolved tube r 4..10 joined to a box in its bore gives at z=8 a 16 mm²
  square and an 84π mm² annulus. Each region's 1 mm Extrude encloses its area
  and passes `Tessellate(0.1 mm, VerifyAll)`. A cube cut by a cylinder over its
  corner and a cylinder of radius 6 mm along X joined to one of radius 4 mm
  along Y refuse, because their sections cut a circle. Unit fixtures merge a
  disjoint union, a circular hole, an island inside a hole, a disk inside a
  square, a hole inside an overlap, a split holed square and a whole arc
  beside certified line cuts, and refuse a line crossing an arc or a circle,
  two regions touching at a corner, an empty result, a square cut by a dart
  of width 2^-30 to 2^-44 mm whose containment Sketch leaves unproven, and a
  square combined with a triangle of area 3·2^-38 to 3·2^-50 mm², axis-aligned
  or not, that Sketch drops below its area floor.
- Cut a real 4×4×4 mm blind pocket from a 10 mm cube. Section at z=8 gives
  84 mm² with one hole spanning (3..7, 3..7), and at z=3 gives 100 mm². Each
  2 mm Extrude passes `Tessellate(0.1 mm, VerifyAll)`. The pocket floor, caps
  and exterior levels, a circular pocket (displaced stack), a translated body
  and a reversed frame are refused.
- Loft three real Sketch triangles with scales 1, 1.5 and 1 at equally spaced
  heights. Section at one-quarter height gives 11.34375 mm²; its 2 mm Extrude
  gives 22.6875 mm³ and passes `Tessellate(0.1 mm, VerifyAll)`. Repeat at the
  middle source level with a translated frame and with a concave star-shaped
  polygon. Refuse cap, exterior, shifted, rotated, inexact and copied cases.
- Extrude a 10×10 mm square by 5 mm. Section at 2.5 mm must give one 100 mm²
  profile. Extruding that returned Sketch/profile by 2 mm gives 200 mm³ and
  passes `Tessellate(VerifyAll)`.
- A rigidly translated prism must section on the correspondingly translated
  plane. A source profile with a hole must return the same outer and hole
  boundaries when Sketch can certify their reconstruction.
- Loft two real Sketch whole-circle profiles of radii 1 and 3 mm across 2 mm.
  A midlevel Section returns radius 2 mm; its 2 mm Extrude encloses 8π mm³
  and passes `Tessellate(0.1 mm, VerifyAll)`. Check a translated source frame,
  a moving outer circle, one moving hole and two paired holes. Refuse cap,
  tilted, shifted source or request, placed and inexact X/Y or radius
  interpolation requests.
- Extrude a 1 mm whole circle by 5 mm. Section it by a cardinal plane through
  its axis and check the exact 2×5 mm rectangle. Re-extrude the returned
  profile by 2 mm and require `Tessellate(0.1 mm, VerifyAll)` to certify the
  resulting 20 mm³ prism. Check a translated cardinal cylinder too.
- Extrude a 5 mm whole circle by 10 mm. Section it by a cardinal plane 3 mm
  from its axis and check the exact 8×10 mm rectangle. Re-extrude the returned
  profile by 2 mm and require `Tessellate(0.1 mm, VerifyAll)` to certify the
  resulting 160 mm³ prism. Check another exact chord, both offset signs, and
  a rigidly placed cylinder.
- Extrude a 10×10 mm rectangle by 5 mm. Its cardinal vertical section through
  the interior is 50 mm². Extrude the returned Sketch/profile by 2 mm and
  require 100 mm³ and `Tessellate(0.1 mm, VerifyAll)`.
- Check a translated, quarter-turned rectangular prism with both radial axes
  and a reversed sweep axis. Refuse tangent, exterior and tapered rectangular
  cases; an interior noncardinal cut uses the bounded class.
- Refuse a tangent or exterior cardinal plane, a non-cardinal plane, and a
  radius or frame whose derived rectangle corner cannot be held exactly.
- A cap plane, unadmitted oblique plane, sheet body, invalid input, and
  canceled context must return their stated errors without changing the
  body's document.
- An interior oblique cut of a real Sketch-produced rectangular prism returns
  one bounded profile. Extruding it by 2 mm yields a volume interval containing
  the exact clipped area times 2 mm and passes `Tessellate(0.1 mm, VerifyAll)`.
- A real whole-circle cylinder of radius 2 mm and height 10 mm cut by the
  central plane spanned by `(1,0,0)` and `(0,1,1)` returns one bounded ellipse
  polygon. Its largest section coordinates enclose 2 mm and 2√2 mm.
  A 2 mm Extrude yields a volume interval containing 8π√2 mm³ and passes
  `Tessellate(0.1 mm, VerifyAll)`. Check a placed cylinder, reversed section
  axis and a cut crossing a cap.
- A radius 5 mm Sketch semicircle revolved to a full sphere and cut by the
  cardinal plane at z = 3 mm returns a circle of radius 4 mm and area 16π mm².
  Extruding it by 2 mm yields 32π mm³ and passes `Tessellate(0.1 mm, VerifyAll)`.
  Check a plane normal to the Revolve axis, signed-cardinal placement and
  reversed section axes. At z = 2 mm, the bounded polygon encloses the true
  21π mm² section and its 2 mm Extrude encloses 42π mm³ while passing the same
  mesh check. A noncardinal frame whose origin is the sphere center encloses
  the true 25π mm² section; its 2 mm Extrude encloses 50π mm³ and passes the
  same mesh check. A noncardinal frame 3 mm from the center encloses the true
  16π mm² section; its 2 mm Extrude encloses 32π mm³ and passes the same mesh
  check. A frame shifted along its own plane retains the same section. Refuse
  tangent and exterior cuts.
- A full Revolve cone with a 5 mm base radius and 3 mm height cut 1 mm from its
  base returns a bounded 10/3 mm radius section. Its 2 mm Extrude encloses
  200π/9 mm³ and passes `Tessellate(0.1 mm, VerifyAll)`; `MeasureProfile`
  refuses the bounded Sketch. Place its axis at X = 1,048,576 mm while keeping
  the plane origin at X = 0 and compare the rounded polygon vertices against
  the exact circle. Vertices may lie outside, and the volume and mesh bounds
  must still enclose the source. With a 10 mm cone shifted by X = 0.1 mm or
  Y = 0.1 mm, a plane origin at 1,048,576 mm in the corresponding local axis
  must refuse when that projected centre is not exactly representable.
- Revolve a stepped meridian (r=10 for z 0..20, r=6 for z 20..40) about world
  Z. The XZ plane, its quarter turn and its reflection each give one region of
  area 2·(10·20 + 6·20) = 640 mm² with the exact ten corners; a 1 mm Extrude
  gives 640 mm³ and passes `Tessellate(0.1 mm, VerifyAll)`. A tube r 4..10
  gives two 120 mm² regions, a capsule with a hemisphere end gives
  100 + 12.5π mm², a torus gives two 25π mm² disks, and a shaft with a toroidal
  void gives one region with two circular holes. Normal planes give a disk of
  100π mm² at z=10, of 16π and 9π mm² on the dome, an annulus of 84π mm² on the
  tube and a disk inside an annulus through the void. A signed-axis placement
  keeps the same sections. Refuse the shoulder, caps, an exterior level, an
  offset or parallel plane, an oblique plane, a pinched meridian, an inexact
  crossing radius or square root, and the dome apex. Unit fixtures refuse each
  source, axial and normal gate on its own.
- Check translated and rigidly placed rectangular prisms, reversed section
  axes, triangle through hexagon section rings, and cuts that refuse at a
  source vertex, tangent, exterior or uncertain topology.
- A changed, stale or serialized bounded Section sketch cannot enter any undecorated
  profile consumer as an exact source section. A forged reference-source marker
  has the same refusal, including when a different profile or chain is selected
  in the same Sketch. Ordinary Sketch profiles keep their current results.
