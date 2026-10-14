# Planar section design

`Body.Section` reads a solid at a requested plane and returns a new Sketch whose
profiles denote the material in that plane. It does not change the body or its
document. An exact returned Sketch may enter any admitted profile consumer. A
bounded section may enter only the straight solid `Extrude` path below.

## Public result

```go
func (b *Body) Section(ctx context.Context, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, error)
```

The returned Sketch uses `plane` as its construction frame. The profiles are
fresh, valid Sketch snapshots; changing the Sketch makes them stale under the
ordinary seam rules or the bounded-source certificate below. A later body
class may return several regions, so the result is a slice even though the
admitted prism carries one material region.
The caller may read a retired body. A nil body or context, or an invalid frame,
is `ErrDegenerate`; a sheet or unsound solid is `ErrNotSolid`.

## Parallel exact prism class

The parallel class admits a `prismPayload` with zero `sectionDelta`,
`z0Delta`, and `z1Delta`. It refuses a plane that touches either cap. A body
outside this prism class tries the authenticated sphere and cone classes below or returns
`ErrUnsupported` without changing the document.

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

## Exact slab of an authenticated Boolean BRep

An A1 planar `Union` may return a BRep with an authenticated stack of prism
slabs. `Body.Section` reads that stack through `stackedUnionOperandOf`, rather
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
ellipse below. This polygon lies inside the true disk. Its lower-circle chords
have the rational `d²/4` unit-circle sagitta bound. Every point of the true
disk can first move radially to the lower disk within the upper-minus-lower
radius gap, then move to the lower polygon within the chord gap. Twice the
radius gap bounds the plane-local L1 displacement; multiplying by the larger
held basis length upper bound covers world displacement. Add these gaps and
the rounded polygon-vertex errors without cancellation to `sectionDelta`.
The same radial map and convex combinations of matching chord endpoints cover
both occupied regions, not only their boundaries.

Reconstruct the polygon through fresh Sketch reference lines and attach the
private revision-bound certificate below. The Sketch polygon is an inner
approximation, not an exact circle. Only straight solid `Extrude` with an
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
radius as a rational value. Require that radius and the projected circle
centre have finite, exactly representable binary64 coordinates. A radius such
as 10/3 mm returns `ErrUnsupported` instead of a rounded Sketch circle.

Reconstruct one complete counterclockwise circle in Sketch and require its
fresh profile record to match before returning it. The exact profile can
enter ordinary profile consumers, including `Extrude`.

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
outer-minus-inner radius bounds radial movement from the true section to the
inner disk. Twice that gap bounds its local L1 move; multiplying by the larger
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
- Check translated and rigidly placed rectangular prisms, reversed section
  axes, triangle through hexagon section rings, and cuts that refuse at a
  source vertex, tangent, exterior or uncertain topology.
- A changed, stale or serialized bounded Section sketch cannot enter any undecorated
  profile consumer as an exact source section. A forged reference-source marker
  has the same refusal, including when a different profile or chain is selected
  in the same Sketch. Ordinary Sketch profiles keep their current results.
