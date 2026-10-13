# Planar section design

`Body.Section` reads a solid at a requested plane and returns a new Sketch whose
profiles denote the material in that plane. It does not change the body or its
document. An exact returned Sketch may enter any admitted profile consumer. A
bounded oblique section may enter only the straight solid `Extrude` path below.

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
whose payload is not this prism class returns `ErrUnsupported` without
changing the document.

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
`recordProfile` and `recordChain` gates scan all Sketch points and entities for
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
- Refuse a tangent or exterior cardinal plane, a positive `q` whose square
  root is not exactly binary64, a non-cardinal plane, and a radius or frame
  whose derived rectangle corner cannot be held exactly.
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
- Check translated and rigidly placed rectangular prisms, reversed section
  axes, triangle through hexagon section rings, and cuts that refuse at a
  source vertex, tangent, exterior or uncertain topology.
- A changed, stale or serialized bounded Section sketch cannot enter any undecorated
  profile consumer as an exact source section. A forged reference-source marker
  has the same refusal, including when a different profile or chain is selected
  in the same Sketch. Ordinary Sketch profiles keep their current results.
