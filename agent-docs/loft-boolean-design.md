# Loft Boolean design

One part per Loft Boolean route. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N".

## Mirror Boolean

This document owns the certified `Union` of one two-section Loft and its
`MirroredCopy` across the Loft's end section. It extends the mirror copy API in
`agent-docs/mirror-pattern-design.md` and the Loft mesh proof in
`agent-docs/loft-design.md`. The result is a faceted solid with retained source and
placement records for later certified consumers.

### Admission and ideal solid

Both operands must be live, solid, authenticated Lofts with the same source
identity, section frames, planes, alignment, and exact recorded profiles.
The lower Loft is unplaced. Its section frames use the XY basis at positive
ascending Z stations. The upper operand has exactly the transform returned by
`r3.Reflection` across the end-section frame. The copy therefore has the same
ideal source, with its Z interval reflected to the other side of the common
section. Their interiors are disjoint and their true middle caps coincide.
Independent equal-looking Lofts cannot establish this identity and refuse the
specialized join.

### Held boundary and proof

Each operand first passes `Tessellate(VerifyAll)`. Its mesh must exactly match
its authenticated Loft payload's vertices and triangles. The end cap vertices
must agree exactly after reflection, and the cap triangulations must have the
same indexed triangles with opposite winding. The join keeps the lower and
upper exterior caps and both sets of source walls. It removes the two middle
caps and identifies their common station vertices. Its three rings comprise
the lower start, common middle, and reflected upper start stations. Whole-mesh
vertex-link and triangle-crossing audits must accept the sewn shell.
The sewn shell is a banded loft whose middle ring is an interior section, so
the crossing audit takes that structure (`loftmesh.LoftCrossingAuditBanded`):
each planar outer cap, with its hole loops, is decided as a whole by
`loftmesh.CapFamilyProof`, with the middle ring on the far side of each cap's
plane, and every other pair is tested exactly. A cap whose proof fails keeps
its pairs in the exact test. The structured audit lets a source with many
stations, such as a twisted gear outline with rounded roots (loft §17), stay
below the audit's candidate ceiling; a rounded Loft keeps its source identity
and authenticated records, so its mirrored copy joins by this route.

The result's boundary displacement is at most the greater of the two source
mesh bounds, because every retained wall and cap is one of their verified
pieces. The occupied-volume symmetric difference is at most the sum of their
source allowances: union is non-expansive for symmetric difference, and the
removed caps occupy no volume. The area allowance conservatively sums both
source area allowances and charges twice the larger allowance for the removed
middle caps. Non-finite proof values refuse. The ordinary faceted builder
then proves topology, mass and bounded area and volume readings from this
record. `Tessellate(VerifyAll)` checks the final result at a caller's target
tolerance.

The `LoftMirrorJoin` record preserves both original Loft operands, separate
lower and upper wall groups, outer cap groups, and the number of vertices in
each station. Both retained outer caps keep their source origins and planar
surface, with new Union cap roles appended. A later rigid `Copy` keeps a
source-backed placement record, gives those caps copy-local roles, and
revalidates the held mesh and displacement bounds. This record does not by
itself authorize any other Boolean case.

### Through-bore of the joined pair

`Cut` accepts an unplaced, contact-audited join record of this route and one
straight prism tool. `loftmirror.ThroughBoreCut` bores the recorded lower Loft
with Loft §18's through-bore (`loftrecord.ThroughBoreCut`), places the bored
Loft under the join's recorded reflection, and joins the two by the held
boundary proof above. The tool's exact ends must lie on or beyond both outer
caps of the join: the start plane and its reflection, twice the section gap
above it (`loftrecord.ThroughToolSpans`). The lower Loft `L` lies in the slab
between its start plane and the mirror plane, and a straight tool `T` along
the Loft axis that reaches both outer caps meets that slab in the same section
prism as its reflection `F(T)`, so `(L ∪ F(L)) − T = (L − T) ∪ F(L − T)`. A
tool short of either outer cap, a placed join, and a lower Loft the
through-bore declines keep the general Boolean.
The bored halves are rebuilt by the Cut itself, so their caps already carry the
Cut's own cap roles; the sewn join drops those before it names its two outer
caps, so `CapStart` and `CapEnd` of the result each name one face, and the
bored join takes Loft §19.1's outer-cap chamfer.

The sewn shell reads each Loft's held vertex layout from its per-wall cell
record: each loop lifts its start stations, then its end stations. Every loop's
end stations are the shared middle ring, and its reflected start stations bound
the top cap, so a bored source joins with its hole loops like its outer loop,
and the banded crossing audit receives every loop of each cap.
`apitest.TestLoftMirrorJoinThroughBore` bores rounded twisted gear joins of five
and seven teeth: each removes `πr²·2h` within the two volume bounds, passes the
verified tessellation with one hole, and keeps every vertex near the axis on
the bore. `TestLoftMirrorJoinShortToolKeepsTheTop` checks that a tool short of
the top cap never removes the full cylinder.

### Scope

`Union` outside this one-source reflection rule follows the existing Boolean
dispatch. A centered-cylinder Join proves its own contacts, sections, and
occupied volume before it consumes this record.

### Centered cylinder and repeated reflected tabs

`Union` also accepts one exact centered circular prism and a mirrored Loft
pair retained by this route. Later unions may add rigid XY rotations of the
same pair, up to the circular Join's 32-source limit. A completed result keeps
the exact blank and pair operands for each next step. A placed result loses
that incremental source record.

The blank must have exact XY sections, one whole circle, and no section
displacement. Its bottom equals the lower Loft's start plane. The mirrored
pair's middle plane must be the exact dyadic midpoint of the blank's two cap
planes. Every later copy must retain the first pair's source identity and
use an axial rotation with no translation. The reflection and each admitted
rotation commute, so the true union is the reflection of a lower-half union
against the exact lower half of the blank. A rotation whose axial basis is
one float step below one uses the bounded bridge below.
`loftcylinder.MirrorBlank` checks the blank and `loftcylinder.MirrorPairMotion`
reads each pair's recorded placement. `loftcylinder.MirrorLevels` checks that
the source middle height is exactly halfway between the blank caps.
`loftcylinder.CheckMirrorPairs` checks the pairs in order and stops at the
first that fails; the lower teeth of the pairs before it are placed before
that refusal returns.

For every step, the existing Loft-to-cylinder Join rebuilds that lower-half
union from the original source Loft and the admitted rotated copies. It proves
the radial seam, pairwise source separation, held boundary, and occupied
volume. The result is reflected using the faceted placement proof. The two
held middle caps must have identical triangles with opposite winding, and
their vertices must coincide exactly. Both caps are removed. Vertex-link and
whole-mesh crossing audits check the sewn shell.

The full body's boundary departure is the larger half bound. Its occupied-
volume error is at most the sum of the two half errors because the true halves
have disjoint interiors. Its area allowance sums the two half allowances and
twice the larger allowance for the removed common cap. The held diameter plus
twice its boundary departure supplies `D` to the faceted centroid allowance
`min(D, volSymDiff × D / volumeFloor)`.
The route refuses a non-finite proof or a boundary departure above 0.1 mm;
ordinary Boolean chord admission is unchanged.

The exterior cap groups keep the original blank and every retained pair's
source origins, plus the new Join cap roles. Exact axial placements keep
both cap planes; a bridged top cap reports a faceted surface with its charged
bound. `VerifyAll` independently
checks the final held shell and occupied-volume allowance at the caller's
chosen tolerance. `loftcylinder.BuildMirrorJoin` reflects the lower half,
sews and audits the shell, and gives its exterior caps these origins and tags.

### One-pass mirrored pattern Join

Multi-body `Union` accepts the same centered prism and two to thirty-two distinct
copies of one authenticated mirrored pair. It checks the common source and
each recorded placement before any operand is consumed. The circular Join
builds the lower half once from every original lower Loft; it proves their
continuous sector separation, root contacts, cap disks, boundary and volume.
The mirrored result is sewn and audited once. This avoids repeatedly
rebuilding all prior teeth for each binary `Union`.

The admitted placed copies have zero translation and an XY rotation. Their
Z basis is either exactly one or `Nextafter(1, 0)`. For the latter, the lower
circular Join uses its authenticated axial proxy and charges the difference
between the original lower Loft and that proxy. Let `F` reflect about the
recorded middle height `m`, and let `T` be the copy's stored affine motion.
The true upper source `T(F(L))` differs from the reflected true lower source
`F(T(L))` by the exact constant Z translation `2m(1-T.EZ.Z)`. The evaluator
computes an upward rational bound for that shift. It adds the shift to all
reflected upper vertex and boundary bounds and charges the source pair's
certified area times the shift to occupied-volume difference
(`loftcylinder.MirrorUpperBridge`). The lower
circular Join's moving-plane tooth-separation test also widens each affected
tooth's margin by the upper shift times its maximum XY radius and the plane's
normal change per unit height. This keeps upper teeth apart, including the
thin level where a shifted upper tooth meets another tooth's lower half. A
fixed through-axis separator needs no height allowance. The lower
circular Join's area allowance already includes the held joined area, blank
area, and every source tooth's area. This also covers any short surface
exposed by the shifted upper tooth. The original pair's common cap moves as
one; the blank fills the small level difference. This proof does not admit an
arbitrary tilted or translated copy.

The final result retains its authenticated private lower-half Join, the
original full blank and pair bodies, and the exact bottom and top cap groups
and outline loops. Both final cap triangulations must tile those outlines;
the loops and lower-half source record support later certified consumers.
The boundary limit remains 0.1 mm, the final mesh still passes whole-shell
vertex-link and crossing audits, and the faceted builder publishes bounded
volume, area, centroid and diameter readings before the atomic commit.

### Centered through-bore

`Cut` accepts an exact centered circular prism through an unplaced, audited
reflected Loft/cylinder join. The join record retains its certified lower-half
Loft/cylinder join and the full body's exterior cap groups and outlines.
Both exterior cap disks are checked against those recorded outlines before
the Cut. The existing lower-half bore route authenticates the cutter and
proves its inner wall and annular caps. The cutter must extend beyond both
exact end planes of the full blank.

The bored lower half is reflected through the original Loft end frame. The
two middle annuli must have matching held vertices and oppositely wound
triangles; they are removed and their vertices are welded. The remaining
mesh passes vertex-link and whole-mesh crossing audits. Its boundary departure
is the greater half bound. The occupied-volume allowance sums both half
allowances, and the area allowance also charges the removed middle annuli.
The final held diameter and departure bound the centroid reading. Both
exterior annuli must tile the blank's exact end planes. The result keeps the
blank's cap origins and receives Cut cap roles; `VerifyAll` checks its boundary
and occupied-volume allowances at the requested tolerance.
`loftcylinder.BuildMirrorBore` reflects, sews, audits and tags the bored shell.

For a near-axis placed pair, the Join record carries its upper source
translation bound and swept-volume charge into the Cut. The reflected upper
vertex and mesh bounds include that translation, and the bored top cap retains
the faceted surface tag when the original upper tooth ends below it.

### Both exterior cap Chamfers after a centered bore

An unplaced, audited mirrored Join with its certified centered through-bore
accepts one equal setback on both complete exterior outer cap loops. The inner
bore loops and all side edges stay sharp. The private bored lower half gets
one start-cap Chamfer. Its opposite cap and middle annulus remain unchanged;
the audited half is then reflected, its matching middle annuli are removed,
and the common middle vertices are welded. Separate cap and wall audits prove
the two exterior annuli, unchanged bore wall, and strict separation of the
lower and upper slabs outside their shared seam.

The lower-half Chamfer is `loftcylinder.ChamferBottomCap`, the two-cap
builder of the loft-cylinder design run with its top cap inactive.
The one-cap cutter uses exact side and inset loops, three source-occupied
split rings, and exact full-band distance checks. Its band can cross the
side-level ring in XY when the three-dimensional contact audit proves the
band touches the old held wall only on the intended side ring. The source
cap and retained side audits still prove both directions of boundary reach.
The reflection uses the placed-mesh rounding envelope
(`loftcylinder.MirrorChamferReflectionReserve`), checked against the
measured upper-half bound. The final boundary allowance reserves this rounding
before building the lower half.

For a near-axis placed pair, the actual upper source tooth lies below its
reflected lower proof tooth by the Join's authenticated axial shift. The
three lower cap-band split rings are checked again against each tooth at its
shifted physical source level. The circular Join sorts lower teeth by angle,
so `loftcylinder.MirrorChamferUpperShifts` matches each shift to its
authenticated placed tooth source instead of its pair input position. An
exact section through the reflected upper band at the lowest possible source
cap proves the intervening cap sliver's
reach. The measured vertical and lateral reach is charged to the upper
vertex and boundary bounds. The Join's swept-volume charge is carried into
the final occupied-volume allowance. Both charges must fit the caller's
0.1 mm `VerifyAll` tolerance.

`loftcylinder.AssembleMirrorChamfer` sews the two halves from their payloads
and the bore record. It runs the seam, vertex-link, exterior annulus and
bore-wall audits and the upper cap section, then proves the result's bounds.
The boundary bound is the larger of the lower half's bound and the upper
half's bound plus the measured cap reach; a bound above 0.1 mm refuses. The
occupied-volume allowance sums both half allowances and the Join's
swept-volume charge. The area allowance sums both half allowances and twice
the larger one for the removed middle annuli. The held diameter and boundary
bound give the centroid allowance. `loftcylinder.BuildMirrorCapChamfer`
reflects the lower half within its document, gives the exterior cap groups the
bored Join's cap origins, and has `loftcylinder.TagCaps` tag them with the
blank's planar cap surfaces.

### One-pass Loft pattern Join

Multi-body `Union` also accepts a centered circular prism followed by distinct
copies of authenticated Lofts, such as the twisted teeth of a circular pattern.
`loftcylinder.UnionTeeth` certifies the blank and every tooth in one circular
Join, the same certificate the binary `Union` extends one tooth at a time. It
runs after the mirrored-pair Join and before the other many-operand routes. A
set it does not admit, or whose boundary departure exceeds 0.1 mm, goes on to
the next route.

## Cylinder Boolean

### Admission

`Union` dispatches to `tryLoftCylinderUnion` for one exact circular prism
and one or more axial Loft copies. The circular prism has one centered circle,
an XY frame, and no section displacement. The Loft copies share one
authenticated source identity. Each copy has two flush XY sections, an axial
rotation without translation, and six ordered outer curves: line, fitted
spline, arc, fitted spline, line, centered root arc. The original admission
arm accepts a prior joined body only at identity placement. Public placement
clears that body's source record.

A second admission arm accepts a common proper signed-axis rigid placement.
It derives the pose from the blank's recorded frame and placement, then
compares their exact rational affine composition with the stored pose. For
each tooth, the two recorded source planes under its placement must equal the
two canonical XY planes under one local axial rotation and that same pose,
as exact rational affine maps. A rounded composition that changes this
equality refuses this arm. The pose may carry a finite translation; a large
one still refuses when the placed result exceeds the 0.1 mm boundary limit.
Other rotations and unequal operand poses continue to the generic Boolean.
In particular, `r3.Rotation` uses rounded sine and cosine. An ordinary
90-degree call can carry a nonzero cosine entry and does not meet the exact
signed-axis gate. The native YZ frame with U=+Y and V=+Z does meet it.
Even with signed axes, a native plane height or accumulated translation whose
float composition changes the exact rational source map misses this arm.

The arm re-evaluates private canonical copies from each operand's recorded
profile with the operand's producer identity and authenticated Loft source
identity. It runs the unchanged local cylinder/Loft certificate, including
the cap, crossing, separation, and occupied-volume audits. `PlaceMesh` maps
the audited shell under the common pose and charges each vertex's coordinate
rounding, boundary bound, volume allowance, and area allowance. The placed
result must still meet the 0.1 mm boundary limit. Its source record names the
actual blank and teeth plus the private local proof bodies and pose. A later
tooth join rebuilds the local certificate from those proof bodies and an
exactly matched new tooth. Ordinary `Transform` or `Copy` of the joined result
clears the record as before.
The source record keeps private local source bodies, not a second joined mesh.
A later centered bore rebuilds the canonical join from those sources only
after its cheap source and pose checks pass.
Nonmatching source shapes and exact-map mismatches leave the generic Boolean
available before private reconstruction. A Union first runs
`loftcylinder.PlacedToothShape` on the new tooth and, for a continuation,
`SourceIdentitiesMatch` on all teeth. `loftcylinder.LocalBlank` checks the
blank's circle shape and pose, and `LocalTooth` checks the tooth's recorded
frames and planes and its exact maps, each before rebuilding its local copy.
A bore runs `SourceIdentitiesMatch` on the recorded teeth and its tool checks
before rebuilding the join. A reconstruction or fixed work-limit
error after those checks is returned, as in the original certified path;
context cancellation is returned unchanged.

When a placed tooth's Z basis is exactly the float immediately below one,
`loftcylinder.ProofTooth` may build a private axial proxy with Z basis one.
Admission requires zero translation, zero XY/Z mixing, a positive blank
height starting at zero, and bit-identical XY basis and held vertex positions.
The proxy uses the original tooth's authenticated sections and source identity.
The original placed tooth remains the exact Boolean operand. If `ε` bounds
the axial distance between corresponding original and proxy source points
and held vertices, the proxy Loft proof adds `ε` to its boundary departure,
`ε·Aupper` to occupied-volume difference, and `(1−s)·Aupper` to area slack,
where `s` is the original Z basis and `Aupper` bounds the proxy surface area.
Axial compression from proxy to original cannot increase surface area during
the sweep, so `ε·Aupper` bounds the swept symmetric difference.

The first and last lines run from the root arc to the flanks. Each has one
Loft cell and natural `0..1` parameters. Their root endpoints equal the
recorded root-arc endpoints. The far endpoint ruling of each line has a
certified minimum projected radius greater than the blank radius plus the
Loft's source-to-held boundary bound. A `LineSeg` Loft wall is its two held
triangles. At a fixed height, those triangles give a two-segment chain from
root endpoint through their common diagonal to flank endpoint. The exact
quadratic radial derivatives on both segments are nonnegative for every
height. An exact root-ruling collar proves the held inner endpoint lies
inside the blank between both cap collars. The far endpoint stays outside,
so that chain crosses the circle once in this interval.

The root arc may have a small radius excess at either cap. Its endpoint
angle intervals must then prove a nonzero twist with one common sign. For
every arc parameter, squared radius along a ruling is convex in height.
`loftcylinder.RootCollar` proves both interior endpoints at `ε` and `1−ε`
lie inside the circle. The root wall is inside the blank between the two
`ε·height` cap collars. The collar, its lateral root-wall sweep, and radial
gap are charged. Both radius readings multiply the recorded squared radius
by the placed body's exact squared XY rotation-basis norm, including its
float-basis residual.

`loftcylinder.RadialGate` computes the exact minimum projected radius of
every held wall triangle. Fitted flank and tip cells must stay outside the
circle after the Loft source bound. Root stations must lie entirely in their
charged radial band. The two connector cells and root cells give the maximum
inward reach of the seam. `loftcylinder.WedgeGate` evaluates the quadratic
cross product of every held triangle against both moving root endpoint rays.
Its Bernstein coefficients bound the whole facet, including the first tooth's
cap facets. The Loft source bound is added to any lateral wedge departure.
`loftcylinder.Separate` orders the teeth by their bottom sector and proves
every pair has a through-axis separator with a margin greater than both
source bounds. The fixed-plane arm checks each held vertex. If it fails,
the moving-plane arm chooses one normal at each cap and interpolates the
normal linearly in height. Its dot product with position is quadratic on
each held triangle. All six exact Bernstein coefficients must lie strictly
on the pair's required sides after the maximum endpoint-normal length times
the authenticated Loft boundary departure is charged. The matched Loft wall
correspondence preserves height, so the margin separates each true wall at
the same height. Both cap contours and the filled sections lie in their
respective open halfplanes because a halfplane is convex. At most eight
million exact coefficients may be checked by this arm per Union; exhaustion
refuses the structural route. The existing cap gap check still applies.

The fixed separator depends only on XY and applies unchanged to a bridged
source. For a moving separator, `N(z)` changes between a proxy point at `z`
and its original point at `s·z`. Its strict margin also charges
`|N1−N0|·Rxy_upper·ε/height`, where `Rxy_upper` encloses proxy source XY.
Thus every original and proxy tooth stays on its assigned side at its own
physical height. Pairwise separation keeps other teeth from hiding matched
boundary points during the axial bridge.

### Constructed shell

`loftcylinder.Join` builds and proves the joined shell, and
`loftcylinder.TagCaps` tags its caps; root keeps both verified meshes. The join
retains each tooth's exterior Loft wall facets, removes its root wall
facets, and fills the gaps with a ring on the circular blank. Every
ring point encloses its true angle with `Atan2Interval` and its trigonometric
coordinate with `RadSinCosInterval`. A quadratic Bernstein test proves each
adjacent pair of ring rulings keeps positive angular order at every height.
The exact projected radius of each ring triangle stays in the charged band.
Continuity then makes each angular ray in an exposed gap meet the ring.

The two planar caps use the ordered tooth exterior paths and ring paths.
The exact cap audit checks that each outline is simple, counterclockwise,
contains the origin, and is tiled by only its cap-group triangles with
matching directed edges and exact area. `LoftCrossingAudit` checks the entire
constructed shell before the payload records `ContactAudited=true`. The
faceted-body audit then checks manifold closure and orientation.
The structural join uses a 16-million candidate-pair scan ceiling in this
audit; other callers keep the generic eight-million ceiling. The triangle
ceiling and all exact pair classifications are unchanged. A full structural
ring join scans more candidate pairs than the generic eight-million ceiling
allows; `AuditMaxScans` in `internal/loftcylinder/xy.go` owns the value.
An exact midpoint and outline-contact audit keeps every omitted held root
chord inside its filled cap; only the two global root endpoints may touch
the outline.

After those audits, each cap group receives the joined body's own `capStart`
or `capEnd` role and the original blank's exact axial `Plane` surface. The
other groups stay `Faceted`. Public `Planar` and `NormalTo(z)` select the two
caps, and `OuterLoopOf(CapStart(result))` and `OuterLoopOf(CapEnd(result))`
select their exterior rims.
If any tooth uses an axial bridge, the top group keeps `capEnd` but uses a
bounded `Faceted` surface: the held cap is at the blank's top plane while
that tooth's true exterior top ends as much as `ε` below it. The bottom
group still has the blank's exact `Plane`. Analytic `Planar` and
`NormalTo(z)` therefore select only the bottom cap in this case; the cap
roles still select both exterior rims.

### Source-circle interval for a later cap cutter

`loftcylinder.SourceCircleOf` accepts one authenticated source Loft retained by
the join or its centered bore. The six-curve admission and matched Loft facet
departure still apply. The root wall stays strictly inside the exact blank
circle at open heights between its certified cap collars. When the existing
collar has zero height, the exact squared root radii at both caps are at
most `R²`; one strict cap radius makes every interior ruling strict by
convexity. If both equal `R²`, a certified nonzero angular change makes
every interior ruling strict instead. The cap planes themselves receive no
circle-occupancy claim.

Every fitted and tip wall stays outside `R` because its held facet minimum
exceeds `R` by more than the matched Loft departure. Each source `LineSeg`
connector uses the same two-triangle diagonal as the Loft wall. The helper
reconstructs its exact source corners from the recorded endpoints, section
frames, and axial placement. Nonnegative quadratic Bernstein coefficients
for both horizontal slice pieces prove that squared radius increases from
the root endpoint to the far endpoint at every height. The root endpoint
is inside `R` at open admitted heights; the far endpoint's exact ruling
minimum exceeds `R`. Each connector therefore crosses the circle once,
transversely. The two crossings are the only source-tooth boundary points
on that circle.

A positive half-plane contains every held tooth vertex by more than the
matched boundary departure. The whole bounded source solid lies in that
half-plane, so the opposite circle ray is exterior. The circle interval
between its two ordered connector crossings is occupied by the tooth;
the opposite interval is exterior. `CrossingBox` brackets both crossings
over any rational axial subinterval with exact endpoint and minimum-radius
tests. It returns only if the two XY boxes preserve positive angular order
and remain in the certified half-plane. A caller may subdivide an interval
that refuses, but it must bound its own subdivision work.

`ContainsTriangleSector` takes an exact triangle and an axial interval that
contains all three vertices. It uses the crossing boxes for that entire
interval. Each projected vertex must lie strictly counterclockwise of every
possible first crossing and strictly clockwise of every possible second
crossing, using exact cross-product lower bounds. Those two fixed halfplanes
are convex, so the whole triangle lies in the angular sector. A failed
strict inequality refuses the triangle. This gate checks only its angle;
radial tooth occupancy and source-wall clearance need separate checks.

This certificate applies only at open heights between the recorded
`safeBottom` and `safeTop`. A cap cutter must separately prove its collars,
cap-plane, bore-wall, and full far-band surface claims. Circle occupancy by
itself does not place a complete cutter facet inside the source union.

### Strict interior split-ring occupancy

The split-ring reader in `internal/loftcylinder` (`ring_inside.go`) accepts
unordered exact edges at one height strictly between the blank's caps and
below every tooth's certified top root collar. It checks exact endpoint
equality, nonzero edges, degree two at every vertex, and one connected
cycle. The entire ring stays outside a centered bore's exact radius. At
least one ring vertex lies strictly inside the blank circle, which gives the
connected ring an occupied witness.

For an admitted one-ULP axial bridge, the ring's physical height is mapped
to `z/s` for each private proof tooth separately. Its XY coordinates stay
exactly the same. The proxy's wall departure includes the original-to-proxy
bridge; its source-circle crossing and connector gates use the mapped
height. The exact original top cap must remain above the physical ring,
and the mapped height must remain between that tooth's root collars.

An edge wholly inside the circle is inside the exact blank. An edge wholly
outside it must clear every exposed source tooth wall. The source-circle
certificate puts the terminal root wall strictly inside the blank at this
height, so that hidden wall cannot bound a wholly outer edge. Both connector
walls, the fitted flanks, and the tip remain checked. The helper measures
its exact minimum distance to every relevant held Loft wall triangle and
requires a strict gap beyond that tooth's parameter-matched wall bound.
The `matchedDelta` and cell twist terms in Loft §5.2 pair every point of a
held wall patch with a point of its true wall patch at the same parameters,
and conversely. Thus that bound excludes a true wall near the edge in
both directions; a one-way held-to-source reading would not suffice.
Expanded coordinate boxes skip only triangles that cannot approach the
edge by the bound. Exact segment-to-triangle distances decide the rest.

For an edge that touches or crosses the blank circle, the whole edge must
lie strictly between one tooth's two source connector crossing rays.
The exact source connector slices are checked for any contact with that
edge, including endpoint touch or collinear overlap. Every tooth's fitted
and tip walls must clear the edge by its matched wall bound. The source
circle record proves each root wall stays inside the blank at this height,
so those hidden walls cannot form the union boundary. An exact whole-edge
radius minimum classifies edges; endpoint radii alone do not.

Cap Chamfer checks one split ring twice, directly and at the reflected tooth
levels, over the same edges. A `RingEdgeRecords` value holds each edge's exact
XY ends, its class (inside, outside, seam, or reaching the bore) and its XY box,
built when the edge is first read. The second check reuses a record only for
the same edge points and the same blank and bore radii; any other input
rebuilds it. The class comparisons and the wall plane test compare exact
fractions by cross-multiplication without cancelling common factors, so every
comparison returns the sign the lowest-terms values give.

The circle sector claim makes every circle point on a crossing edge
strictly occupied by the tooth. No edge touches any source tooth wall or
an exposed blank or bore wall. Since the exact edge set is one connected
cycle with a strict blank-inside witness, every point on the ring lies
inside the original union. This proof covers the split ring only. The cap
planes, both collars, and the cutter band's two-dimensional surfaces
need separate bounds and occupancy proofs.

### Boundary and occupied-volume bounds

Away from the root band, the retained tooth walls have the Loft source
boundary bound and the original circular blank is unchanged. In a gap, the
ordered ring covers the exposed cylinder. At a connector, its held two-
segment slice crosses the circle once; the subchain inside the disc lies in
the charged annulus. The source-circle proof gives one occupied angular
sector per tooth; the held ring has its own exact angular ordering. On either
side, a hidden connector point reaches its same-height circle crossing by
at most the certified connector reach plus the matched Loft departure.
This reach claims the crossing on the slice's first piece. At the cap where
that piece has zero length, its far end is the root endpoint, which the
section records on the circle; float rounding puts it inside or outside.
When it is inside, the crossing lies on the second piece at heights just
above that cap. `loftcylinder.ConnectorReach` therefore widens the tooth's
cap collar. It tries dyadic fractions `2⁻⁵²` through `2⁻⁸` of the height
in increasing order, starting from the tooth's existing collar. It keeps
the first collar at whose two interior heights every source and held
connector starts strictly inside the circle and its far end stays strictly
outside it. Squared radius is convex along each root ruling, so its two
strict endpoint tests cover every height between the collars. The far end
moves along one segment between those heights, and the test reads that
segment's exact minimum radius. The join charges the wider collar like
the root collar below. If no fraction settles the endpoints, the tooth
lacks the connector certificate.
The exact radial derivative also bounds the shift between source and held
crossings: the matched point lies within `δ` of the crossing, so its squared
radius differs from `R²` by at most `2Rδ+δ²`. Dividing by the certified
radial derivative and multiplying by the connector piece length bounds the
remaining shift. The cap's vanishing axial factor cancels in this ratio.
An exposed source-cylinder point between a held root ray and its connector
crossing reaches the retained held connector within the charged inward reach.
Between the held and source crossings, it reaches the held crossing within
the certified crossing shift. A held ring point inside the
source tooth's sector first projects radially to `R`, then moves to the source
crossing. The ring begins at the held root ray, so this second leg includes
the exact held-to-source root endpoint displacement and the true connector's
tangential motion before its crossing. The latter uses a cubic Bernstein
test for the same-height radial deficit and cross product; its height and
connector parameters stay paired. A strict bound below one quarter of the
radius keeps each angular move on the short circle arc. The source-circle
crossing boxes are checked over 256 contiguous exact height slabs, so their
positive cyclic order holds throughout the admitted interval. Outside these
narrow disagreements, same-angle projection pairs the ring and cylinder. A point
inside either root collar first uses the charged radial seam reach to the
matching cap region. The audited simple cap outline follows the tooth
exterior and gap ring. Every omitted held root chord, including the closing
chord, has an exact midpoint strictly inside that polygon. It has no outline
contact except at the two global root endpoints, so the entire chord lies
in the filled cap. The true root arc lies within its matched Loft departure
`δ` of the chord.
Its XY ruling moves by at most twice the maximum authenticated source radius
times `collar/height`; the axial move is at most the collar height. The
published boundary bound includes the Loft source bound, the maximum of
connector reach plus departure and radial ring reach plus root-angle reach,
outward root excess, one cap collar height, and its XY sweep. A result above
0.1 mm refuses.
If any tooth lacks the connector certificate, the join uses the
two-times-inward-reach bound for the complete pattern. That bound uses no
connector certificate, so when every tooth has one the join uses the smaller
of the two complete-pattern seam reaches. It never mixes the two bounds
across teeth within one result.

For volume, let `B` be the exact blank and `T` the union of exact teeth.
Replacing each true tooth by its held Loft changes occupied volume by at most
the sum of its authenticated `volSymDiff` values; union with `B` cannot
increase that symmetric difference. Outside the charged radial annulus, the
retained wall facets and cap exterior contours agree with those of the held
teeth. The omitted root facets, changed cylinder facets, and their cap seams
are entirely inside the annulus. The exact cap tiling and crossing audits
exclude an extra cap region or folded shell outside it. Charge the entire
annular slab, `π·(2R(δin+δout)+δout²)·height`, as an upper bound on the
remaining occupied difference. The blank mesh's own symmetric-difference
bound is added conservatively.

The faceted result uses the held mesh's coordinate-box diagonal plus twice
its two-way boundary bound as an upper diameter for the centroid reading.
That diameter multiplies the occupied-volume difference in the faceted
centroid allowance. A non-finite diameter refuses the join.

The axial bridge also transfers the constructed shell's two-way boundary
bound to the exact Union. Outside the blank cylinder, each tooth boundary
point maps to its original or proxy at identical XY and within `ε` in Z.
Inside the cylinder, the blank occupies every height from zero to its top
and neither tooth extends beyond those levels. On the cylinder side, a
tooth's exposed-height set maps by `z ↔ s·z`; the original's unmatched
top slab reaches the blank's top boundary within `ε`. The strict tooth
separators, including the moving-normal bridge margin, prevent another
tooth from hiding these matched points. The constructed-shell seam bound
then adds the proxy's enlarged Loft departure once, rather than treating
the proxy as an exact input.

### Source record for a later bore

The payload records the original blank and ordered teeth, both exact cap
outlines and their group IDs, and the exact minimum projected radius of every
non-cap held triangle. It also records the upward-rounded inward and outward
radial seam reaches and the symmetric cap collar height used by the boundary
and volume proofs. Those reaches enclose the replaced root seam; retained
tooth exterior walls use their Loft source bounds. A centered bore must remain inside the
exact blank, below the side clearance after the joined boundary bound, and
inside the seam annulus's inner radius. The cap audit proves each recorded
outline is one disk containing the axis. Placement clears the record, so a
later Boolean cannot rebuild from unplaced operands.
The common-placement arm stores its local source bodies alongside the actual
source identities. It does not retain the local joined mesh. This record
exists only on the result built by that arm; public placement still clears it.
It also stores the private proxy teeth and each axial bridge distance. A
later Union rebuilds proofs from the original teeth. A centered through-bore
copies both tooth lists and the bridge distances into its own source record.
Cap Chamfer checks each original/proxy pair again. Its split-ring reader keeps
the blank and bore at physical height `z`, and reads a proxy tooth at exact
height `z/s` for the recorded one-ULP axial scale `s`. The proxy's Loft wall
bound includes the bridge to the original exact source. The top-cap proof
checks a rational section of the bevel band below every original tooth cap.

### Centered through-bore Cut

`Cut` accepts an unplaced, audited join record and one centered circular
prism whose exact axial ends extend past both blank caps. The tool has an XY
frame, one whole-circle section, and no section displacement. Its radius is
smaller than the exact blank radius. `loftcylinder.BoreCut` admits the tool
and builds the bore; its tool-profile and through-span checks are
`loftcylinder.BoreTool` and `BoreThrough`. The exact cap audit runs again before
either disk is replaced. The held bore radius, its certified polygon error,
and the joined boundary bound must together stay below the recorded minimum
projected radius of every retained side triangle.

The bore wall uses 256 ordered circular stations. Interval trigonometry
bounds each rounded station, and the angular step bounds the chord sagitta.
An exact planar check proves the inner ring is simple, counterclockwise, and
contains the axis. Each audited cap disk becomes an annulus with that ring
as its hole. The new inward wall keeps the tool's cylinder face provenance.
The final mesh passes orientation, vertex-link, and exact crossing audits.
This specialized bore uses the structural join's 16-million scan ceiling;
other `LoftCrossingAudit` callers keep their generic eight-million ceiling.
The exact pair classifications and triangle ceiling are unchanged. The real
17-tooth, 1 mm centered bore audited 10,408 triangles, 11,281,630 box scans,
and 3,555,085 overlapping candidate pairs in a diagnostic run.
An exact cap audit also checks that each cap group's triangles have the
area of its outer polygon less its inner polygon, and that their directed
edges pair internally and match both recorded boundary loops. The result
gets new `capStart` and `capEnd` roles and the blank's axial `Plane` tags
only after this audit. Each cap has an inner bore loop; `OuterLoopOf` excludes
its edges.
If the joined source carried an axial bridge, the bored top cap keeps
`capEnd` but uses a bounded `Faceted` surface. Its held triangles lie at
the blank's top plane while an original tooth ends below it. The bottom
annulus keeps its exact `Plane` tag.

Cutting the same exact cylinder from the true and held joined solids cannot
increase their occupied-volume difference. The polygonal bore differs from
that cylinder only in the certified radial shell around its wall. The result
adds the whole shell volume, `4π·radius·boreBound·height`, to the join's
occupied-volume bound. The result carries the larger of the join and bore
boundary bounds. Its held vertex box and that resulting boundary bound
recompute the centroid diameter after the bore mesh is built. It replaces
the join source record with the certified outer and inner cap loops, their
facet groups, the bore-wall group, the original blank and tooth sources,
the private proof teeth and bridge distances, and the unchanged seam and
collar bounds. The strict bore-to-blank and bore-to-side
clearances place its exact cylinder inside the common blank of both the
original and proxy unions. Thus the unchanged outer boundary keeps the joined
bound, while the new bore wall and inner annular rims use `boreBound`.
Placement clears this post-bore record.

For a common-placement join, `Cut` first checks that the tool's recorded
circle, radius, axial span, plane and placement match a local centered bore
under the same exact pose, running the same `BoreTool` and `BoreThrough`
checks. An ineligible tool reaches generic Cut before
either private body is rebuilt. For an eligible tool, `Cut` rebuilds the
canonical join from the retained local sources and then builds a private tool
with the actual tool's producer identity. Reconstruction errors propagate.
The existing bore audits and bounds run in local coordinates; `PlaceMesh`
charges the final rigid placement, and the result must still meet the 0.1 mm
boundary limit. The placed bore's face origins name the actual blank, teeth,
and tool. The placed bore does not gain an unplaced chamfer source record.

The bore record also retains the upward `BoreBound` used in Cut's wall proof.
That value bounds the two-way, same-height correspondence between the exact
bore cylinder and its held wall. A later outer-cap edit must recheck Cut's
strict bore-to-blank clearance before using this local bound.

### Both outer-cap Chamfers on the joined gear

`Chamfer` accepts only an identity-placed, audited Union record or its
certified centered-bore Cut, with both complete outer cap loops selected and
one equal setback. Partial loops, mixed edges, asymmetric setbacks, and other
faceted receivers still refuse. The cap loops in the immutable record must
again tile exactly one disk or annulus each. The side must still be the
recorded paired two-triangle strips. All selected public edges must cover
the two recorded outer walks and no inner bore edge.

The setback and loop admission (`ChamferSource.BindCapSetback`,
`ChamferSelectsOuterLoops`) and the build (`ChamferBothCaps`) live in
`internal/loftcylinder` over topology nodes. Root reads the receiver's cap
refs and maps the result node to its handle.

This narrow route extends Loft §18's held-polygon cap semantics. It constructs
one rational polyhedral retained solid `C` from the certified held outer
loops and source-ordered offset samples. The exact modified source is
`S∩C`, where `S` is the original blank-and-teeth union after its optional
centered bore. The construction uses the same `C` for the held solid `H`.
The inset-sampling bound records deviation from the requested held-polygon
offset; it does not switch to a smooth-spline offset. Other Chamfer routes
keep their ordinary analytic semantics.

At each cap, the old side strips are clipped at the exact rational level
`z0+d` or `z1−d`. An inset cap, its source-ordered bevel strip, and the
retained middle side form the ideal shell. The builder checks the side and
inset loops for exact simplicity, nesting, and nonzero edges. Directed edge
and exact area audits prove cap tiling, including the unchanged bore hole.
The bevel bands are connected and embedded. Exact band-to-old-wall contacts
are confined to their intended side-ring edges; an interior witness then
places the bands inside `H`. The ideal shell has outward orientation and
no unrecorded self-contact. Its retained middle facets are exact subsets
of old wall facets, and the inset caps lie inside the old held cap loops.
Those checks give `C⊂H` and make the ideal held result exactly `H∩C=C`.

A complete rational split ring through each bevel band lies strictly inside
`S` by the source-ring certificate above. One near-band proof builds each
tooth's ring record, its source circle and wall records, once and reads it
for every split ring it checks, the reflected checks included. One ring
check builds each tooth's crossing boxes and connector slices at the ring
level once for all its seam edges, on the first seam edge that reads them.
Each is a function of the immutable tooth, the blank radius and the level
alone. Each ideal band facet has a
certified distance below the published boundary allowance to that ring.
Since the ring is also on `∂C`, every band point has a nearby point on
`∂(S∩C)`. The cap proof separates the exact blank disk, a narrow root-seam
annulus, and the exterior source tooth caps. The inset stays farther than
the joined boundary bound from the held outer walk; every source tooth's
exterior cap chord belongs to that walk and has no greater Loft departure.
The exterior inset walk is inside the matched source tooth cap by exact
contour clearance and winding. Monotone radial runs through the thin top
root slit reach an inset/blank-circle crossing; the certified run length
plus slit width is charged. The bore remains inside the blank and clear of
all new outer geometry.

Every retained middle-side facet uses a same-height inward patch shorter
than the remaining boundary allowance. An exact prism contact audit proves
the path from that facet to the patch stays in `H=C` in the open middle slab.
The patch is inside the original source tooth when it clears every held
tooth wall by its two-sided Loft departure and has a strict held-tooth
inside witness. Near a root wall, the patch instead clears every nonroot
wall; the true root stays inside the blank between its certified collars, and
each held root triangle clipped to the patch's height slab is strictly
inside the blank too. The convex blank disk contains their matched wall
motion. Every patch component outside the disk reaches an outside patch
vertex, whose strict held-tooth parity transfers through the Loft boundary
homotopy. The portion inside the disk belongs to the exact blank. Thus a
middle facet either already borders `S∩C` or reaches its boundary along
that certified inward path. The unchanged bore wall and inner cap loop use
Cut's local `boreBound` correspondence and exact outer-band separation.
The source-circle proof depends only on the immutable tooth and blank radius.
The middle-side audit retains each successful proof for later facets of that
tooth; every facet still runs its own wall, height, and inward-patch checks.
It also keeps, per tooth, the z interval spanned by the height slabs whose
clipped held root triangles it has proved inside the blank disk, adding a slab
only when it overlaps that interval. A later slab inside the interval needs no
new clip: each clipped part is a subset of the parts already proved inside the
disk. A gap between proved slabs is never covered, so a slab spanning it runs
the exact clip.
The prism witness against the old shell and the held-tooth witnesses scan
only the facets that a per-axis grid over the projection plane lists for the
query's cell. A facet is omitted from a cell only if its closed projection,
widened by the margin, is disjoint from the cell, and the margin exceeds the
maximum float error of the placement arithmetic, so float placement can only
add facets to a list. For a facet projecting to a segment the whole line is
listed. A facet omitted from a cell because its widened projection is disjoint
from it ends strictly outside in the exact scan. A later slanted-segment facet
with exactly the same projected endpoints is omitted because the first such
facet has the same exact classification for every query; endpoint order and
signed zero do not change that key. Different segments on the same slanted
supporting line remain listed. Of the facets on one axis-parallel line only the
first is listed, since it is the one that ends the scan. The membership cap
counts the remaining placements after these duplicate skips; when that count
exceeds the cap, the axis uses the plain scan. Every skipped duplicate keeps
its original facet position, so exact scanning still polls cancellation on the
plain scan's schedule.

The placement runs under a range guard. Each projected vertex coordinate is
zero or between `2⁻³⁰⁰` and `2²⁴⁰` in magnitude; each coordinate difference
and run offset is zero or between `2⁻³⁰⁰` and `2²⁵⁰`; each slope and product
is between `2⁻⁶⁰⁰` and `2⁵⁰⁰`, a slope being zero exactly when its
difference is zero and a product exactly when a factor is. A mesh with any
quantity outside the guard gets no index on that swept axis, and the scan on
that axis is the plain all-facets scan, so the guard can only choose the
exact scan. Inside it no multiplication or division underflows or overflows
and a subtraction with a subnormal result is exact, so every operation has a
relative error of at most `2⁻⁵³` and the computed slope has the exact slope's
sign. On a line through `(a, b)` with float slope `s`, the margin is
`1e-12·K + 2⁻¹⁰⁰⁰` with `K = |b| + (|x|+|a|)·(1+|s|)`. The computed height is
within `8·2⁻⁵³·K` of the exact one, and rounding a band end adds at most
`2⁻⁵³` of its magnitude plus of the margin, so the total is below
`9.5·2⁻⁵³·K` plus `2⁻⁵³` of the margin. `K` is zero or at least `2⁻³⁰⁰`, so
`1e-12·K` is a normal float. The computed margin is within a relative
`7·2⁻⁵³` of the real `1e-12·K + 2⁻¹⁰⁰⁰` (four roundings in the computed `K`,
two more for the product and the floor sum), so it is at least
`(1-7·2⁻⁵³)·1e-12·K`, about 900 times the total; the floor only serves the
case `K = 0`. Every cell strip is also widened by one
float step on each side (`2⁻²⁹⁹` around an edge of magnitude at most `2⁻³⁰⁰`),
because the query's coordinate is the nearest float to an exact value that
may lie just outside a cell edge. The grid is built once per parity mesh and
decides no sign: the exact scan visits the listed facets in ascending order
and polls the context at every multiple of 256 in original facet position,
as the plain scan does.

An axis whose placement would list more than `2²³` (cell, facet) memberships
gets no index and the scan on that axis is the plain one. The placement
collects each facet's cells as runs, one per grid column (one per grid row for
a steep segment), and the runs are expanded in ascending facet order into a
counting sort, so the lists are those one append per membership would build. A
run costs 8 bytes and lists at least one membership, the run slice is
allocated once at its final bound, and a facet adds at most 258 runs before the
check, so the cap bounds the collected runs at about 64 MiB; the built index
costs 4 bytes per membership, at most 32 MiB. The shells here list about 43
thousand per index. This bounds the memberships only: the build also holds the
facet boxes (40 bytes per facet), the facet kinds and run offsets, two per-cell
count arrays and the vertex projection cache of segment facets. The count is
checked after every facet, before any offset is summed, so the 32-bit offsets
cannot overflow, and an index of exactly the cap is built.
An outward-rounded box gap can omit a held wall triangle only when it strictly
exceeds that tooth's matched Loft departure; all remaining pairs use the exact
triangle clearance check.

For the opposite boundary direction, any point of `∂(S∩C)` inside `C`
belongs to `∂S`; the original two-way joined or bored boundary bound takes
it to `∂H`. Since the containment audit proves `C⊂H`, that matched point
cannot lie in the interior of `C`. The segment to it reaches `∂C` within the
old bound. Points already on `∂C` need no transfer. This closed-solid argument
does not need a crossing-angle rim bound; the exact band/old-wall contact
audit still proves cutter containment. Intersecting both `S` and `H`
with one fixed `C` cannot increase their occupied symmetric difference,
so the old `volSymDiff` remains valid for the ideal shell. The actual mesh
rounds only certified clip vertices. Both ideal and rounded shells are
closed and embedded with matching triangle indices; their vertex-linear
surface motion adds the explicit `SweptVolumeAllow` charge and a boundary
distance of at most the clip-rounding bound. The published boundary bound
is 0.1 mm only after every band, cap, middle-side, bore, and rounding
charge fits strictly below it.

For a one-ULP axial tooth bridge, every positive-bridge tooth has one common
exact top source level. The proxy and original retain identical XY sections,
wall indices, and source identity; the proxy's source bound includes their
axial difference. The bottom cap is unchanged. On the top band, the exact
section at the lowest original top station must be one simple loop enclosing
the inset, while the embedded two-level band prevents a pinch at any higher
section. If `ε` bounds the axial cap shift, `D` bounds matched cap-to-side XY
motion, and `d` is the setback, each section in that slab stays within
`L=(ε/d)D` of the inset. The original top-cap footprint matches the proxy
footprint in XY. The exterior-contour clearance is checked with `L` added;
the top cap and band receive an explicit reverse reach `ε+2L`, and the root
slit's forward reach adds the same amount. Each remaining source-ring query
uses its tooth's exact mapped height, which lies below that original cap.

The route has private limits of 4,096 outer vertices per rim, 4,096 inset
vertices, and 32,768 output triangles. Its exact crossing audit allows at most
the lesser of 64 million scans or 2,048 scans per output triangle.
Every proof gate refuses when its own bounded work or strict inequality
cannot be completed.

The mirrored-bore consumer privately rewrites only the lower exterior cap.
It leaves the opposite cap and upper side strips in place for an exact middle
seam with its reflection. Three exact split rings at one-quarter, one-half,
and three-quarters of the active band are checked inside the source union;
an exact distance check covers each complete band facet from those rings.
The one-cap contact audit partitions old cap facets, inherited held-wall
pairs, and changed band contacts. It checks every changed non-cap contact
exactly while the already audited unchanged pairs retain their certificate.
For near-axis mirrored copies, each split ring is checked a second time at
the corresponding original tooth's shifted physical source level. The
reflected upper band also gets the exact source-cap section proof above.
