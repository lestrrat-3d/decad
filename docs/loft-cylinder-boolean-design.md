# Loft and coaxial cylinder Union

## Admission

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
available before private reconstruction. `loftcylinder.PlacedToothShape` and
`SourceIdentitiesMatch` are those cheap source checks, and
`loftcylinder.LocalBlank` and `LocalTooth` check the exact maps and rebuild
the local copies. A reconstruction or fixed work-limit
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

## Constructed shell

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
ceiling and all exact pair classifications are unchanged. In the real
17-tooth gallery build, the generic ceiling stopped tooth 16 after 67.59 s;
the structural ceiling completed Build and `VerifyAll` in 78.00 s. A
diagnostic run counted 9,466,787 scanned pairs and 3,054,048 overlapping
candidate pairs in the final 9,380-triangle join.
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

## Source-circle interval for a later cap cutter

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

## Strict interior split-ring occupancy

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

## Boundary and occupied-volume bounds

Away from the root band, the retained tooth walls have the Loft source
boundary bound and the original circular blank is unchanged. In a gap, the
ordered ring covers the exposed cylinder. At a connector, its held two-
segment slice crosses the circle once; the subchain inside the disc lies in
the charged annulus. The source-circle proof gives one occupied angular
sector per tooth; the held ring has its own exact angular ordering. On either
side, a hidden connector point reaches its same-height circle crossing by
at most the certified connector reach plus the matched Loft departure.
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
If any tooth lacks the tighter connector certificate, the join retains the
prior two-times-inward-reach bound for the complete pattern. It never mixes
the two bounds within one result.

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

## Source record for a later bore

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

## Centered through-bore Cut

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

## Both outer-cap Chamfers on the joined gear

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
listed. Such an omitted facet ends strictly outside in the exact scan. Of the
facets on one axis-parallel line only the first is listed, since it is the one
that ends the scan.

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
gets no index and the scan on that axis is the plain one. The lists cost 8
bytes per membership while they are collected (16 with slice growth) and 4
bytes once built, so the cap bounds the membership lists at about 130 MB
while collected (a facet may append up to 66 564 more before the check) and a
built index at 32 MB; the shells here list about 43 thousand per index. This
bounds the memberships only: the build also holds the facet boxes (40 bytes
per facet) and the vertex projection cache of segment facets, allocated before
the check. The count is
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
