# Loft and coaxial cylinder Union

## Admission

`Union` dispatches to `tryLoftCylinderUnion` for one exact circular prism
and one or more axial Loft copies. The circular prism has one centered circle,
an XY frame, and no section displacement. The Loft copies share one
authenticated source identity. Each copy has two flush XY sections, an axial
rotation without translation, and six ordered outer curves: line, fitted
spline, arc, fitted spline, line, centered root arc. A prior joined body is
accepted only at identity placement; placing it clears its source record.

The first and last lines run from the root arc to the flanks. Each has one
Loft cell and natural `0..1` parameters. Their root endpoints equal the
recorded root-arc endpoints. The far endpoint ruling of each line has a
certified minimum projected radius greater than the blank radius plus the
Loft's source-to-held boundary bound. A `LineSeg` Loft wall is its two held
triangles. At a fixed height, those triangles give a two-segment chain from
root endpoint through their common diagonal to flank endpoint. The exact
quadratic radial derivatives on both segments are nonnegative for every
height. An exact root-ruling collar proves the held inner endpoint lies
inside the blank below it. The far endpoint stays outside, so that chain
crosses the circle once below the collar.

The root arc's bottom squared radius is at most the blank's exact squared
radius. If its top is larger, the arc's endpoint angle intervals must prove
a nonzero twist. For every arc parameter, squared radius along a ruling is
convex in height. `loftCylinderRootCollar` proves both endpoints of the
interval from the bottom to `1−ε` lie inside the circle; the root wall is
inside the blank below the resulting `ε·height` top collar. The measured
source may have a small top radius mismatch, so the collar and radial gap
are charged instead of silently treating the surfaces as coincident. Both
readings multiply the recorded squared radius by the exact squared norm of
the placed body's XY rotation basis, including its float-basis residual.

`loftCylinderRadialGate` computes the exact minimum projected radius of
every held wall triangle. Fitted flank and tip cells must stay outside the
circle after the Loft source bound. Root stations must lie entirely in their
charged radial band. The two connector cells and root cells give the maximum
inward reach of the seam. `loftCylinderWedgeGate` evaluates the quadratic
cross product of every held triangle against both moving root endpoint rays.
Its Bernstein coefficients bound the whole facet, including the first tooth's
cap facets. The Loft source bound is added to any lateral wedge departure.
Every pair of teeth also has an exact through-axis separating plane with a
margin greater than both source bounds.

## Constructed shell

The join retains each tooth's exterior Loft wall facets, removes its root
wall facets, and fills the gaps with a ring on the circular blank. Every
ring point encloses its true angle with `Atan2Interval` and its trigonometric
coordinate with `RadSinCosInterval`. A quadratic Bernstein test proves each
adjacent pair of ring rulings keeps positive angular order at every height.
The exact projected radius of each ring triangle stays in the charged band.
Continuity then makes each angular ray in an exposed gap meet the ring.

The two planar caps use the ordered tooth exterior paths and ring paths.
The exact cap audit checks that each outline is simple, counterclockwise,
contains the origin, and is tiled by only its cap-group triangles with
matching directed edges and exact area. `LoftCrossingAudit` checks the entire
constructed shell before the payload records `contactAudited=true`. The
faceted-body audit then checks manifold closure and orientation.

After those audits, each cap group receives the joined body's own `capStart`
or `capEnd` role and the original blank's exact axial `Plane` surface. The
other groups stay `Faceted`. Public `Planar` and `NormalTo(z)` select the two
caps, and `OuterLoopOf(CapStart(result))` and `OuterLoopOf(CapEnd(result))`
select their exterior rims.

## Source-circle interval for a later cap cutter

`loftCylinderSourceCircleOf` accepts one authenticated source Loft retained by
the join or its centered bore. The six-curve admission and matched Loft facet
departure still apply. The root wall stays strictly inside the exact blank
circle at open heights below its certified top collar. When the existing
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

This certificate applies only at open heights above the start cap and below
the recorded `safeTop`. A cap cutter must separately prove its top-collar,
cap-plane, bore-wall, and full far-band surface claims. Circle occupancy by
itself does not place a complete cutter facet inside the source union.

## Strict interior split-ring occupancy

`loftCylinderSourceRingInside` accepts unordered exact edges at one height
strictly between the blank's caps and below every tooth's certified top root
collar. It checks exact endpoint equality, nonzero edges, degree two at every
vertex, and one connected cycle. The entire ring stays outside a centered
bore's exact radius. At least one ring vertex lies strictly inside the
blank circle, which gives the connected ring an occupied witness.

An edge wholly inside the circle is inside the exact blank. An edge wholly
outside it must clear every source tooth's true wall. The helper measures
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

The circle sector claim makes every circle point on a crossing edge
strictly occupied by the tooth. No edge touches any source tooth wall or
an exposed blank or bore wall. Since the exact edge set is one connected
cycle with a strict blank-inside witness, every point on the ring lies
inside the original union. This proof covers the split ring only. The cap
planes, the top collar, and the cutter band's two-dimensional surfaces
need separate bounds and occupancy proofs.

## Boundary and occupied-volume bounds

Away from the root band, the retained tooth walls have the Loft source
boundary bound and the original circular blank is unchanged. In a gap, the
ordered ring covers the exposed cylinder. At a connector, its held two-
segment slice crosses the circle once; the subchain inside the disc lies in
the charged annulus. The moving-wedge bound and ring ordering pair
both sides of the exposed seam to that connector or ring. Above the root
collar, the cap is within the charged axial collar height. The published
boundary bound includes the Loft source bound, twice the maximum certified
radial or lateral seam reach, the ring roundoff, the outward root excess, and
the top collar height. A result whose bound exceeds 0.1 mm is refused.

The root connector also supports a local bound for a later cap-band cutter.
At an exact height below both admitted top collars, its two held triangles
form the radial-monotone chain through their common diagonal. The source
connector begins inside the blank and ends outside it, so it crosses the
exact cylinder. The supplied `δ` covers the matched Loft facet departure.
Every source crossing corresponds to a held point whose radius lies in
`[R−δ,R+δ]`.
`loftCylinderConnectorContactBound` brackets that held strip with exact
rational signs. It requires an exact cutter contact segment on one piece of
the chain at the same height. The exact contact is split at held radius
`R+δ`: beyond it the matched source wall is outside the blank and keeps its
ordinary `δ` bound. For the possibly hidden portion, the helper bounds the
distance from its endpoints to the entire crossing strip and adds `δ`.
Convexity covers each portion of the full contact segment.
The helper returns only this source-seam distance. Its caller separately
charges cutter rounding and proves the cutter contact and output shell.

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

## Source record for a later bore

The payload records the original blank and ordered teeth, both exact cap
outlines and their group IDs, and the exact minimum projected radius of every
non-cap held triangle. It also records the upward-rounded inward and outward
radial seam reaches and the top collar height used by the boundary and volume
proofs. Those reaches enclose the replaced root seam; retained tooth exterior
walls use their Loft source bounds. A centered bore must remain inside the
exact blank, below the side clearance after the joined boundary bound, and
inside the seam annulus's inner radius. The cap audit proves each recorded
outline is one disk containing the axis. Placement clears the record, so a
later Boolean cannot rebuild from unplaced operands.

## Centered through-bore Cut

`Cut` accepts an unplaced, audited join record and one centered circular
prism whose exact axial ends extend past both blank caps. The tool has an XY
frame, one whole-circle section, and no section displacement. Its radius is
smaller than the exact blank radius. The exact cap audit runs again before
either disk is replaced. The held bore radius, its certified polygon error,
and the joined boundary bound must together stay below the recorded minimum
projected radius of every retained side triangle.

The bore wall uses 256 ordered circular stations. Interval trigonometry
bounds each rounded station, and the angular step bounds the chord sagitta.
An exact planar check proves the inner ring is simple, counterclockwise, and
contains the axis. Each audited cap disk becomes an annulus with that ring
as its hole. The new inward wall keeps the tool's cylinder face provenance.
The final mesh passes orientation, vertex-link, and exact crossing audits.
An exact cap audit also checks that each cap group's triangles have the
area of its outer polygon less its inner polygon, and that their directed
edges pair internally and match both recorded boundary loops. The result
gets new `capStart` and `capEnd` roles and the blank's axial `Plane` tags
only after this audit. Each cap has an inner bore loop; `OuterLoopOf` excludes
its edges.

Cutting the same exact cylinder from the true and held joined solids cannot
increase their occupied-volume difference. The polygonal bore differs from
that cylinder only in the certified radial shell around its wall. The result
adds the whole shell volume, `4π·radius·boreBound·height`, to the join's
occupied-volume bound. The result carries the larger of the join and bore
boundary bounds. It replaces the join source record with the certified outer
and inner cap loops, their facet groups, the bore-wall group, the original
blank and tooth sources, and the unchanged seam and collar bounds. Placement
clears this post-bore record.
