# Finite Revolve and trimmed point-Loft Union

## Source admission

The first public `Union` route accepts a finite, unplaced, full Revolve of one
closed Sketch section and one point-Loft sector after authenticated toe Cut
and heel Intersect. The Revolve's axis is world X, or any axis through the
world origin that lies in a world coordinate plane; every step reads that
axis as `agent-docs/revolve-axis-design.md` states. This document states the
construction in its world X form, which that document generalizes. The section consists of whole straight edges.
Exactly forward-collinear edges become one geometric span; every such span
keeps the complete set of recorded source roles and its retained face. The
section has one axial span. A candidate root span must have a finite positive
conical surface, and its recorded meridian is compared with that surface over
the source fan's complete axial reach. No fixed number of geometric spans
enters this source certificate.

The point-Loft source must still be the current Sketch profile, revision,
frame and ordered entity list recorded by Loft. Sketch alone certifies two
exact crossings against the reported cone. The first admitted class has two
connector-line crossings next to one strictly inside root arc. The second
has two fitted-spline crossings with strict source-side transitions and a
strict local coordinate derivative. All other whole entities need strict
Sketch side certificates with margins above the source-fan displacement and
finite-meridian allowance. A missing, stale, approximate or ambiguous answer
refuses the route. A Partial edge is admitted only on the fitted route and only
when Sketch reports its bounds exact (`TExact`). Two kinds qualify. A fitted
spline fragment must record exactly the span Sketch reports, in the edge's walk
direction, and its cone crossing must lie strictly inside that span. A circle
fragment, the root of a tooth whose involute base circle lies inside the root
circle, has no Sketch side certificate, so decad proves its side itself: an
exact rational interval box around the reported parameter range, widened by a
fixed relative slack and one ulp, with the sign of the cone residual over the
box and a margin above the same displacement allowance. The fragment may span
less than a quarter turn. It serves as the inside segment. Any other Partial
edge refuses the route.

The certificate retains ordered section spans, both axis endpoints through
the axial span, root face and cone, Sketch crossing/side answers, source
revision, sector endpoints, pre-refinement exposed loop, and crossing bound.
The exterior path chooses the unique pair of source fan cells that contain
the two contacts. The fitted route uses Sketch's parameter brackets, local
coordinate derivative and recorded Loft stations. Both routes charge the
crossing enclosure separately from the source fan displacement.
The loop keeps source group and root-contact labels on each edge. The joined
body retains the original sources and published boundary, occupied-volume
and area allowances. Placement clears
the direct join certificate and records an authenticated rigid copy of the
original trimmed source.

## Sector construction

The builder meshes every recorded meridian span at one shared angular plan.
It omits a root-face interval only where a certified sector cap replaces that
interval. Its rings follow section order, so a shoulder or split cap adds a
ring without changing any positional profile role. The source cap is
triangulated and refined before placement by each authenticated axis
rotation. The original unplaced source controls each cone map; placement is
applied once after that map. The placed caps share the exact ring vertex IDs
at every root station.

A copied sector's axis-X placement has zero translation and no reflection.
Its X/YZ cross terms are exactly zero, its YZ block has the proper rotation
relations, and its held X basis length differs from one by at most one float
step. `MapChargeOf` charges that axial basis defect in source and cap placement
bounds. The builder also checks each placed cap's measured snap to the shared
root ring. A larger axial defect or a tilted axis refuses this route. About a
general axis, `massmoment.AxisRotation` admits the copy and charges its drift
(`agent-docs/revolve-axis-design.md` §4.5, §5.5).

For repeated sectors, the complete trimmed source solids must be separate by
one certified projection for every pair. The builder first checks expanded
world-coordinate ranges. When those ranges overlap, it chooses a YZ direction
from the held box centers and projects every held source vertex with exact
rational arithmetic. Each projection range expands by the source spatial
bound and authenticated placement charge. A direction whose enlarged ranges
overlap refuses the pair; choosing the direction alone proves nothing.
A seam is selected from the
unused angular complement. Each pair of root intervals must remain separate
after source mesh, crossing, and placement bounds are converted to an angular
allowance at the finite root radius. The seam uses held root intervals and
those pairwise margins; it is not a separate Sketch side certificate. All
admitted sectors use one ring mesh.
Serial and variadic `Union` use the same constructor; serial joins retain the
original Revolve, source certificate, placed sources, and dynamic ring IDs.

Each construction must pass connected orientation, vertex links, the exact
triangle contact audit, a 0.1 mm boundary bound, finite area and occupied
volume bounds, and a measured-volume increase over the original Revolve.
For each cap whose held triangles form a single graph over the plane normal
to the Revolve axis, the audit may omit pairs within that cap. It proves
strict and consistent projected triangle orientation, opposite use of each
internal edge, one directed boundary loop, and a simple exact projected
boundary. These facts make the oriented triangle chain cover its boundary
region once, so distinct triangle interiors cannot meet. Failed cap proofs
leave every pair in the exact audit. Separate cap families and all cap-wall
and wall-wall pairs always remain in the audit. A family-aware box tree
counts its node visits and leaf comparisons under the existing per-call
scan ceiling before allocating candidate pairs. Large candidate sets use
bounded row batches and compact exact-coordinate tables. Small candidate
sets retain a bounded list. Both paths keep indices over the full mesh.
The collapse check and projected-cap proof inspect every triangle first;
structured Loft audits continue to prepare their complete exact tables.
The audit triangle allowance follows the section-ring and cap cell counts,
with a 1.2-million-triangle memory ceiling. The crossing scan allowance is
the smaller of all triangle pairs and 25 million scans per admitted sector.
If the cone cap interpolation term exceeds the remaining 0.1 mm boundary
budget, the builder splits only cap cells above that term's target. Shared
edges split together, including root edges. It rebuilds the shared ring and
recomputes the cone cap and full boundary bounds after refinement. Each
construction attempt permits at most three additional cap passes within the
same mesh ceiling; a cap that still exceeds the budget refuses the join.
A cap station of the tool's upper cone whose Q falls short of 1 by more than the
sample's error is refused, because the cone cap would then reach the far source
plane. When the Boolean that recorded the cone was called with
`WithMatchingFaces`, a shortfall of at most 64 steps at 1 is taken as the
intended match instead: the station takes the far point, charged for its
distance from the smooth cone point like a station inside the error band. A
larger shortfall refuses. The refusal names the shortfall, and when it is within
the 64 steps it says to pass `WithMatchingFaces` to the Cut or Intersect that made
the cone cut. Every sector carries the same cap, so the joined mesh grows with the sector
count. With fewer than four sources, the first attempt subdivides each cap four
times; with four or more sources, it subdivides each cap twice. When the
mesh would pass the triangle ceiling or the crossing audit's scan allowance, the builder
retries with one fewer subdivision, down to none, without using up an attempt, and cap refinement adds cells back only
where the bound needs them. The sector count has no fixed limit of its own.
The builder makes at most eight construction attempts. Before the next one, a cap
bound refusal lowers the cone cap limit, a specific cap/root adjacency contact
or a placed cap whose root contact misses the ring stations by more than the
seam allowance quarters the angular sag allowance, and an occupied-volume
refusal scales that allowance by a factor from 0.1 to 0.5 set by its measured
margin. Each new
mesh must pass the same exact audit. Refusal leaves all operands live. The
source class requires an unplaced, full Revolve about an axis that
`agent-docs/revolve-axis-design.md` §3 admits. Transformed Revolves and
other axes use the ordinary Boolean route or refuse.

## Coaxial cylinder Cut

`Cut` accepts a full centered cylinder spanning both axial ends of a joined
Revolve/point-Loft sector body. This section states the route for a
certificate about world X from a Revolve that passes the frame predicate.
A certificate from the general record branch reads the same route about its
own axis (`agent-docs/revolve-axis-design.md` §9).
It reads the same authenticated section,
ordered ring IDs, source groups, and published boundary, occupied-volume and
area bounds as `Union`. The two spans adjacent to the section's sole axial
span must be planar radial caps. Their ring IDs identify one complete axis
fan per angular station. The cylinder side keeps its source face and origins.
The result clears the join record because a second structural rewrite has no
unmodified source mesh to authenticate.

When the bore radius is strictly inside both cap outer radii, `Cut` removes
both complete axis fans. It inserts two annuli and one cylindrical wall at
the joined ring's angular stations. Every other meridian span and every
untouched triangle must clear the cylinder by the joined and new bore bounds.
The cap census removes one old triangle per cap and station, then adds six;
the net count increases by four per station.

When the bore radius lies strictly between the low cap outer radius and its
adjacent straight band's outer radius, `Cut` solves that band's one crossing
with exact rational source coordinates. It removes both cap fans and exactly
two band triangles per station. It inserts the clipped band, the high annulus
and the cylindrical wall. Every other section span and untouched triangle
must clear the bore. This census increases the triangle count by two per
station. Equality at either band radius refuses the route.

The retained Union station gaps are rechecked with rational intervals. The
circular bore ring bounds its own trigonometric rounding and chord sag at
those stations. Existing bore callers keep their 0.025 radian station limit;
this route supplies the certified joined ring's largest step and requires a
0.1 mm result boundary bound. The rewrite limits stations to 2,304, audited
triangles to 1,321,696 and audit scans to 400 million. It checks connected
orientation, vertex links and triangle contacts before building a result.
The result audit proposes each copied face group and each half-annulus as a
projected YZ patch. It proves each patch from the result triangles; failed
proposals retain every pair. Only pairs inside a proven patch leave the exact
contact audit. Cross-patch, patch-to-wall and all unproved pairs remain in it.
The grouped box tree counts its work under the same 400-million ceiling.
Above one million candidates, its row queries are separately preflighted
under that ceiling and classified in bounded batches without a complete pair
list. The result does not reuse the earlier Union's contact verdict.
The occupied-volume allowance adds the cylinder's radial shell to the
joined allowance. A clipped band also adds its side shell and crossing slab.
The area allowance includes the held areas and new wall. Failed checks leave
the target and tool bodies live.

The real neutral acceptance model uses a tapered Revolve hub whose outer cap
has two exactly collinear source edges. A separate Sketch makes a radial lug,
point-Loft makes its solid fan, and two real conical tools trim its toe and
heel. Public `Union` passes `Tessellate(0.1 mm, VerifyAll)` for both straight
connector and fitted flank sources. The mesh has Euler characteristic two,
both verification flags are true, and the joined computed-volume lower bound
exceeds the hub's computed-volume upper bound. Its joined faces retain both
split-cap source roles and an analytic root cone.

## Public routing

Public serial and variadic `Union` use the same certified section-ring and
sector-cap builder. The root routes collect the placed copies;
`internal/revolveshell` certifies the sources (the recorded Revolve section,
its conical roots and the trimmed sector), traces the exterior loop, builds,
refines and audits the joined body, and attaches its join record. Public `Cut` uses the
joined section record for a centered, spanning cylinder; the Boolean dispatch
hands both operand nodes to `internal/revolveshell`, which admits the cylinder
and builds the annular or straight-band result. The legacy six-line join and
bore constructors are absent. A section with unsupported source geometry or an
unproved contact refuses. Full gear acceptance tests run in `decad-gallery`;
Decad tests use small Revolve and point-Loft producers.
