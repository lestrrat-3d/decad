# Point-section loft

`LoftFromPoint(ctx, doc, apex, sketch, profile)` builds the cone over one
authenticated, closed Sketch region. The point is a world coordinate in
millimetres. The region keeps its recorded segment kinds and exact Sketch trim
ranges. The function accepts a single outer loop and refuses holes: coning every
hole loop to the same point would give the apex a non-disk vertex link.

The apex must lie off the profile plane. The evaluator keeps one apex vertex,
one far vertex per certified chord station, one wall triangle per station
interval, and a triangulated far cap. It checks the held triangles for
crossings, then orients the complete shell by its signed volume. Wall faces
carry `side(0,j)` roles for the original segment `j`; the cap carries
`capEnd`. The body records the original profile and plane beside the
held mesh, so placement and later consumers can identify the source curve.
It also retains certified source station bounds, per-cell arc-speed uppers
and parameter-matched departures for a later face-fed loft.
No small profile stands in for the point.

The profile passes the ordinary Sketch seam and area falsifier. The existing
Loft station rules chord each recorded segment against its own certified
bound. A sampled loop whose non-adjacent chord tubes overlap is refused before
construction. The body refuses a cap triangulation or a fan that collapses.

For each wall cell, the source boundary lies within its certified chord
departure of the held far chord. Linear interpolation from the common apex
scales this departure by at most one, so the same bound covers the whole fan.
The cap's region difference is bounded by the section displacement area tube.
Its cone has at most the distance from the apex to the plane origin times that
area, divided by three, in occupied-volume difference. A separate swept-mesh
term covers rounding of the held far vertices. The area proof adds the cap
region difference and an upper bound on both the true and held wall areas.
Every mesh facet retains a source face role; `VerifyAll` checks the closed
boundary and publishes the finite occupied-volume allowance.

The result has the ordinary `BodySolid` readings and faceted restatement.
It accepts placement through the faceted payload. A chord tolerance below its
held mesh bound is `ErrUnsupported`; the caller may use a larger tolerance.
This first point-section form has no holes, section alignment or surface
result option.

## Planar split of a point loft

`Split` admits an unplaced `LoftFromPoint` body and one unplaced,
single-line chain `Extrude` sheet whose source sketch is world XY and whose
straight sweep is along world Z. The line and sweep must span every cut
section, and every source ray must cross the sheet plane strictly between the
apex and the far profile. A split result may be split again by another sheet
in this class. Other point-loft/sheet pairs keep `ErrUnsupported`.

The sheet's stored line endpoints define its exact rational plane. For each
far-section point `p`, the ray from the recorded apex `a` meets that plane
at `a + α(p)(p-a)`, with `α(p) = d/(n·(p-a))`. Here `n` is the exact cross
product of the line direction and world Z, and `d` is the exact plane
offset from the apex. The entry gate uses rational signs and the original
point loft's boundary displacement to prove `0 < α < 1` over its whole
source cap. Consecutive planes are ordered over that same cap before any
piece is built. A sheet missing a ray, crossing an existing cut plane, or
ending inside the section refuses without consuming either operand.

Every result is rebuilt from the original point-loft fan, never from a
previously rounded slab. The evaluator maps each original far-ring station
to its lower and upper plane, adds the corresponding cap triangles, and
keeps the original side face roles beside fresh `capStart` and `capEnd`
roles. A boundary crossing audit must accept each rounded result before
the document commits both pieces. The cap faces report the cutting sheet's
`Plane` surface. Their outward signs come from comparing the actual sheet
face frame with the exact apex-away plane normal, independently of Sketch
line endpoint order. The original far cap keeps its source-plane tag and
takes its outward sign from the exact side of that plane holding the apex.
Their selectors remain planar after subsequent splits.

The source-to-held boundary bound is carried through the rational ray-plane
map using a lower bound on its denominator and an upper bound on its
Lipschitz factor. Exact-rational intersections are rounded once; their
coordinate errors add to that bound. The source point loft's occupied-volume
allowance also bounds each exact clipped piece: intersecting two sets with
the same half-spaces cannot enlarge their symmetric difference. A separate
swept-boundary allowance covers rounding of the clipped held mesh. Area
slack bounds each true and held patch separately before summing their
non-cancelling difference. `Tessellate(0.1 mm, VerifyAll)` exercises both
the boundary and occupied-volume certificates on the real tooth.

`Face.Centroid` returns a bounded area centroid for each complete far or
split cap. The far cap lifts the authenticated profile centroid through its
recorded plane. A split cap integrates the exact rational image of the
original far station polygon. The validated ray map preserves its simple
boundary, and the mapped source curve lies within the proved displacement
of its chord walk. A tube around that walk bounds the region difference.
If the tube area is Δ, the polygon area has lower bound A, and every point
in the difference lies at most D from the polygon centroid, the centroid
bound is ΔD/(A−Δ) plus coordinate rounding. A nonpositive area floor leaves
the Split valid but makes this query return `ErrUnsupported`. A rigid
`Transform` maps the certified centroid and widens its bound by the
placement's proved stretch and rounding. A source group split into separate
face patches cannot lend its whole-group centroid to either patch.

`Body.ExtremeFacesByCentroid` certifies the least and greatest centroid
projections across **all** faces of a complete point-loft Split slab. Its two
caps have bounded `Face.Centroid` readings. Each side patch is a subset of
the source cone between the lower and upper cap rays. Along one ray, conical
area density is proportional to its ray fraction `t`, so the side patch's
conditional centroid fraction lies between 1/2 and 2/3 of the cap interval.
The selector bounds both cap contour projection ranges from held ring
vertices, then evaluates both fraction endpoints for a forward or reversed
axis. It returns cap faces only when their centroid intervals strictly
separate the complete side interval. Otherwise it returns `ErrUnsupported`.

Rigid placement retains the immutable Split source record and every applied
rigid delta in order for this query and a later face-fed loft.
The cap contour displacement is carried separately from the generic held
mesh bound: each placement charges the exact basis stretch of the previous
source gap plus the new rounding and embedding bound. A separate original
source marker prevents any placed copy, including one moved back to its
original pose, from becoming a structural Split source again.

## Anchored uniform scale of a Split slab

`ScaledAt(ctx, b, anchor, factor)` accepts a complete authenticated Split
slab with a finite, positive dimensionless factor. The anchor is a world
point in millimetres. It maps each held vertex by `anchor + factor*(p-anchor)`;
an exact rational image of the stored floats encloses the rounding of that
evaluation. The mesh embedding audit may adjust a rounded vertex within its
float box or refuse the image. Each source vertex bound grows by the factor
plus that vertex's image rounding. The occupied-volume gap grows by the cube
of the factor plus the rounded boundary's swept volume. Area slack grows by
the square of the factor plus the rounded facets' area allowance.
The body centroid's diameter allowance includes the scaled source diameter
and the new held coordinate-box diagonal plus twice its mesh displacement.

The result keeps the original `pointPlaneSliceRecord`, including the source
Sketch revision, far station order and rational lower/upper ray-plane maps.
A separate record appends the accumulated rigid placement, each positive
scale with its anchor, and later rigid placements in order. Cap origins and
the original cap-role selectors stay on their faces. Scaling the cap-contour
displacement and both cap-centroid bounds lets
`ExtremeFacesByCentroid` reselect the ends after the scale. The result clears
the original-source marker, so `Split` cannot re-enter the structural arm.
Rebuilding a cap `Plane` frame may round its normal again; `NormalAt` reports
the conservative two-unit direction bound used for placed planar tags.

A point loft that is not a Split slab scales as an unmarked faceted solid
(`agent-docs/evaluator-design.md` "Positive uniform scale of a prism, stack,
solid Revolve, or faceted solid"). Its image drops the `PointSection` and
`PointCone` records, so it no longer feeds a cone trim, a `Split` or a
face-fed loft as a point-loft source.

## Coaxial finite cone trims

`Cut` and `Intersect` retain the point loft's authenticated fan when the
tool is an unplaced, full Revolve of a triangular straight meridian about an
axis through the world origin. The axis is world X, or an exact vector `A`
that lies in a world coordinate plane and that the tool Revolve records
(`agent-docs/revolve-axis-design.md` §3, §4.3). The tool's apex lies on the
positive side of the origin along the axis, and its finite far cap on the
negative side. Every source fan vertex minus the source mesh bound must lie
strictly ahead of that far cap. A tool outside this class takes the ordinary
mesh Boolean path. A second limit joins a trim only about the axis of the
limit the trim already holds; a toe about X and a heel about Y take the mesh
path.
An untrimmed point-loft target enters this arm only while its source record
survives. Ordinary placement keeps that record only for exactly identity
deltas; an exact rotation copy drops it regardless of angle. Later float
composition to identity cannot restore it, so that pair takes the mesh path.

The cone is recorded as `g(p)=x+k·hypot(y,z)` and `alpha(p)=a/g(p)`,
with exact-rational apex `a` and slope `k` read from the meridian. About a
general axis `x` is the exact `s(p)=p·A`, `hypot(y,z)` is `σ(p)=|p×A|`, and
`a` is the A-scaled apex `c=P·A`, so `alpha` is the same crossing. Orderings,
the far-cap test and the witness widen the mesh bound by `|A|↑`, and the cell
bounds below read millimetre geometry and `a↑ ≥ c/|A|`
(`agent-docs/revolve-axis-design.md` §3.4, §3.5, §5.4). A
`Cut` retains rays after the cone crossing; an `Intersect` retains rays
before it. The outside result requires a far-cap vertex with
`g(p)/a−1` strictly larger than its arithmetic error and the source
mesh displacement. This is the nonempty witness for a thin outside piece
even when its volume interval includes zero. Chained cuts and
intersections rebuild from the same source fan. Two limits of either kind
must be strictly ordered over the whole far cap; a lower Cut limit must
precede an upper Intersect limit. If the reverse proof establishes that the
upper Intersect limit precedes the lower Cut limit over the whole far cap, the
band is empty. Overlapping or uncertain orders take the mesh path.

Each far-cap triangle is subdivided before its cone image is chorded.
Its coordinate box proves `rho>=rhoMin>0` and `g>=gMin>0`; its longest
edge is `h` and its greatest source reach is `R`. On that cell,
`||grad g||<=1+k` and `||H g||<=k/rhoMin`, so
`||H alpha||<=2a(1+k)^2/gMin^3 + ak/(rhoMin·gMin^2)`.
The interpolation allowance for `alpha` is `h²/2` times that bound.
The cone-image allowance adds `R` times the alpha allowance and
`a(1+k)h²/gMin²` for the product `p·alpha`. The largest cell
allowance, exact-to-float crossing errors, the source fan bound, and
subdivision/contact rounding form the published mesh displacement.

The cap-area volume term multiplies interpolation displacement by the
source cap area and the cone image-area factor
`max(1,L²)`, where `L=a/gMin+R·a(1+k)/gMin²` bounds the crossing
map's stretch on each cell. The source fan's occupied-volume allowance
and the final vertex sweep are added. Retained flank patches carry
`NURBSSurface` tags; the cut or intersection boundary carries the
source `Cone` tag, origins, orientation and denoted-normal certificate.
Placement moves the cone tag and denotation, and drops the radial-chain
record.

## Finite Revolve and point-Loft booleans

A finite full Revolve section can join one or more authenticated cone-trimmed
point-Loft sectors. Serial and variadic `Union` use the same section-ring and
sector-cap construction. A centered spanning cylinder can `Cut` its completed
caps or one straight band. The source, construction, and proof contracts live
in `agent-docs/revolve-loft-sector-design.md`. Full gear models and their acceptance
tests live in `decad-gallery`.
