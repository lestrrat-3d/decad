# Point-section loft

`Document.LoftFromPoint(ctx, apex, sketch, profile)` builds the cone over one
authenticated, closed Sketch region. The point is a world coordinate in
millimetres. The region keeps its recorded segment kinds and exact Sketch trim
ranges. The method accepts a single outer loop and refuses holes: coning every
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

`Document.Split` admits an unplaced `LoftFromPoint` body and one unplaced,
single-line `ExtrudeChain` sheet whose source sketch is world XY and whose
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
`Placed` copy maps the certified centroid and widens its bound by the
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

`Body.ScaledAt(ctx, anchor, factor)` accepts a complete authenticated Split
slab with a finite, positive dimensionless factor. The anchor is a world
point in millimetres. It maps each held vertex by `anchor + factor*(p-anchor)`;
an exact rational image of the stored floats encloses the rounding of that
evaluation. The mesh embedding audit may adjust a rounded vertex within its
float box or refuse the image. Each source vertex bound grows by the factor
plus that vertex's image rounding. The occupied-volume gap grows by the cube
of the factor plus the rounded boundary's swept volume. Area slack grows by
the square of the factor plus the rounded facets' area allowance.

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

## Coaxial finite cone trims

`Cut` and `Intersect` retain the point loft's authenticated fan when the
tool is an unplaced, full Revolve of a triangular straight meridian. The
tool axis is world X, its apex is at positive X, and its finite far cap is
at negative X. Every source fan vertex minus the source mesh bound must
lie strictly ahead of that far cap. A tool outside this class takes the
ordinary mesh Boolean path.

The cone is recorded as `g(p)=x+k·hypot(y,z)` and `alpha(p)=a/g(p)`,
with exact-rational apex `a` and slope `k` read from the meridian. A
`Cut` retains rays after the cone crossing; an `Intersect` retains rays
before it. The outside result requires a far-cap vertex with
`g(p)/a−1` strictly larger than its arithmetic error and the source
mesh displacement. This is the nonempty witness for a thin outside piece
even when its volume interval includes zero. Chained cuts and
intersections rebuild from the same source fan. Two limits of either kind
must be strictly ordered over the whole far cap; a lower Cut limit must
precede an upper Intersect limit. Other orders take the mesh path.

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

## One-tooth bevel blank union

`Union` admits one unplaced, axis-X, six-line Revolve blank and one unplaced
point loft after its toe `Cut` and heel `Intersect` when their source records
match. The blank has a finite root meridian between its toe root and dedendum
stations. The tooth profile has two whole fitted flanks, a tip arc, a root
arc, and two root connector lines. A different shape uses the usual Boolean
path and may refuse on its held-mesh bound.

The join reads the original Sketch entities and revision recorded by
`LoftFromPoint`. It asks Sketch for exact line crossings with the blank's
reported root cone and whole-curve side certificates. The two flanks and tip
arc must lie outside that cone; the root arc must lie inside. Every
certificate must name the recorded entity, cone, frame and current sketch.
Its exact implicit margin must exceed the source fan's certified displacement
and the finite meridian's difference from the reported cone. The line-root
contacts are then checked against the actual finite meridian, including the
Sketch crossing interval and float point-evaluation error. The toe and heel
tool meridians must meet the corresponding blank stations within the charged
seam allowance.

The mesh removes only the covered angular span of the blank root face. Its
replacement tooth caps and side walls reuse the blank's root-ring vertex
indices. A regular 256-chord angular grid bounds blank circular departure;
four cap subdivision rounds bound the cone-map interpolation. Construction
checks directed edge pairing, one component, every vertex link, and exact
nonadjacent triangle contacts under an 82,000-triangle and fixed
64-million-scan work limit.
The resulting faceted body carries a boundary displacement below 0.1 mm,
an occupied-volume bound, and the original Cone and NURBSSurface face tags.
`Tessellate(0.1 mm, VerifyAll)` restates the audited mesh and both proofs.

The reference 8/8 blank and one trimmed tooth pass this path. The four-round
cap bound proves positive inertia in `MassProperties` at a density of
0.001 kg/mm³ while retaining the occupied-volume and mesh-bound proofs.
Further `PlacedCopy` teeth can join through the shared-ring continuations
below, up to eight certified sectors.

For a blank and tooth built from the same unplaced gear source, the public
operation order is:

```go
toeKeeper, err := decad.Cut(ctx, tooth, toeCone)
if err != nil { return err }
trimmed, err := decad.Intersect(ctx, toeKeeper, heelCone)
if err != nil { return err }
joined, err := decad.Union(ctx, blank, trimmed)
if err != nil { return err }
mesh, err := joined.Tessellate(ctx, units.Millimeters(0.1),
    decad.WithVerification(decad.VerifyAll))
if err != nil { return err }
if !mesh.BoundaryVerified() || !mesh.VolumeVerified() {
    return fmt.Errorf("joined bevel tooth lacks a verified solid")
}
```

`apitest/bevel_join_test.go` builds and verifies the 8/8 blank and its first
trimmed tooth through public Sketch and Decad APIs.

## Two-tooth bevel blank union

`Union` admits the one-tooth joined body and one `PlacedCopy` of the same
trimmed tooth when the copy is a non-identity rotation about the blank's X
axis. The first join carries its immutable blank and trimmed-tooth records;
the copy carries the original cone-trim record and its rigid motion. Placement
does not preserve a cone-trim record for another Cut or Intersect. An unrelated
faceted pair follows the ordinary mesh Boolean path.

Before rebuilding, the join checks the original Sketch source again and asks
Sketch for fresh cone-crossing and whole-curve side certificates. The two
complete trimmed tooth bodies must have separated Z ranges after each held
mesh's boundary displacement and the placed basis's stretch are charged.
Their root angular sectors must remain separate after crossing, placement
and stretched mesh errors become an angular allowance at the smaller finite
root radius. A stale profile, a different source tooth, or sectors whose
separation cannot be proved do not enter this structural path.

Both caps are built from the original source's subdivisions. The copied cap
points move through its recorded rotation. The blank's root face is removed
over each certified sector, and both cap boundaries use the same root-ring
vertex indices as the blank. Inside either sector, only that cap inserts
angular stations into the ring; a second regular-grid station can differ by a
few ulps after rotation and leave an open edge. The final mesh passes directed
edge pairing, one component, vertex links and exact nonadjacent triangle
contact checks. The two-tooth contact audit has a fixed 256-million-scan cap,
four times the one-tooth cap for up to twice as many triangles.

The boundary bound charges the blank ring once and takes the larger of the
two tooth errors. The held rotation basis's exact stretch and determinant
charge its boundary, source-area and occupied-volume allowances. The
occupied-volume allowance charges both source fans and both cap maps, then
sweeps the one shared held boundary by its construction error. The copied
tooth's placed source faces and the original tooth's faces
retain their Cone and NURBSSurface tags. The reference 8/8 pair passes
`Tessellate(0.1 mm, VerifyAll)`. Its result retains the immutable blank,
original trimmed tooth and placed copy for the following continuation.

## Three-to-eight-tooth bevel pattern union

The canonical gallery order in
`fusion360-gear-generator/spec/bevelgear/instructions.md` (steps 991–1027)
revolves the blank, lofts one point tooth, cuts its toe and heel with cones,
makes seven 45° `PlacedCopy` instances, and joins all eight teeth in order.
A conditional centered bore Cut follows the joined gear body.

For each third through eighth Union, the receiver must carry the retained
blank and every prior tooth from this same live Sketch and cone-trim source.
The incoming body must be a recorded axis-X rotation of that source. The
join rechecks the live Sketch certificates, proves every pair of complete
trimmed tooth solids disjoint with their held coordinate boxes and boundary
bounds, including each held rotation basis's stretch, and proves every pair
of root sectors separate after crossing, placement and mesh errors. A
different source, non-rigid placement or uncertified overlap follows the
ordinary Boolean path.

All admitted teeth are rebuilt on one blank root ring. The ring starts at
−22.5° so the fifth, 180° tooth remains inside one angular interval rather
than crossing its seam. Each cap owns the root stations inside its sector;
the blank and caps share the same root vertex indices. Directed edge pairing,
one component, vertex links and exact nonadjacent triangle contacts are
checked again for every result. The audit allows 44,480 triangles for the
two-tooth baseline plus 22,000 for each added sector, and up to 12 million
scanned pairs per tooth. The real 8/8 final mesh has 171,776 triangles and
its audit scanned 73,504,567 pairs, both below the eight-tooth limits of
176,480 triangles and 96 million scans.

The boundary allowance charges the shared blank ring once and uses the
largest local tooth error. Occupied-volume and area allowances charge every
source fan and cap map, including the exact stretch and determinant of each
held placement basis. The shared result's swept-boundary allowance is charged
once. Every source face retains its Cone or NURBSSurface tag. The real 8/8
producer passes eight sequential Unions and `Tessellate(0.1 mm, VerifyAll)`.
`decad-gallery/models/bevelgear/join_test.go` checks the complete eight-tooth
build. Decad checks cap-owned root stations across the 180° seam and circular
sector separation in `bevel_join_pattern_test.go`.
The final body retains the immutable blank and all eight certified tooth
sources and both complete cap rings. A post-pattern Cut admits a single
centered, axis-X, full-circle Sketch prism that spans both blank caps.
The tool's held ends are compared with the cap stations as exact sums, so
a rounded end cannot pass a cap it only touches. The tool's cylindrical
side face supplies the inward wall source.

The bore rewrite first proves that every non-cap facet lies outside the
cylinder by its held bound and the new circle bound. It checks one axis-fan
triangle per angular sector on each cap, replaces both fans with annular
strips, and sews the cylindrical wall between their inner rings. Directed
edges, vertex links, and exact nonadjacent triangle contacts are checked
again. The 1,280-station bore mesh has 176,896 triangles, below its bounded
181,600-triangle audit limit, and uses a 128-million-pair scan limit.

The original joined body's occupied-volume allowance remains valid when
the exact cylinder is removed from both the true and held solids. The
circle-to-polygon radial shell adds the bore's own occupied-volume allowance
over the blank's exact cap-to-cap span. The area allowance also covers both
removed cap discs and the new cylindrical wall.
`decad-gallery/models/bevelgear/bore_test.go` checks the complete eight-tooth
body with a through-bore. Decad checks cap-fan sector pairing and bore
clearance decisions in `bevel_join_bore_test.go`.
