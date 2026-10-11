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
three cap subdivision rounds bound the cone-map interpolation. Construction
checks directed edge pairing, one component, every vertex link, and exact
nonadjacent triangle contacts under a fixed 64-million-scan work limit.
The resulting faceted body carries a boundary displacement below 0.1 mm,
an occupied-volume bound, and the original Cone and NURBSSurface face tags.
`Tessellate(0.1 mm, VerifyAll)` restates the audited mesh and both proofs.

The reference 8/8 blank and one trimmed tooth pass this path. Its current
three-round cap bound proves occupied volume but is too broad to prove
positive inertia in `MassProperties`; that call returns `ErrUnsupported`.
One further `PlacedCopy` tooth can join through the two-tooth continuation
below. More teeth use the ordinary Boolean path.

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
mesh's boundary displacement is charged. Their root angular sectors must be
separated after the crossing, placement and mesh errors are converted to an
angular allowance at the smaller finite root radius. A stale profile, a
different source tooth, or sectors whose separation cannot be proved do not
enter this structural path.

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
`Tessellate(0.1 mm, VerifyAll)`. The result has no retained join record, so a
third tooth follows the ordinary mesh Boolean path. In the real 8/8 gallery
sequence, that third Union refuses because the two-tooth body's 0.0774 mm
held boundary bound exceeds its 0.000236 mm pair chord. Joining the full
pattern needs a separate source-certified continuation for more sectors.
