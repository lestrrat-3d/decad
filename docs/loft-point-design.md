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
